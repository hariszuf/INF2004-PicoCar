
/**
 * @file line_follow.c
 * @brief Two-sensor line following (decision logic from ir_motor_curve_test.c)
 *        driving the wheels through Buddy 2's motion_drive().
 *
 *   both on line   -> straight
 *   one on line    -> steer toward the side that still sees it (inside wheel slower)
 *   none on line   -> keep turning toward the last side for LINE_RECOVER_MS, then stop
 *
 * Straddle mode (LINE_SENSORS_STRADDLE, line between the sensors) with
 * LINE_BOTH_BLACK_STOP: white = move, black = stop.
 *   both white     -> straight
 *   one black      -> that side's wheel stops, the car turns toward that side
 *   both black     -> stop; moves again as soon as a sensor sees white
 */
#include "line_follow.h"

#include "line_config.h"
#include "line_sensors.h"
#include "motion_control.h"

#ifdef LINE_BARCODE_DECODE
#include "barcode_scan.h"   /* also compile barcode_scan.c and barcode39.c */
static uint32_t s_clock_us;  /* line_follow_update() runs every LINE_UPDATE_PERIOD_MS */
#endif

static line_state_t s_state = LINE_STATE_IDLE;
static uint8_t s_mask;
static float s_left_cmd;
static float s_right_cmd;
static int32_t s_last_side;      /* +1 steer left, -1 steer right */
static uint32_t s_lost_ms;
static uint32_t s_side_ms;       /* time steering in the same direction */
static bool s_active;

#if LINE_JUNCTION
typedef enum
{
    J_NONE = 0,
    J_STOPPING,    /* waiting for the car to come to a halt */
    J_WAITING,     /* standing still at the junction for LINE_JUNCTION_PAUSE_MS */
    J_ADVANCING,   /* driving forward to the middle of the junction */
    J_TURNING,     /* open-loop first part of the turn */
    J_SEARCHING,   /* slow pivot until a sensor sees the new line */
    J_CROSSING,    /* driving straight across */
    J_CENTRING,    /* rejoin only: keep pivoting until the line is between the sensors */
    J_CENTRE_BACK  /* rejoin only: pivot back to the middle of the two sensor crossings */
} junction_phase_t;

static junction_phase_t s_junction;
static bool s_junction_armed = true;   /* false while still on the bar we just handled */
static bool s_junction_halted;         /* stopped at a junction with no barcode letter: stay put */
static junction_action_t s_junction_action;
static uint32_t s_junction_ms;         /* time in the current junction step, for the timeout */
/* Rejoin: the same junction turn, used by the obstacle bypass to get back onto the line. */
static bool s_rejoin;
static int8_t s_rejoin_result;         /* 0 running / not started, 1 done (centred, stopped), -1 failed */
static uint8_t s_centre_first;         /* the sensor that saw the line first */
static bool s_centre_have_exit, s_centre_have_enter;
static float s_centre_exit_deg, s_centre_enter_deg, s_centre_target_deg, s_centre_dir;
#if LINE_JUNCTION_USE_BARCODE
extern bool buddy3_barcode_latest(char *letter, uint32_t *read_ms);   /* barcode_task.c */
static bool s_barcode_used;
static uint32_t s_barcode_used_ms;
#endif
#endif

static void drive(float left, float right)
{
    /* Give Buddy 2 equal targets on straight segments so it can balance them. */
    s_left_cmd = left == right ? left : left * LINE_LEFT_WHEEL_SCALE;
    s_right_cmd = left == right ? right : right * LINE_RIGHT_WHEEL_SCALE;
    (void)motion_drive(s_left_cmd, s_right_cmd);
}

static void steer(int32_t side, float inner)
{
    if (side > 0)
    {
        drive(inner, LINE_TURN_OUTER_SPEED_CM_S);      /* turn left */
    }
    else
    {
        drive(LINE_RIGHT_OUTER_SPEED_CM_S, inner);     /* turn right: left wheel faster */
    }
}

#if LINE_JUNCTION
static junction_action_t junction_choose(void)
{
#if LINE_JUNCTION_USE_BARCODE
    char letter = 0;
    uint32_t read_ms = 0u;

    if (buddy3_barcode_latest(&letter, &read_ms) && ((!s_barcode_used) || (read_ms != s_barcode_used_ms)))
    {
        junction_action_t from_barcode = junction_action_for_letter(letter);

        if (from_barcode != JUNCTION_NONE)
        {
            s_barcode_used = true;
            s_barcode_used_ms = read_ms;
            return from_barcode;
        }
    }
#endif
    return LINE_JUNCTION_DEFAULT_ACTION;
}

static void junction_begin(void)
{
    s_junction_action = junction_choose();
    s_junction_ms = 0u;
    s_junction_armed = false;
    s_state = LINE_STATE_JUNCTION;
    s_left_cmd = 0.0f;
    s_right_cmd = 0.0f;
    motion_stop();
    if (s_junction_action == JUNCTION_NONE)
    {
        /* No barcode letter for this junction: do not guess. Stop here and stay stopped. */
        s_junction = J_NONE;
        s_junction_halted = true;
        return;
    }
    s_junction = J_STOPPING;
}

static void rejoin_finish(int8_t result)
{
    s_rejoin = false;
    s_rejoin_result = result;
    s_active = false;   /* hold still; the bypass restarts normal following with line_follow_start() */
    s_junction = J_NONE;
    s_state = LINE_STATE_STOPPED;
    s_left_cmd = 0.0f;
    s_right_cmd = 0.0f;
    motion_stop();
}

static float reported_heading(void)
{
    motion_status_t m;
    motion_get_status(&m);
    return m.heading_deg;
}

static void junction_abort(void)
{
    if (s_rejoin)
    {
        rejoin_finish(-1);
        return;
    }
    s_junction = J_NONE;
    s_state = LINE_STATE_STOPPED;
    s_left_cmd = 0.0f;
    s_right_cmd = 0.0f;
    motion_stop();
}

static void junction_start_search(void)
{
    float v = LINE_JUNCTION_SEARCH_SPEED_CM_S;

    s_junction_ms = 0u;
    s_junction = J_SEARCHING;
    s_state = LINE_STATE_JUNCTION;
    s_left_cmd = (s_junction_action == JUNCTION_LEFT) ? -v : v;
    s_right_cmd = -s_left_cmd;
    (void)motion_drive(s_left_cmd, s_right_cmd);
}

static void junction_start_turn(void)
{
    float first_deg = LINE_JUNCTION_TURN_DEG - LINE_JUNCTION_PRETURN_MARGIN_DEG;
    motion_result_t result;

    if (first_deg < 2.0f)
    {
        junction_start_search();   /* no open-loop part: go straight to the sensor search */
        return;
    }
    result = (s_junction_action == JUNCTION_LEFT) ? motion_turn_left(first_deg) : motion_turn_right(first_deg);

    s_junction_ms = 0u;
    if (result != MOTION_OK)
    {
        junction_abort();
        return;
    }
    s_junction = J_TURNING;
}

/* One 5 ms step of the junction sequence: stop, (advance,) turn or cross, then resume following. */
static void junction_step(void)
{
    s_junction_ms += LINE_UPDATE_PERIOD_MS;
    if (s_junction_ms > LINE_JUNCTION_TIMEOUT_MS)
    {
        junction_abort();
        return;
    }
    if (s_junction == J_SEARCHING)
    {
        if (((s_mask & (LINE_SENSOR_LEFT | LINE_SENSOR_RIGHT)) != 0u) && s_rejoin)
        {
#if LINE_REJOIN_CENTRE
            /* Keep pivoting: note the heading where the first sensor leaves the tape and where the other
               sensor reaches it. The middle of those two headings puts the tape midway between the sensors. */
            s_centre_first = (s_mask & LINE_SENSOR_LEFT) ? LINE_SENSOR_LEFT : LINE_SENSOR_RIGHT;
            s_centre_have_exit = false;
            s_centre_have_enter = false;
            s_junction = J_CENTRING;
            s_junction_ms = 0u;
#else
            rejoin_finish(1);
#endif
        }
        else if ((s_mask & (LINE_SENSOR_LEFT | LINE_SENSOR_RIGHT)) != 0u)
        {
            /* Found the new line: hand over to normal line following, which centres it. */
            s_junction = J_NONE;
            s_state = LINE_STATE_STRAIGHT;
            s_lost_ms = 0u;
            s_side_ms = 0u;
        }
        else if (s_junction_ms > LINE_JUNCTION_SEARCH_MS)
        {
            junction_abort();
        }
        return;
    }
    if (s_junction == J_CENTRING)
    {
        uint8_t other = (uint8_t)((LINE_SENSOR_LEFT | LINE_SENSOR_RIGHT) & ~s_centre_first);
        float h = reported_heading();

        if (!s_centre_have_enter && ((s_mask & other) != 0u))
        {
            s_centre_have_enter = true;
            s_centre_enter_deg = h;
        }
        if (!s_centre_have_exit && ((s_mask & s_centre_first) == 0u))
        {
            s_centre_have_exit = true;
            s_centre_exit_deg = h;
        }
        if (s_centre_have_enter && s_centre_have_exit)
        {
            s_centre_target_deg = 0.5f * (s_centre_enter_deg + s_centre_exit_deg);
            s_centre_dir = (s_centre_target_deg > h) ? 1.0f : -1.0f;   /* +: heading must increase (left) */
            s_left_cmd = -s_centre_dir * LINE_REJOIN_CENTRE_SPEED_CM_S;
            s_right_cmd = s_centre_dir * LINE_REJOIN_CENTRE_SPEED_CM_S;
            (void)motion_drive(s_left_cmd, s_right_cmd);
            s_junction = J_CENTRE_BACK;
            s_junction_ms = 0u;
        }
        else if (s_junction_ms > LINE_REJOIN_CENTRE_MS)
        {
            rejoin_finish(1);   /* tape already between the sensors, the other edge was not reached */
        }
        return;
    }
    if (s_junction == J_CENTRE_BACK)
    {
        float h = reported_heading();

        if (((s_centre_dir > 0.0f) && (h >= s_centre_target_deg)) ||
            ((s_centre_dir < 0.0f) && (h <= s_centre_target_deg)) ||
            (s_junction_ms > LINE_REJOIN_CENTRE_MS))
        {
            rejoin_finish(1);
        }
        return;
    }
    if (!motion_is_idle())
    {
        return;   /* previous motion still running */
    }

    switch (s_junction)
    {
    case J_STOPPING:
        s_junction_ms = 0u;
        s_junction = J_WAITING;
        break;
    case J_WAITING:
        if (s_junction_ms < LINE_JUNCTION_PAUSE_MS)
        {
            break;
        }
        s_junction_ms = 0u;
        if (s_junction_action == JUNCTION_STRAIGHT)
        {
            if (motion_move_forward(LINE_JUNCTION_CROSS_CM) != MOTION_OK)
            {
                junction_abort();
                return;
            }
            s_junction = J_CROSSING;
        }
        else if (LINE_JUNCTION_ADVANCE_CM > 0.0f)
        {
            if (motion_move_forward(LINE_JUNCTION_ADVANCE_CM) != MOTION_OK)
            {
                junction_abort();
                return;
            }
            s_junction = J_ADVANCING;
        }
        else
        {
            junction_start_turn();
        }
        break;
    case J_ADVANCING:
        junction_start_turn();
        break;
    case J_TURNING:
        junction_start_search();
        break;
    case J_CROSSING:
        /* Done: carry on following the line. Detection stays off until a sensor sees white. */
        s_junction = J_NONE;
        s_state = LINE_STATE_STRAIGHT;
        s_lost_ms = 0u;
        s_side_ms = 0u;
        break;
    default:
        s_junction = J_NONE;
        break;
    }
}
#endif

void line_follow_init(void)
{
    line_sensors_init();
#ifdef LINE_BARCODE_DECODE
    barcode_scan_init();
#endif
    s_state = LINE_STATE_IDLE;
    s_active = false;
    s_last_side = 0;
    s_lost_ms = 0u;
    s_side_ms = 0u;
#if LINE_JUNCTION
    s_junction = J_NONE;
    s_junction_armed = true;
    s_junction_halted = false;
#endif
}

bool line_follow_start(void)
{
    motion_status_t status;

    motion_get_status(&status);
    if (status.state == MOTION_STATE_ERROR)
    {
        s_state = LINE_STATE_FAULT;
        return false;
    }
#if defined(B3_NO_LINE) && (B3_NO_LINE != 0)
    /* Bench-test mode: report "started" but never activate, so no wheel commands are sent. */
    return true;
#endif
    s_active = true;
    s_last_side = 0;
    s_lost_ms = 0u;
    s_side_ms = 0u;
    s_state = LINE_STATE_STOPPED;
#if LINE_JUNCTION
    s_junction = J_NONE;
    /* Armed only once both sensors read white. A (re)start with both sensors on the tape, e.g. when the
       obstacle bypass hands back to the line across it, must not be taken as a "+" junction. */
    s_junction_armed = false;
    s_junction_halted = false;
#endif
    return true;
}

void line_follow_stop(void)
{
    s_active = false;
    s_state = LINE_STATE_IDLE;
#if LINE_JUNCTION
    s_junction = J_NONE;
    s_junction_halted = false;
    s_rejoin = false;
#endif
    motion_stop();
}

#if LINE_JUNCTION
bool line_follow_rejoin(int direction)
{
    if (!line_follow_start())
    {
        return false;
    }
    s_junction_action = (direction > 0) ? JUNCTION_LEFT : JUNCTION_RIGHT;
    s_rejoin = true;
    s_rejoin_result = 0;
    s_junction_ms = 0u;
    s_state = LINE_STATE_JUNCTION;
    if (LINE_JUNCTION_ADVANCE_CM > 0.0f)
    {
        if (motion_move_forward(LINE_JUNCTION_ADVANCE_CM) != MOTION_OK)
        {
            rejoin_finish(-1);
            return false;
        }
        s_junction = J_ADVANCING;
    }
    else
    {
        junction_start_turn();
    }
    return true;
}

int line_follow_rejoin_status(void)
{
    return s_rejoin_result;
}
#endif

void line_follow_update(void)
{
    motion_status_t status;
    bool left;
    bool right;

    if ((!s_active) || (s_state == LINE_STATE_FAULT))
    {
        return;
    }
#if LINE_JUNCTION
    if (s_junction_halted)
    {
        return;   /* waiting at a junction without a barcode letter: send no commands */
    }
#endif

    motion_get_status(&status);
    if (status.state == MOTION_STATE_ERROR)
    {
        /* Do not automatically clear motor faults. A stuck wheel needs
           inspection before the vehicle can resume. */
        s_state = LINE_STATE_FAULT;
        s_left_cmd = 0.0f;
        s_right_cmd = 0.0f;
        return;
    }

    s_mask = line_sensors_read();
#ifdef LINE_BARCODE_DECODE
    /* Barcode sensor: measure bars while following the line. Only the ratio of bar
       widths matters, so the 5 ms update tick is accurate enough as a clock. */
    s_clock_us += (LINE_UPDATE_PERIOD_MS * 1000u);
    barcode_scan_update((s_mask & LINE_SENSOR_BARCODE) != 0u, s_clock_us);
#endif
#if LINE_JUNCTION
    if (s_junction != J_NONE)
    {
        junction_step();   /* the junction sequence drives the car; sensors are ignored */
        return;
    }
#endif
#if LINE_TERRAIN_HOLD
    /* Hump/slope freeze: a raised wheel lifts its sensor (reads black). While tilted, keep the last
       commands and ignore the sensors. No timer: released the moment the tilt is gone. */
    {
        bool tilted = (status.terrain != MOTION_TERRAIN_FLAT);
        float pitch_deg = 0.0f;
        float roll_deg = 0.0f;

        if (motion_get_tilt(&pitch_deg, &roll_deg))
        {
            if ((roll_deg > LINE_FREEZE_ROLL_DEG) || (roll_deg < -LINE_FREEZE_ROLL_DEG) ||
                (pitch_deg > LINE_FREEZE_PITCH_DEG) || (pitch_deg < -LINE_FREEZE_PITCH_DEG))
            {
                tilted = true;
            }
        }
        if (tilted)
        {
            s_lost_ms = 0u;
            s_state = LINE_STATE_FROZEN; /* visible in telemetry as line_state=7 */
            return;
        }
    }
#endif
    left = ((s_mask & LINE_SENSOR_LEFT) != 0u);
    right = ((s_mask & LINE_SENSOR_RIGHT) != 0u);
#if LINE_JUNCTION
    if (!left && !right)
    {
        s_junction_armed = true;   /* both white again (line between the sensors): ready for the next junction */
    }
#endif

    if (left && right)
    {
        s_lost_ms = 0u;
        s_side_ms = 0u;
#if (LINE_SENSORS_STRADDLE != 0u) && (LINE_BOTH_BLACK_STOP != 0u) && (LINE_JUNCTION != 0u)
        /* "+" junction: stop, then turn / cross by the barcode letter (or the test default). */
        if (s_junction_armed)
        {
            junction_begin();
        }
        else
        {
            /* Still on the bar of the junction we just handled: keep going. */
            s_state = LINE_STATE_STRAIGHT;
            drive(LINE_STRAIGHT_SPEED_CM_S, LINE_STRAIGHT_SPEED_CM_S);
        }
#elif (LINE_SENSORS_STRADDLE != 0u) && (LINE_BOTH_BLACK_STOP != 0u)
        /* Straddle mode: both sensors over black is not "on the line". Black = stop. */
        if (s_state != LINE_STATE_STOPPED)
        {
            s_state = LINE_STATE_STOPPED;
            s_left_cmd = 0.0f;
            s_right_cmd = 0.0f;
            motion_stop();
        }
#else
        s_state = LINE_STATE_STRAIGHT;
        drive(LINE_STRAIGHT_SPEED_CM_S, LINE_STRAIGHT_SPEED_CM_S);
#endif
    }
    else if (left || right)
    {
        int32_t side = left ? 1 : -1;
        line_state_t next_state = side > 0 ? LINE_STATE_STEER_LEFT : LINE_STATE_STEER_RIGHT;
        s_lost_ms = 0u;
        s_last_side = side;
        s_side_ms = (s_state == next_state) ? (s_side_ms + LINE_UPDATE_PERIOD_MS) : 0u;
        s_state = next_state;
#if LINE_ESCALATE_AFTER_MS != 0u
        steer(side, s_side_ms >= LINE_ESCALATE_AFTER_MS ? LINE_ESCALATE_INNER_CM_S
                                                        : LINE_TURN_INNER_SPEED_CM_S);
#else
        steer(side, LINE_TURN_INNER_SPEED_CM_S);
#endif
    }
    else if (LINE_SENSORS_STRADDLE != 0u)
    {
        /* Both sensors are over white in the normal straddle position. */
        s_side_ms = 0u;
        s_state = LINE_STATE_STRAIGHT;
        drive(LINE_STRAIGHT_SPEED_CM_S, LINE_STRAIGHT_SPEED_CM_S);
    }
    else
    {
        s_lost_ms += LINE_UPDATE_PERIOD_MS;
        if (s_lost_ms <= LINE_RECOVER_MS)
        {
            s_state = LINE_STATE_RECOVERING;
            if (s_last_side != 0)
            {
                steer(s_last_side, LINE_RECOVER_INNER_CM_S);
            }
            else
            {
                /* Lost directly from centred-on-line: hold heading briefly. */
                drive(LINE_STRAIGHT_SPEED_CM_S, LINE_STRAIGHT_SPEED_CM_S);
            }
        }
        else
        {
            if (s_state != LINE_STATE_STOPPED)
            {
                s_state = LINE_STATE_STOPPED;
                s_left_cmd = 0.0f;
                s_right_cmd = 0.0f;
                motion_stop();
            }
        }
    }
}

void line_follow_get_status(line_status_t *p_status)
{
    if (p_status != 0)
    {
        p_status->state = s_state;
        p_status->sensor_mask = s_mask;
        p_status->left_cmd_cm_s = s_left_cmd;
        p_status->right_cmd_cm_s = s_right_cmd;
    }
}


/**
 * @file line_follow.c
 * @brief Two-sensor line following (decision logic from ir_motor_curve_test.c)
 *        driving the wheels through Buddy 2's motion_drive().
 *
 *   both on line   -> straight
 * one on line -> steer toward the side that still sees it (inside wheel slower)
 * none on line -> keep turning toward the last side for LINE_RECOVER_MS, then
 * stop
 *
 * Straddle mode (LINE_SENSORS_STRADDLE, line between the sensors) with
 * LINE_BOTH_BLACK_STOP: white = move, black = stop.
 *   both white     -> straight
 *   one black      -> that side's wheel stops, the car turns toward that side
 *   both black     -> stop; moves again as soon as a sensor sees white
 */
#include "../barrc_types.h"
#include "line_follow.h"
#include <math.h>

#include "line_config.h"
#include "line_sensors.h"
#include "motion_control.h"

#ifdef LINE_BARCODE_DECODE
#include "barcode_scan.h"   /* also compile barcode_scan.c and barcode39.c */
/* line_follow_update() runs every LINE_UPDATE_PERIOD_MS */
static uint32_t s_clock_us;
#endif

static line_state_t s_state = LINE_STATE_IDLE;
static uint8_t s_mask;
static float32_t s_left_cmd;
static float32_t s_right_cmd;
static int32_t s_last_side;      /* +1 steer left, -1 steer right */
static uint32_t s_lost_ms;
static uint32_t s_side_ms;       /* time steering in the same direction */
static bool s_active;

#if LINE_JUNCTION
typedef enum
{
    J_NONE = 0,
    J_STOPPING,    /* waiting for the car to come to a halt */
    /* standing still at the junction for LINE_JUNCTION_PAUSE_MS */
    J_WAITING,
    J_ADVANCING,   /* driving forward to the middle of the junction */
    J_TURNING,     /* open-loop first part of the turn */
    J_SEARCHING,   /* slow pivot until a sensor sees the new line */
    J_CROSSING,    /* driving straight across */
    /* rejoin only: pivot until the second sensor has crossed the tape */
    J_CENTRING,
    /* rejoin only: pivot to the middle of the two sensor crossings */
    J_CENTRE_BACK
} junction_phase_t;

static junction_phase_t s_junction;
/* false while still on the bar we just handled */
static bool s_junction_armed = true;
/* stopped at a junction with no barcode letter: stay put */
static bool s_junction_halted;
static junction_action_t s_junction_action;
/* time in the current junction step, for the timeout */
static uint32_t s_junction_ms;
/* Rejoin: the same junction turn, used by the obstacle bypass to get back onto
 * the line. */
/* After line_follow_start() the tilt freeze is ignored for this long: a freeze
 * keeps the last wheel commands,
   which are zero right after the bypass hand-back, so a car that finished the
   turn slightly tilted would sit
   still on the tape and never follow it. */
#define LINE_START_FREEZE_GRACE_MS (1500u)
static uint32_t s_freeze_grace_ms;
/* The freeze uses raw (unfiltered) tilt, which acceleration, braking and bumps
 * also disturb. A freeze that
   never ends keeps the last wheel commands and ignores the sensors, so the car
   drives off the line. Limit how
   long one freeze may last, then follow the line for a while before another
   freeze is allowed. */
#define LINE_FREEZE_MAX_MS      (2000u)
#define LINE_FREEZE_COOLDOWN_MS (1000u)
static uint32_t s_freeze_ms;
static uint32_t s_freeze_cool_ms;
static bool s_rejoin;
/* 0 running / not started, 1 done (centred, stopped), -1 failed */
static int8_t s_rejoin_result;
#if LINE_JUNCTION_USE_BARCODE
/* barcode_task.c */
extern bool buddy3_barcode_latest (char * letter, uint32_t * p_read_ms);
static bool s_barcode_used;
static uint32_t s_barcode_used_ms;
#endif
#endif

static void drive (float32_t left, float32_t right)
{
    /* Give Buddy 2 equal targets on straight segments so it can balance them.
     */
    s_left_cmd = left == right ? left : left * LINE_LEFT_WHEEL_SCALE;
    s_right_cmd = left == right ? right : right * LINE_RIGHT_WHEEL_SCALE;
    (void)motion_drive(s_left_cmd, s_right_cmd);
}

static void steer (int32_t side, float32_t inner)
{
    if (side > 0)
    {
        drive(inner, LINE_TURN_OUTER_SPEED_CM_S);      /* turn left */
    }
    else
    {
        /* turn right: left wheel faster */
        drive(LINE_RIGHT_OUTER_SPEED_CM_S, inner);
    }
}

#if LINE_JUNCTION
static junction_action_t junction_choose (void)
{
#if LINE_JUNCTION_USE_BARCODE
    char letter = 0;
    uint32_t read_ms = 0u;

    if ((buddy3_barcode_latest(&letter, &read_ms))
        && ((!s_barcode_used) || (read_ms != s_barcode_used_ms)))
    {
        junction_action_t from_barcode = junction_action_for_letter(letter);

        if (JUNCTION_NONE != from_barcode)
        {
            s_barcode_used = true;
            s_barcode_used_ms = read_ms;
            return from_barcode;
        }
    }
#endif
    return LINE_JUNCTION_DEFAULT_ACTION;
}

static void junction_begin (void)
{
    s_junction_action = junction_choose();
    s_junction_ms = 0u;
    s_junction_armed = false;
    s_state = LINE_STATE_JUNCTION;
    s_left_cmd = 0.0f;
    s_right_cmd = 0.0f;
    motion_stop();

    if (JUNCTION_NONE == s_junction_action)
    {
        /* No barcode letter for this junction: do not guess. Stop here and stay
         * stopped. */
        s_junction = J_NONE;
        s_junction_halted = true;
        return;
    }

    s_junction = J_STOPPING;
}

static void rejoin_finish (int8_t result)
{
    s_rejoin = false;
    s_rejoin_result = result;
    /* hold still; the bypass restarts normal following with line_follow_start()
     */
    s_active = false;
    s_junction = J_NONE;
    s_state = LINE_STATE_STOPPED;
    s_left_cmd = 0.0f;
    s_right_cmd = 0.0f;
    motion_stop();
}

static float32_t reported_heading (void)
{
    motion_status_t m;
    motion_get_status(&m);
    return m.heading_deg;
}

static void junction_abort (void)
{
    if (s_rejoin)
    {
        /* The search gave up, but a sensor is on the tape: the car IS on the
         * line. Report success so the
           bypass hands back to normal line following (which steers onto the
           tape) instead of failing,
           which restarted the whole obstacle process and left the car turning
           around. */
        rejoin_finish((0u != (s_mask & (LINE_SENSOR_LEFT | LINE_SENSOR_RIGHT)))
                      ? 1 : -1);
        return;
    }

    s_junction = J_NONE;
    s_state = LINE_STATE_STOPPED;
    s_left_cmd = 0.0f;
    s_right_cmd = 0.0f;
    motion_stop();
}

#if LINE_REJOIN_CENTRE
/* ------------------------------------------------------------------ */
/* Bypass hand-back: put the tape midway between the two sensors.      */
/* The car pivots on the spot and the heading at which each sensor     */
/* crosses the tape is recorded. The middle of the two crossings is    */
/* the heading at which the tape lies exactly between the sensors.     */
/* It does not matter how far apart the sensors are: the sweep goes on */
/* until BOTH sensors have crossed the tape (first one way, then the   */
/* other if the second sensor lies behind), then pivots to the middle. */
/* ------------------------------------------------------------------ */
typedef struct
{
    bool in;        /* tape under the sensor right now */
    bool done;      /* crossed completely: centre is valid */
    /* was already on the tape when the sweep began: wait until it is white */
    bool ignore;
    float32_t enter;
    float32_t centre;
} rj_event_t;

/* 0 = LEFT sensor, 1 = RIGHT sensor */
static rj_event_t s_ev[2];
/* +1 pivoting left (heading rising), -1 pivoting right */
static float32_t s_rj_dir;
/* heading where the current pivot began */
static float32_t s_rj_start_deg;
/* heading with the tape midway between the sensors */
static float32_t s_rj_target_deg;
/* sensor still to be measured after the first crossing */
static int32_t s_rj_second;
static bool s_rj_reversed_once;
static bool s_rj_fast;

static uint8_t rj_bit (int32_t idx)
{
    return (0 == idx) ? (uint8_t)LINE_SENSOR_LEFT : (uint8_t)LINE_SENSOR_RIGHT;
}

static void rj_pivot (float32_t dir, float32_t speed)
{
    s_rj_dir = dir;
    s_left_cmd = -dir * speed;
    s_right_cmd = dir * speed;
    (void)motion_drive(s_left_cmd, s_right_cmd);
}

static void rj_clear (int32_t idx)
{
    s_ev[idx].in = false;
    s_ev[idx].done = false;
    /* already black: measure it on a later pass */
    s_ev[idx].ignore = (0u != (s_mask & rj_bit(idx)));
}

static void rj_track (float32_t h)
{
    int32_t idx;

    for (idx = 0; idx < 2; ++idx)
    {
        bool black = (0u != (s_mask & rj_bit(idx)));
        rj_event_t * p_e = &s_ev[idx];

        if (p_e->done)
        {
            continue;
        }

        if (p_e->ignore)
        {
            if (!black)
            {
                p_e->ignore = false;
            }

            continue;
        }

        if (black && (!p_e->in))
        {
            p_e->in = true;
            p_e->enter = h;
        }
        else if ((!black) && (p_e->in))
        {
            p_e->in = false;
            p_e->done = true;
            p_e->centre = 0.5f * (p_e->enter + h);
        }
        else
        {
            /* no action required */
        }
    }
}

/* Forget both crossings and pivot in direction dir at the search speed. */
static void rj_begin_search (float32_t dir)
{
    rj_clear(0);
    rj_clear(1);
    s_junction_ms = 0u;
    s_rj_start_deg = reported_heading();
    rj_pivot(dir, LINE_JUNCTION_SEARCH_SPEED_CM_S);
}

/* Both crossings known: pivot to the middle of the two. */
static void rj_go_to_middle (float32_t h)
{
    s_rj_target_deg = 0.5f * (s_ev[0].centre + s_ev[1].centre);
    s_rj_fast = (fabsf(s_rj_target_deg - h) > LINE_REJOIN_SLOW_ZONE_DEG);
    rj_pivot((s_rj_target_deg > h) ? 1.0f : -1.0f,
             s_rj_fast ? LINE_JUNCTION_SEARCH_SPEED_CM_S
             : LINE_REJOIN_CENTRE_SPEED_CM_S);
    s_junction = J_CENTRE_BACK;
    s_junction_ms = 0u;
}
#endif

static void junction_start_search (void)
{
    float32_t v = LINE_JUNCTION_SEARCH_SPEED_CM_S;

    s_junction_ms = 0u;
    s_junction = J_SEARCHING;
    s_state = LINE_STATE_JUNCTION;
#if LINE_REJOIN_CENTRE
    if (s_rejoin)
    {
        s_rj_reversed_once = false;
        rj_begin_search((JUNCTION_LEFT == s_junction_action) ? 1.0f : -1.0f);
        return;
    }
#endif
    s_left_cmd = (JUNCTION_LEFT == s_junction_action) ? -v : v;
    s_right_cmd = -s_left_cmd;
    (void)motion_drive(s_left_cmd, s_right_cmd);
}

static void junction_start_turn (void)
{
    float32_t turn_deg = (JUNCTION_UTURN == s_junction_action)
            ? 180.0f : LINE_JUNCTION_TURN_DEG;
    float32_t first_deg = turn_deg - LINE_JUNCTION_PRETURN_MARGIN_DEG;
    motion_result_t result;

    if (first_deg < 2.0f)
    {
        /* no open-loop part: go straight to the sensor search */
        junction_start_search();
        return;
    }

    result = (JUNCTION_LEFT == s_junction_action) ? motion_turn_left(first_deg)
            : motion_turn_right(first_deg);

    s_junction_ms = 0u;

    if (MOTION_OK != result)
    {
        junction_abort();
        return;
    }

    s_junction = J_TURNING;
}

/* One 5 ms step of the junction sequence: stop, (advance,) turn or cross, then
 * resume following. */
static void junction_step (void)
{
    s_junction_ms += LINE_UPDATE_PERIOD_MS;

    if (s_junction_ms > LINE_JUNCTION_TIMEOUT_MS)
    {
        junction_abort();
        return;
    }
#if LINE_REJOIN_CENTRE
    if (s_rejoin && ((J_SEARCHING == s_junction) || (J_CENTRING == s_junction)
                     || (J_CENTRE_BACK == s_junction)))
    {
        float32_t h = reported_heading();

        if (J_SEARCHING == s_junction)
        {
            /* First pass: pivot until one sensor has crossed the tape
             * completely. */
            float32_t travelled = (h - s_rj_start_deg) * s_rj_dir;

            rj_track(h);

            if ((s_ev[0].done) && (s_ev[1].done))
            {
                rj_go_to_middle(h);
            }
            else if ((s_ev[0].done) || (s_ev[1].done))
            {
                int32_t f = s_ev[0].done ? 0 : 1;
                int32_t g = 1 - f;
                /* Pivoting left meets the LEFT sensor first, pivoting right
                 * meets the RIGHT sensor first. */
                int32_t expected_first = (s_rj_dir > 0.0f) ? 0 : 1;

                s_rj_second = g;
                s_junction = J_CENTRING;
                s_junction_ms = 0u;

                if (f != expected_first)
                {
                    /* The other sensor crossed the tape earlier in this turn:
                     * turn back to measure it. */
                    rj_clear(g);
                    rj_pivot(-s_rj_dir, LINE_JUNCTION_SEARCH_SPEED_CM_S);
                }

                s_rj_start_deg = h;
            }
            else if (travelled > LINE_REJOIN_SEARCH_SPAN_DEG)
            {
                if (!s_rj_reversed_once)
                {
                    /* the turn may have overshot the tape: look the other way
                     */
                    s_rj_reversed_once = true;
                    rj_begin_search(-s_rj_dir);
                }
                else
                {
                    junction_abort();
                }
            }
            else
            {
                /* no action required */
            }

            return;
        }

        if (J_CENTRING == s_junction)
        {
            /* Second pass: pivot until the other sensor has crossed the tape
             * completely. */
            float32_t travelled = (h - s_rj_start_deg) * s_rj_dir;

            rj_track(h);

            if (s_ev[s_rj_second].done)
            {
                rj_go_to_middle(h);
            }
            else if (travelled > 2.0f * LINE_REJOIN_SEARCH_SPAN_DEG)
            {
                junction_abort();
            }
            else
            {
                /* no action required */
            }

            return;
        }

        /* J_CENTRE_BACK: pivot to the middle of the two crossings, slowly for
         * the last few degrees. */
        {
            /* > 0 while still short of the target */
            float32_t remaining = (s_rj_target_deg - h) * s_rj_dir;

            if (s_rj_fast && (remaining < LINE_REJOIN_SLOW_ZONE_DEG))
            {
                s_rj_fast = false;
                rj_pivot(s_rj_dir, LINE_REJOIN_CENTRE_SPEED_CM_S);
            }

            /* Stop slightly early: the car keeps turning a little while it
             * brakes. */
            if ((remaining <= LINE_REJOIN_CENTRE_LEAD_DEG)
                || (s_junction_ms > LINE_REJOIN_CENTRE_MS))
            {
                rejoin_finish(1);
            }
        }

        return;
    }
#endif
    if (J_SEARCHING == s_junction)
    {
        if (0u != (s_mask & (LINE_SENSOR_LEFT | LINE_SENSOR_RIGHT)))
        {
            /* Found the new line: hand over to normal line following, which
             * centres it. */
            s_junction = J_NONE;
            s_state = LINE_STATE_STRAIGHT;
            s_lost_ms = 0u;
            s_side_ms = 0u;
        }
        else if (s_junction_ms > LINE_JUNCTION_SEARCH_MS)
        {
            junction_abort();
        }
        else
        {
            /* no action required */
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

        if (JUNCTION_STRAIGHT == s_junction_action)
        {
            if (MOTION_OK != motion_move_forward(LINE_JUNCTION_CROSS_CM))
            {
                junction_abort();
                return;
            }

            s_junction = J_CROSSING;
        }
        else if (LINE_JUNCTION_ADVANCE_CM > 0.0f)
        {
            if (MOTION_OK != motion_move_forward(LINE_JUNCTION_ADVANCE_CM))
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
        /* Done: carry on following the line. Detection stays off until a sensor
         * sees white. */
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

void line_follow_init (void)
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

bool line_follow_start (void)
{
    motion_status_t status;

    motion_get_status(&status);

    if (MOTION_STATE_ERROR == status.state)
    {
        s_state = LINE_STATE_FAULT;
        return false;
    }
#if defined(B3_NO_LINE) && (B3_NO_LINE != 0)
    /* Bench-test mode: report "started" but never activate, so no wheel
     * commands are sent. */
    return true;
#endif
    s_active = true;
    s_last_side = 0;
    s_lost_ms = 0u;
    s_side_ms = 0u;
    s_freeze_grace_ms = LINE_START_FREEZE_GRACE_MS;
    s_freeze_ms = 0u;
    s_freeze_cool_ms = 0u;
    s_state = LINE_STATE_STOPPED;
#if LINE_JUNCTION
    s_junction = J_NONE;
    /* Armed only once both sensors read white. A (re)start with both sensors on
     * the tape, e.g. when the
       obstacle bypass hands back to the line across it, must not be taken as a
       "+" junction. */
    s_junction_armed = false;
    s_junction_halted = false;
#endif
    return true;
}

void line_follow_stop (void)
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
bool line_follow_rejoin (int32_t direction, float32_t advance_cm)
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
    /* Put the axle on the tape before turning, exactly like the junction (the
     * caller measured the distance). */
    if ((advance_cm > 0.3f) || (advance_cm < -0.3f))
    {
        motion_result_t r =
                (advance_cm > 0.0f) ? motion_move_forward(advance_cm)
                : motion_move_backward(-advance_cm);

        if (MOTION_OK != r)
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

int32_t line_follow_rejoin_status (void)
{
    return s_rejoin_result;
}
#endif

void line_follow_update (void)
{
    motion_status_t status;
    bool left;
    bool right;

    if ((!s_active) || (LINE_STATE_FAULT == s_state))
    {
        return;
    }
#if LINE_JUNCTION
    if (s_junction_halted)
    {
        /* waiting at a junction without a barcode letter: send no commands */
        return;
    }
#endif

    motion_get_status(&status);

    if (MOTION_STATE_ERROR == status.state)
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
    /* Barcode sensor: measure bars while following the line. Only the ratio of
     * bar
       widths matters, so the 5 ms update tick is accurate enough as a clock. */
    s_clock_us += (LINE_UPDATE_PERIOD_MS * 1000u);
    barcode_scan_update(0u != (s_mask & LINE_SENSOR_BARCODE), s_clock_us);
#endif
#if LINE_JUNCTION
    if (J_NONE != s_junction)
    {
        /* the junction sequence drives the car; sensors are ignored */
        junction_step();
        return;
    }
#endif
#if LINE_TERRAIN_HOLD
    if (s_freeze_grace_ms > 0u)
    {
        s_freeze_grace_ms =
                (s_freeze_grace_ms > LINE_UPDATE_PERIOD_MS)
                ? (s_freeze_grace_ms - LINE_UPDATE_PERIOD_MS) : 0u;
    }
    else
    /* Hump/slope freeze: a raised wheel lifts its sensor (reads black). While
     * tilted, keep the last
       commands and ignore the sensors. No timer: released the moment the tilt
       is gone. */
    {
        bool b_tilted = (MOTION_TERRAIN_FLAT != status.terrain);
        float32_t pitch_deg = 0.0f;
        float32_t roll_deg = 0.0f;

        if (motion_get_tilt(&pitch_deg, &roll_deg))
        {
            if ((roll_deg > LINE_FREEZE_ROLL_DEG)
                || (roll_deg < -LINE_FREEZE_ROLL_DEG) ||
                (pitch_deg > LINE_FREEZE_PITCH_DEG)
                || (pitch_deg < -LINE_FREEZE_PITCH_DEG))
            {
                b_tilted = true;
            }
        }

        if (s_freeze_cool_ms > 0u)
        {
            /* just released a long freeze: follow the line now whatever the
             * tilt reads */
            s_freeze_cool_ms =
                    (s_freeze_cool_ms > LINE_UPDATE_PERIOD_MS)
                    ? (s_freeze_cool_ms - LINE_UPDATE_PERIOD_MS) : 0u;
            b_tilted = false;
        }

        if (!b_tilted)
        {
            s_freeze_ms = 0u;
        }
        else
        {
            s_freeze_ms += LINE_UPDATE_PERIOD_MS;

            if (s_freeze_ms >= LINE_FREEZE_MAX_MS)
            {
                s_freeze_ms = 0u;
                s_freeze_cool_ms = LINE_FREEZE_COOLDOWN_MS;
                b_tilted = false;
            }
        }

        if (b_tilted)
        {
            s_lost_ms = 0u;
            /* visible in telemetry as line_state=7 */
            s_state = LINE_STATE_FROZEN;
            return;
        }
    }
#endif
    left = (0u != (s_mask & LINE_SENSOR_LEFT));
    right = (0u != (s_mask & LINE_SENSOR_RIGHT));
#if LINE_JUNCTION
    if ((!left) && (!right))
    {
        /* both white again (line between the sensors): ready for the next
         * junction */
        s_junction_armed = true;
    }
#endif

    if (left && right)
    {
        s_lost_ms = 0u;
        s_side_ms = 0u;
#if (LINE_SENSORS_STRADDLE != 0u) && (LINE_BOTH_BLACK_STOP != 0u) \
    && (LINE_JUNCTION != 0u)
        /* "+" junction: stop, then turn / cross by the barcode letter (or the
         * test default). */
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
        /* Straddle mode: both sensors over black is not "on the line". Black =
         * stop. */
        if (LINE_STATE_STOPPED != s_state)
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
        line_state_t next_state =
                side > 0 ? LINE_STATE_STEER_LEFT : LINE_STATE_STEER_RIGHT;
        s_lost_ms = 0u;
        s_last_side = side;
        s_side_ms = (s_state == next_state)
                ? (s_side_ms + LINE_UPDATE_PERIOD_MS) : 0u;
        s_state = next_state;
#if LINE_ESCALATE_AFTER_MS != 0u
        steer(side, s_side_ms >= LINE_ESCALATE_AFTER_MS
              ? LINE_ESCALATE_INNER_CM_S
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

            if (0 != s_last_side)
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
            if (LINE_STATE_STOPPED != s_state)
            {
                s_state = LINE_STATE_STOPPED;
                s_left_cmd = 0.0f;
                s_right_cmd = 0.0f;
                motion_stop();
            }
        }
    }
}

bool line_follow_restart_required (void)
{
#if LINE_JUNCTION
    return s_junction_halted;
#else
    return false;
#endif
}

void line_follow_discard_barcode (void)
{
#if LINE_JUNCTION && LINE_JUNCTION_USE_BARCODE
    char letter;
    uint32_t read_ms;

    if (buddy3_barcode_latest(&letter, &read_ms))
    {
        s_barcode_used = true;
        s_barcode_used_ms = read_ms;
    }
#endif
}

void line_follow_get_status (line_status_t * p_status)
{
    if (0 != p_status)
    {
        p_status->state = s_state;
        p_status->sensor_mask = s_mask;
        p_status->left_cmd_cm_s = s_left_cmd;
        p_status->right_cmd_cm_s = s_right_cmd;
    }
}

/*** end of file ***/

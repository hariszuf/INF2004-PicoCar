/**
 * @file motion_control.c
 * @brief Motion Control subsystem: speed PID, straight-line correction,
 *        encoder-based turning, stopping and fault handling.
 *
 * All control runs inside motion_update(), called at a fixed period by the
 * Motion Control task. Public setters only queue a command, so there is one
 * writer of the control state. motion_emergency_stop() is the exception: it
 * brakes the motors straight away, then lets motion_update() record the fault.
 */
#include "../barrc_types.h"
#include "motion_control.h"

#include <math.h>

#include "motion_config.h"
#include "motion_encoder.h"
#include "motion_motor.h"
#include "motion_pid.h"

#if defined(MOTION_HOST_TEST)
#include "host_platform.h"
#elif defined(MOTION_USE_UTKERNEL)
/* micro T-Kernel trial: app_program/buddy2/mtk_trial */
#include "motion_mtk_hw.h"
#else
#include "hardware/sync.h"
#include "pico/time.h"
#endif

/* Packed yes/no flags shared between tasks. Writes use the same critical
 * section as the command queue below. */
static uint32_t critical_enter (void);
static void critical_exit (uint32_t saved);
#define ROBOT_FLAGS_IRQ_T uint32_t
#define ROBOT_FLAGS_LOCK(s) ((s) = critical_enter())
#define ROBOT_FLAGS_UNLOCK(s) critical_exit(s)
#include "../robot_flags.h"

/* ------------------------------------------------------------------ */
/* Private constants and types                                         */
/* ------------------------------------------------------------------ */
#define PERIOD_S            ((float)MOTION_UPDATE_PERIOD_MS / 1000.0f)
#define DEG_TO_RAD          (MOTION_PI / 180.0f)
#define RAD_TO_DEG          (180.0f / MOTION_PI)
#define MAX_TURN_DEG        (720.0f)
/* below this a set-point means "stopped" */
#define ZERO_SPEED_CM_S     (0.5f)
#define DUTY_ZERO           (0.001f)
#define US_PER_SECOND       (1000000.0f)
#define US_PER_MS           (1000u)

typedef enum
{
    COMMAND_NONE = 0,
    COMMAND_MOVE,
    COMMAND_TURN,
    COMMAND_DRIVE,
    COMMAND_STOP
} command_type_t;

typedef struct
{
    command_type_t type;
    /* MOVE: signed cm (+ forward). TURN: signed deg (+ left) */
    float32_t value;
    float32_t left_cm_s;    /* DRIVE only */
    float32_t right_cm_s;
} command_t;

/* ------------------------------------------------------------------ */
/* Private state (single writer: motion_update, except where noted)    */
/* ------------------------------------------------------------------ */
#define MF_INITIALISED (1u << 0)
#define MF_ESTOP_REQUEST (1u << 1)   /* written by any task */
#define MF_PENDING_VALID (1u << 2)   /* written by any task */
#define MF_TERRAIN_HAVE (1u << 3)    /* written by the IMU task */
static robot_flags_t s_flags = 0u;
#define MF(mask) robot_flags_test(&s_flags, (mask))
/* motion_state_t, 2 bits needed */
static volatile uint8_t s_state = MOTION_STATE_IDLE;
/* motion_fault_t, 2 bits needed */
static volatile uint8_t s_fault = MOTION_FAULT_NONE;
static command_t s_pending;
static command_t s_active;
static float32_t s_cruise_cm_s = MOTION_DEFAULT_SPEED_CM_S;

static float32_t s_cm_per_count[MOTION_SIDE_COUNT];
static int32_t s_base_count[MOTION_SIDE_COUNT];
static float32_t s_position_cm[MOTION_SIDE_COUNT];
static float32_t s_previous_position_cm[MOTION_SIDE_COUNT];
static float32_t s_start_position_cm[MOTION_SIDE_COUNT];
static float32_t s_speed_cm_s[MOTION_SIDE_COUNT];
static float32_t s_setpoint_cm_s[MOTION_SIDE_COUNT];
static int8_t s_direction[MOTION_SIDE_COUNT];   /* -1 or +1 */
static motion_pid_t s_pid[MOTION_SIDE_COUNT];
static uint32_t s_stall_ms[MOTION_SIDE_COUNT];
static uint32_t s_settle_cycles = 0u;
static uint32_t s_settle_ms = 0u;
static uint64_t s_last_update_us = 0u;

/* Terrain compensation (input written by the IMU task, rest by motion_update).
 */
static motion_terrain_input_t s_terrain_input;
static volatile uint64_t s_terrain_time_us = 0u;
/* motion_terrain_t, 3 bits needed */
static uint8_t s_terrain = MOTION_TERRAIN_FLAT;
static float32_t s_terrain_scale = 1.0f;
/* slewed assist, applied */
static float32_t s_terrain_duty[MOTION_SIDE_COUNT];
/* wanted gravity term */
static float32_t s_terrain_grade_duty = 0.0f;
/* wanted hump term (magnitude) */
static float32_t s_terrain_hump_duty[MOTION_SIDE_COUNT];

/* Latest values for motion_get_status(). */
static int32_t s_status_count[MOTION_SIDE_COUNT];

/* ------------------------------------------------------------------ */
/* Platform helpers                                                    */
/* ------------------------------------------------------------------ */
static uint64_t now_us (void)
{
#if defined(MOTION_HOST_TEST)
    return host_time_us();
#elif defined(MOTION_USE_UTKERNEL)
    return motion_mtk_time_us();
#else
    return time_us_64();
#endif
}

static uint32_t critical_enter (void)
{
#if defined(MOTION_HOST_TEST)
    return 0u;
#elif defined(MOTION_USE_UTKERNEL)
    return motion_mtk_irq_save();
#else
    return save_and_disable_interrupts();
#endif
}

static void critical_exit (uint32_t saved)
{
#if defined(MOTION_HOST_TEST)
    (void)saved;
#elif defined(MOTION_USE_UTKERNEL)
    motion_mtk_irq_restore(saved);
#else
    restore_interrupts(saved);
#endif
}

/* ------------------------------------------------------------------ */
/* Small maths helpers                                                 */
/* ------------------------------------------------------------------ */
static float32_t abs_f (float32_t value)
{
    return (value < 0.0f) ? -value : value;
}

static float32_t sign_f (float32_t value)
{
    return (value < 0.0f) ? -1.0f : 1.0f;
}

static float32_t clamp_f (float32_t value, float32_t low, float32_t high)
{
    float32_t result = value;

    if (result < low)
    {
        result = low;
    }

    if (result > high)
    {
        result = high;
    }

    return result;
}

static float32_t approach (float32_t current, float32_t target,
                           float32_t max_step)
{
    float32_t delta = target - current;

    if (delta > max_step)
    {
        delta = max_step;
    }

    if (delta < -max_step)
    {
        delta = -max_step;
    }

    return current + delta;
}

/* ------------------------------------------------------------------ */
/* Sensing                                                             */
/* ------------------------------------------------------------------ */
static void read_sensors (float32_t dt_s)
{
    uint32_t idx;
    motion_side_t side;
    int32_t counts;
    float32_t position;
    float32_t raw_speed;

    for (idx = 0u; idx < (uint32_t)MOTION_SIDE_COUNT; idx++)
    {
        side = (motion_side_t)idx;
        counts = motion_encoder_get_count(side) - s_base_count[idx];
        s_status_count[idx] = counts;
        position = (float32_t)counts * s_cm_per_count[idx];
        raw_speed = (position - s_previous_position_cm[idx]) / dt_s;
        s_speed_cm_s[idx] += MOTION_SPEED_FILTER_ALPHA
                * (raw_speed - s_speed_cm_s[idx]);
        s_previous_position_cm[idx] = position;
        s_position_cm[idx] = position;
    }
}

static float32_t heading_deg_from (float32_t left_cm, float32_t right_cm)
{
    return ((right_cm - left_cm) / MOTION_WHEEL_BASE_CM) * RAD_TO_DEG;
}

/* ------------------------------------------------------------------ */
/* Terrain compensation (Buddy 4 IMU -> Buddy 2 speed and power)       */
/* ------------------------------------------------------------------ */
/*
 * Turns the latest IMU report into:
 *  - a speed scale for moves and motion_drive() (smoothed later by the
 *    normal accel / decel ramp, so speed changes are gentle), and
 *  - a duty assist: a gravity term on a slope (both wheels) plus extra push
 *    on the raised wheel on a one-side hump. The assist is rate limited.
 * A missing, invalid or stale report means flat ground: normal behaviour.
 */
static void update_terrain (uint64_t now, float32_t dt_s)
{
    motion_terrain_input_t input;
    bool b_fresh;
    uint32_t saved;
    uint32_t idx;
    float32_t pitch;
    float32_t roll;
    float32_t scale = 1.0f;
    float32_t wanted;

    saved = critical_enter();
    input = s_terrain_input;
    /* A report stamped after 'now' (written by another task meanwhile) is
     * fresh. */
    b_fresh = (MF(MF_TERRAIN_HAVE)) && (input.valid) &&
            ((s_terrain_time_us >= now) ||
             ((now - s_terrain_time_us) <= ((uint64_t)MOTION_TERRAIN_STALE_MS
                                            * US_PER_MS)));
    critical_exit(saved);

    s_terrain = MOTION_TERRAIN_FLAT;
    s_terrain_grade_duty = 0.0f;
    s_terrain_hump_duty[MOTION_SIDE_LEFT] = 0.0f;
    s_terrain_hump_duty[MOTION_SIDE_RIGHT] = 0.0f;

    if (b_fresh)
    {
        pitch = clamp_f(input.pitch_deg, -MOTION_TERRAIN_MAX_ANGLE_DEG,
                        MOTION_TERRAIN_MAX_ANGLE_DEG);
        roll = clamp_f(input.roll_deg, -MOTION_TERRAIN_MAX_ANGLE_DEG,
                       MOTION_TERRAIN_MAX_ANGLE_DEG);

        /* Slope: only act once Buddy 4's hysteresis says the car is tilted. */
        if (input.pitch_state > 0)
        {
            s_terrain = MOTION_TERRAIN_UPHILL;
            scale = MOTION_UPHILL_SPEED_SCALE;
            s_terrain_grade_duty =
                    MOTION_GRADE_DUTY_PER_G * sinf(abs_f(pitch) * DEG_TO_RAD);
        }
        else if (input.pitch_state < 0)
        {
            s_terrain = MOTION_TERRAIN_DOWNHILL;
            scale = MOTION_DOWNHILL_SPEED_SCALE;
            s_terrain_grade_duty =
                    -MOTION_GRADE_DUTY_PER_G * sinf(abs_f(pitch) * DEG_TO_RAD);
        }
        else
        {
            /* flat in pitch */
        }

        /* Hump / one-side ramp: slow down and help the raised wheel. */
        if (0 != input.roll_state)
        {
            motion_side_t raised =
                    (input.roll_state < 0) ? MOTION_SIDE_LEFT
                    : MOTION_SIDE_RIGHT;

            s_terrain = (MOTION_SIDE_LEFT == raised) ? MOTION_TERRAIN_HUMP_LEFT
                    : MOTION_TERRAIN_HUMP_RIGHT;

            if (MOTION_HUMP_SPEED_SCALE < scale)
            {
                scale = MOTION_HUMP_SPEED_SCALE;
            }

            s_terrain_hump_duty[raised] =
                    MOTION_HUMP_DUTY_PER_G * sinf(abs_f(roll) * DEG_TO_RAD);
        }
    }

    s_terrain_scale = clamp_f(scale, 0.0f, 1.0f);

    /* Slew the applied assist so power changes are gradual. The hump part
       follows the direction the wheel is asked to turn. */
    for (idx = 0u; idx < (uint32_t)MOTION_SIDE_COUNT; idx++)
    {
        wanted = s_terrain_grade_duty;

        if (abs_f(s_setpoint_cm_s[idx]) > ZERO_SPEED_CM_S)
        {
            wanted += sign_f(s_setpoint_cm_s[idx]) * s_terrain_hump_duty[idx];
        }

        wanted = clamp_f(wanted, -MOTION_TERRAIN_DUTY_MAX,
                         MOTION_TERRAIN_DUTY_MAX);
        s_terrain_duty[idx] = approach(s_terrain_duty[idx], wanted,
                                       MOTION_TERRAIN_DUTY_RATE_PER_S * dt_s);
    }
}

/* ------------------------------------------------------------------ */
/* Actuation                                                           */
/* ------------------------------------------------------------------ */
/*
 * A single-phase encoder cannot sense direction, so the count sign is the last
 * driven direction. Never flip that direction while the wheel is still
 * turning: brake instead and wait.
 */
static float32_t drive_wheel (motion_side_t side, float32_t duty)
{
    int32_t wanted_direction = (duty < 0.0f) ? -1 : 1;
    float32_t applied = duty;

    if (abs_f(duty) < DUTY_ZERO)
    {
        motion_motor_brake(side);
        applied = 0.0f;
    }
    else if ((wanted_direction != s_direction[side]) &&
             (abs_f(s_speed_cm_s[side]) > MOTION_REVERSAL_SPEED_CM_S))
    {
        motion_motor_brake(side);
        applied = 0.0f;
    }
    else
    {
        s_direction[side] = wanted_direction;
        motion_encoder_set_direction(side, wanted_direction);
        motion_motor_set(side, duty);
    }

    return applied;
}

static void reset_controllers (void)
{
    uint32_t idx;

    for (idx = 0u; idx < (uint32_t)MOTION_SIDE_COUNT; idx++)
    {
        motion_pid_reset(&s_pid[idx]);
        s_setpoint_cm_s[idx] = 0.0f;
        s_stall_ms[idx] = 0u;
    }
}

/* Speed loop for one wheel. Returns the duty actually applied. */
static float32_t wheel_control (motion_side_t side, float32_t setpoint,
                                float32_t dt_s)
{
    float32_t duty;

    if (abs_f(setpoint) <= ZERO_SPEED_CM_S)
    {
        if ((MOTION_PID_HOLD_INTEGRAL_ON_ZERO == 0u)
            || (COMMAND_DRIVE != s_active.type))
        {
            motion_pid_reset(&s_pid[side]);
        }

        /* else: keep the integral (frozen, not accumulating) for the next push
         */
        return drive_wheel(side, 0.0f);
    }

    duty = (MOTION_FEEDFORWARD_DUTY_PER_CM_S * setpoint) +
            motion_pid_update(&s_pid[side], setpoint, s_speed_cm_s[side], dt_s);

    /* Continuous drive must be able to back off below the duty floor
       when a wheel has reached its requested speed. */
    bool b_needs_min_duty = (COMMAND_DRIVE != s_active.type) ||
            (sign_f(setpoint)
             * s_speed_cm_s[side] < abs_f(setpoint));

    if (b_needs_min_duty && (abs_f(duty) < MOTION_MIN_DUTY))
    {
        duty = sign_f(setpoint) * MOTION_MIN_DUTY;
    }

    if ((MOTION_BREAKAWAY_DUTY > 0.0f) &&
        (abs_f(s_speed_cm_s[side]) < MOTION_BREAKAWAY_SPEED_CM_S) &&
        (abs_f(duty) < MOTION_BREAKAWAY_DUTY))
    {
        /* wheel is stuck: give it a push */
        duty = sign_f(setpoint) * MOTION_BREAKAWAY_DUTY;
    }

    /* Terrain assist from the IMU (zero on flat ground). */
    duty += s_terrain_duty[side];

    /* Never drive a wheel against its requested direction (downhill assist):
       brake instead. This also protects the single-phase encoder count. */
    if ((duty * setpoint) < 0.0f)
    {
        duty = 0.0f;
    }

    duty = clamp_f(duty, -1.0f, 1.0f);
    return drive_wheel(side, duty);
}

/*
 * True when the speed loop is pushing a wheel as hard as it is allowed to:
 * duty at or above MOTION_STALL_DUTY, or the PID integral saturated in the
 * driving direction. The second case matters at slow line-following speeds,
 * where feed-forward + PID can never reach MOTION_STALL_DUTY.
 */
static bool pushing_hard (uint32_t idx, float32_t applied)
{
    float32_t drive_sign = sign_f(s_setpoint_cm_s[idx]);

    if (abs_f(applied) >= MOTION_STALL_DUTY)
    {
        return true;
    }

    return (abs_f(applied) > DUTY_ZERO) &&
            (abs_f(s_setpoint_cm_s[idx]) > ZERO_SPEED_CM_S) &&
            ((drive_sign * s_pid[idx].integral) >= (0.99f
            * MOTION_PID_INTEGRAL_LIMIT));
}

/* ------------------------------------------------------------------ */
/* State transitions                                                   */
/* ------------------------------------------------------------------ */
static void begin_stop (void)
{
    motion_motor_brake_all();
    reset_controllers();
    s_settle_cycles = 0u;
    s_settle_ms = 0u;
    s_state = MOTION_STATE_STOPPING;
}

static void enter_error (motion_fault_t fault)
{
    motion_motor_set_latch(true);   /* also brakes both motors */
    reset_controllers();
    robot_flags_clear(&s_flags, MF_PENDING_VALID);
    s_active.type = COMMAND_NONE;
    s_fault = fault;
    s_state = MOTION_STATE_ERROR;
}

static void start_command (const command_t * p_command)
{
    uint32_t idx;

    if (COMMAND_STOP == p_command->type)
    {
        if ((MOTION_STATE_EXECUTING == s_state))
        {
            begin_stop();
        }

        return;
    }

    /* Line following re-sends motion_drive() every few ms. A new DRIVE while
       already driving only changes the wheel speeds: keep the stall timers
       running, otherwise a stuck wheel is never detected. */
    if ((COMMAND_DRIVE == p_command->type)
        && (MOTION_STATE_EXECUTING == s_state) &&
        (COMMAND_DRIVE == s_active.type))
    {
        /* Start measuring straight travel from the end of a steering
         * correction. */
        if ((p_command->left_cm_s == p_command->right_cm_s) &&
            ((s_active.left_cm_s != s_active.right_cm_s) ||
             (p_command->left_cm_s * s_active.left_cm_s < 0.0f)))
        {
            s_start_position_cm[MOTION_SIDE_LEFT] =
                    s_position_cm[MOTION_SIDE_LEFT];
            s_start_position_cm[MOTION_SIDE_RIGHT] =
                    s_position_cm[MOTION_SIDE_RIGHT];
        }

        s_active = *p_command;
        return;
    }

    if (MOTION_STATE_EXECUTING != s_state)
    {
        reset_controllers();
    }

    for (idx = 0u; idx < (uint32_t)MOTION_SIDE_COUNT; idx++)
    {
        s_start_position_cm[idx] = s_position_cm[idx];
        s_stall_ms[idx] = 0u;
    }

    s_active = *p_command;
    s_state = MOTION_STATE_EXECUTING;
}

/* ------------------------------------------------------------------ */
/* Motion profiles: each fills target[] (cm/s per wheel), returns done */
/* ------------------------------------------------------------------ */

/* Distance the robot still travels after the brake is applied. */
static float32_t stop_lead_cm (void)
{
    float32_t mean_speed = 0.5f * (abs_f(s_speed_cm_s[MOTION_SIDE_LEFT]) +
                               abs_f(s_speed_cm_s[MOTION_SIDE_RIGHT]));

    return mean_speed * MOTION_STOP_LEAD_S;
}

static float32_t straight_trim (float32_t distance_difference, float32_t limit)
{
    return clamp_f(MOTION_STRAIGHT_KP * distance_difference, -limit, limit);
}

static bool profile_move (float32_t target[MOTION_SIDE_COUNT])
{
    float32_t direction = sign_f(s_active.value);
    float32_t goal = abs_f(s_active.value);
    float32_t left =
            s_position_cm[MOTION_SIDE_LEFT]
            - s_start_position_cm[MOTION_SIDE_LEFT];
    float32_t right =
            s_position_cm[MOTION_SIDE_RIGHT]
            - s_start_position_cm[MOTION_SIDE_RIGHT];
    float32_t travelled = (left + right) * 0.5f;
    float32_t remaining = goal - (direction * travelled);
    float32_t minimum =
            (MOTION_MIN_SPEED_CM_S < s_cruise_cm_s) ? MOTION_MIN_SPEED_CM_S
            : s_cruise_cm_s;
    float32_t speed;
    float32_t trim;
    float32_t lead = stop_lead_cm();

    if (remaining <= (MOTION_DISTANCE_TOL_CM + lead))
    {
        return true;
    }

    speed = sqrtf(2.0f * MOTION_DECEL_CM_S2 * remaining);
    speed = clamp_f(speed, minimum, s_cruise_cm_s);

    /* left - right mismatch: positive means the left wheel is ahead */
    trim = straight_trim(left - right, MOTION_STRAIGHT_TRIM_MAX_CM_S);

    target[MOTION_SIDE_LEFT] = (direction * speed) - trim;
    target[MOTION_SIDE_RIGHT] = (direction * speed) + trim;
    return false;
}

static bool profile_turn (float32_t target[MOTION_SIDE_COUNT])
{
    float32_t direction = sign_f(s_active.value);
    float32_t goal_deg = abs_f(s_active.value) * MOTION_TURN_SCALE;
    float32_t left =
            s_position_cm[MOTION_SIDE_LEFT]
            - s_start_position_cm[MOTION_SIDE_LEFT];
    float32_t right =
            s_position_cm[MOTION_SIDE_RIGHT]
            - s_start_position_cm[MOTION_SIDE_RIGHT];
    float32_t turned_deg = heading_deg_from(left, right);
    float32_t remaining_deg = goal_deg - (direction * turned_deg);
    float32_t arc_cm;
    float32_t speed;
    float32_t lead_deg =
            (stop_lead_cm() / (MOTION_WHEEL_BASE_CM * 0.5f)) * RAD_TO_DEG;

    if (remaining_deg <= (MOTION_ANGLE_TOL_DEG + lead_deg))
    {
        return true;
    }

    arc_cm = remaining_deg * DEG_TO_RAD * (MOTION_WHEEL_BASE_CM * 0.5f);
    speed = sqrtf(2.0f * MOTION_DECEL_CM_S2 * arc_cm);
    speed = clamp_f(speed, MOTION_MIN_TURN_SPEED_CM_S, MOTION_TURN_SPEED_CM_S);

    /* Left turn: left wheel backward, right wheel forward. */
    target[MOTION_SIDE_LEFT] = -direction * speed;
    target[MOTION_SIDE_RIGHT] = direction * speed;
    return false;
}

static void profile_drive (float32_t target[MOTION_SIDE_COUNT])
{
    target[MOTION_SIDE_LEFT] = s_active.left_cm_s;
    target[MOTION_SIDE_RIGHT] = s_active.right_cm_s;

    if (s_active.left_cm_s == s_active.right_cm_s)
    {
        float32_t left =
                s_position_cm[MOTION_SIDE_LEFT]
                - s_start_position_cm[MOTION_SIDE_LEFT];
        float32_t right =
                s_position_cm[MOTION_SIDE_RIGHT]
                - s_start_position_cm[MOTION_SIDE_RIGHT];
        float32_t speed = abs_f(s_active.left_cm_s);
        /* At crawl speed, keep the correction small enough for both wheels to
         * drive. */
        float32_t limit = clamp_f((speed - ZERO_SPEED_CM_S) * 0.5f,
                              0.0f, MOTION_STRAIGHT_TRIM_MAX_CM_S);
        float32_t trim = straight_trim(left - right, limit);
        target[MOTION_SIDE_LEFT] -= trim;
        target[MOTION_SIDE_RIGHT] += trim;
    }
}

/* ------------------------------------------------------------------ */
/* State handlers                                                      */
/* ------------------------------------------------------------------ */
static void run_executing (float32_t dt_s)
{
    float32_t target[MOTION_SIDE_COUNT] = { 0.0f, 0.0f };
    bool b_done = false;
    uint32_t idx;
    motion_side_t side;
    float32_t rate;
    float32_t applied;

    switch (s_active.type)
    {
        case COMMAND_MOVE:
        b_done = profile_move(target);
        break;
        case COMMAND_TURN:
        b_done = profile_turn(target);
        break;
        case COMMAND_DRIVE:
        profile_drive(target);
        break;
        default:
        b_done = true;
        break;
    }

    if (b_done)
    {
        begin_stop();
        return;
    }

    /* Terrain speed scale for straight moves and continuous drive (line
       following). Spins in place keep their speed. The accel / decel ramp
       below makes the change smooth. */
    if ((COMMAND_MOVE == s_active.type) || (COMMAND_DRIVE == s_active.type))
    {
        target[MOTION_SIDE_LEFT] *= s_terrain_scale;
        target[MOTION_SIDE_RIGHT] *= s_terrain_scale;
    }

    for (idx = 0u; idx < (uint32_t)MOTION_SIDE_COUNT; idx++)
    {
        side = (motion_side_t)idx;
        rate = (abs_f(target[idx]) < abs_f(s_setpoint_cm_s[idx]))
                ? MOTION_DECEL_CM_S2
                : MOTION_ACCEL_CM_S2;
        s_setpoint_cm_s[idx] =
                approach(s_setpoint_cm_s[idx], target[idx], rate * dt_s);
        applied = wheel_control(side, s_setpoint_cm_s[idx], dt_s);

        if ((pushing_hard(idx, applied)) &&
            (abs_f(s_speed_cm_s[idx]) < MOTION_STALL_SPEED_CM_S))
        {
            s_stall_ms[idx] += MOTION_UPDATE_PERIOD_MS;
        }
        else
        {
            s_stall_ms[idx] = 0u;
        }

        if (s_stall_ms[idx] >= MOTION_STALL_TIME_MS)
        {
            enter_error(MOTION_FAULT_STALL);
            return;
        }
    }
}

static void run_stopping (void)
{
    motion_motor_brake_all();

    if ((abs_f(s_speed_cm_s[MOTION_SIDE_LEFT]) < MOTION_SETTLE_SPEED_CM_S) &&
        (abs_f(s_speed_cm_s[MOTION_SIDE_RIGHT]) < MOTION_SETTLE_SPEED_CM_S))
    {
        s_settle_cycles++;
    }
    else
    {
        s_settle_cycles = 0u;
    }

    s_settle_ms += MOTION_UPDATE_PERIOD_MS;

    if ((s_settle_cycles >= MOTION_SETTLE_CYCLES) ||
        (s_settle_ms >= MOTION_SETTLE_TIMEOUT_MS))
    {
        s_active.type = COMMAND_NONE;
        s_state = MOTION_STATE_IDLE;
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */
motion_result_t motion_init (void)
{
    uint32_t idx;
    float32_t circumference_cm = MOTION_PI * MOTION_WHEEL_DIAMETER_CM;

    if (MF(MF_INITIALISED))
    {
        return MOTION_OK;
    }

    s_cm_per_count[MOTION_SIDE_LEFT] =
            circumference_cm / MOTION_COUNTS_PER_REV_LEFT;
    s_cm_per_count[MOTION_SIDE_RIGHT] =
            circumference_cm / MOTION_COUNTS_PER_REV_RIGHT;

    motion_motor_init();
    motion_encoder_init();

    for (idx = 0u; idx < (uint32_t)MOTION_SIDE_COUNT; idx++)
    {
        motion_pid_init(&s_pid[idx], MOTION_PID_KP, MOTION_PID_KI,
                        MOTION_PID_KD,
                        MOTION_PID_INTEGRAL_LIMIT, -1.0f, 1.0f);
        s_base_count[idx] = motion_encoder_get_count((motion_side_t)idx);
        s_position_cm[idx] = 0.0f;
        s_previous_position_cm[idx] = 0.0f;
        s_start_position_cm[idx] = 0.0f;
        s_speed_cm_s[idx] = 0.0f;
        s_setpoint_cm_s[idx] = 0.0f;
        s_direction[idx] = 1;
        s_status_count[idx] = 0;
        s_stall_ms[idx] = 0u;
        s_terrain_duty[idx] = 0.0f;
        s_terrain_hump_duty[idx] = 0.0f;
    }

    robot_flags_clear(&s_flags, MF_TERRAIN_HAVE);
    s_terrain_input.valid = false;
    s_terrain = MOTION_TERRAIN_FLAT;
    s_terrain_scale = 1.0f;
    s_terrain_grade_duty = 0.0f;
    robot_flags_clear(&s_flags, MF_PENDING_VALID | MF_ESTOP_REQUEST);
    s_active.type = COMMAND_NONE;
    s_fault = MOTION_FAULT_NONE;
    s_state = MOTION_STATE_IDLE;
    s_cruise_cm_s = MOTION_DEFAULT_SPEED_CM_S;
    s_last_update_us = now_us();
    robot_flags_set(&s_flags, MF_INITIALISED);
    return MOTION_OK;
}

void motion_update (void)
{
    uint64_t now;
    float32_t dt_s;
    command_t command;
    bool b_have_command;
    uint32_t saved;

    if (!MF(MF_INITIALISED))
    {
        return;
    }

    now = now_us();
    dt_s = (float32_t)(now - s_last_update_us) / US_PER_SECOND;
    s_last_update_us = now;
    dt_s = clamp_f(dt_s, 0.5f * PERIOD_S, 2.0f * PERIOD_S);

    read_sensors(dt_s);
    update_terrain(now, dt_s);

    if ((MF(MF_ESTOP_REQUEST)) && ((MOTION_STATE_ERROR != s_state) ||
                                   (MOTION_FAULT_ESTOP != s_fault)))
    {
        enter_error(MOTION_FAULT_ESTOP);
    }

    saved = critical_enter();
    b_have_command = MF(MF_PENDING_VALID);
    command = s_pending;
    robot_flags_clear_nl(&s_flags, MF_PENDING_VALID);
    critical_exit(saved);

    if (b_have_command && (MOTION_STATE_ERROR != s_state))
    {
        start_command(&command);
    }

    switch (s_state)
    {
        case MOTION_STATE_EXECUTING:
        run_executing(dt_s);
        break;
        case MOTION_STATE_STOPPING:
        run_stopping();
        break;
        case MOTION_STATE_IDLE:
        case MOTION_STATE_ERROR:
        default:
        motion_motor_brake_all();
        break;
    }
}

static motion_result_t queue_command (const command_t * p_command)
{
    uint32_t saved;

    if (!MF(MF_INITIALISED))
    {
        return MOTION_ERR_NOT_INIT;
    }

    if ((MOTION_STATE_ERROR == s_state) || (MF(MF_ESTOP_REQUEST)))
    {
        return MOTION_ERR_FAULT;
    }

    saved = critical_enter();
    s_pending = *p_command;
    robot_flags_set_nl(&s_flags, MF_PENDING_VALID);
    critical_exit(saved);
    return MOTION_OK;
}

static motion_result_t queue_move (float32_t signed_distance_cm)
{
    command_t command;

    if (abs_f(signed_distance_cm) < MOTION_DISTANCE_TOL_CM)
    {
        return MOTION_ERR_INVALID_ARG;
    }

    command.type = COMMAND_MOVE;
    command.value = signed_distance_cm;
    command.left_cm_s = 0.0f;
    command.right_cm_s = 0.0f;
    return queue_command(&command);
}

static motion_result_t queue_turn (float32_t signed_angle_deg)
{
    command_t command;

    if ((abs_f(signed_angle_deg) < MOTION_ANGLE_TOL_DEG) ||
        (abs_f(signed_angle_deg) > MAX_TURN_DEG))
    {
        return MOTION_ERR_INVALID_ARG;
    }

    command.type = COMMAND_TURN;
    command.value = signed_angle_deg;
    command.left_cm_s = 0.0f;
    command.right_cm_s = 0.0f;
    return queue_command(&command);
}

motion_result_t motion_move_forward (float32_t distance_cm)
{
    return (distance_cm > 0.0f) ? queue_move(distance_cm)
            : MOTION_ERR_INVALID_ARG;
}

motion_result_t motion_move_backward (float32_t distance_cm)
{
    return (distance_cm > 0.0f) ? queue_move(-distance_cm)
            : MOTION_ERR_INVALID_ARG;
}

motion_result_t motion_turn_left (float32_t angle_deg)
{
    return (angle_deg > 0.0f) ? queue_turn(angle_deg) : MOTION_ERR_INVALID_ARG;
}

motion_result_t motion_turn_right (float32_t angle_deg)
{
    return (angle_deg > 0.0f) ? queue_turn(-angle_deg) : MOTION_ERR_INVALID_ARG;
}

motion_result_t motion_drive (float32_t left_cm_s, float32_t right_cm_s)
{
    command_t command;

    command.value = 0.0f;
    command.left_cm_s =
            clamp_f(left_cm_s, -MOTION_MAX_SPEED_CM_S, MOTION_MAX_SPEED_CM_S);
    command.right_cm_s =
            clamp_f(right_cm_s, -MOTION_MAX_SPEED_CM_S, MOTION_MAX_SPEED_CM_S);

    if ((abs_f(command.left_cm_s) < ZERO_SPEED_CM_S) &&
        (abs_f(command.right_cm_s) < ZERO_SPEED_CM_S))
    {
        command.type = COMMAND_STOP;
    }
    else
    {
        command.type = COMMAND_DRIVE;
    }

    return queue_command(&command);
}

motion_result_t motion_set_speed (float32_t speed_cm_s)
{
    if ((speed_cm_s <= 0.0f) || (speed_cm_s > MOTION_MAX_SPEED_CM_S))
    {
        return MOTION_ERR_INVALID_ARG;
    }

    s_cruise_cm_s = speed_cm_s;
    return MOTION_OK;
}

bool motion_get_tilt (float32_t * p_pitch_deg, float32_t * p_roll_deg)
{
    motion_terrain_input_t input;
    bool b_have;
    uint64_t stamp;
    uint64_t now;
    uint32_t saved;

    if ((p_pitch_deg == (float32_t *)0) || (p_roll_deg == (float32_t *)0)
        || (!MF(MF_INITIALISED)))
    {
        return false;
    }

    now = now_us();
    saved = critical_enter();
    input = s_terrain_input;
    b_have = MF(MF_TERRAIN_HAVE);
    stamp = s_terrain_time_us;
    critical_exit(saved);

    if ((!b_have) || (!input.valid) ||
        ((now > stamp) && ((now - stamp) > ((uint64_t)MOTION_TERRAIN_STALE_MS
                                            * US_PER_MS))))
    {
        return false;
    }

    *p_pitch_deg = input.pitch_deg;
    *p_roll_deg = input.roll_deg;
    return true;
}

motion_result_t motion_set_terrain (const motion_terrain_input_t * p_input)
{
    uint32_t saved;

    if (!MF(MF_INITIALISED))
    {
        return MOTION_ERR_NOT_INIT;
    }

    if (p_input == (const motion_terrain_input_t *)0)
    {
        return MOTION_ERR_INVALID_ARG;
    }

    if ((p_input->valid) &&
        ((!isfinite(p_input->pitch_deg)) || (!isfinite(p_input->roll_deg)) ||
         (p_input->pitch_state < -1) || (p_input->pitch_state > 1) ||
         (p_input->roll_state < -1) || (p_input->roll_state > 1)))
    {
        return MOTION_ERR_INVALID_ARG;
    }

    saved = critical_enter();
    s_terrain_input = *p_input;
    s_terrain_time_us = now_us();
    robot_flags_set_nl(&s_flags, MF_TERRAIN_HAVE);
    critical_exit(saved);
    return MOTION_OK;
}

void motion_stop (void)
{
    command_t command;

    command.type = COMMAND_STOP;
    command.value = 0.0f;
    command.left_cm_s = 0.0f;
    command.right_cm_s = 0.0f;
    (void)queue_command(&command);
}

void motion_emergency_stop (void)
{
    robot_flags_set(&s_flags, MF_ESTOP_REQUEST);
    motion_motor_set_latch(true);   /* brakes both motors immediately */
}

motion_result_t motion_reset (void)
{
    uint32_t saved;

    if (!MF(MF_INITIALISED))
    {
        return MOTION_ERR_NOT_INIT;
    }

    if (MOTION_STATE_ERROR != s_state)
    {
        return MOTION_OK;
    }

    saved = critical_enter();
    robot_flags_clear_nl(&s_flags, MF_ESTOP_REQUEST | MF_PENDING_VALID);
    s_fault = MOTION_FAULT_NONE;
    reset_controllers();
    motion_motor_set_latch(false);
    s_state = MOTION_STATE_IDLE;
    critical_exit(saved);
    return MOTION_OK;
}

motion_result_t motion_reset_odometry (void)
{
    uint32_t idx;
    uint32_t saved;

    if (!MF(MF_INITIALISED))
    {
        return MOTION_ERR_NOT_INIT;
    }

    if ((MOTION_STATE_IDLE != s_state) || (MF(MF_PENDING_VALID)))
    {
        return MOTION_ERR_BUSY;
    }

    saved = critical_enter();

    for (idx = 0u; idx < (uint32_t)MOTION_SIDE_COUNT; idx++)
    {
        s_base_count[idx] = motion_encoder_get_count((motion_side_t)idx);
        s_position_cm[idx] = 0.0f;
        s_previous_position_cm[idx] = 0.0f;
        s_start_position_cm[idx] = 0.0f;
        s_status_count[idx] = 0;
    }

    critical_exit(saved);
    return MOTION_OK;
}

bool motion_is_idle (void)
{
    return (MOTION_STATE_IDLE == s_state) && (!MF(MF_PENDING_VALID));
}

void motion_get_status (motion_status_t * p_status)
{
    uint32_t saved;

    if (p_status == (motion_status_t *)0)
    {
        return;
    }

    saved = critical_enter();
    p_status->state = s_state;
    p_status->fault = s_fault;
    p_status->left_count = s_status_count[MOTION_SIDE_LEFT];
    p_status->right_count = s_status_count[MOTION_SIDE_RIGHT];
    p_status->left_speed_cm_s = s_speed_cm_s[MOTION_SIDE_LEFT];
    p_status->right_speed_cm_s = s_speed_cm_s[MOTION_SIDE_RIGHT];
    p_status->speed_cm_s = 0.5f * (s_speed_cm_s[MOTION_SIDE_LEFT] +
                                   s_speed_cm_s[MOTION_SIDE_RIGHT]);
    p_status->distance_cm = 0.5f * (s_position_cm[MOTION_SIDE_LEFT] +
                                    s_position_cm[MOTION_SIDE_RIGHT]);
    p_status->heading_deg = heading_deg_from(s_position_cm[MOTION_SIDE_LEFT],
                                             s_position_cm[MOTION_SIDE_RIGHT]);
    p_status->terrain = s_terrain;
    p_status->terrain_speed_scale = s_terrain_scale;
    p_status->left_terrain_duty = s_terrain_duty[MOTION_SIDE_LEFT];
    p_status->right_terrain_duty = s_terrain_duty[MOTION_SIDE_RIGHT];
    critical_exit(saved);
}

/*** end of file ***/

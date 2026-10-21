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
#include "motion_control.h"

#include <math.h>

#include "motion_config.h"
#include "motion_encoder.h"
#include "motion_motor.h"
#include "motion_pid.h"

#ifdef MOTION_HOST_TEST
#include "host_platform.h"
#else
#include "hardware/sync.h"
#include "pico/time.h"
#endif

/* ------------------------------------------------------------------ */
/* Private constants and types                                         */
/* ------------------------------------------------------------------ */
#define PERIOD_S            ((float)MOTION_UPDATE_PERIOD_MS / 1000.0f)
#define DEG_TO_RAD          (MOTION_PI / 180.0f)
#define RAD_TO_DEG          (180.0f / MOTION_PI)
#define MAX_TURN_DEG        (720.0f)
#define ZERO_SPEED_CM_S     (0.5f)   /* below this a set-point means "stopped" */
#define DUTY_ZERO           (0.001f)
#define US_PER_SECOND       (1000000.0f)

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
    float value;        /* MOVE: signed cm (+ forward). TURN: signed deg (+ left) */
    float left_cm_s;    /* DRIVE only */
    float right_cm_s;
} command_t;

/* ------------------------------------------------------------------ */
/* Private state (single writer: motion_update, except where noted)    */
/* ------------------------------------------------------------------ */
static bool s_initialised = false;
static volatile motion_state_t s_state = MOTION_STATE_IDLE;
static volatile motion_fault_t s_fault = MOTION_FAULT_NONE;
static volatile bool s_estop_request = false;   /* written by any task */
static volatile bool s_pending_valid = false;   /* written by any task */
static command_t s_pending;
static command_t s_active;
static float s_cruise_cm_s = MOTION_DEFAULT_SPEED_CM_S;

static float s_cm_per_count[MOTION_SIDE_COUNT];
static int32_t s_base_count[MOTION_SIDE_COUNT];
static float s_position_cm[MOTION_SIDE_COUNT];
static float s_previous_position_cm[MOTION_SIDE_COUNT];
static float s_start_position_cm[MOTION_SIDE_COUNT];
static float s_speed_cm_s[MOTION_SIDE_COUNT];
static float s_setpoint_cm_s[MOTION_SIDE_COUNT];
static int32_t s_direction[MOTION_SIDE_COUNT];
static motion_pid_t s_pid[MOTION_SIDE_COUNT];
static uint32_t s_stall_ms[MOTION_SIDE_COUNT];
static uint32_t s_settle_cycles = 0u;
static uint32_t s_settle_ms = 0u;
static uint64_t s_last_update_us = 0u;

/* Latest values for motion_get_status(). */
static int32_t s_status_count[MOTION_SIDE_COUNT];

/* ------------------------------------------------------------------ */
/* Platform helpers                                                    */
/* ------------------------------------------------------------------ */
static uint64_t now_us(void)
{
#ifdef MOTION_HOST_TEST
    return host_time_us();
#else
    return time_us_64();
#endif
}

static uint32_t critical_enter(void)
{
#ifdef MOTION_HOST_TEST
    return 0u;
#else
    return save_and_disable_interrupts();
#endif
}

static void critical_exit(uint32_t saved)
{
#ifdef MOTION_HOST_TEST
    (void)saved;
#else
    restore_interrupts(saved);
#endif
}

/* ------------------------------------------------------------------ */
/* Small maths helpers                                                 */
/* ------------------------------------------------------------------ */
static float abs_f(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float sign_f(float value)
{
    return (value < 0.0f) ? -1.0f : 1.0f;
}

static float clamp_f(float value, float low, float high)
{
    float result = value;

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

static float approach(float current, float target, float max_step)
{
    float delta = target - current;

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
static void read_sensors(float dt_s)
{
    uint32_t i;
    motion_side_t side;
    int32_t counts;
    float position;
    float raw_speed;

    for (i = 0u; i < (uint32_t)MOTION_SIDE_COUNT; i++)
    {
        side = (motion_side_t)i;
        counts = motion_encoder_get_count(side) - s_base_count[i];
        s_status_count[i] = counts;
        position = (float)counts * s_cm_per_count[i];
        raw_speed = (position - s_previous_position_cm[i]) / dt_s;
        s_speed_cm_s[i] += MOTION_SPEED_FILTER_ALPHA * (raw_speed - s_speed_cm_s[i]);
        s_previous_position_cm[i] = position;
        s_position_cm[i] = position;
    }
}

static float heading_deg_from(float left_cm, float right_cm)
{
    return ((right_cm - left_cm) / MOTION_WHEEL_BASE_CM) * RAD_TO_DEG;
}

/* ------------------------------------------------------------------ */
/* Actuation                                                           */
/* ------------------------------------------------------------------ */
/*
 * A single-phase encoder cannot sense direction, so the count sign is the last
 * driven direction. Never flip that direction while the wheel is still
 * turning: brake instead and wait.
 */
static float drive_wheel(motion_side_t side, float duty)
{
    int32_t wanted_direction = (duty < 0.0f) ? -1 : 1;
    float applied = duty;

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

static void reset_controllers(void)
{
    uint32_t i;

    for (i = 0u; i < (uint32_t)MOTION_SIDE_COUNT; i++)
    {
        motion_pid_reset(&s_pid[i]);
        s_setpoint_cm_s[i] = 0.0f;
        s_stall_ms[i] = 0u;
    }
}

/* Speed loop for one wheel. Returns the duty actually applied. */
static float wheel_control(motion_side_t side, float setpoint, float dt_s)
{
    float duty;

    if (abs_f(setpoint) <= ZERO_SPEED_CM_S)
    {
        motion_pid_reset(&s_pid[side]);
        return drive_wheel(side, 0.0f);
    }

    duty = (MOTION_FEEDFORWARD_DUTY_PER_CM_S * setpoint) +
           motion_pid_update(&s_pid[side], setpoint, s_speed_cm_s[side], dt_s);

    if (abs_f(duty) < MOTION_MIN_DUTY)
    {
        duty = sign_f(setpoint) * MOTION_MIN_DUTY;
    }
    if ((MOTION_BREAKAWAY_DUTY > 0.0f) &&
        (abs_f(s_speed_cm_s[side]) < MOTION_BREAKAWAY_SPEED_CM_S) &&
        (abs_f(duty) < MOTION_BREAKAWAY_DUTY))
    {
        duty = sign_f(setpoint) * MOTION_BREAKAWAY_DUTY;   /* wheel is stuck: give it a push */
    }
    duty = clamp_f(duty, -1.0f, 1.0f);
    return drive_wheel(side, duty);
}

/* ------------------------------------------------------------------ */
/* State transitions                                                   */
/* ------------------------------------------------------------------ */
static void begin_stop(void)
{
    motion_motor_brake_all();
    reset_controllers();
    s_settle_cycles = 0u;
    s_settle_ms = 0u;
    s_state = MOTION_STATE_STOPPING;
}

static void enter_error(motion_fault_t fault)
{
    motion_motor_set_latch(true);   /* also brakes both motors */
    reset_controllers();
    s_pending_valid = false;
    s_active.type = COMMAND_NONE;
    s_fault = fault;
    s_state = MOTION_STATE_ERROR;
}

static void start_command(const command_t *p_command)
{
    uint32_t i;

    if (p_command->type == COMMAND_STOP)
    {
        if ((s_state == MOTION_STATE_EXECUTING))
        {
            begin_stop();
        }
        return;
    }

    if (s_state != MOTION_STATE_EXECUTING)
    {
        reset_controllers();
    }
    for (i = 0u; i < (uint32_t)MOTION_SIDE_COUNT; i++)
    {
        s_start_position_cm[i] = s_position_cm[i];
        s_stall_ms[i] = 0u;
    }
    s_active = *p_command;
    s_state = MOTION_STATE_EXECUTING;
}

/* ------------------------------------------------------------------ */
/* Motion profiles: each fills target[] (cm/s per wheel), returns done */
/* ------------------------------------------------------------------ */

/* Distance the robot still travels after the brake is applied. */
static float stop_lead_cm(void)
{
    float mean_speed = 0.5f * (abs_f(s_speed_cm_s[MOTION_SIDE_LEFT]) +
                               abs_f(s_speed_cm_s[MOTION_SIDE_RIGHT]));

    return mean_speed * MOTION_STOP_LEAD_S;
}

static bool profile_move(float target[MOTION_SIDE_COUNT])
{
    float direction = sign_f(s_active.value);
    float goal = abs_f(s_active.value);
    float left = s_position_cm[MOTION_SIDE_LEFT] - s_start_position_cm[MOTION_SIDE_LEFT];
    float right = s_position_cm[MOTION_SIDE_RIGHT] - s_start_position_cm[MOTION_SIDE_RIGHT];
    float travelled = (left + right) * 0.5f;
    float remaining = goal - (direction * travelled);
    float minimum = (MOTION_MIN_SPEED_CM_S < s_cruise_cm_s) ? MOTION_MIN_SPEED_CM_S
                                                            : s_cruise_cm_s;
    float speed;
    float trim;
    float lead = stop_lead_cm();

    if (remaining <= (MOTION_DISTANCE_TOL_CM + lead))
    {
        return true;
    }

    speed = sqrtf(2.0f * MOTION_DECEL_CM_S2 * remaining);
    speed = clamp_f(speed, minimum, s_cruise_cm_s);

    /* left - right mismatch: positive means the left wheel is ahead */
    trim = clamp_f(MOTION_STRAIGHT_KP * (left - right),
                   -MOTION_STRAIGHT_TRIM_MAX_CM_S, MOTION_STRAIGHT_TRIM_MAX_CM_S);

    target[MOTION_SIDE_LEFT] = (direction * speed) - trim;
    target[MOTION_SIDE_RIGHT] = (direction * speed) + trim;
    return false;
}

static bool profile_turn(float target[MOTION_SIDE_COUNT])
{
    float direction = sign_f(s_active.value);
    float goal_deg = abs_f(s_active.value) * MOTION_TURN_SCALE;
    float left = s_position_cm[MOTION_SIDE_LEFT] - s_start_position_cm[MOTION_SIDE_LEFT];
    float right = s_position_cm[MOTION_SIDE_RIGHT] - s_start_position_cm[MOTION_SIDE_RIGHT];
    float turned_deg = heading_deg_from(left, right);
    float remaining_deg = goal_deg - (direction * turned_deg);
    float arc_cm;
    float speed;
    float lead_deg = (stop_lead_cm() / (MOTION_WHEEL_BASE_CM * 0.5f)) * RAD_TO_DEG;

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

static void profile_drive(float target[MOTION_SIDE_COUNT])
{
    target[MOTION_SIDE_LEFT] = s_active.left_cm_s;
    target[MOTION_SIDE_RIGHT] = s_active.right_cm_s;
}

/* ------------------------------------------------------------------ */
/* State handlers                                                      */
/* ------------------------------------------------------------------ */
static void run_executing(float dt_s)
{
    float target[MOTION_SIDE_COUNT] = { 0.0f, 0.0f };
    bool done = false;
    uint32_t i;
    motion_side_t side;
    float rate;
    float applied;

    switch (s_active.type)
    {
        case COMMAND_MOVE:
            done = profile_move(target);
            break;
        case COMMAND_TURN:
            done = profile_turn(target);
            break;
        case COMMAND_DRIVE:
            profile_drive(target);
            break;
        default:
            done = true;
            break;
    }

    if (done)
    {
        begin_stop();
        return;
    }

    for (i = 0u; i < (uint32_t)MOTION_SIDE_COUNT; i++)
    {
        side = (motion_side_t)i;
        rate = (abs_f(target[i]) < abs_f(s_setpoint_cm_s[i])) ? MOTION_DECEL_CM_S2
                                                              : MOTION_ACCEL_CM_S2;
        s_setpoint_cm_s[i] = approach(s_setpoint_cm_s[i], target[i], rate * dt_s);
        applied = wheel_control(side, s_setpoint_cm_s[i], dt_s);

        if ((abs_f(applied) >= MOTION_STALL_DUTY) &&
            (abs_f(s_speed_cm_s[i]) < MOTION_STALL_SPEED_CM_S))
        {
            s_stall_ms[i] += MOTION_UPDATE_PERIOD_MS;
        }
        else
        {
            s_stall_ms[i] = 0u;
        }
        if (s_stall_ms[i] >= MOTION_STALL_TIME_MS)
        {
            enter_error(MOTION_FAULT_STALL);
            return;
        }
    }
}

static void run_stopping(void)
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
motion_result_t motion_init(void)
{
    uint32_t i;
    float circumference_cm = MOTION_PI * MOTION_WHEEL_DIAMETER_CM;

    if (s_initialised)
    {
        return MOTION_OK;
    }

    s_cm_per_count[MOTION_SIDE_LEFT] = circumference_cm / MOTION_COUNTS_PER_REV_LEFT;
    s_cm_per_count[MOTION_SIDE_RIGHT] = circumference_cm / MOTION_COUNTS_PER_REV_RIGHT;

    motion_motor_init();
    motion_encoder_init();

    for (i = 0u; i < (uint32_t)MOTION_SIDE_COUNT; i++)
    {
        motion_pid_init(&s_pid[i], MOTION_PID_KP, MOTION_PID_KI, MOTION_PID_KD,
                        MOTION_PID_INTEGRAL_LIMIT, -1.0f, 1.0f);
        s_base_count[i] = motion_encoder_get_count((motion_side_t)i);
        s_position_cm[i] = 0.0f;
        s_previous_position_cm[i] = 0.0f;
        s_start_position_cm[i] = 0.0f;
        s_speed_cm_s[i] = 0.0f;
        s_setpoint_cm_s[i] = 0.0f;
        s_direction[i] = 1;
        s_status_count[i] = 0;
        s_stall_ms[i] = 0u;
    }

    s_pending_valid = false;
    s_estop_request = false;
    s_active.type = COMMAND_NONE;
    s_fault = MOTION_FAULT_NONE;
    s_state = MOTION_STATE_IDLE;
    s_cruise_cm_s = MOTION_DEFAULT_SPEED_CM_S;
    s_last_update_us = now_us();
    s_initialised = true;
    return MOTION_OK;
}

void motion_update(void)
{
    uint64_t now;
    float dt_s;
    command_t command;
    bool have_command;
    uint32_t saved;

    if (!s_initialised)
    {
        return;
    }

    now = now_us();
    dt_s = (float)(now - s_last_update_us) / US_PER_SECOND;
    s_last_update_us = now;
    dt_s = clamp_f(dt_s, 0.5f * PERIOD_S, 2.0f * PERIOD_S);

    read_sensors(dt_s);

    if (s_estop_request && ((s_state != MOTION_STATE_ERROR) ||
                            (s_fault != MOTION_FAULT_ESTOP)))
    {
        enter_error(MOTION_FAULT_ESTOP);
    }

    saved = critical_enter();
    have_command = s_pending_valid;
    command = s_pending;
    s_pending_valid = false;
    critical_exit(saved);

    if (have_command && (s_state != MOTION_STATE_ERROR))
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

static motion_result_t queue_command(const command_t *p_command)
{
    uint32_t saved;

    if (!s_initialised)
    {
        return MOTION_ERR_NOT_INIT;
    }
    if ((s_state == MOTION_STATE_ERROR) || s_estop_request)
    {
        return MOTION_ERR_FAULT;
    }

    saved = critical_enter();
    s_pending = *p_command;
    s_pending_valid = true;
    critical_exit(saved);
    return MOTION_OK;
}

static motion_result_t queue_move(float signed_distance_cm)
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

static motion_result_t queue_turn(float signed_angle_deg)
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

motion_result_t motion_move_forward(float distance_cm)
{
    return (distance_cm > 0.0f) ? queue_move(distance_cm) : MOTION_ERR_INVALID_ARG;
}

motion_result_t motion_move_backward(float distance_cm)
{
    return (distance_cm > 0.0f) ? queue_move(-distance_cm) : MOTION_ERR_INVALID_ARG;
}

motion_result_t motion_turn_left(float angle_deg)
{
    return (angle_deg > 0.0f) ? queue_turn(angle_deg) : MOTION_ERR_INVALID_ARG;
}

motion_result_t motion_turn_right(float angle_deg)
{
    return (angle_deg > 0.0f) ? queue_turn(-angle_deg) : MOTION_ERR_INVALID_ARG;
}

motion_result_t motion_drive(float left_cm_s, float right_cm_s)
{
    command_t command;

    command.value = 0.0f;
    command.left_cm_s = clamp_f(left_cm_s, -MOTION_MAX_SPEED_CM_S, MOTION_MAX_SPEED_CM_S);
    command.right_cm_s = clamp_f(right_cm_s, -MOTION_MAX_SPEED_CM_S, MOTION_MAX_SPEED_CM_S);

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

motion_result_t motion_set_speed(float speed_cm_s)
{
    if ((speed_cm_s <= 0.0f) || (speed_cm_s > MOTION_MAX_SPEED_CM_S))
    {
        return MOTION_ERR_INVALID_ARG;
    }
    s_cruise_cm_s = speed_cm_s;
    return MOTION_OK;
}

void motion_stop(void)
{
    command_t command;

    command.type = COMMAND_STOP;
    command.value = 0.0f;
    command.left_cm_s = 0.0f;
    command.right_cm_s = 0.0f;
    (void)queue_command(&command);
}

void motion_emergency_stop(void)
{
    s_estop_request = true;
    motion_motor_set_latch(true);   /* brakes both motors immediately */
}

motion_result_t motion_reset(void)
{
    uint32_t saved;

    if (!s_initialised)
    {
        return MOTION_ERR_NOT_INIT;
    }
    if (s_state != MOTION_STATE_ERROR)
    {
        return MOTION_OK;
    }

    saved = critical_enter();
    s_estop_request = false;
    s_pending_valid = false;
    s_fault = MOTION_FAULT_NONE;
    reset_controllers();
    motion_motor_set_latch(false);
    s_state = MOTION_STATE_IDLE;
    critical_exit(saved);
    return MOTION_OK;
}

motion_result_t motion_reset_odometry(void)
{
    uint32_t i;
    uint32_t saved;

    if (!s_initialised)
    {
        return MOTION_ERR_NOT_INIT;
    }
    if ((s_state != MOTION_STATE_IDLE) || s_pending_valid)
    {
        return MOTION_ERR_BUSY;
    }

    saved = critical_enter();
    for (i = 0u; i < (uint32_t)MOTION_SIDE_COUNT; i++)
    {
        s_base_count[i] = motion_encoder_get_count((motion_side_t)i);
        s_position_cm[i] = 0.0f;
        s_previous_position_cm[i] = 0.0f;
        s_start_position_cm[i] = 0.0f;
        s_status_count[i] = 0;
    }
    critical_exit(saved);
    return MOTION_OK;
}

bool motion_is_idle(void)
{
    return (s_state == MOTION_STATE_IDLE) && !s_pending_valid;
}

void motion_get_status(motion_status_t *p_status)
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
    critical_exit(saved);
}

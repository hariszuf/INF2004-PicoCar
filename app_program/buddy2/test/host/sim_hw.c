/**
 * @file sim_hw.c
 * @brief Simulated motor driver and encoders. Implements motion_motor.h and
 *        motion_encoder.h so motion_control.c can run unchanged on a PC.
 */
#include "../../../barrc_types.h"
#include <math.h>
#include <stdlib.h>

#include "sim_hw.h"
#include "host_platform.h"
#include "motion_config.h"
#include "motion_encoder.h"
#include "motion_motor.h"

/* Plant parameters (illustrative only). */
#define SIM_MAX_SPEED_CM_S      (45.0)
#define SIM_RIGHT_GAIN          (1.05)    /* right motor 5 % stronger */
#define SIM_MOTOR_TAU_S         (0.12)
#define SIM_BRAKE_TAU_S         (0.04)
#define SIM_DEAD_ZONE_DUTY      (0.10)
#define SIM_STEP_S              (0.001)

static uint64_t s_time_us;
static float64_t s_duty[MOTION_SIDE_COUNT];
static bool s_braked[MOTION_SIDE_COUNT];
static bool s_stalled[MOTION_SIDE_COUNT];
static bool s_latched;
static float64_t s_load[MOTION_SIDE_COUNT];
static float64_t s_speed[MOTION_SIDE_COUNT];
static float64_t s_position[MOTION_SIDE_COUNT];
static float64_t s_pulse_accum[MOTION_SIDE_COUNT];
static int32_t s_count[MOTION_SIDE_COUNT];
static int32_t s_direction[MOTION_SIDE_COUNT];

static float64_t cm_per_count (motion_side_t side)
{
    float64_t cpr =
            (MOTION_SIDE_LEFT == side) ? (float64_t)MOTION_COUNTS_PER_REV_LEFT
            : (float64_t)MOTION_COUNTS_PER_REV_RIGHT;
    return ((float64_t)MOTION_PI * (float64_t)MOTION_WHEEL_DIAMETER_CM) / cpr;
}

void sim_reset (void)
{
    int32_t idx;

    s_time_us = 0u;
    s_latched = false;

    for (idx = 0; idx < (int32_t)MOTION_SIDE_COUNT; idx++)
    {
        s_duty[idx] = 0.0;
        s_braked[idx] = true;
        s_stalled[idx] = false;
        s_speed[idx] = 0.0;
        s_position[idx] = 0.0;
        s_pulse_accum[idx] = 0.0;
        s_count[idx] = 0;
        s_direction[idx] = 1;
        s_load[idx] = 0.0;
    }
}

uint64_t host_time_us (void)
{
    return s_time_us;
}

static void step_wheel (motion_side_t side)
{
    float64_t gain = (MOTION_SIDE_RIGHT == side) ? SIM_RIGHT_GAIN : 1.0;
    float64_t target = 0.0;
    float64_t delta;
    float64_t travelled;

    float64_t effective;

    if (s_braked[side])
    {
        s_speed[side] -= s_speed[side] * (SIM_STEP_S / SIM_BRAKE_TAU_S);
    }
    else
    {
        effective = s_duty[side] - s_load[side];

        if (fabs(effective) >= SIM_DEAD_ZONE_DUTY)
        {
            target = effective * SIM_MAX_SPEED_CM_S * gain;
        }
        else if ((s_load[side] < 0.0) && (s_duty[side] > 0.0))
        {
            /* gravity overcomes friction */
            target = effective * SIM_MAX_SPEED_CM_S * gain;
        }
        else
        {
            /* no action required */
        }

        s_speed[side] += (target - s_speed[side])
                * (SIM_STEP_S / SIM_MOTOR_TAU_S);
    }

    if (s_stalled[side])
    {
        s_speed[side] = 0.0;
    }

    delta = s_speed[side] * SIM_STEP_S;
    s_position[side] += delta;

    /* Single-phase encoder: pulses carry the last driven direction's sign. */
    travelled = fabs(delta);
    s_pulse_accum[side] += travelled / cm_per_count(side);

    while (s_pulse_accum[side] >= 1.0)
    {
        s_count[side] += s_direction[side];
        s_pulse_accum[side] -= 1.0;
    }
}

void sim_advance_ms (uint32_t milliseconds)
{
    uint32_t idx;

    for (idx = 0u; idx < milliseconds; idx++)
    {
        step_wheel(MOTION_SIDE_LEFT);
        step_wheel(MOTION_SIDE_RIGHT);
        s_time_us += 1000u;
    }
}

float64_t sim_true_position_cm (motion_side_t side)
{
    return s_position[side];
}

float64_t sim_true_heading_deg (void)
{
    float64_t radians =
            (s_position[MOTION_SIDE_RIGHT] - s_position[MOTION_SIDE_LEFT]) /
            (float64_t)MOTION_WHEEL_BASE_CM;
    return radians * (180.0 / (float64_t)MOTION_PI);
}

float64_t sim_true_distance_cm (void)
{
    return 0.5 * (s_position[MOTION_SIDE_LEFT] + s_position[MOTION_SIDE_RIGHT]);
}

void sim_set_stall (motion_side_t side, bool b_stalled)
{
    s_stalled[side] = b_stalled;
}

void sim_set_load (motion_side_t side, float64_t load_duty)
{
    s_load[side] = load_duty;
}

bool sim_is_braked (motion_side_t side)
{
    return s_braked[side];
}

float64_t sim_last_duty (motion_side_t side)
{
    return s_braked[side] ? 0.0 : s_duty[side];
}

/* ---- motion_motor.h ---- */
void motion_motor_init (void)
{
    sim_reset();
}

void motion_motor_brake (motion_side_t side)
{
    s_braked[side] = true;
    s_duty[side] = 0.0;
}

void motion_motor_brake_all (void)
{
    motion_motor_brake(MOTION_SIDE_LEFT);
    motion_motor_brake(MOTION_SIDE_RIGHT);
}

void motion_motor_set (motion_side_t side, float32_t duty)
{
    if (s_latched)
    {
        motion_motor_brake(side);
        return;
    }

    s_braked[side] = false;
    s_duty[side] = (float64_t)duty;
}

void motion_motor_set_latch (bool b_latched)
{
    s_latched = b_latched;

    if (b_latched)
    {
        motion_motor_brake_all();
    }
}

/* ---- motion_encoder.h ---- */
void motion_encoder_init (void)
{
    /* Counts are owned by sim_reset(). */
}

void motion_encoder_irq_handler (uint32_t gpio, uint32_t events)
{
    /* cast: result intentionally ignored */
    (void)gpio;
    /* cast: result intentionally ignored */
    (void)events;
}

int32_t motion_encoder_get_count (motion_side_t side)
{
    return s_count[side];
}

void motion_encoder_set_direction (motion_side_t side, int32_t direction)
{
    s_direction[side] = (direction < 0) ? -1 : 1;
}

void motion_encoder_clear (motion_side_t side)
{
    s_count[side] = 0;
}

/*** end of file ***/

/**
 * @file sim_hw.c
 * @brief Simulated motor driver and encoders. Implements motion_motor.h and
 *        motion_encoder.h so motion_control.c can run unchanged on a PC.
 */
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
static double s_duty[MOTION_SIDE_COUNT];
static bool s_braked[MOTION_SIDE_COUNT];
static bool s_stalled[MOTION_SIDE_COUNT];
static bool s_latched;
static double s_load[MOTION_SIDE_COUNT];
static double s_speed[MOTION_SIDE_COUNT];
static double s_position[MOTION_SIDE_COUNT];
static double s_pulse_accum[MOTION_SIDE_COUNT];
static int32_t s_count[MOTION_SIDE_COUNT];
static int32_t s_direction[MOTION_SIDE_COUNT];

static double cm_per_count(motion_side_t side)
{
    double cpr = (side == MOTION_SIDE_LEFT) ? (double)MOTION_COUNTS_PER_REV_LEFT
                                            : (double)MOTION_COUNTS_PER_REV_RIGHT;
    return ((double)MOTION_PI * (double)MOTION_WHEEL_DIAMETER_CM) / cpr;
}

void sim_reset(void)
{
    int i;

    s_time_us = 0u;
    s_latched = false;
    for (i = 0; i < (int)MOTION_SIDE_COUNT; i++)
    {
        s_duty[i] = 0.0;
        s_braked[i] = true;
        s_stalled[i] = false;
        s_speed[i] = 0.0;
        s_position[i] = 0.0;
        s_pulse_accum[i] = 0.0;
        s_count[i] = 0;
        s_direction[i] = 1;
        s_load[i] = 0.0;
    }
}

uint64_t host_time_us(void)
{
    return s_time_us;
}

static void step_wheel(motion_side_t side)
{
    double gain = (side == MOTION_SIDE_RIGHT) ? SIM_RIGHT_GAIN : 1.0;
    double target = 0.0;
    double delta;
    double travelled;

    double effective;

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
            target = effective * SIM_MAX_SPEED_CM_S * gain;   /* gravity overcomes friction */
        }
        s_speed[side] += (target - s_speed[side]) * (SIM_STEP_S / SIM_MOTOR_TAU_S);
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

void sim_advance_ms(uint32_t milliseconds)
{
    uint32_t i;

    for (i = 0u; i < milliseconds; i++)
    {
        step_wheel(MOTION_SIDE_LEFT);
        step_wheel(MOTION_SIDE_RIGHT);
        s_time_us += 1000u;
    }
}

double sim_true_position_cm(motion_side_t side)
{
    return s_position[side];
}

double sim_true_heading_deg(void)
{
    double radians = (s_position[MOTION_SIDE_RIGHT] - s_position[MOTION_SIDE_LEFT]) /
                     (double)MOTION_WHEEL_BASE_CM;
    return radians * (180.0 / (double)MOTION_PI);
}

double sim_true_distance_cm(void)
{
    return 0.5 * (s_position[MOTION_SIDE_LEFT] + s_position[MOTION_SIDE_RIGHT]);
}

void sim_set_stall(motion_side_t side, bool stalled)
{
    s_stalled[side] = stalled;
}

void sim_set_load(motion_side_t side, double load_duty)
{
    s_load[side] = load_duty;
}

bool sim_is_braked(motion_side_t side)
{
    return s_braked[side];
}

double sim_last_duty(motion_side_t side)
{
    return s_braked[side] ? 0.0 : s_duty[side];
}

/* ---- motion_motor.h ---- */
void motion_motor_init(void)
{
    sim_reset();
}

void motion_motor_brake(motion_side_t side)
{
    s_braked[side] = true;
    s_duty[side] = 0.0;
}

void motion_motor_brake_all(void)
{
    motion_motor_brake(MOTION_SIDE_LEFT);
    motion_motor_brake(MOTION_SIDE_RIGHT);
}

void motion_motor_set(motion_side_t side, float duty)
{
    if (s_latched)
    {
        motion_motor_brake(side);
        return;
    }
    s_braked[side] = false;
    s_duty[side] = (double)duty;
}

void motion_motor_set_latch(bool latched)
{
    s_latched = latched;
    if (latched)
    {
        motion_motor_brake_all();
    }
}

/* ---- motion_encoder.h ---- */
void motion_encoder_init(void)
{
    /* Counts are owned by sim_reset(). */
}

void motion_encoder_irq_handler(unsigned int gpio, uint32_t events)
{
    (void)gpio;
    (void)events;
}

int32_t motion_encoder_get_count(motion_side_t side)
{
    return s_count[side];
}

void motion_encoder_set_direction(motion_side_t side, int32_t direction)
{
    s_direction[side] = (direction < 0) ? -1 : 1;
}

void motion_encoder_clear(motion_side_t side)
{
    s_count[side] = 0;
}

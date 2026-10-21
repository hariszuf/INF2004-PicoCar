/**
 * @file motion_pid.c
 * @brief PID controller implementation. See motion_pid.h.
 */
#include "motion_pid.h"

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

void motion_pid_init(motion_pid_t *p_pid, float kp, float ki, float kd,
                     float integral_limit, float out_min, float out_max)
{
    if (p_pid == (motion_pid_t *)0)
    {
        return;
    }
    p_pid->kp = kp;
    p_pid->ki = ki;
    p_pid->kd = kd;
    p_pid->integral_limit = integral_limit;
    p_pid->out_min = out_min;
    p_pid->out_max = out_max;
    motion_pid_reset(p_pid);
}

void motion_pid_reset(motion_pid_t *p_pid)
{
    if (p_pid == (motion_pid_t *)0)
    {
        return;
    }
    p_pid->integral = 0.0f;
    p_pid->previous_measurement = 0.0f;
    p_pid->primed = 0;
}

float motion_pid_update(motion_pid_t *p_pid, float setpoint,
                        float measurement, float dt_s)
{
    float error;
    float derivative = 0.0f;
    float output;

    if ((p_pid == (motion_pid_t *)0) || (dt_s <= 0.0f))
    {
        return 0.0f;
    }

    error = setpoint - measurement;

    p_pid->integral += p_pid->ki * error * dt_s;
    p_pid->integral = clamp_f(p_pid->integral, -p_pid->integral_limit,
                              p_pid->integral_limit);

    if (p_pid->primed != 0)
    {
        derivative = -(measurement - p_pid->previous_measurement) / dt_s;
    }
    p_pid->previous_measurement = measurement;
    p_pid->primed = 1;

    output = (p_pid->kp * error) + p_pid->integral + (p_pid->kd * derivative);
    return clamp_f(output, p_pid->out_min, p_pid->out_max);
}

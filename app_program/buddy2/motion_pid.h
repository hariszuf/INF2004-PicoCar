/**
 * @file motion_pid.h
 * @brief Small PID controller used for wheel speed control.
 *
 * Derivative acts on the measurement (no kick on set-point changes) and the
 * integrator is clamped (anti-windup). No hardware dependencies.
 */
#ifndef MOTION_PID_H
#define MOTION_PID_H
#include "../barrc_types.h"

typedef struct
{
    float32_t kp;
    float32_t ki;
    float32_t kd;
    float32_t integral;
    float32_t integral_limit;
    float32_t previous_measurement;
    float32_t out_min;
    float32_t out_max;
    int32_t   primed;   /* 0 until the first update after a reset */
} motion_pid_t;

/** Initialise gains and output limits, and clear internal state. */
void motion_pid_init (motion_pid_t * p_pid, float32_t kp, float32_t ki,
                      float32_t kd,
                      float32_t integral_limit, float32_t out_min,
                      float32_t out_max);

/** Clear integrator and derivative history. Gains are kept. */
void motion_pid_reset (motion_pid_t * p_pid);

/** Run one step. dt_s must be greater than zero. Returns the clamped output. */
float32_t motion_pid_update (motion_pid_t * p_pid, float32_t setpoint,
                             float32_t measurement, float32_t dt_s);

#endif /* MOTION_PID_H */

/*** end of file ***/

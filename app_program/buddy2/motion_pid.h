/**
 * @file motion_pid.h
 * @brief Small PID controller used for wheel speed control.
 *
 * Derivative acts on the measurement (no kick on set-point changes) and the
 * integrator is clamped (anti-windup). No hardware dependencies.
 */
#ifndef MOTION_PID_H
#define MOTION_PID_H

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float integral_limit;
    float previous_measurement;
    float out_min;
    float out_max;
    int   primed;   /* 0 until the first update after a reset */
} motion_pid_t;

/** Initialise gains and output limits, and clear internal state. */
void motion_pid_init(motion_pid_t *p_pid, float kp, float ki, float kd,
                     float integral_limit, float out_min, float out_max);

/** Clear integrator and derivative history. Gains are kept. */
void motion_pid_reset(motion_pid_t *p_pid);

/** Run one step. dt_s must be greater than zero. Returns the clamped output. */
float motion_pid_update(motion_pid_t *p_pid, float setpoint,
                        float measurement, float dt_s);

#endif /* MOTION_PID_H */

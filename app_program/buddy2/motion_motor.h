/**
 * @file motion_motor.h
 * @brief PWM H-bridge driver for the two Robo Pico DC motor channels.
 *
 * Robo Pico truth table (datasheet Table 3): A high / B low = forward,
 * A low / B high = backward, both low = brake, both high = coast.
 * Forward is driven by PWM on input A with B held low, so the PWM off-time is
 * a brake (low-low) phase.
 */
#ifndef MOTION_MOTOR_H
#define MOTION_MOTOR_H

#include <stdbool.h>
#include "motion_types.h"

/** Configure the four motor pins as PWM outputs, all motors braked. */
void motion_motor_init(void);

/**
 * Drive one motor. duty is -1.0 (full backward) to +1.0 (full forward).
 * Ignored (motor braked) while the safety latch is set.
 */
void motion_motor_set(motion_side_t side, float duty);

/** Brake one motor (both inputs low). */
void motion_motor_brake(motion_side_t side);

/** Brake both motors. Safe to call from any task. */
void motion_motor_brake_all(void);

/**
 * Safety latch. While set, motion_motor_set() only brakes. Used by the
 * emergency stop so normal movement commands cannot override it.
 */
void motion_motor_set_latch(bool latched);

#endif /* MOTION_MOTOR_H */

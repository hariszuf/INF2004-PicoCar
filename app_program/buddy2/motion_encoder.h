/**
 * @file motion_encoder.h
 * @brief Single-phase wheel encoder counting (Pico SDK GPIO interrupts).
 *
 * Each wheel has one encoder input (phase A). A single phase gives no
 * direction, so the count is signed using the last driven direction of that
 * motor, which motion_control sets with motion_encoder_set_direction().
 * Assumption: the wheel turns the way it was last driven. motion_control
 * enforces this by never reversing a motor while the wheel is still turning.
 */
#ifndef MOTION_ENCODER_H
#define MOTION_ENCODER_H

#include <stdint.h>
#include "motion_types.h"

/**
 * Configure encoder pins and enable interrupts.
 * If the team owns one shared GPIO interrupt callback (the RP2040 allows only
 * one per core), define MOTION_EXTERNAL_IRQ_DISPATCHER and call
 * motion_encoder_irq_handler() from it. Otherwise this registers its own.
 */
void motion_encoder_init(void);

/** Handler to call from a shared dispatcher for the encoder GPIOs. */
void motion_encoder_irq_handler(unsigned int gpio, uint32_t events);

/** Signed count since init or the last clear. */
int32_t motion_encoder_get_count(motion_side_t side);

/** Set the sign applied to new pulses: +1 forward, -1 backward. */
void motion_encoder_set_direction(motion_side_t side, int32_t direction);

/** Zero a wheel count. */
void motion_encoder_clear(motion_side_t side);

#endif /* MOTION_ENCODER_H */

/**
 * @file sim_hw.h
 * @brief Simulated robot (motors, wheels, single-phase encoders) for host tests.
 *
 * This is a simple first-order motor model, NOT a measurement of the real
 * robot. It checks the control logic and state machine only. Real tuning data
 * must come from the car (docs/PID_Tuning_Report.md).
 */
#ifndef SIM_HW_H
#define SIM_HW_H

#include <stdbool.h>
#include <stdint.h>
#include "motion_types.h"

/** Reset the simulated robot and clock. */
void sim_reset(void);

/** Advance simulated time, integrating the plant in 1 ms steps. */
void sim_advance_ms(uint32_t milliseconds);

/** True distance travelled by a wheel since reset (cm, signed). */
double sim_true_position_cm(motion_side_t side);

/** True heading since reset (degrees, left positive). */
double sim_true_heading_deg(void);

/** True distance of the robot centre since reset (cm). */
double sim_true_distance_cm(void);

/** Lock a wheel to emulate a stalled motor. */
void sim_set_stall(motion_side_t side, bool stalled);

/** True if the motor is currently braked (both inputs low). */
bool sim_is_braked(motion_side_t side);

/** Last duty written to a motor (0 when braked). */
double sim_last_duty(motion_side_t side);

#endif /* SIM_HW_H */

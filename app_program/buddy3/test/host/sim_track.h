/**
 * @file sim_track.h
 * @brief Simulated black line track and IR sensors for host tests (Buddy 3).
 *
 * Uses Buddy 2's simulated robot (sim_hw) for wheel motion. Geometry only;
 * not a measurement of the real track or sensors.
 */
#ifndef SIM_TRACK_H
#define SIM_TRACK_H

#include <stdbool.h>

/** Build the track and put the robot at its start. Call after motion_init(). */
void track_reset(void);

/** Advance 1 ms: move the robot pose from the wheel motion and refresh sensors. */
void track_step_1ms(void);

/** Distance (mm) from the target point (between the sensors) to the line. */
double track_error_mm(void);

/** Fraction (0..1) of the track length covered by the nearest-point progress. */
double track_progress(void);

/** Robot pose (mm, deg) for printing. */
void track_pose(double *p_x_mm, double *p_y_mm, double *p_heading_deg);

#endif

/**
 * @file line_config.h
 * @brief Configuration for Line Following (Buddy 3).
 *
 * Based on the team's two-IR line follower (ir_motor_curve_test.c). Motors,
 * encoders and wheel-speed PID are NOT in this module any more: they come from
 * the Buddy 2 Motion Control library (motion_drive()).
 * Values marked TUNE / MEASURE are placeholders for the real car.
 */
#ifndef LINE_CONFIG_H
#define LINE_CONFIG_H

#include "junction_map.h"

/* ------------------------------------------------------------------ */
/* Pins (from ir_motor_curve_test.c)                                   */
/* ------------------------------------------------------------------ */
#define LINE_PIN_GP0_IR             (0u)   /* line sensor seen on the left of the car in the photo */
#define LINE_PIN_GP2_IR             (2u)   /* line sensor seen on the right of the car in the photo */
#define LINE_PIN_BARCODE_IR         (26u)  /* barcode sensor, monitored only, not decoded */

/** 1 if GP0 is on the car's LEFT when facing the direction of travel, 0 if on its RIGHT. */
#define LINE_GP0_IS_CAR_LEFT        (0u)   /* MEASURE */
/** Pin level that means black. Confirmed 1 = black on this car. */
#define LINE_BLACK_LEVEL            (1u)

/**
 * How the two line sensors sit relative to the 18 mm line.
 *  1 = STRADDLE: the sensors are far apart (one each side of the chassis) and the
 *      line runs BETWEEN them. Both white = on track, drive straight. A sensor that
 *      sees black means the line is drifting toward that side, so steer that way.
 *  0 = BOTH ON THE LINE: the sensors are close together (under 18 mm apart) and
 *      both start on the line. Both white = line lost.
 * Use straddle mode when the sensing spots are wider apart than the line.
 * Place one sensor on each side, with both seeing white when centred.
 */
#define LINE_SENSORS_STRADDLE       (1u)     /* centred with both sensors over white */

/**
 * Straddle mode only: white = move, black = stop.
 *  1 = both sensors over black stops the car. It moves again as soon as at least
 *      one sensor sees white. One sensor over black still steers toward that side
 *      (that side's wheel stops, so the car turns).
 *  0 = both sensors over black drives straight (previous behaviour).
 */
#define LINE_BOTH_BLACK_STOP        (1u)

/** Hump/ramp freeze. A raised wheel lifts its IR sensor off the floor, which reads as black and would
 *  stop that wheel. While Buddy 4 reports the car tilted, the follower FREEZES: it keeps the last wheel
 *  commands and ignores the sensors. It releases as soon as the tilt is gone (no extra delay), so turns
 *  are not slowed. Uses the raw tilt angles (faster than Buddy 4/2's terrain state) and the terrain state.
 *  Needs the IMU; with no IMU the freeze never triggers. 0 = off. */
#define LINE_TERRAIN_HOLD           (1u)
#define LINE_FREEZE_ROLL_DEG        (1.5f)   /* TUNE: freeze when |roll| is above this */
#define LINE_FREEZE_PITCH_DEG       (2.0f)   /* TUNE: freeze when |pitch| is above this */

/** "+" junction (straddle mode). Both sensors over black means the car has reached a junction: it
 *  stops, then turns left / goes straight / turns right, then carries on following the line.
 *  The direction comes from the barcode letter (see junction_map.h: A=left, B=straight, C=right, D=left ...).
 *  Needs LINE_SENSORS_STRADDLE and LINE_BOTH_BLACK_STOP. 0 = off (both black simply stops the car). */
#ifndef LINE_JUNCTION
#define LINE_JUNCTION               (1u)
#endif
/** 1 = use the latest barcode letter for each junction. 0 = always use LINE_JUNCTION_DEFAULT_ACTION (testing,
 *  no barcode yet). With 1 and no new letter since the last junction, the default action is used. */
#ifndef LINE_JUNCTION_USE_BARCODE
#define LINE_JUNCTION_USE_BARCODE   (1u)
#endif
/** Action when there is no new barcode letter: JUNCTION_NONE = stop and stay stopped (default, never guess),
 *  or JUNCTION_LEFT / JUNCTION_STRAIGHT / JUNCTION_RIGHT to force a turn for testing. */
#ifndef LINE_JUNCTION_DEFAULT_ACTION
#define LINE_JUNCTION_DEFAULT_ACTION JUNCTION_NONE
#endif
/** The sensors sit ahead of the wheel axle. Before turning, drive forward this far so the car turns at the
 *  middle of the junction. TUNE: about the distance from the axle to the sensors. 0 = turn on the spot. */
/** Pause at the junction (car stopped) before it advances and turns. 0 = no pause. */
#ifndef LINE_JUNCTION_PAUSE_MS
#define LINE_JUNCTION_PAUSE_MS      (3000u)
#endif
#define LINE_JUNCTION_ADVANCE_CM    (8.0f)
/** Distance driven to cross the junction when the action is STRAIGHT (past the cross bar). TUNE. */
#define LINE_JUNCTION_CROSS_CM      (10.0f)
#define LINE_JUNCTION_TURN_DEG      (90.0f)
/** The turn is done in two parts. First TURN_DEG minus this margin by encoder (sensors ignored), then a slow
 *  pivot until a sensor sees the new line. Open-loop turns are a few degrees off and the line only just fits
 *  between the sensors, so the last part is closed on the sensors. TUNE: bigger = safer, smaller = quicker. */
#define LINE_JUNCTION_PRETURN_MARGIN_DEG (20.0f)
/** Wheel speed (one forward, one backward) of the slow pivot that looks for the new line. */
#define LINE_JUNCTION_SEARCH_SPEED_CM_S (4.0f)
/** Give up looking for the new line after this long (the car stops). */
#define LINE_JUNCTION_SEARCH_MS     (3000u)
/** Give up on a junction step that takes longer than this (the car stops). */
#define LINE_JUNCTION_TIMEOUT_MS    (8000u)
/* Bypass hand-back: after the junction-style search, centre the tape between the two IR sensors. */
#ifndef LINE_REJOIN_CENTRE
#define LINE_REJOIN_CENTRE          (1)
#endif
#define LINE_REJOIN_CENTRE_SPEED_CM_S (2.0f)   /* final approach to the middle heading */
#define LINE_REJOIN_CENTRE_LEAD_DEG (1.0f)   /* braking allowance for the final pivot to the middle */
#define LINE_REJOIN_SLOW_ZONE_DEG   (8.0f)   /* pivot at the search speed until this close to the middle, then slow */
#define LINE_REJOIN_CENTRE_MS       (8000u)  /* upper limit for the pivot to the middle */
#define LINE_REJOIN_SEARCH_SPAN_DEG (110.0f) /* each sweep may pivot this far (the second one twice this) before giving up */

/* ------------------------------------------------------------------ */
/* Timing                                                              */
/* ------------------------------------------------------------------ */
/** line_follow_update() period: sensors are checked every 5 ms for quick steering.
 *  Motion Control keeps its own 20 ms period (MOTION_UPDATE_PERIOD_MS). */
#define LINE_UPDATE_PERIOD_MS       (5u)

/* ------------------------------------------------------------------ */
/* Speeds (cm/s). Much slower than the Buddy 2 demo.                   */
/* Team test used 300 counts/s straight, 120 inner (about 5 and 2 cm/s). */
/* ------------------------------------------------------------------ */
#define LINE_STRAIGHT_SPEED_CM_S    (3.0f)
#define LINE_TURN_OUTER_SPEED_CM_S  (3.0f)
#ifndef LINE_RIGHT_OUTER_SPEED_CM_S
#define LINE_RIGHT_OUTER_SPEED_CM_S LINE_TURN_OUTER_SPEED_CM_S
#endif
#define LINE_TURN_INNER_SPEED_CM_S  (0.0f)   /* Stop the inner wheel for a clear correction at low speed. */

/** Optional (off by default). Escalating turn: if only one sensor keeps seeing the line for this long, the
 *  turn is not tight enough (sharp curve), so the inside wheel is driven at
 *  LINE_ESCALATE_INNER_CM_S (negative = backwards) to pivot harder. */
#define LINE_ESCALATE_AFTER_MS      (0u)     /* 0 = off (basic rule). TUNE: e.g. 150 to pivot harder on sharp curves */
#define LINE_ESCALATE_INNER_CM_S    (-2.0f)  /* TUNE */

/** Per-wheel speed scale during turns (1.0 = none).
 *  Straight segments use equal targets and Buddy 2's encoder correction. */
#ifndef LINE_LEFT_WHEEL_SCALE
#define LINE_LEFT_WHEEL_SCALE       (0.94f)  /* Trial trim for observed right drift. */
#endif
#define LINE_RIGHT_WHEEL_SCALE      (1.0f)   /* TUNE */

/**
 * Buddy 2's default minimum motor duty (15 %) makes a wheel run much faster than
 * a few cm/s, so the CMake file lowers it for this program with
 *   MOTION_MIN_DUTY=0.10f  MOTION_FEEDFORWARD_DUTY_PER_CM_S=0.0125f
 * adds a break-away push (MOTION_BREAKAWAY_DUTY=0.22f) so a still wheel always starts,
 * and relaxes the stall fault (MOTION_STALL_DUTY=0.6f, MOTION_STALL_TIME_MS=1500u),
 * because a slow turn can legitimately make a wheel push hard for a moment.
 * (about the 6 % the team's own test used at 300 counts/s). The speed PID then
 * raises the duty until the wheel really reaches the requested speed.
 */

/* ------------------------------------------------------------------ */
/* Lost line                                                           */
/* ------------------------------------------------------------------ */
/** Neither sensor sees black: keep turning toward the side the line was last
 *  seen for this long, then stop. Set to 0 for the original behaviour (stop at once). */
#define LINE_RECOVER_MS             (500u)   /* tolerate a brief sensor dropout before stopping */
#define LINE_RECOVER_INNER_CM_S     (0.0f)   /* search without reversing an inner wheel */

/* ------------------------------------------------------------------ */
/* Sensor geometry (host simulation only)                              */
/* ------------------------------------------------------------------ */
#define LINE_SENSOR_SPACING_MM      (12.0f)  /* MEASURE: distance between the two line sensors (host simulation only; use ~110 with STRADDLE) */
#define LINE_SENSOR_FORWARD_MM      (50.0f)  /* MEASURE */
#define LINE_WIDTH_MM               (18.0f)

#endif /* LINE_CONFIG_H */

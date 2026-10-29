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
 * Which one applies depends on how the sensors are mounted on the car. Default is 0
 * (the team code's assumption); set 1 only if the sensors are wider apart than the line.
 */
#define LINE_SENSORS_STRADDLE       (0u)     /* MEASURE: unconfirmed, see README */
/** In STRADDLE mode, stop if neither sensor has seen the line for this long (end of track). */
#define LINE_STRADDLE_STOP_MS       (12000u) /* TUNE */

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
#define LINE_STRAIGHT_SPEED_CM_S    (3.0f)   /* TUNE: both sensors on the line */
#define LINE_TURN_OUTER_SPEED_CM_S  (3.5f)   /* left-turn outside wheel (right motor) */
/* TEMP TEST: right turns depend on the LEFT motor, so raise its target gently.
   If testing shows actual left wheel speed cannot reach the command, investigate
   hardware / wheel stiction instead of increasing speed again. */
#define LINE_RIGHT_OUTER_SPEED_CM_S (5.0f)
#define LINE_TURN_INNER_SPEED_CM_S  (0.0f)   /* TUNE: inside wheel while steering (0 = brake and pivot) */

/** Optional (off by default). Escalating turn: if only one sensor keeps seeing the line for this long, the
 *  turn is not tight enough (sharp curve), so the inside wheel is driven at
 *  LINE_ESCALATE_INNER_CM_S (negative = backwards) to pivot harder. */
#define LINE_ESCALATE_AFTER_MS      (0u)     /* 0 = off (basic rule). TUNE: e.g. 150 to pivot harder on sharp curves */
#define LINE_ESCALATE_INNER_CM_S    (-2.0f)  /* TUNE */

/** Per-wheel speed scale to even out the two motors (1.0 = none). If the car
 *  pulls to one side on a straight, lower the scale of the wheel that is faster. */
#define LINE_LEFT_WHEEL_SCALE       (1.0f)   /* TUNE */
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
#define LINE_RECOVER_INNER_CM_S     (-2.0f)  /* inside wheel while recovering (negative = backwards) */

/* ------------------------------------------------------------------ */
/* Sensor geometry (host simulation only)                              */
/* ------------------------------------------------------------------ */
#define LINE_SENSOR_SPACING_MM      (12.0f)  /* MEASURE: distance between the two line sensors (host simulation only; use ~110 with STRADDLE) */
#define LINE_SENSOR_FORWARD_MM      (50.0f)  /* MEASURE */
#define LINE_WIDTH_MM               (18.0f)

#endif /* LINE_CONFIG_H */

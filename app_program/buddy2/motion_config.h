/**
 * @file motion_config.h
 * @brief Compile-time configuration for Motion Control (Buddy 2).
 *
 * Pins follow the approved Week 6 design review (Section 2).
 * Values marked TUNE are placeholders that must be replaced with
 * measured values (see docs/PID_Tuning_Report.md).
 */
#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

/* ------------------------------------------------------------------ */
/* Pins (Week 6 design review, Section 2)                              */
/* ------------------------------------------------------------------ */
#define MOTION_PIN_LEFT_MOTOR_A     (8u)   /* M1A */
#define MOTION_PIN_LEFT_MOTOR_B     (9u)   /* M1B */
#define MOTION_PIN_RIGHT_MOTOR_A    (10u)  /* M2A */
#define MOTION_PIN_RIGHT_MOTOR_B    (11u)  /* M2B */
#define MOTION_PIN_LEFT_ENCODER     (6u)   /* phase A only */
#define MOTION_PIN_RIGHT_ENCODER    (4u)   /* phase A only */

/**
 * Set to 1u if a motor turns the wrong way when driven "forward".
 * Equivalent to swapping the MxA / MxB wires (Robo Pico datasheet).
 */
#define MOTION_LEFT_MOTOR_INVERT    (0u)
#define MOTION_RIGHT_MOTOR_INVERT   (0u)

/**
 * Encoder edge counted. The counts-per-revolution values below MUST have been
 * measured with the same setting (rising edge only, falling edge only, or
 * both), otherwise every distance and angle is scaled wrongly.
 * 1u = rising edge only, 2u = falling edge only, 3u = both edges.
 */
#define MOTION_ENCODER_EDGE_MODE    (1u)   /* TUNE: match the hand-turn test */

/* ------------------------------------------------------------------ */
/* Measured geometry (team measurements)                               */
/* ------------------------------------------------------------------ */
#define MOTION_WHEEL_DIAMETER_CM        (6.5f)
#define MOTION_WHEEL_BASE_CM            (12.2f)   /* centre of tyre to centre of tyre */
#define MOTION_COUNTS_PER_REV_LEFT      (1265.8f) /* 12658 counts / 10 turns */
#define MOTION_COUNTS_PER_REV_RIGHT     (1268.5f) /* 12685 counts / 10 turns */
#define MOTION_PI                       (3.14159265f)

/**
 * Multiplies the commanded turn angle. 1.0 = ideal geometry. Wheel slip makes
 * the real robot turn less than the geometry predicts, so calibrate this with
 * the 360 degree spin test (docs/Motion_Accuracy_Evaluation.md).
 */
#define MOTION_TURN_SCALE               (0.88f)   /* TUNE: 180 / ~205 deg measured on the car */

/* ------------------------------------------------------------------ */
/* Timing                                                              */
/* ------------------------------------------------------------------ */
/** Period at which motion_update() must be called. One encoder count is
 *  about 0.016 cm, so 20 ms keeps speed quantisation near 0.8 cm/s. */
#define MOTION_UPDATE_PERIOD_MS         (20u)
#define MOTION_PWM_FREQ_HZ              (15000u)  /* driver maximum is 20 kHz */

/* ------------------------------------------------------------------ */
/* Motion profile                                                      */
/* ------------------------------------------------------------------ */
#define MOTION_DEFAULT_SPEED_CM_S       (20.0f)   /* TUNE */
#define MOTION_MAX_SPEED_CM_S           (40.0f)   /* TUNE: measure at full duty */
#define MOTION_MIN_SPEED_CM_S           (6.0f)    /* creep speed near target */
#define MOTION_TURN_SPEED_CM_S          (15.0f)   /* wheel speed during a spin */
#define MOTION_MIN_TURN_SPEED_CM_S      (5.0f)
#define MOTION_ACCEL_CM_S2              (60.0f)   /* TUNE */
#define MOTION_DECEL_CM_S2              (80.0f)   /* TUNE */
#define MOTION_DISTANCE_TOL_CM          (0.3f)
#define MOTION_ANGLE_TOL_DEG            (1.0f)

/* ------------------------------------------------------------------ */
/* Control gains                                                       */
/* ------------------------------------------------------------------ */
/** Wheel speed PID. Output is duty (-1..1) added to the feed-forward term. */
#define MOTION_PID_KP                   (0.020f)  /* TUNE */
#define MOTION_PID_KI                   (0.100f)  /* TUNE */
#define MOTION_PID_KD                   (0.000f)  /* TUNE */
#define MOTION_PID_INTEGRAL_LIMIT       (0.30f)
/* Both can be overridden from the build (e.g. Buddy 3 line following runs much slower). */
#ifndef MOTION_FEEDFORWARD_DUTY_PER_CM_S
#define MOTION_FEEDFORWARD_DUTY_PER_CM_S (0.025f) /* TUNE: duty / (cm/s) */
#endif
/** Break-away: while a wheel that is asked to turn is still (measured speed below
 *  MOTION_BREAKAWAY_SPEED_CM_S), apply at least this duty to overcome stiction.
 *  0 = off. Used by slow line following, where the normal duty floor is very low. */
#ifndef MOTION_BREAKAWAY_DUTY
#define MOTION_BREAKAWAY_DUTY           (0.0f)
#endif
#define MOTION_BREAKAWAY_SPEED_CM_S     (0.3f)

#ifndef MOTION_MIN_DUTY
#define MOTION_MIN_DUTY                 (0.15f)   /* TUNE: stiction compensation */
#endif

/** Straight-line correction: speed trim (cm/s) per cm of left/right mismatch. */
#define MOTION_STRAIGHT_KP              (1.5f)    /* TUNE */
#define MOTION_STRAIGHT_TRIM_MAX_CM_S   (5.0f)

/** Low-pass filter weight for measured wheel speed (0..1, higher = faster). */
#define MOTION_SPEED_FILTER_ALPHA       (0.5f)

/* ------------------------------------------------------------------ */
/* Stopping, reversal and fault detection                              */
/* ------------------------------------------------------------------ */
/** Braking is not instant. A move or turn ends early by (wheel speed x this
 *  time) so the robot stops on target. TUNE: measure with the braking test,
 *  then adjust until the mean distance and angle error are close to zero. */
#define MOTION_STOP_LEAD_S              (0.03f)

#define MOTION_SETTLE_SPEED_CM_S        (0.5f)
#define MOTION_SETTLE_CYCLES            (3u)
#define MOTION_SETTLE_TIMEOUT_MS        (1000u)

/** Single-phase encoder cannot sense direction. Do not reverse a motor
 *  while the wheel is still turning faster than this, or counts go wrong. */
#define MOTION_REVERSAL_SPEED_CM_S      (2.0f)

#ifndef MOTION_STALL_DUTY
#define MOTION_STALL_DUTY               (0.25f)
#endif
#define MOTION_STALL_SPEED_CM_S         (0.5f)
#ifndef MOTION_STALL_TIME_MS
#define MOTION_STALL_TIME_MS            (500u)
#endif

/* ------------------------------------------------------------------ */
/* Terrain (slope / hump) speed control                                */
/* Input comes from Buddy 4's IMU through motion_set_terrain().        */
/* Buddy 2 owns the response: it scales the wheel speed targets and    */
/* adds a duty assist so the speed loop does not have to wait for the  */
/* integral to wind up on a ramp. All values are TUNE on the car.      */
/* ------------------------------------------------------------------ */
/** Terrain input older than this is ignored (car treated as on flat ground). */
#define MOTION_TERRAIN_STALE_MS         (200u)
/** Speed target multipliers while the IMU reports each condition. */
#define MOTION_UPHILL_SPEED_SCALE       (1.0f)    /* TUNE: hold speed, add power */
#define MOTION_DOWNHILL_SPEED_SCALE     (0.7f)    /* TUNE: slow down going down */
#define MOTION_HUMP_SPEED_SCALE         (0.8f)    /* TUNE: one wheel on a hump/ramp */
/** Gravity feed-forward: extra duty per g of slope (x sin(pitch)), both wheels. */
#define MOTION_GRADE_DUTY_PER_G         (0.8f)    /* TUNE */
/** Hump assist: extra duty on the RAISED wheel per sin(|roll|). */
#define MOTION_HUMP_DUTY_PER_G          (0.8f)    /* TUNE */
/** Limits on the terrain duty assist and how fast it may change. */
#define MOTION_TERRAIN_DUTY_MAX         (0.30f)
#define MOTION_TERRAIN_DUTY_RATE_PER_S  (1.5f)
/** Angles beyond this are clamped (sensor glitch / car being handled). */
#define MOTION_TERRAIN_MAX_ANGLE_DEG    (30.0f)

#endif /* MOTION_CONFIG_H */

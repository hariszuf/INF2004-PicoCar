#ifndef PICOCAR_BUDDY5_CONFIG_H
#define PICOCAR_BUDDY5_CONFIG_H
/* User measurements, mm, including protrusions. */
#define B5_CAR_LENGTH_MM 355
#define B5_CAR_WIDTH_MM 170
#define B5_FRONT_OVERHANG_MM 28
#define B5_SENSOR_TO_REAR_MM (B5_CAR_LENGTH_MM - B5_FRONT_OVERHANG_MM)
/* Provisional allowances: validate on the actual track. */
#define B5_CLEARANCE_MM 50
#define B5_EDGE_ALLOWANCE_MM 30
/* HC-SR04 edge/clear-space estimate uncertainty used only when deciding
 * whether a measured side is long enough; planned body clearance is retained. */
#define B5_ROUTE_UNCERTAINTY_MM 30
#define B5_STOP_FRONT_MM 200
#define B5_MAX_PROFILE_MM 1200
#define B5_MAX_OFFSET_MM 700
#define B5_STEP_MM 60
#define B5_MAX_EDGE_BRACKET_MM 100
#define B5_SIDE_MATCH_MM 60
#define B5_CLEAR_JUMP_MM 100
#define B5_EDGE_CONFIRMATIONS 3
#define B5_FEEDBACK_MAX_AGE_MS 250u
#define B5_REQUEST_TIMEOUT_MS 90000u
#define B5_SCAN_TIMEOUT_MS 30000u
#define B5_AVOID_TIMEOUT_MS 120000u /* long open-loop bypass + line recovery */
/* STATIONARY demonstration; motor power off. No motor commands issued.
 * Set 0 only with registered, tested Vehicle Controller callbacks. */
#ifndef B5_BENCH_MODE
#define B5_BENCH_MODE 0
#endif
/* Logical 0=forward, positive=left. CALIBRATE -60/0/+60 on YOUR SG90.
 * Pulses below are a commissioning window, not a guaranteed angle mapping.
 * Geometry remains provisional until angles and bracket clearance are checked. */
/* Provisional photo estimates; pulses retain the previous uncalibrated slope.
 * These are NOT measured mechanical stops. Full test needs physical validation. */
#define B5_SCAN_MIN_DEG (-35)
#define B5_SCAN_MAX_DEG 25
#define B5_SCAN_ZERO_SLOT 7
#define B5_TURN_CLEARANCE_MM (B5_SENSOR_TO_REAR_MM+B5_CAR_LENGTH_MM+B5_CAR_WIDTH_MM/2+B5_CLEARANCE_MM)
#define B5_SERVO_PIN 12u
#define B5_SERVO_RIGHT_US 2000u
#define B5_SERVO_CENTRE_US 1500u
#define B5_SERVO_LEFT_US 1000u
#define B5_SERVO_SETTLE_MS 300u
#define B5_PING_INTERVAL_MS 70u
#define B5_SAMPLE_COUNT 3u
#define B5_MIN_VALID_SAMPLES 2u
#define B5_SAMPLES_MAX_SPREAD_MM 60u
#define B5_TELEMETRY_INTERVAL_MS 1000u
#define B5_TASK_PRIORITY 11
/* Proposed IDs to agree with Buddy 1; existing message layout unchanged. */
#define B5_TELEM_STATE 50u
#define B5_TELEM_SIZE 51u
#define B5_TELEM_PLAN 52u
#define B5_TELEM_DEPTH_BOUNDS 53u
#define B5_TELEM_FRONT 54u
#define B5_TELEM_RAW_RANGE 58u /* Last HC-SR04+ result: mm and ultrasonic_status_t. */
/* Automatic powered-on driving profile: continuous forward readings. */
#define B5_GUARD_FRONT_MM 150
/* After a full scan has selected a side, allow the initial lateral bypass to
 * approach the obstacle edge more closely; the normal forward guard remains. */
#define B5_BYPASS_GUARD_FRONT_MM 80
/* Turn calibration. The old value 4.0 compensated the encoders under-counting before the Buddy 2 encoder fix
 * (a requested 90-degree turn was a full rotation). With correct counts Buddy 2 turns accurately, so 1.0.
 * If bypass turns come out too small/large, TUNE this (-DB5_TURN_ODOMETRY_CORRECTION=..). */
#ifndef B5_TURN_ODOMETRY_CORRECTION
#define B5_TURN_ODOMETRY_CORRECTION 1.0f
#endif

/* Simple bypass: after the scan picks a side, drive sideways, pass the box in ONE move (assumed depth),
 * come back to the line and find it with the IR sensors. 0 = the original step-by-step side-wall sampling. */
#ifndef B5_SIMPLE_BYPASS
#define B5_SIMPLE_BYPASS 1
#endif
/* TUNE: depth of the box along the line (mm) when the scan could not measure it. Bigger = safer. */
#ifndef B5_ASSUMED_DEPTH_MM
#define B5_ASSUMED_DEPTH_MM 350
#endif
/* The scan only sees about +/-35 degrees, so a wide box's edges can be out of view and its width underestimated.
 * Always keep the car at least this far (mm) from the line while passing. TUNE: >= half the widest box + 85 + 50. */
#ifndef B5_MIN_BYPASS_OFFSET_MM
#define B5_MIN_BYPASS_OFFSET_MM 350
#endif
/* Measuring the real box width: if the box fills the +/-35 degree scan window the edges cannot be seen, so the car
 * reverses this far (mm) and scans again, up to B5_MAX_RESCANS times. Needs free space behind the car. */
#ifndef B5_RESCAN_BACKUP_MM
#define B5_RESCAN_BACKUP_MM 150
#endif
#ifndef B5_MAX_RESCANS
#define B5_MAX_RESCANS 2
#endif
/* The straight leg past the box is B5_PASS_WIDTH_FACTOR x the measured box width (B5_UNKNOWN_WIDTH_MM if it could
 * not be measured), counted from where the car first stopped, plus B5_PASS_SAFETY_MM. A deeper box is caught by the
 * sonar check after turning toward the line (see B5_MAX_EXTENDS), which extends the leg. */
#define B5_PASS_WIDTH_FACTOR 2
#define B5_UNKNOWN_WIDTH_MM 500
#define B5_MIN_PASS_MM 400
#ifndef B5_PASS_SAFETY_MM
#define B5_PASS_SAFETY_MM 120
#endif
/* After the pass, turning toward the line: if the sonar still sees something closer than the distance to the
 * line, the car is still beside the box. It reverses a little, faces forward, drives a little further and
 * tries again (up to B5_MAX_EXTENDS times). */
#define B5_RETURN_REVERSE_MM 60
#define B5_EXTEND_MM 150
#define B5_MAX_EXTENDS 3
/* Extra distance past the box rear so the car's tail clears it before turning back (mm). */
#define B5_BYPASS_END_MARGIN_MM 40
/* A long open-loop forward move drifts a little: allowed end-position error (mm). */
#if B5_SIMPLE_BYPASS
#define B5_ADV_TOL_X_MM 60
#define B5_ADV_TOL_Y_MM 80
#else
#define B5_ADV_TOL_X_MM 5
#define B5_ADV_TOL_Y_MM 20
#endif
/* Sideways sweep back to the line: how far past the expected line position to keep looking (mm). */
#define B5_LINE_SWEEP_EXTRA_MM 100
/* Wheel speed (cm/s) for the straight legs of the bypass. */
#ifndef B5_BYPASS_SPEED_CM_S
#define B5_BYPASS_SPEED_CM_S 8.0f
#endif
/* Slow pivot that finds the line after the return turn: speed (cm/s) and timeout (ms). */
#define B5_LINE_SEARCH_SPEED_CM_S 4.0f
#define B5_LINE_SEARCH_MS 4000u

#endif

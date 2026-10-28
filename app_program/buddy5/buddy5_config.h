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
#define B5_STOP_FRONT_MM 200
#define B5_MAX_PROFILE_MM 2500
#define B5_MAX_OFFSET_MM 700
#define B5_STEP_MM 20
#define B5_MAX_EDGE_BRACKET_MM 100
#define B5_SIDE_MATCH_MM 60
#define B5_CLEAR_JUMP_MM 100
#define B5_EDGE_CONFIRMATIONS 3
#define B5_FEEDBACK_MAX_AGE_MS 250u
#define B5_REQUEST_TIMEOUT_MS 15000u
#define B5_SCAN_TIMEOUT_MS 30000u
#define B5_AVOID_TIMEOUT_MS 180000u
/* STATIONARY demonstration; motor power off. No motor commands issued.
 * Set 0 only with registered, tested Vehicle Controller callbacks. */
#ifndef B5_BENCH_MODE
#define B5_BENCH_MODE 1
#endif
/* Logical 0=forward, positive=left. CALIBRATE -60/0/+60 on YOUR SG90.
 * Pulses below are a commissioning window, not a guaranteed angle mapping.
 * Geometry remains provisional until angles and bracket clearance are checked. */
#define B5_SERVO_PIN 12u
#define B5_SERVO_RIGHT_US 1000u
#define B5_SERVO_CENTRE_US 1500u
#define B5_SERVO_LEFT_US 2000u
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
#endif
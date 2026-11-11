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
#define B5_STOP_FRONT_MM 350   /* stop this far (mm) from the box: the +25/-35 degree scan then covers ~160 mm left and ~245 mm right of the line */
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
/* Sideways leg after the first turn: the measured box width + B5_SIDE_EXTRA_MM, but never less than
 * B5_MIN_BYPASS_OFFSET_MM and never so little that the pivot beside the box would clip it (B5_PIVOT_CLEAR_MM).
 * A 200 mm box on the line gives about 300 mm. */
#ifndef B5_MIN_BYPASS_OFFSET_MM
#define B5_MIN_BYPASS_OFFSET_MM 220
#endif
#ifndef B5_SIDE_EXTRA_MM
#define B5_SIDE_EXTRA_MM 60
#endif
/* Measuring the real box width: if the box fills the +/-35 degree scan window the edges cannot be seen, so the car
 * reverses this far (mm) and scans again, up to B5_MAX_RESCANS times. Needs free space behind the car. */
#ifndef B5_RESCAN_BACKUP_MM
#define B5_RESCAN_BACKUP_MM 150
#endif
/* Reverse and scan again ONLY when the scan cannot tell which side is better (the obstacle fills the whole window,
 * e.g. two boxes side by side). Each reverse widens what the +25/-35 degree window can see. */
#ifndef B5_MAX_RESCANS
#define B5_MAX_RESCANS 0   /* 0 = never reverse after detecting the obstacle; the front check picks the other side if needed */
#endif
/* The straight leg past the box is B5_PASS_WIDTH_FACTOR x the measured box width (B5_UNKNOWN_WIDTH_MM if it could
 * not be measured), counted from where the car first stopped, plus B5_PASS_SAFETY_MM. A deeper box is caught by the
 * sonar check after turning toward the line (see B5_MAX_EXTENDS), which extends the leg. */
#define B5_PASS_WIDTH_FACTOR 2
/* Straight leg beside the box, one move: the axle ends B5_PASS_BESIDE_MM + B5_PASS_TAIL_MM past the box FRONT face,
 * then the car turns toward the line once. The sonar check there catches a deeper box (reverse, extend, try again). */
#ifndef B5_PASS_BESIDE_MM
#define B5_PASS_BESIDE_MM 100   /* a few cm past the box front, then face the line and check */
#endif
#define B5_AXLE_TO_SENSOR_MM 80
/* Sonar check after turning toward the line: wait this long (ms) for post-turn pings, then the box counts as
 * still beside the car only if it is at least this far (mm) short of the line. */
#define B5_CHECK_SETTLE_MS 500u
/* While sweeping back across the line: black longer than this (mm) is not the tape edge-to-edge. */
#define B5_TAPE_MAX_MM 40.0f
/* ...and black shorter than this (mm) is not the line either (the edge of the white paper, a mark on the floor):
 * the car keeps driving until it crosses the real tape. */
#ifndef B5_TAPE_MIN_MM
#define B5_TAPE_MIN_MM 4.0f
#endif
/* Lateral room (mm) from the box edge to the axle so a 90 degree pivot beside the box cannot clip it:
 * corner radius sqrt(177^2+85^2) = 196 plus a margin. */
#define B5_PIVOT_CLEAR_MM 160   /* TUNE: raise if the car's corner touches the box when it turns toward the line */
#define B5_CHECK_MARGIN_MM 30
/* The sonar reports anything nearer than its 90 mm dead zone as "no echo", i.e. as free space. After turning to
 * face the line the box can be that close, so the car first reverses this far (mm, away from the line) and only
 * then judges the readings. */
#ifndef B5_CHECK_BACK_MM
#define B5_CHECK_BACK_MM 120
#endif
/* While driving back to the line: a forward echo nearer than this (mm) and short of the line stops the car;
 * it reverses, faces forward, drives B5_EXTEND_MM further and tries again. */
#ifndef B5_SWEEP_STOP_MM
#define B5_SWEEP_STOP_MM 180
#endif
/* Side check by turning the car: the sonar beam is so wide that from the stop point a box usually fills the whole
 * +25/-35 degree servo scan and neither edge shows. Unless both edges were measured, the car pivots left to
 * B5_PROBE_DEG and pings, then B5_PROBE_STEP_DEG further (up to B5_PROBE_LOOKS looks) until the box is out of view,
 * then does the same to the right and turns back. The side where the box drops out of view sooner has the nearer
 * edge: the car passes on that side. 0 = off. */
#ifndef B5_PROBE_DEG
#define B5_PROBE_DEG 30
#endif
#define B5_PROBE_STEP_DEG 15
#define B5_PROBE_LOOKS 4u
#define B5_PROBE_SETTLE_MS 600u    /* wait after the pivot so every ping of the batch is taken standing still */
#define B5_PROBE_TIMEOUT_MS 3000u  /* no usable reading in this time: that look counts as unknown */
/* Extra sideways distance (mm) on a side whose box edge the scan could not see (its position is only a low guess). */
#ifndef B5_UNSEEN_EDGE_EXTRA_MM
#define B5_UNSEEN_EDGE_EXTRA_MM 80
#endif
/* 1 = during the return the sonar looks back toward the box (servo at its limit on that side). Echoes up to
 * B5_CHECK_BEYOND_MM past the line still count as the box (it straddles the line). */
#ifndef B5_RECOVERY_LOOK_BACK
#define B5_RECOVERY_LOOK_BACK 1
#endif
#define B5_CHECK_BEYOND_MM 100
#define B5_LOOK_PERIOD_MS 800u   /* servo dwell per look direction during the return */
/* 1 = the forward-range self-halt ("emergency stop") is overridden during ADVANCE / RECOVER_LINE / BACKUP. */
#ifndef B5_BYPASS_IGNORE_GUARD
#define B5_BYPASS_IGNORE_GUARD 1
#endif
/* Once the sonar sees nothing beside the car, drive this much further (mm) so the car's tail clears the box
 * rear when it pivots back onto the line. About the half car length (~177) plus a margin. */
#ifndef B5_PASS_TAIL_MM
#define B5_PASS_TAIL_MM 0
#endif
#define B5_UNKNOWN_WIDTH_MM 500
#define B5_MIN_PASS_MM 400
#ifndef B5_PASS_SAFETY_MM
#define B5_PASS_SAFETY_MM 120
#endif
/* After the pass, turning toward the line: if the sonar still sees something closer than the distance to the
 * line, the car is still beside the box. It reverses a little, faces forward, drives a little further and
 * tries again (up to B5_MAX_EXTENDS times). */
#define B5_RETURN_REVERSE_MM 60
#define B5_EXTEND_MM 200
#define B5_MAX_EXTENDS 10
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
#define B5_LINE_SWEEP_EXTRA_MM 250   /* drive up to this far (mm) past where the line should be; the IR sensors stop the car on the line */
/* Wheel speed (cm/s) for the straight legs of the bypass. */
#ifndef B5_BYPASS_SPEED_CM_S
#define B5_BYPASS_SPEED_CM_S 8.0f
#endif
/* Slow pivot that finds the line after the return turn: speed (cm/s) and timeout (ms). */
#define B5_LINE_SEARCH_SPEED_CM_S 4.0f
#define B5_LINE_SEARCH_MS 4000u

/* Side choice: two box edges must differ by at least this (mm) before the nearer one decides; otherwise the side with
 * more free space beyond the box decides. */
#ifndef B5_SIDE_PREFER_MM
#define B5_SIDE_PREFER_MM 40
#endif
/* A turn that ends more than the deadband short of its target is repeated up to this many times. */
#ifndef B5_TURN_RETRIES
#define B5_TURN_RETRIES 3
#endif
/* Bypass: if the sonar still sees something in front for this long (ms) before a sideways move, or the guard trips
 * during it, the car did not turn enough. It stops and turns a little more (B5_BLOCK_EXTRA_TURN_DEG), up to
 * B5_BLOCK_MAX_FIXES times, then gives up (and the process restarts). */
#ifndef B5_BLOCK_WAIT_MS
#define B5_BLOCK_WAIT_MS 800u
#endif
#ifndef B5_BLOCK_EXTRA_TURN_DEG
#define B5_BLOCK_EXTRA_TURN_DEG 8.0f
#endif
#ifndef B5_BLOCK_MAX_FIXES
#define B5_BLOCK_MAX_FIXES 4
#endif
/* Before the straight pass beside the box, the sonar (now facing forward from the side position) must see clear space
 * for the whole pass plus this margin (mm). Otherwise something else (a second box) is on that side: take the other
 * side, or stop if both are blocked. */
#ifndef B5_ROUTE_MARGIN_MM
#define B5_ROUTE_MARGIN_MM 80
#endif
#ifndef B5_ROUTE_CHECK_MS
#define B5_ROUTE_CHECK_MS 500u   /* wait for fresh forward readings after the turn */
#endif
/* During the straight pass: anything closer than this ahead stops the car. */
#ifndef B5_ADVANCE_STOP_MM
#define B5_ADVANCE_STOP_MM 170
/* Front check before the straight pass: only an echo no more than this many mm beyond the box-front distance
 * counts as "in the path". A longer one is the corner of the box being passed (the sonar beam is wide). */
#define B5_ROUTE_CORNER_MARGIN_MM 30
#endif
/* Telemetry src5 type 59: left/right box edge, why that side was chosen, turn and sonar corrections. */
#define B5_TELEM_DECISION 59u

#endif

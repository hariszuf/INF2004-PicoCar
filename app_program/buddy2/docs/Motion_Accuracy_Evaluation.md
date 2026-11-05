# Motion Accuracy Evaluation — Motion Control (Buddy 2)

Team 08, INF2004. Author: Afiq.
Status: **mostly not tested (5 Nov 2026).** Only the encoder calibration check and the 20 cm/s speed-consistency data exist. The distance, turn, drift, battery, emergency-stop and stall tests were **not run**, so no distance or angle accuracy figures are claimed. Section 6 lists what is measured and what is not.

## 1. Purpose

Show, with repeated measurements, how accurately the car moves a commanded distance, turns a commanded angle, keeps a straight line, and holds speed. This supports requirement FR2 and the Motion Control performance objectives (consistent speed, accurate distance, repeatable turns, stable motion).

## 2. Proposed requirements (the team must confirm)

The Week 6 document has no numeric motion targets. These are proposals to add as non-functional requirements.

| ID | Proposed requirement |
|---|---|
| MC1 | Distance error within ±2 % or ±1 cm (whichever is larger) over 30 to 200 cm |
| MC2 | Turn angle error within ±3° for 90° and ±5° for 180° |
| MC3 | Sideways drift under 2 cm over 100 cm |
| MC4 | Speed ripple under 1.5 cm/s at 20 cm/s |
| MC5 | Emergency stop: motors braked within 20 ms of the call (software), wheels stopped within 300 ms |

## 3. Calibration constants used

| Item | Value | Source |
|---|---|---|
| Wheel diameter | 6.5 cm | team measurement |
| Wheel gap | 12.2 cm | team measurement |
| Counts per turn, left / right | 1265.8 / 1268.5 | 10 hand turns each (12658, 12685), counted on both edges. Firmware set to `MOTION_ENCODER_EDGE_MODE` 3 on 5 Nov 2026 after a hand-turn check (5 turns gave 3162 / 3178 counts in mode 1, so mode 1 counted half). |
| Turn scale (`MOTION_TURN_SCALE`) | 1.00 until calibrated | see section 5.2 |
| Stop lead (`MOTION_STOP_LEAD_S`) | 0.045 s | brake test, 3 runs (PID report section 6) |

Formulas used by the firmware:

- Distance per count = π × 6.5 / counts per turn (about 0.0161 cm).
- Heading (degrees) = (right distance − left distance) / 12.2 × 180 / π.
- Spin turn of angle θ: each wheel moves π × 12.2 × θ / 360 cm (90° → 9.58 cm, 180° → 19.16 cm).

## 4. Measurement method

- Mark a start line at the wheel contact points. Mark the target distance with tape.
- Distance: tape measure from the start mark to the same point on the wheels after the move. Record to 1 mm.
- Angle: draw the start heading on paper (a long straight edge along the car side), measure the final heading with a protractor or by the angle between the two lines. Alternatively use a floor marking at 90° and 180°.
- Drift: distance from the straight reference line to the car centre at 100 cm.
- Run each test 5 times from the same start, same surface, battery level recorded.
- Log the firmware's own reported distance and heading (`p` command or CSV) next to each measurement, so odometry error can be compared to the tape measure.

Per set of trials: mean error = mean(measured − target); repeatability = standard deviation of the measured values; worst case = largest absolute error.

Conditions for the data that exists (speed consistency, 5.4): tile floor, battery full to half, 20 cm/s, 5 Nov 2026.

## 5. Tests

**Status, 5 Nov 2026:** 5.1 (distance), 5.2 (turning), 5.3 (drift), 5.5 (battery), 5.6 (emergency stop) and 5.7 (stall) were **not run**; their tables are left blank on purpose. Only 5.4 has data, and only at 20 cm/s.

### 5.1 Straight distance

| Target (cm) | Trial | Measured (cm) | Error (cm) | Firmware distance (cm) |
|---|---|---|---|---|
| 30 | 1–5 | | | |
| 50 | 1–5 | | | |
| 100 | 1–5 | | | |
| 200 | 1–5 | | | |

Summary per target:

| Target | Mean error | Std dev | Worst case | MC1 met? |
|---|---|---|---|---|
| 30 cm | | | | |
| 50 cm | | | | |
| 100 cm | | | | |
| 200 cm | | | | |

Backward: repeat 50 cm backward, 5 trials.

| Trial | Measured (cm) | Error (cm) |
|---|---|---|
| 1–5 | | |

### 5.2 Turning

Calibrate `MOTION_TURN_SCALE` first with the 360° spin (`L`): mark the car heading, spin, and measure how much it over- or under-rotates. Set the scale to commanded ÷ actual rotation, then repeat.

| Angle | Trial | Measured (deg) | Error (deg) |
|---|---|---|---|
| 360 (calibration) | 1–3 | | |
| 90 left | 1–5 | | |
| 90 right | 1–5 | | |
| 180 (U-turn) | 1–5 | | |

Summary:

| Turn | Mean error | Std dev | Worst case | MC2 met? |
|---|---|---|---|---|
| 90 left | | | | |
| 90 right | | | | |
| 180 | | | | |

Final `MOTION_TURN_SCALE`: ____

### 5.3 Straight-line drift

| Trial | Drift at 100 cm (cm) | Heading error (deg) |
|---|---|---|
| 1–5 | | |

MC3 met? ____

### 5.4 Speed consistency

Run the step test (`t`) at 10, 20 and 30 cm/s. Using the log, take the last second of steady speed.

Measured: three step tests at 20 cm/s (the bench step test is fixed at 20 cm/s). Values are the average of the three runs over the last second (2.0 to 3.0 s) with the old feed-forward 0.025; individual runs: mean left 19.97 to 20.00, mean right 19.85 to 20.08, std dev left 0.19 to 0.26, std dev right 0.23 to 0.29. Raw logs were not archived. See the PID report for the full step response.

| Set-point (cm/s) | Mean left | Mean right | Std dev left | Std dev right | MC4 met? |
|---|---|---|---|---|---|
| 10 | not tested | | | | |
| 20 | 19.99 | 19.96 | 0.22 | 0.25 | Yes (under 1.5 cm/s) |
| 30 | not tested | | | | |

### 5.5 Battery dependence

Repeat the 100 cm move 3 times at full and at low battery.

| Battery | Mean error (cm) | Std dev (cm) |
|---|---|---|
| Full | | |
| Low | | |

### 5.6 Emergency stop

While moving at 20 cm/s, trigger `e`. Use the log to find the time from the command to zero encoder counts.

| Trial | Time to zero speed (ms) | Distance after command (cm) |
|---|---|---|
| 1–3 | | |

Also check: movement commands are rejected after the stop, and the car only moves again after `x` (reset).

### 5.7 Stall and fault check

Hold one wheel briefly while driving (with the car on a block). The car should enter ERROR (stall) after about 0.5 s and stop both motors. Record: ____

## 6. Results summary

| Requirement | Result | Met? |
|---|---|---|
| MC1 distance | not tested | Not tested |
| MC2 turn angle | not tested | Not tested |
| MC3 drift | not tested (the firmware heading changed by +0.53°, -2.04° and -0.53° in three 20 cm/s step runs without straight-line correction; this is its own estimate, not a measurement) | Not tested |
| MC4 speed ripple | std dev 0.19 to 0.29 cm/s at 20 cm/s | Yes, at 20 cm/s only (10 and 30 cm/s not tested) |
| MC5 emergency stop | not tested (the normal `motion_stop` brake test stops with about 2 cm of coast and a 0.045 s lead, but this is not the emergency-stop test) | Not tested |

## 7. Limitations and discussion

- **Calibration problem found.** The encoder edge mode did not match the calibration constants, so the firmware counted half of the real distance (a 100 cm command would have driven about 200 cm). It was found with a hand-turn check (5 turns gave 3162 and 3178 counts, about 634 per turn instead of about 1266) and fixed by setting `MOTION_ENCODER_EDGE_MODE` to 3. After the fix the same check was reported to be in the expected range (about 6330 and 6340 after 5 turns; the exact counts were not recorded). No tape-measure distance check has been done yet, so the calibration is confirmed by counts per turn only, not by a measured travelled distance.
- **Not tested:** distance (30 to 200 cm, forward and backward), turn angles (360° calibration, 90°, 180°), straight-line drift, battery dependence, emergency stop, and stall. `MOTION_TURN_SCALE` is still 1.00 (uncalibrated).
- **Speed consistency:** only 20 cm/s was tested, with one battery state and one surface (tile).
- **One encoder phase per wheel:** no direction sensing. Direction is the last driven direction, and the firmware brakes before reversing. No case of a wrong count was observed in the tests that were run.
- **Wheel slip during spin turns:** not measured; this is why `MOTION_TURN_SCALE` is needed.
- **Measurement error** of the tape measure and protractor does not apply yet because those tests were not run.
- **What would improve results:** a second encoder phase, and IMU yaw to cross-check turns.
- **Next step to complete this report:** run the 100 cm straight test (5 trials) and the 90° turn tests, and compare the tape measure with the firmware distance and heading.

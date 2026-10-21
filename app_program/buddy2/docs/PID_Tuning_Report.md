# PID Tuning Report — Motion Control (Buddy 2)

Team 08, INF2004. Author: Afiq.
Status: **procedure ready, results to be filled from the car.** Nothing in the results tables has been measured yet.

## 1. Purpose

Tune the wheel speed controller in `motion_control.c` so each wheel holds a commanded speed with little overshoot and no sustained oscillation, across the battery range the car will see in the demo.

## 2. Controller

Per wheel, every 20 ms:

```
duty = FF * setpoint + PID(setpoint, measured_speed)
duty = at least MOTION_MIN_DUTY in magnitude (stiction), limited to -1..1
```

- Measured speed comes from encoder counts, low-pass filtered (`MOTION_SPEED_FILTER_ALPHA`).
- PID has a clamped integrator and derivative on the measurement.
- Set-points ramp with limited acceleration and deceleration.
- Straight-line correction trims the two wheel set-points from the left/right distance mismatch.

Starting values (all placeholders until this report is filled):

| Parameter | Config name | Start value | Final value |
|---|---|---|---|
| Feed-forward (duty per cm/s) | `MOTION_FEEDFORWARD_DUTY_PER_CM_S` | 0.025 | |
| Kp | `MOTION_PID_KP` | 0.020 | |
| Ki | `MOTION_PID_KI` | 0.100 | |
| Kd | `MOTION_PID_KD` | 0.000 | |
| Integral limit | `MOTION_PID_INTEGRAL_LIMIT` | 0.30 | |
| Minimum duty | `MOTION_MIN_DUTY` | 0.15 | |
| Straight-line Kp | `MOTION_STRAIGHT_KP` | 1.5 | |
| Acceleration (cm/s²) | `MOTION_ACCEL_CM_S2` | 60 | |
| Deceleration (cm/s²) | `MOTION_DECEL_CM_S2` | 80 | |
| Stop lead (s) | `MOTION_STOP_LEAD_S` | 0.03 | |

## 3. Measured constants (team measurements)

| Item | Value |
|---|---|
| Wheel diameter | 6.5 cm |
| Wheel gap (tyre centre to tyre centre) | 12.2 cm |
| Counts per wheel turn, left | 12658 / 10 = 1265.8 |
| Counts per wheel turn, right | 12685 / 10 = 1268.5 |
| Distance per count | about 0.0161 cm |
| Encoder edge counted during that test | (fill in; must match `MOTION_ENCODER_EDGE_MODE`) |

## 4. Test set-up

- Battery or power bank: ____ , state of charge at start: ____
- Surface: ____
- Firmware: `motion_bench` (see README), USB serial log saved as CSV.
- Wheels off the ground for open-loop tests (`d`, `k`).

## 5. Step 1 — plant identification (open loop)

Run `d` (duty sweep). Record steady speed at each duty.

| Duty | Left (cm/s) | Right (cm/s) |
|---|---|---|
| 0.10 | | |
| 0.20 | | |
| 0.30 | | |
| 0.40 | | |
| 0.50 | | |
| 0.60 | | |
| 0.70 | | |
| 0.80 | | |
| 0.90 | | |
| 1.00 | | |

From the table:

- Dead zone (lowest duty that moves both wheels): ____  → set `MOTION_MIN_DUTY` slightly above it.
- Maximum speed at full duty: ____ cm/s → set `MOTION_MAX_SPEED_CM_S` below it with margin.
- Slope of the linear part, duty per cm/s: ____ → set `MOTION_FEEDFORWARD_DUTY_PER_CM_S`.
- Left/right mismatch at the same duty: ____ % (this is what straight-line correction must absorb).

## 6. Step 2 — brake test

Run `k`. It prints the coast distance after braking and a suggested `MOTION_STOP_LEAD_S`.

| Trial | Speed before brake (cm/s) | Distance after brake (cm) | Lead (s) |
|---|---|---|---|
| 1 | | | |
| 2 | | | |
| 3 | | | |

Chosen `MOTION_STOP_LEAD_S`: ____

## 7. Step 3 — closed-loop tuning

Method (manual, one gain at a time):

1. Set Ki = 0 and Kd = 0. Set the feed-forward from step 1.
2. Run `t` (step to 20 cm/s for 3 s, then stop). Log the CSV.
3. Raise Kp until the response is fast but starts to oscillate, then back off by about half.
4. Add Ki until the steady-state error is gone within about 1 s, without overshoot above the limit below.
5. Leave Kd at 0 unless noise is not a problem and overshoot remains (speed quantisation makes derivative noisy).
6. Repeat at 10, 20 and 30 cm/s, and at low and high battery.

Tuning log:

| Run | FF | Kp | Ki | Kd | Rise time (s) | Overshoot (%) | Settling time (s) | Steady-state error (cm/s) | Notes |
|---|---|---|---|---|---|---|---|---|---|
| 1 | | | | | | | | | |
| 2 | | | | | | | | | |
| 3 | | | | | | | | | |
| 4 | | | | | | | | | |
| 5 | | | | | | | | | |

Definitions: rise time = 10 % to 90 % of the set-point; overshoot = (peak − set-point) / set-point; settling time = time to stay within ±5 % of the set-point; steady-state error = mean(set-point − speed) over the last second.

Include one plot of speed against time for the final gains at 20 cm/s (both wheels, with the set-point).

## 8. Step 4 — straight-line correction

1. Drive `F` (100 cm) on a marked straight line, 5 times.
2. Record the sideways drift at the end and the heading error.
3. Raise `MOTION_STRAIGHT_KP` until drift is small without the car weaving.

| Trial | Sideways drift at 100 cm (cm) | Heading error (deg) |
|---|---|---|
| 1 | | |
| 2 | | |
| 3 | | |
| 4 | | |
| 5 | | |

## 9. Acceptance targets (proposed — the team must confirm these)

| Metric | Target |
|---|---|
| Overshoot at 20 cm/s | under 10 % |
| Settling time at 20 cm/s | under 0.8 s |
| Steady-state error at 20 cm/s | under 1 cm/s |
| Speed ripple (standard deviation, steady state) | under 1.5 cm/s |
| Sideways drift over 100 cm | under 2 cm |
| Performance at low and high battery | all of the above met |

## 10. Final gains and results

| Metric | Result | Target met? |
|---|---|---|
| Overshoot | | |
| Settling time | | |
| Steady-state error | | |
| Speed ripple | | |
| Drift over 100 cm | | |

Final values are copied into `motion_config.h`.

## 11. Conclusions and limitations

(Fill in after the tests: what worked, what was hard, what would change with a quadrature (two-phase) encoder, effect of battery level.)

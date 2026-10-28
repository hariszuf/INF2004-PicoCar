# Buddy 3 — Line Following (on the Buddy 2 library)

Team 08, INF2004 PicoCar. Based on the team's `ir_motor_curve_test.c`. Its own motors, encoder interrupts and PID were removed: **all wheel control now goes through Buddy 2's `motion_drive()`**, so Buddy 3 depends on the Buddy 2 library and cannot clash with its encoder interrupts.

## Run it (autonomous)

The project `CMakeLists.txt` builds `test/line_follow_demo_main.c` together with the Buddy 2 sources into `EmbeddedProject.uf2`. Flash, switch on, and place the car on the line within 5 seconds with both line sensors on the 18 mm black line. It drives by itself. USB serial output is optional.

Slopes and humps: the program also polls Buddy 4's IMU and passes the result to Buddy 2, which slows the wheels / adds power on the ramp. Buddy 3's logic and speeds are unchanged; the scaling happens inside Buddy 2.

## Logic (same as the team code)

| Sensors on the line | Action |
|---|---|
| both | straight at `LINE_STRAIGHT_SPEED_CM_S` |
| only one | steer toward the side that still sees the line: inside wheel `LINE_TURN_INNER_SPEED_CM_S`, outside wheel `LINE_TURN_OUTER_SPEED_CM_S` |
| (optional, off by default) only one, for more than `LINE_ESCALATE_AFTER_MS` | the curve is too tight for the normal turn: the inside wheel reverses slowly to pivot harder (`LINE_ESCALATE_*`) |
| none | **stop** (wheels only move while a black line is detected). Resumes when a sensor sees the line again. Optional: `LINE_RECOVER_MS` > 0 keeps turning toward the last side for that long first |

Added to the team code: the short recovery turn (set `LINE_RECOVER_MS` to 0 for the original "stop at once"), and speeds in cm/s (about 2.5 cm/s, slow) instead of encoder counts/s.

The line sensors are checked every 5 ms (`LINE_UPDATE_PERIOD_MS`) for quicker steering; Motion Control keeps its 20 ms speed loop.

Why it can now go this slow: Buddy 2's default minimum motor duty (15 %) kept a wheel far above a few cm/s. The project `CMakeLists.txt` lowers it for this program (`MOTION_MIN_DUTY=0.05f`, `MOTION_FEEDFORWARD_DUTY_PER_CM_S=0.0125f`, near the 6 % the team's own test used), and Buddy 2's speed PID raises the duty until the wheel reaches the requested speed. `LINE_LEFT_WHEEL_SCALE` / `LINE_RIGHT_WHEEL_SCALE` even out the two motors.

## Settings to check (`line_config.h`)

- `LINE_GP0_IS_CAR_LEFT`: 0 means GP0 is on the car's right when facing forward. If the car steers away from the line, flip it.
- `LINE_BLACK_LEVEL`: 1 = black (confirmed by the team).
- Speeds: `LINE_STRAIGHT_SPEED_CM_S`, `LINE_TURN_OUTER_SPEED_CM_S`, `LINE_TURN_INNER_SPEED_CM_S`. Raise them slowly once it follows well.
- Pins are GP0 and GP2 for the line sensors and GP26 for the barcode (monitored only), as in the team code. These differ from the Week 6 document, which listed GP26 left, GP2 centre, GP0 right.

## Files

| File | Purpose |
|---|---|
| `line_config.h` | Pins, speeds, lost-line timing |
| `line_sensors.h/.c` | Reads GP0, GP2 and the barcode pin as left/right/barcode bits |
| `line_follow.h/.c` | Decision logic, calls `motion_drive()` |
| `test/line_follow_demo_main.c` | Autonomous demo program |
| `test/host/` | PC simulation on a track with 10 cm radius curves |

## Host simulation

```
cd "Buddy 3"
sh test/host/run_tests.sh
```

Uses Buddy 2's simulated robot on a made-up track (straights, 10 cm U-turn, S-curve, 270° loop, bend, line end). The sensor spacing (12 mm) and forward offset (50 mm) are assumptions. It checks the logic only, not the real car.

## Verification status

- Logic passes the host simulation (follows the whole track, stops at the line end).
- Pico files were syntax-checked against stubs only; not built with the SDK or run on the car.
- Barcode decoding is not implemented. `line_sensors_read()` already reports the barcode pin.

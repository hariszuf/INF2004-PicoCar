# Buddy 2 — Motion Control Library

Team 08, INF2004 PicoCar. Owner: Afiq (Motion Control).
Follows the approved Week 6 design review: motors on GP8–GP11, wheel encoders on GP4 (right) and GP6 (left), one encoder phase per wheel, a Motion Control RTOS task with Idle / Executing Motion / Stopping / Error states, and an emergency-stop path from the Safety Monitor.

## Deliverables (from the project brief)

| Brief deliverable | Where |
|---|---|
| Motion control library | `motion_control.[ch]`, `motion_motor.[ch]`, `motion_encoder.[ch]`, `motion_pid.[ch]`, `motion_config.h` |
| PID tuning report | `docs/PID_Tuning_Report.md` (procedure ready, results to fill from the car) |
| Motion accuracy evaluation | `docs/Motion_Accuracy_Evaluation.md` (test plan ready, results to fill from the car) |
| Statechart for the design review | `docs/Motion_Statechart.md` |

## Files

| File | Purpose |
|---|---|
| `motion_config.h` | Pins, measured geometry, gains, limits. Values marked `TUNE` are placeholders. |
| `motion_control.h/.c` | Public API and state machine. This is the only header other tasks include. |
| `motion_pid.h/.c` | PID with anti-windup, derivative on measurement. No hardware access. |
| `motion_motor.h/.c` | PWM H-bridge driver for the Robo Pico, with an e-stop latch. |
| `motion_encoder.h/.c` | Single-phase encoder counting with GPIO interrupts. |
| `motion_task.c` | micro T-Kernel task skeleton (untested, needs the team template). |
| `test/motion_bench_main.c` | Serial bench firmware for collecting tuning data on the car. |
| `test/host/` | PC simulation tests for the control logic. |

## API (what the other buddies call)

```c
motion_init();                        // once at start-up
motion_update();                      // every MOTION_UPDATE_PERIOD_MS (20 ms) from the Motion Control task

motion_move_forward(distance_cm);     // non-blocking, straight line with correction
motion_move_backward(distance_cm);
motion_turn_left(angle_deg);          // spin in place, encoder based
motion_turn_right(angle_deg);
motion_drive(left_cm_s, right_cm_s);  // continuous, for line-following steering
motion_set_speed(cm_s);               // cruise speed for later moves
motion_stop();                        // controlled stop (brake)
motion_emergency_stop();              // immediate brake + latch, callable from any task
motion_reset();                       // leave ERROR
motion_reset_odometry();              // only while idle
motion_set_terrain(&report);          // latest slope / hump report from Buddy 4 (every IMU sample)
motion_is_idle();
motion_get_status(&status);           // state, fault, counts, speeds, distance, heading
```

Commands only queue; they are applied at the next `motion_update()`. A new command replaces the current one (as in the Week 6 text). Commands are rejected with `MOTION_ERR_FAULT` while in ERROR.

Telemetry for Buddy 1 comes from `motion_get_status()`: encoder counts, wheel speeds, distance travelled, heading, state and fault.

### Notes for the other buddies

- **Buddy 3 (line / barcode):** use `motion_drive(left, right)` for steering, or send moves and turns through the Vehicle Controller. Barcode decoding is timing based, so hold a constant speed over the barcode (`motion_set_speed()` or a fixed `motion_drive`). Agree that speed with Motion Control.
- **Buddy 5 (obstacle):** `motion_move_backward()`, `motion_turn_left/right()` and `motion_drive()` cover reverse, turn and bypass arcs.
- **Safety Monitor:** call `motion_emergency_stop()`. It brakes the motors in the caller's context and latches them, so no movement command can restart them until `motion_reset()`.
- **Vehicle Controller:** poll `motion_is_idle()` to know when a move or turn has finished.

### Slope and hump speed control (Buddy 4 -> Buddy 2)

Buddy 4 only observes the terrain. After each IMU sample the main loop passes its result to Buddy 2 with `motion_set_terrain()` (glue: `shared/terrain_link.h`, which is the only file that knows both libraries). Buddy 2 then decides speed and power on its own:

| IMU report (Buddy 4 hysteresis) | Speed target (moves and `motion_drive`) | Power assist |
|---|---|---|
| FLAT | unchanged | none |
| UPHILL (pitch >= 6 deg) | x `MOTION_UPHILL_SPEED_SCALE` (1.0) | + `MOTION_GRADE_DUTY_PER_G` x sin(pitch) on both wheels |
| DOWNHILL | x `MOTION_DOWNHILL_SPEED_SCALE` (0.7) | - same gravity term (less power, brakes rather than reverses) |
| LEFT/RIGHT_WHEEL_UP (one-side hump/ramp, roll >= 4 deg) | x `MOTION_HUMP_SPEED_SCALE` (0.8) | + `MOTION_HUMP_DUTY_PER_G` x sin(roll) on the raised wheel only |

- Buddy 3 needs no change: its `motion_drive()` speeds are scaled inside Buddy 2, so line following slows down on the ramp by itself. Spins in place are not slowed.
- Speed changes go through the normal accel / decel ramp, and the assist is rate limited (`MOTION_TERRAIN_DUTY_RATE_PER_S`) and capped (`MOTION_TERRAIN_DUTY_MAX`), so there is no jerk.
- A wheel is never driven against its requested direction: if the downhill term would do that, it brakes instead (protects the single-phase encoder count).
- No report for `MOTION_TERRAIN_STALE_MS` (200 ms), or `valid = false` (IMU missing / I2C error) = flat ground, i.e. normal behaviour. Stall detection still applies.
- `motion_get_status()` reports `terrain`, `terrain_speed_scale` and the per-wheel assist for telemetry.
- All terrain values are `TUNE` placeholders: check them on the real ramp at line-following speed.

## Design decisions

- **One encoder phase per wheel.** The count sign is the last driven direction. To keep that valid the library never reverses a motor while the wheel is still turning: it brakes first, then reverses. Drag or being pushed while stopped is not detected.
- **Brake, not coast, to stop.** Both inputs low (Robo Pico truth table). PWM is applied to input A (or B for backward) with the other held low, so the PWM off-time is also a brake phase.
- **Speed control is feed-forward plus PID per wheel**, with a trapezoidal profile (ramp up, then slow down as `sqrt(2 * decel * remaining)`), straight-line correction from the left/right distance mismatch, and a tunable stop-lead (`MOTION_STOP_LEAD_S`) because braking takes a moment.
- **Faults:** stall (wheel pushed as hard as allowed - duty at `MOTION_STALL_DUTY` or the PID integral saturated - but not turning for `MOTION_STALL_TIME_MS`) and emergency stop both go to ERROR, brake and latch the motors. Repeated `motion_drive()` calls only update the wheel speeds, so stall detection keeps working during line following.
- **No RTOS dependency in the core.** The library is called by a task; it does not create tasks or use kernel calls.
- **Floating point** is used (the RP2040 has no FPU). At 50 Hz this is fine. Check CPU load at integration.

## Things you must confirm on the car

1. **Encoder edge mode.** `MOTION_ENCODER_EDGE_MODE` must match whatever you used when you counted 12658 / 12685 counts in 10 turns. If the counts were taken on both edges, set it to 3; otherwise every distance and angle is wrong.
2. **Motor direction.** If a wheel drives backward on "forward", set `MOTION_LEFT_MOTOR_INVERT` or `MOTION_RIGHT_MOTOR_INVERT` to 1.
3. **Left/right naming.** The design lists the left motor as M1 (GP8/GP9) and the right as M2 (GP10/GP11). Check against the wiring.
4. **Every `TUNE` value** in `motion_config.h`, using the bench tests.
5. **Shared GPIO interrupt.** The RP2040 allows one GPIO interrupt callback per core. By default `motion_encoder_init()` registers it. If another module (for example the ultrasonic echo) also needs GPIO interrupts, define `MOTION_EXTERNAL_IRQ_DISPATCHER` and call `motion_encoder_irq_handler()` from the shared dispatcher.

## Build

This folder does not change the project's `CMakeLists.txt`. Add this to it when ready (the folder name has a space, so keep the quotes):

```cmake
set(BUDDY2_DIR "${CMAKE_CURRENT_LIST_DIR}/Buddy 2")

add_library(motion_control STATIC
    "${BUDDY2_DIR}/motion_control.c"
    "${BUDDY2_DIR}/motion_pid.c"
    "${BUDDY2_DIR}/motion_motor.c"
    "${BUDDY2_DIR}/motion_encoder.c")
target_include_directories(motion_control PUBLIC "${BUDDY2_DIR}")
target_link_libraries(motion_control PUBLIC
    pico_stdlib hardware_gpio hardware_pwm hardware_clocks hardware_sync)

# Stand-alone bench for tuning (own UF2, USB serial enabled)
add_executable(motion_bench "${BUDDY2_DIR}/test/motion_bench_main.c")
target_link_libraries(motion_bench motion_control)
pico_enable_stdio_usb(motion_bench 1)
pico_enable_stdio_uart(motion_bench 0)
pico_add_extra_outputs(motion_bench)
```

For the full robot, link `motion_control` into the main executable and create the Motion Control task (see `motion_task.c`).

## Bench commands (`motion_bench`)

Open a serial terminal at 115200 baud. `h` prints the list: forward/backward, 90 and 180 degree turns, 360 degree spin, stop, e-stop, reset, a 3 s step test with CSV output, an open-loop duty sweep, and a brake/coast test. The sweep and brake tests drive the motors directly, so put the car on a block first.

## Host simulation tests

```
cd "Buddy 2"
sh test/host/run_tests.sh
```

This compiles the control code on a PC against a simulated motor and encoder model and runs 42 checks, then the terrain tests (`test_terrain.c`, 30 checks: flat unchanged, uphill assist, downhill slow-down without reversing, left-wheel hump, stale / invalid IMU fallback, distance and turn accuracy with terrain active). The 42 motion checks are: move and turn accuracy, straight-line correction, reversal safety with the single-phase encoder, emergency stop latch, stall detection (also while line following re-sends `motion_drive()` every 5 ms), argument validation. The motion checks then run a second time with the slow line-following tuning. It checks logic only. The simulated motor is a made-up first-order model, so its accuracy numbers say nothing about the real car.

## Verification status

- Control logic: passes all 42 host simulation checks (default and line-following tuning) and 30 terrain checks.
- Built with Pico SDK 2.3.1 (EmbeddedProject and buddy234_driving_test), 28 Oct 2026. Terrain gains not yet tuned on the car.
- Pico-specific files (`motion_motor.c`, `motion_encoder.c`, the bench): syntax-checked against stub headers only. They have not been built with the real Pico SDK or run on hardware.
- `motion_task.c`: not built (no micro T-Kernel template available).
- Coding style follows Barr-C conventions by hand (braces everywhere, fixed-width types, `s_` for file statics, no magic numbers in logic). No Barr-C checker was run.
- Real PID gains, `MOTION_STOP_LEAD_S`, speed limits and accuracy figures are not yet measured.

# Correctness and performance

## Stage 1: controller flags and fault mask
- Host test `testing/system/test_robot_flags.c`: 18 checks pass (set, clear, update, test-and-clear, balanced locking).
- Firmware builds (make exit code 0). Flash +224 B, RAM unchanged.
- On-car check: passed on the final firmware (see Stage 4 section): STOP halts, START restarts, dashboard shows Running / Stopped.

## Stage 2: Buddy 5 structs
- Host test `testing/system/test_bitpack.c`: 9 checks pass (all widths 1..32, 13 scan slots independent, clamp).
- Behaviour unchanged: the host simulation (`testing/subsystem5/new_logic/run_tests.sh 2`) and the scan-only run
  (`run_scanonly.sh`) print byte-for-byte identical output before and after the change (diff is empty).
- On-car check: passed on the final firmware (see Stage 4 section): obstacle bypass works as before.

## Stage 3: binary MQTT
- C codec: `testing/system/test_telemetry_frame.c` passes (round trip, extreme values, malformed frames).
- Python codec and dashboard: `dashboard/test_app.py` 13 tests pass, using the exact bytes the C codec prints.
- Sizes: see `MESSAGE_SIZES.md` (telemetry payloads 1594 B to 207 B for 23 message types).
- On the car: START, STOP, barcode map and live telemetry all work with the new dashboard.

## Stage 4: Buddy 2, 3 and 4
Host tests are run before and after the change and their output is compared with `diff` (empty diff = identical).

| Module | Test | Result before | Result after |
|---|---|---|---|
| Buddy 2 motion control (shared flags in one locked word, enums in one byte) | `app_program/buddy2/test/host/run_tests.sh` | 42 checks, 0 failed | identical output |
| Buddy 3 barcode scanner (4 flags in one locked word) | `app_program/buddy3/test/host/run_barcode_tests.sh` | 233 + 50 checks, 0 failed | identical output |
| Buddy 4 IMU, terrain, lift | `app_program/buddy4/tests/test_imu.c`, `test_lift.c` | pass | identical output |

- The Buddy 4 test caught one mistake during the work: `terrain` can be -1, so it must stay a signed byte.
  It is `int8_t` in the final code.
- Timing: the only added work in the control path is a short interrupt-masked read-modify-write on a flag bit,
  the same DI/EI section the code already used for its command queue. Reading a flag is one load. The Buddy 2
  PID and terrain tests check timing-dependent behaviour (stall detection after 2140 ms, speed settling) and are
  unchanged.
- On-car check (2026-11-21, final firmware with Stages 1 to 4): line following, barcode read, slope / hump, obstacle bypass,
  START, STOP, barcode map and live dashboard telemetry all behaved normally, same as before the change.

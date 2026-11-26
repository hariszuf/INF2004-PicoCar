# Subsystem 5 — test log

**Subsystem:** Buddy 5: obstacle detection  ·  **Student ID:** _add student ID_  ·  **Code:** `app_program/buddy5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | Basic sensor response |  | manual | Move a flat object closer and farther away (car stationary) | Distance decreases/increases respectively |
| S5-U02 | State logic (host) |  | auto | `g++ -std=c++11 -Wall -Wextra -Werror testing/subsystem5/test_obstacle_state.c -o test_obstacle_state` then run it | All assertions pass, including exact boundaries and every sensor error code |
| S5-U03 | Obstacle entry |  | manual | Begin above 250 mm; move object to 200 mm or less | OBSTACLE at <= 200 mm |
| S5-U04 | Hysteresis |  | manual | From OBSTACLE move to 225 mm, then >= 250 mm; from CLEAR move to 225 mm, then <= 200 mm | Retain OBSTACLE at 225, clear at >= 250; retain CLEAR at 225, obstacle at <= 200 |
| S5-U05 | Fault and recovery |  | manual | Power off, disconnect sensor, power on; then reconnect and retest | Invalid readings report SENSOR FAULT; valid <= 200 OBSTACLE; valid >= 250 CLEAR; a valid reading between thresholds keeps the previous state |

*Method:* **auto** = `TEST_CHECK` or host program, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app...).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-15 | S5-U01 | not recorded | Vijay | PASS (direction only) | serial screenshot, not saved | accuracy against a ruler not measured |

## Notes

- Thresholds are provisional, not validated driving stop distances.
- Earlier IDs: `B5-U01`/`B5-U02` are now `S5-U01`/`S5-U02`; `B5-H01`..`B5-H03` are now `S5-U03`..`S5-U05`.
- **S5-U02 is currently broken:** `test_obstacle_state.c` includes `app_program/buddy5/obstacle_state.h`, which no longer exists (the logic moved into `buddy5_core.c`/`buddy5_plan.c`). The test must be rewritten before it can be run. Found while restructuring; not changed here.
- (Old note) `test_obstacle_state.c` was written against the state logic used by the firmware. `test_distance.c` tests a separate copy of the conversion formula, not the driver.
- Initial state is SENSOR FAULT until a valid reading crosses either threshold.
- Sampling keeps the 100 ms delay after each measurement. State reporting does not issue motor commands or detect a stalled sensor task.
- Do not unplug sensor wiring while powered. An unobstructed sensor may report a distant surface or a timeout.
- S5-U03 to S5-U05 need the new firmware flashed; no hardware PASS is claimed here.
- Host simulation: `testing/subsystem5/new_logic/run_tests.sh`, `run_scanonly.sh`.
- Save serial evidence under `logs/` and record the firmware commit with every result.

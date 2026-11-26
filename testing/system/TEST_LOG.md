# System — test log

**Subsystem:** whole car against the mission requirements  ·  **Student ID:** _add student ID_  ·  **Code:** `app_program/vehicle/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | TBD |  | TBD | TBD | TBD |

*Method:* **auto** = `TEST_CHECK` or host program, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app...).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | SYS-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

## Host test programs

`test_bitpack.c`, `test_robot_flags.c`, `test_telemetry_frame.c` and `measure_message_sizes.py` in this folder are host checks of the shared data format (see `docs/data-representation/`).

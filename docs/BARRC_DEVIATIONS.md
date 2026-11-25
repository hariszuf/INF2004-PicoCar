# BARR-C:2018 deviations

This file records where the code intentionally does not follow BARR-C:2018,
and why. Everything else in `app_program/`, `testing/` and `tools/` was
reformatted to the standard (Allman braces, 4-space indent, 80 columns,
fixed-width types, `float32_t`, `p_`/`b_`/`s_`/`g_` prefixes, constant on the
left of `==`, `else` at the end of every `else if` chain).

## Not covered

| Folder | Reason |
|---|---|
| `kernel/`, `device/`, `lib/`, `include/`, `config/` | vendor micro T-Kernel and Pico code |
| `app_program/buddy4/mtk_trial/overlay/` | trial copies of vendor files |
| `tools/elf2uf2/` | third-party tool |

## Deviations

| Rule | Deviation | Reason |
|---|---|---|
| 1.6 | Casts are not individually commented (about 760) | A useful comment needs a reason per cast. Generic comments would add noise. |
| 4.1 | `buddy4_*.c` file names share their first 8 characters | Renaming changes includes and build scripts that cannot be built without the Pico SDK. |
| 5.2 | Plain `char` is kept for text, `int main` and `__wrap___errno` keep `int` | Required by the C standard or the linker wrap. |
| 6.2a | Functions over 100 lines (table below) | Splitting them changes shared local state and cannot be tested without the car. |
| - | `setjmp`/`longjmp` in `app_program/buddy4/sdk_integration_test/test_main_telemetry.c` | Host test only, never built for the car. |

## Functions over 100 lines

| Function | File | Lines |
|---|---|---|
| `controller_task` | `app_program/vehicle/vehicle_controller.c` | 998 |
| `full_nav_update` | `app_program/vehicle/support/vehicle_nav.c` | 813 |
| `veh_step` | `testing/buddy5/new_logic/sim_b5.c` | 398 |
| `b5_core_update` | `app_program/buddy5/buddy5_core.c` | 329 |
| `main` | `app_program/buddy4/sdk_integration_test/main.c` | 312 |
| `b5_plan_make` | `app_program/buddy5/buddy5_plan.c` | 304 |
| `main` | `tools/buddy1_startup_tests/network_test.c` | 251 |
| `buddy4_poll` | `app_program/buddy4/buddy4_imu.c` | 215 |
| `line_follow_update` | `app_program/buddy3/line_follow.c` | 208 |
| `main` | `app_program/buddy4/tests/test_imu.c` | 205 |
| `junction_step` | `app_program/buddy3/line_follow.c` | 205 |
| `buddy4_obstruction_update` | `app_program/buddy4/buddy4_obstruction.c` | 204 |
| `scan_task` | `app_program/buddy5/buddy5_task.c` | 195 |
| `test_measurement_quality` | `app_program/buddy4/tests/test_imu.c` | 195 |
| `network_status_task` | `app_program/buddy1/network.c` | 173 |
| `imu_quality_reproduction` | `app_program/buddy4/sdk_integration_test/test_drive_control.c` | 164 |
| `run` | `testing/buddy5/new_logic/sim_b5.c` | 162 |
| `main` | `testing/buddy5/new_logic/sim_b5.c` | 162 |
| `brief_pitch_pause_safety` | `app_program/buddy4/sdk_integration_test/test_obstruction_sdk.c` | 156 |
| `process_once` | `app_program/buddy4/rtos/buddy4_task.c` | 156 |
| `timing_and_invalid` | `app_program/buddy4/sdk_integration_test/test_obstruction_sdk.c` | 144 |
| `buddy4_lift_update` | `app_program/buddy4/buddy4_lift.c` | 140 |
| `asymmetric_transition` | `app_program/buddy4/sdk_integration_test/test_obstruction.c` | 135 |
| `boundaries_and_history` | `app_program/buddy4/sdk_integration_test/test_impact.c` | 132 |
| `main` | `app_program/buddy3/test/host/test_junction.c` | 131 |
| `recorded_angles_and_cooperative_schedule` | `app_program/buddy4/sdk_integration_test/test_obstruction_sdk.c` | 129 |
| `buddy4_impact_update` | `app_program/buddy4/buddy4_impact.c` | 128 |
| `full_nav_accept` | `app_program/vehicle/support/vehicle_nav.c` | 126 |
| `acquisition_faults` | `app_program/buddy4/sdk_integration_test/test_impact.c` | 125 |
| `main` | `app_program/buddy3/test/host/test_barcode39.c` | 122 |
| `buddy1_mqtt_command_ingest` | `app_program/buddy1/command.c` | 122 |
| `recorded_ramp` | `app_program/buddy4/sdk_integration_test/test_obstruction_sdk.c` | 121 |
| `main` | `testing/system/test_telemetry_frame.c` | 109 |
| `test_error_recovery` | `app_program/buddy4/tests/test_imu.c` | 106 |
| `main` | `app_program/buddy3/test/host/test_barcode_scan.c` | 106 |
| `main` | `testing/system/test_bitpack.c` | 101 |

Split these only when a hardware run is available to confirm behaviour.

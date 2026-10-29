# Buddy 4 integration with the professor's micro T Kernel port

Checkpoint: 28 October 2026. The team application now registers Buddy 4 with the existing Buddy 1 and Buddy 5 tasks. The native team USB firmware compiles/links, and the new host tests pass. Hardware timing, bus operation and coexistence have not yet been verified. Nothing was flashed, operated, committed or pushed during this change.

## Base and scope

The professor's hub links to [the RP2040 port](https://github.com/sirfonzie/mtk3smp-rp2040). The reference downloaded for comparison was commit `2e8e6ad480c2d84a369b12995694b9bf5e9ddfb3`. The team's existing kernel tree already shares its headers/configuration and most implementation files. This integration uses that team-controlled tree; it does not replace it or change the professor's repository.

No replacement kernel, alternate polling I2C implementation or separate build profile is introduced. The application uses `tk_cre_tsk`, `tk_sta_tsk`, `tk_dly_tsk`, `tk_get_otm`, priority-inheritance mutexes, and the port's registered `iica` device via `tk_swri_dev` / `T_I2C_EXEC`.

Buddy 4 core algorithms and calibration are unchanged: roll offset 0.61 degrees, pitch offset 0.97 degrees, existing acceleration coefficients, sensor axes, 11.3 cm spacing and thresholds. Standalone CMake and integration driving firmware remain available. Tribunal 1 packing is deferred.

## Files

| File | Responsibility |
|---|---|
| `buddy4_task.c/.h` | Native IMU task, protected snapshots and encoder input mailbox |
| `buddy4_device.c/.h` | Core bus callbacks mapped to the registered kernel I2C device |
| `tests/test_mtk_task.c` | Real processing/bridge tests with kernel/device mocks |
| `tests/test_native_i2c.c` | Actual native driver state-machine tests with register/IRQ mocks |
| `tests/mtk_stubs/` | Host-only test definitions; not firmware sources |
| `tests/test_mtk.ps1` | New native integration test runner |
| `../app_main.c` | Adds Buddy 4 to the existing two-phase init/start and low-priority diagnostics |
| `../../build_make/mtkernel_3/app_program/subdir.mk` | Explicit native sources; excludes SDK mains and host tests |
| `../../build_make/pico_rp2040.mk` / `makefile` | Feature guard, profile stamp, math library and link settings |
| `../../device/i2c/sysdepend/rp2040/` | Board pin mapping, native timeout and tested STOP/restart corrections |

Buddy 1/2/3/5 source, kernel scheduler, common headers and kernel configuration remain unchanged. Buddy 2/3 do not yet have migrated task owners in this application, so native builds deliberately exclude their SDK-specific programs. The ordinary CMake driving application still includes them.

## Build

From the team repository root, using the existing GNU Make toolchain:

```powershell
$env:PATH = "$env:USERPROFILE/.pico-sdk/toolchain/15_2_Rel1/bin;C:/msys64/ucrt64/bin;C:/msys64/usr/bin;" + $env:PATH
Push-Location build_make
try {
    & make SMP=0 CONSOLE=usb_cdc "PICO_SDK_PATH=$env:USERPROFILE/.pico-sdk/sdk/2.3.1" -j4
    if ($LASTEXITCODE) { throw 'Team micro T Kernel build failed' }
} finally { Pop-Location }
```

Output: `build_make/mtk3pico_smp0_usb_cdc.uf2` with corresponding ELF/map. This is the team RTOS application, not `buddy234_driving_test.uf2` and not the untouched professor demo.

`BUDDY4_ENABLED=1` is the default; `BUDDY4_ENABLED=0` excludes its task/core from the native application. `BUDDY1_ONLY=1` guards both Buddy 4 and Buddy 5 startup. That optional compatibility build was not run because approval was declined; do not claim it was verified. Do not run concurrent profiles in the shared Make object tree.

The standard USB build initially completed using Make. Final changed sources were rebuilt using the same native Windows compiler/linker commands from that build log, then repackaged with elf2uf2; this avoided the sandbox's MSYS runtime restriction. Final build section sizes and warnings are compile/link evidence, not live RAM, stack or timing measurements.

`libm` is required by existing IMU processing. The native bare-metal link uses `nosys.specs` and emits four warnings that `_close`, `_lseek`, `_read`, `_write` are not implemented. No adapter code uses libc file I/O or heap allocation. Diagnostics use `tm_printf`. Team changes introducing newlib stdio, allocation or task-specific errno need separate review.

## Task and data interfaces

`buddy4_task_init()` creates a priority-inheritance data mutex and opens `iica` exclusively. It is called in the existing initialisation phase before `buddy4_task_start()`. These names avoid collision with the core's `buddy4_init(imu,bus,config)`.

The sensor owner targets absolute 20 ms releases, using kernel monotonic operating time in milliseconds converted to 64-bit microseconds. The kernel tick is 10 ms. Late releases skip slots rather than producing catch-up polls; actual timing needs measurement. Configuration is attempted at most once per second while not ready. Sensor errors invalidate output and abort interrupted events through the unchanged core.

Only the owner task touches mutable IMU/impact/motion/yaw state. `buddy4_task_get()` returns a mutex-protected copy, with IMU/impact validity aged after 40 ms and encoder validity at 100 ms. Numeric fields can remain diagnostic after invalidation; always check validity. Never call these APIs from an ISR.

The snapshot includes corrected/uncorrected acceleration, raw counts, filtered tilt, height, terrain/side, errors, latest completed event with explicit estimator validity, completed-event count, possible-impact count, and estimated motion/yaw. Sequence numbers and timestamps identify repeated reads. One-sample pulses can be read more than once or missed by slow clients: this is a latest-value API, not a reliable event queue. Completed-event summaries and counters are retained, but a future lossless delivery requirement needs team queue design.

An eventual native Buddy 2 task should submit one coherent signed encoder observation in cm/s using `buddy4_task_submit_speed()`. Obtain the timestamp with `buddy4_task_time_us()`; a bare SDK timestamp or millisecond count has a different representation/epoch. Duplicate samples do not advance confirmation, reversed timestamps invalidate evidence, and missing/stale input remains invalid. Until this producer is connected, native motion is UNKNOWN and yaw invalid. No zero-speed observation is fabricated.

Priority 10 and an 8192-byte sensor task stack are provisional. No mutex is held across device I/O, processing, logging or delays. No motor command, control decision or automatic stop depends on this task. No new ultrasonic/IMU collision fusion or MQTT payload is claimed: Buddy 5 continues its existing task and Buddy 1 continues its existing telemetry behaviour.

## Native I2C board adaptation and corrections

The professor-derived sample driver originally assigned I2C0 to GP8/9, used by the car motors. The board configuration now assigns **GP16 SDA / GP17 SCL**, with nominal 100 kHz standard mode at the BSP's 125 MHz peripheral clock. I2C1 fast-mode settings remain unchanged.

The IMU exclusively uses the native I2C0 owner. Register reads stage the core's register-pointer callback, then submit one combined `T_I2C_EXEC` transaction; the staged pointer alone does not prove an ACK. Native short transfers and failures propagate as core I/O errors. The application does not call the Pico SDK I2C runtime or register another I2C IRQ.

Targeted native driver corrections:

- Request STOP on the final byte of multi-byte reads.
- Wait for physical STOP completion before disabling the controller after writes/reads.
- Request repeated START on the first read command following a prefix write.
- Clear only observed interrupt causes, avoiding loss of a newly arriving STOP.
- Use a 10 ms I2C0 per-request timeout rather than 1000 ms. Kernel-tick rounding and scheduling mean this is not a hard wall-clock bound; an entire configuration includes multiple transactions.
- In the SMP=0 trial, notify completion after restoring the IRQ mask so the kernel can request PendSV; ignore pending NVIC IRQs with no active controller cause.

The host register/IRQ tests cover these paths, 1..6-byte reads with varied STOP delivery, dispatch, aborts, timeout races and recovery. B4-DIAG-E retains 32 timeout fields, including actual wait registration/deadline, expiry, flag release and task return. The trial adds observation hooks in its copy of kernel wait.c; scheduling and timeout behavior are unchanged. Tests using real kernel timer/wait code reproduce both completed-after-expiry and partial-transfer errors. The USB trial disables the polled UART mirror, which otherwise blocks IRQs during tm_printf; its early/panic UART mirror is consequently unavailable. The live console/kernel remain unchanged. Hardware validation is still required. See the trial's top-level README for evidence and the exact isolated UF2 path.

## Serial diagnostics and initial physical check

After all registered buddies start, the initial task lowers its priority to 14 and prints one read-only `B4` line per second. Its fields are:

```text
B4,sequence,valid,io_errors,pitch_mdeg,roll_mdeg,height_milli_cm,completed_events,stable_milli_cm,median_milli_cm,skipped_slots
```

`1000` in a height field means 1.000 cm; `1000` in an angle field means 1 degree. `-1` in a completed peak field means unconfirmed. These logs are separate from the unchanged standalone/integration 33/69-column CSV formats. Other Buddy 1/5 text may appear between lines.

This RTOS application currently has no migrated Buddy 2/3 motor controls or `M/L/R/G/S` driving-command handler. Do not assume the driving-test serial commands work here.

First hardware qualification should keep motor power off, confirm identification and valid samples, inspect errors/skips, verify settled flat/left/right response and check disconnect/recovery. Measure sampling cadence, stack use and effects on Buddy 5 echo timing before claiming coexistence. The current task priorities and synchronous driver behaviour require course-specific validation.

The previous course-board roll baseline and height bias remain unresolved. Do not tune calibration to hide that condition or infer general accuracy from earlier hard-surface checks.

## Verification checkpoint

- New native task/device bridge tests: passed, including configuration, short/error propagation, startup cleanup, recovery, clock failure, duplicate/reversed/stale encoders, retained event summaries and snapshot age checks.
- Native I2C register/IRQ regressions: passed.
- Existing IMU, median, impact, motion, yaw, control and scheduler suites: passed.
- Existing CSV schema checks: passed, 33/69 columns.
- Both CMake firmware builds and native USB team build: passed in the staging workspace.
- Optional Buddy-1-only compatibility profile: not run.
- Physical operation and timing: not tested by this change.

Final vehicle integration, collision discrimination, independently referenced turn-rate accuracy, Tribunal 1 comparisons and the report remain pending. The source integration is not completion of the assessed project.

### Readable trial diagnostics

The trial application defaults to readable status at 1 Hz, with decimal degrees,
cm, named terrain/side, saved failure traces, error and skipped-slot counts.
Invalid current measurements display N/A. Historical tilt records/counts are
hidden; their internal data and optional CSV fields are retained.
Set `BUDDY4_CONSOLE_CSV=1` in the proposed `app_main.c` and rebuild for the original
11-field numeric CSV; the default is 0. This setting affects display only.
The slow monitor may miss event pulses; it does not replace high-rate recording.

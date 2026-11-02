# Buddy 4 ownership, verification and team integration

Checkpoint: 2 November 2026. Ready for team integration work; not yet registered
or qualified in the live team MTK application. Runtime algorithms are unchanged
by this handover. Existing code, tests, recordings and prior checkpoints are retained.

## Start with the reusable IMU module

**`buddy4_imu.c` and `buddy4_imu.h` are the reusable IMU module.** They have no
Pico SDK or kernel dependency. Supply bounded bus callbacks, monotonic timestamps
and a persistent, exclusively owned `buddy4_imu_t`. Link the math library.

Call `buddy4_init(state, bus, buddy4_default_config())`, then
`buddy4_configure(state)`. Schedule `buddy4_poll(state, monotonic_us)` every
20 ms. It checks data-ready once and processes at most one sample. Reconfiguration
after an error is the application's responsibility. Preserve current calibration,
mounting signs/offsets, 11.3 cm geometry and validity semantics; do not auto-zero.

`output.valid` means acquisition success, `acceleration_valid` permits non-clipped
finite acceleration processing, and `tilt_valid` permits gravity-based tilt/height.
GRAVITY_LIKE is necessary, not proof of static orientation. Invalid terrain state
zero is not evidence of flat ground. Retained values and completed history need
their own validity/timestamps. Do not let multiple tasks mutate the same instance.

## Folder and ownership map

All paths below are relative to `app_program/`. Files stay at their existing paths
to preserve build scripts, include paths and evidence provenance.

| Path | Ownership and reuse |
| --- | --- |
| `buddy4/buddy4_imu.c/.h` | Buddy 4 portable acquisition, calibration, tilt/terrain, side events and height/peak processing; primary teammate entry point |
| `buddy4/buddy4_impact.c/.h` | Buddy 4 portable vector possible-impact observer; acceleration input, no motor commands |
| `buddy4/buddy4_motion.c/.h` | Buddy 4 portable signed-encoder motion classifier |
| `buddy4/buddy4_encoder_yaw.c/.h` | Buddy 4 portable encoder yaw estimate; no gyro |
| `buddy4/buddy4_obstruction.c/.h` | Buddy 4 portable possible-obstruction state machine; qualified command/speed context supplied by caller |
| `buddy4/main.c`, `CMakeLists.txt`, `pico_sdk_import.cmake` | Standalone Pico SDK IMU monitor/build, not part of the reusable core or MTK startup |
| `buddy4/tests/test_imu.c` | Authoritative standalone IMU/calibration/quality tests |
| `integration_test/buddy4_obstruction_sdk.c/.h` | Buddy 4 SDK-specific pairing/eligibility adapter; depends on SDK test control context, not a native MTK adapter |
| `integration_test/main.c`, `drive_control.c/.h`, `scheduler.h`, `CMakeLists.txt` | Combined SDK test application, controls, safety, cooperative scheduling and 86-column telemetry; not the final team app |
| `integration_test/test*.c`, `test.ps1`, `fixtures/`, `host_sdk/` | Buddy 4/integration regressions and test-only hardware stubs; exclude from firmware targets |
| `shared/terrain_link.h` | Integration glue from Buddy 4 terrain to Buddy 2's public API; neither portable library owns the other's control policy |
| `buddy4_mtk/overlay/app_program/buddy4/buddy4_task.c/.h` | Buddy 4 isolated native task, mutex-protected snapshot and speed mailbox |
| `buddy4_mtk/overlay/app_program/buddy4/buddy4_device.c/.h` | Buddy 4 native `iica`/`T_I2C_EXEC` bus adapter |
| `buddy4_mtk/overlay/app_program/app_main.c` | Isolated trial startup/monitor, default IMU-only; not the live `app_program/app_main.c` |
| `buddy4_mtk/overlay/{device,kernel,build_make}/` | Trial board/driver, instrumentation and build adaptations; require team review, never bulk-copy over the team tree |
| `buddy4_mtk/{prepare_source.py,test.ps1,build.ps1,base_hashes.json}` | Isolated trial tooling and base-file change detection |
| `buddy4_mtk/build/source/` | Generated disposable trial tree, not a second authoritative module |
| `buddy4/evidence/`, `buddy4_mtk/evidence/` | Original captures, derived manifests, analysis rules and verification scripts |
| `buddy4/*CHECKPOINT.md`, `OBSTRUCTION_*.md`, review notes | Dated investigations; historical claims apply to their recorded versions |
| `buddy4/README.md`, `TODO.md`, this file, report draft | Current navigation, verified status, pending work and report |

Buddy 2 owns its encoder/motor implementations; Buddy 3 owns line following.
No Buddy 2/3 sources were edited. Only one application entry point belongs in
an image; do not link the standalone/SDK mains into MTK.

## Hardware status and evidence

The operator reports completion of controlled SDK tilt/terrain, one-tyre ramp,
motion classification and turning checks. Local height captures support the
limited 1 cm results in the report; historical turn summaries support direction
checks, not independently referenced yaw accuracy. This handover ran no hardware.

The tested **1.00 cm SDK peak-height checkpoint is complete on the operator's
report**. Latest reported results are: flat 505 valid samples/10.08 s, 0.319 cm
mean live height difference, FLAT/SIDE_LEVEL with no I2C errors/clipping; left
saved stable/median 1.242/1.312 cm in Test 1, left live maximum 1.360 cm in Test 2;
right live maxima 0.798/0.750 cm. Right Test 2 was operator-confirmed complete
and unassisted, with positive roll reported. Original recordings for this series
were not identified during finalisation. Right Test 1's 0.640/0.467 saved values
cannot yet be assigned exact fields or side. Use the
[report table and limitations](Buddy4_Terrain_Analysis_Report_Draft.md#57-reported-2-november-1-cm-sdk-height-validation),
not these summaries as a substitute for raw evidence. The supplied estimates
meet +/-1 cm in these tested conditions; broader accuracy remains open.

Buddy 4 next needs the missing raw files, saved/live/side verification and
broader height/yaw/collision evidence. Buddy 2 should investigate the reported
rightward drift and provide coherent encoder/command context. Teammates then
need to complete the native responsibilities below. No calibration or motor
correction is implied by drift or the nonzero flat baseline.

Both latest foam recordings were inspected unchanged. Each contains one new
POSSIBLE_OBSTRUCTION count (0 -> 1), repeated event-flag rows for the same event,
continued commanded forward state at the event, and later SERIAL_STOP. Test 1
contact is explicitly operator-confirmed; Test 2 contact timing/physical details
are not independently verified. Neither adds a possible-impact event. See
[the exact evidence and timing distinctions](evidence/2026-11-02/FOAM_SUCCESS.md).

These are successful bounded observations, not confirmed collision classification,
a measured sensitivity/false-positive rate or proof that all contacts are detectable.
An isolated IMU impulse can remain unconfirmed; prompt S cancels an unfinished
obstruction candidate. Ramps, slip, mechanical load and encoder failure remain
ambiguous. Preserve immediate stop and explicit rearm; events do not stop motors.

## Fresh verification on 2 November 2026

The results below record the earlier software handover run. This later
documentation-only finalisation did not rerun tests or rebuild firmware.
`buddy4_mtk/base_hashes.json` fingerprints the Buddy 4 README; its stored README
fingerprint predates this edit. Before the next isolated preparation/test/build,
review and refresh that documentation entry using the preparer's normalized
UTF-8 hashing convention. The metadata is outside this documentation-only scope
and was left untouched; preserve every code-file fingerprint.

| Check | Result |
| --- | --- |
| Standalone CMake native target + CTest | PASS, 1/1 |
| Complete `integration_test/test.ps1` | PASS: IMU, impact, obstruction core/adapter, motion/yaw, peaks, scheduler, drive safety, real-main telemetry and Buddy 2/3 read-only simulations |
| Complete `buddy4_mtk/test.ps1` | PASS: native task/device/I2C/kernel-wait, console, schemas and shared integration suite |
| CSV schemas/formatter checks | PASS: standalone 39 / SDK 86 / optional isolated MTK 17 fields |
| Standalone Pico W SDK build | PASS |
| SDK driving-test build | PASS |
| Isolated MTK `SMP=0 CONSOLE=usb_cdc` build | PASS; default readable monitor, optional 17-column CSV checked by tests |
| Original evidence manifests and latest foam verifier | PASS; incomplete rows excluded without reconstruction |
| Preservation and `git diff --check` | PASS |

Existing Buddy 3 unsigned-comparison warnings and MTK newlib unimplemented
file-I/O stub linker warnings remain. No new runtime change was required.

Commands from the repository root (native GCC/Python, installed CMake/Pico SDK
and the documented MSYS Make/toolchain environment are prerequisites):

```powershell
$cm = "$env:USERPROFILE/.pico-sdk/cmake/v4.3.4/bin/cmake.exe"
& $cm -S app_program/buddy4 -B app_program/buddy4/build-host -DBUDDY4_HOST_TESTS=ON
& $cm --build app_program/buddy4/build-host
& "$env:USERPROFILE/.pico-sdk/cmake/v4.3.4/bin/ctest.exe" --test-dir app_program/buddy4/build-host --output-on-failure
powershell -NoProfile -ExecutionPolicy Bypass -File app_program/integration_test/test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File app_program/buddy4_mtk/test.ps1
python app_program/buddy4/evidence/verify_foam_success.py
& $cm --build app_program/buddy4/build --target buddy4
& $cm --build app_program/integration_test/build --target buddy234_driving_test
powershell -NoProfile -ExecutionPolicy Bypass -File app_program/buddy4_mtk/build.ps1
```

Do not run trial test/build scripts concurrently: preparation recreates the
same disposable tree. SDK builds are separate. See the SDK README for first-time
configuration. This handover does not flash any generated UF2.

| Artifact relative to repository | SHA-256 |
| --- | --- |
| `app_program/buddy4/build/buddy4.uf2` | `5b5f6baf9292df397e592474aa35a1609c6aea084509380c453361fdb4c6edd2` |
| `app_program/integration_test/build/buddy234_driving_test.uf2` | `c5f3f5599d2b3cb19cf0ae07c87838e994b37ba5a4fd3a2e200cd19eaea67e73` |
| `app_program/buddy4_mtk/build/mtk3pico_smp0_usb_cdc.uf2` | `63b907ee14c85faa7bc3fc5202f3b21398afd3feceb51d9968650ef0211a35d4` |

The SDK artifact is unchanged from the successful-test checkpoint. Captures
identify it as updated SDK firmware by operator provenance, not an embedded hash.

## Concrete team MTK integration blockers

1. **Live registration/build:** live `app_program/app_main.c` starts Buddy 1/5,
   not Buddy 4. Live `build_make/mtkernel_3/app_program/subdir.mk` does not list
   Buddy 4. Integrate task init/start and the portable files explicitly after
   agreeing ownership; the isolated overlay is a reference, not a merged result.
2. **Native I2C board/driver and console:** live driver still selects GP8/9 for
   I2C0, conflicting with motor wiring; the isolated board path uses GP16/17 and
   tested native completion/recovery handling. Review those changes with owners.
   The trial opens `iica` exclusively; agree bus ownership. Its USB-only profile
   disables polled UART mirroring to control IRQ blockage and sacrifices that
   early/panic mirror. Do not assume live console timing has the same properties.
3. **Fresh motor/encoder producer:** the native speed mailbox can support motion/
   yaw, but no Buddy 2 producer is wired. Use coherent signed cm/s pairs, validity,
   a common monotonic epoch and real observation provenance, not fresh getter
   timestamps on cached readings. Direction is command-derived for single-phase
   encoders. Do not alter Buddy 2 calibration/geometry silently (11.3 vs 12.2 cm).
4. **Obstruction integration missing:** native task/snapshot has no obstruction
   state or full command context. The speed mailbox alone is insufficient.
   A future native adapter must receive forward targets, mode/session changes,
   stops/braking/faults, motor state, terrain compensation, fresh IMU and qualified
   encoder timing. Preserve pause/asymmetry/confirmation/cooldown cancellation
   and expose count/time/validity. Do not link SDK test controls into MTK.
5. **Safety and telemetry ownership:** SDK R/G/S and quality-stop/rearm logic
   are test-application behavior, absent from the IMU-only trial. The team motor
   owner must implement the agreed safety contract. Define validity, units,
   current vs saved state, event delivery/loss policy and MQTT encoding with
   Buddy 1. Snapshots retain latest history, not a lossless event queue.
6. **Integrated qualification:** measured task priorities/stacks, timing under
   USB/Wi-Fi/ultrasonic load, bus/error recovery and full course start/stop/turn/
   ramp/contact behavior remain to be tested. The 8192-byte sensor task stack
   and priority 10 are provisional; build size is not measured runtime headroom.

Buddy 5 is optional for additional proximity context; it is not a prerequisite
for the existing encoder-assisted obstruction observation or initial IMU task
integration. Proximity cannot independently confirm contact. General height and
yaw accuracy, the prior sensor/USB interruption root causes and Tribunal 1
before/after measurements remain open. These do not justify changing working
algorithms in this documentation handover.

The terrain report is an updated evidence-backed **draft**, not a final course/
team acceptance report. Complete traceability for historical summaries, final
team results, rubric/references and Tribunal evidence before submission.

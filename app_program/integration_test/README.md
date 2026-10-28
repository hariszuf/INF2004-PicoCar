# Buddy 2 + 3 + 4 integration prototype checkpoint

Standalone Pico SDK/CMake firmware; report and micro-T-Kernel integration deferred.
Existing Buddy sources are referenced directly; standalone mains/motion_task.c are
excluded and parent build targets are not imported. Buddy 2/3 files are read-only.

## Build and verification

Run from the repository root. SDK/tool versions below match the verified machine;
the commands resolve the user home and native GCC instead of embedding local paths.

```powershell
# From the repository root; GCC and Python must already be on PATH.
$checkpointTools = Join-Path $env:USERPROFILE '.pico-sdk'
$checkpointCmake = Join-Path $checkpointTools 'cmake/v4.3.4/bin/cmake.exe'
$checkpointHostGcc = (Get-Command gcc).Source
$env:PATH = "$(Join-Path $checkpointTools 'ninja/v1.13.2');$(Join-Path $checkpointTools 'toolchain/15_2_Rel1/bin');" + $env:PATH

powershell -NoProfile -ExecutionPolicy Bypass -File app_program/integration_test/test.ps1
python app_program/integration_test/test_csv_schema.py

& $checkpointCmake -S app_program/buddy4 -B app_program/integration_test/build-buddy4-host -G Ninja -DBUDDY4_HOST_TESTS=ON "-DCMAKE_C_COMPILER=$checkpointHostGcc"
& $checkpointCmake --build app_program/integration_test/build-buddy4-host
& (Join-Path $checkpointTools 'cmake/v4.3.4/bin/ctest.exe') --test-dir app_program/integration_test/build-buddy4-host --output-on-failure

& $checkpointCmake -S app_program/buddy4 -B app_program/integration_test/build-buddy4 -G Ninja "-DPICO_SDK_PATH=$checkpointTools/sdk/2.3.1" -DPICO_BOARD=pico_w -DPICO_NO_PICOTOOL=0 "-Dpicotool_DIR=$checkpointTools/picotool/2.3.1/picotool"
& $checkpointCmake --build app_program/integration_test/build-buddy4 --target buddy4

& $checkpointCmake -S app_program/integration_test -B app_program/integration_test/build -G Ninja "-DPICO_SDK_PATH=$checkpointTools/sdk/2.3.1" -DPICO_BOARD=pico_w -DPICO_NO_PICOTOOL=0 "-Dpicotool_DIR=$checkpointTools/picotool/2.3.1/picotool"
& $checkpointCmake --build app_program/integration_test/build --target buddy234_driving_test
```

Firmware: `app_program/integration_test/build/buddy234_driving_test.uf2` and .elf.
Standalone firmware: `app_program/integration_test/build-buddy4/buddy4.uf2`.
No flashing command is part of the checkpoint.

Verification checkpoint: 2026-10-22 (Asia/Singapore). Native GCC 15.2.0,
Arm GCC 15.2.1, Pico SDK 2.3.1, CMake 4.3.4, Ninja 1.13.2; board pico_w.

| Check | Result |
| --- | --- |
| Authoritative IMU/calibration/configuration/recovery suite | Pass |
| Standalone native CTest | 1/1 pass |
| Median peak window/lifecycle regressions | Pass |
| Encoder motion classification | Pass |
| Possible-impact synthetic tests | Pass |
| Encoder yaw-rate estimate | Pass |
| Integration controls, safety and both timed turn modes | Pass |
| Cooperative scheduler | Pass |
| Buddy 2 motion simulation | 39 checks, 0 failures |
| Buddy 3 line simulation | 5 checks, 0 failures |
| CSV schemas/printf argument counts | Pass: standalone 33, integration 69 columns |
| Standalone buddy4 and integration buddy234_driving_test builds | Pass |

No remaining test/build failures. Buddy 3 line_follow.c:127/136 emits two existing
unsigned comparison warnings because turn escalation is disabled at zero; no
Buddy 3 source was changed. Tests are synthetic host checks and compile/link
verification, not hardware accuracy or safety validation. No firmware was flashed
and no motors were operated. Existing recording analysis is observational evidence,
not validation of the prototype's measurement accuracy.

## Provisional mounting roll update (28 October 2026)

Roll offset is +0.61 degrees, replacing -0.25 degrees. The two flat recordings
at the previous setting averaged +0.913 and +0.800 degrees; subtracting the
additional 0.86 degrees predicts +0.053 and -0.060 degree residuals. These are
predictions only. Hardware verification remains pending: manually flash the new
UF2 and record flat, left-held and right-held positions using the 1 cm ramp.
No startup auto-zero is added. All other calibration, axes, geometry, thresholds,
estimators, motion classification, controls and safety behaviour are unchanged.

Software verification for this update passed: the complete `test.ps1` host runner
(IMU/recovery, median, motion classification, impact, yaw, controls/safety,
scheduler, Buddy 2 simulation: 39 checks, Buddy 3 simulation: 5 checks), CSV schema
and printf argument checks (33 standalone / 69 integration columns), native CTest
(1/1), standalone `buddy4` build and integration `buddy234_driving_test` build.
No test/build failures occurred. The two documented Buddy 3 unsigned-comparison
warnings remain. Independent literal expectations and all tolerances are retained.
Only Buddy 4's roll default changes firmware behaviour; Buddy 2/3 files are unchanged.
The newly built integration firmware is `build/buddy234_driving_test.uf2` relative
to this directory. No flashing, motor operation, commit or push was performed.

## Test ownership

`app_program/buddy4/tests/test_imu.c` is authoritative for IMU/calibration and
recovery tests. The native CMake target and integration test runner use it directly.
`integration_test/test_imu_updated.c` is now only a compatibility include, with no
separate expectations. Median tests reuse the authoritative fake-I2C fixtures and
exercise the internal window algorithm separately. The other authoritative
feature suites are test_median.c, test_motion_class.c, test_impact.c,
test_encoder_yaw.c, test_drive_control.c and test_scheduler.c in integration_test.
The runner also compiles existing Buddy 2/3 simulation tests without modifying them.

The test reconciliation changes only stale mounting-offset constants, synthetic
angle inputs and the recalibration flag expectation. Exact independent bias/scale,
sign and geometry assertions are retained/explicit; expected acceleration and angle
calculations use literal reference coefficients, not values read back from the
implementation. Functional assertions and tolerances are preserved. An explicit
return 0 permits reusing the test entry point without suppressing compiler warnings.

## Preserved settings and wiring

Motor update 20 ms; line update 5 ms; IMU poll 20 ms. Parent line-demo overrides:
MOTION_MIN_DUTY=0.10f, MOTION_BREAKAWAY_DUTY=0.22f,
MOTION_FEEDFORWARD_DUTY_PER_CM_S=0.0125f, MOTION_STALL_DUTY=0.6f,
MOTION_STALL_TIME_MS=1500u. Other tuning comes unchanged from original headers.
Buddy 4 calibration: bias {0.03799f,0.03790f,0.08411f}, scale
{1.01301f,1.03419f,1.01223f}, pitch sign +1/offset 0.97 degrees, roll sign
+1/offset +0.61 degrees (provisional; pending hardware verification), mounting_offsets_need_recalibration=false, spacing
11.3 cm confirmed. Buddy 2 motion geometry remains 12.2 cm.

Motors GP8/9 and GP10/11; encoders GP6/4; IR GP0/2, barcode GP26. I2C0 GP16/17,
100 kHz, LSM303D 0x1D. Confirm wiring and line sensor side/geometry before hardware
use. The parent CMake's historical Buddy 2/Buddy 3 paths differ from the current
buddy2/buddy3 directory names; integration uses current paths directly.

## Controls and safety

USB CDC terminal, DTR asserted, conventional baud 115200. Uppercase single keys,
no Enter required; disable automatic command macros.

| Key | Action |
| --- | --- |
| M | Select manual equal-speed driving, default mode 0; 3.0/3.0 cm/s, no line needed |
| L | Select line following, mode 1; Buddy 3 chooses wheel targets |
| A | Select left turn test, mode 2; -2.0/+2.0 cm/s |
| D | Select right turn test, mode 3; +2.0/-2.0 cm/s |
| R | Explicitly rearm stopped latch with healthy fresh IMU; does not start |
| G | Start selected mode after R |
| S or space | Brake and latch off |

Boot is stopped/latched. Select mode only when stopped; selection revokes rearm.
Wait for the motor error/latch state, then explicitly R/G each run. Mode changes,
R and G during motion do not change/extend the run. G after stopping cannot restart
without rearm. Clear a fault only after inspection. No automatic clearing/restart.

USB/DTR disconnection, motor faults, >=500 ms without fresh IMU data, serial stop,
and line loss in line mode latch off. Line loss does not stop manual/turn modes;
Buddy 3 start/update runs only in line mode. Manual drive is equal-speed control,
not IMU heading control. Estimates/observers never issue motor commands or affect
safety decisions. Low target speed does not guarantee actual speed because original
closed-loop and breakaway tuning remains active.

Turn tests stop with TURN_TEST_TIMEOUT at the first cooperative check of elapsed
>=1000000 us from accepted G. Brake/latch does not wait for the next motor update.
No repeat or timeout extension. TURN_TEST_SPEED_CM_S defaults to 2.0f and can be
configured via target compile definition; nonfinite/nonpositive/>40 cm/s is rejected
at start. The timeout is fixed. USB/fault/IMU protections remain active. Loop latency,
hangs and physical stopping time prevent a hard real-time guarantee.

A serial stop is NOT a hardware emergency stop: it depends on working firmware,
USB and motor electronics. Use an accessible physical motor power cutoff. SDK
USB/DTR connection status is not a host-command heartbeat.

## Scheduler and recording

One cooperative core checks motor, line and IMU tasks in order; phase-locked due
checks skip missed releases without catch-up bursts. Each I2C transfer has a 1000 us
timeout. Reconfiguration retries only while stopped, at most once per second;
multiple bounded transfers can delay other work. Diagnostics expose skipped
releases, worst lateness and runtime; small nonzero lateness is normal jitter.
USB input is zero-timeout, one byte per pass outside updates. Output is best effort,
with SDK stdout timeout 500 us per write; formatting/multiple writes add latency.

Enable terminal file logging before starting; reconnect for a fresh header if
needed. Record stationary baseline, selected mode/targets and the entire run, then
S to stop. Keep USB connected. Save # metadata separately from CSV rows; parse by
header and discard malformed/truncated rows. No lossless recording guarantee.

Current schema: **69 integration columns**, **33 standalone columns**. Existing
IMU measurements/events, stability/median peaks, encoder speeds/distance, line state,
faults and timing fields are retained. Added observer fields have explicit names:
encoder_motion_state/valid/sample_us, possible_impact_valid/event/count/sample_us/
event_us/magnitude_g/baseline_g/deviation_g, encoder_yaw_rate_estimate_rad_s/deg_s/
valid/sample_us, selected_target_left_cm_s/right_cm_s and control_reason. Consult
the emitted full header for exact names. Time units are microseconds since boot.
Selected targets remain visible while stopped; non-line active command fields go
to zero. Selected line targets are zero placeholders; its dynamic commands remain
in left_cmd/right_cmd. CONTROL summaries expose mode/targets/reason/test limit.

Invalid IMU rows retain old measurements: check valid and sample_us. Each observer
has separate validity. Do not equate invalid zero-initialized numbers with measured
zero. Repeated encoder sample timestamps are held observations, not new updates.
The classifier/speed estimate is updated immediately after each actual 20 ms motor
update because Buddy 2 status lacks its own timestamp. Sample time is software
observation time, not the time of an individual encoder edge.

## Observational features (unchanged criteria)

IMU smoothing alpha=0.15; pitch hysteresis 6/3 degrees, roll 4/2 degrees. Original
stability peak and 11-sample median candidate are documented in the
[Buddy 4 README](../buddy4/README.md). Median validity=false means unconfirmed.
Original bump flag stays separate from possible-impact logging.

Motion labels: STOPPED, FORWARD, REVERSE, TURNING_LEFT, TURNING_RIGHT, UNKNOWN.
Provisional stop thresholds 0.5/0.8 cm/s (max absolute wheel speed); turn thresholds
1.5/1.0 cm/s (signed right-minus-left difference); 100 ms confirmation and 100 ms
expiry. Turning takes precedence over mean-speed travel outside stop deadband.
Opposing speeds can cancel the mean without enough turn evidence, giving UNKNOWN.
Slow-turn recording analysis found this behavior consistent with thresholds and
confirmation; incomplete logged updates prevented an exact internal-state replay.
No threshold tuning followed. Invalid inputs immediately yield UNKNOWN; otherwise
old confirmed labels can persist during pending transitions. Duplicate/reversed/
future timestamps, nonfinite inputs and stale samples invalidate. Recovery needs
full confirmation. Direction is inferred from commanded motion by single-phase
encoders, not independent ground truth.

Possible-impact signal: absolute deviation of corrected acceleration magnitude from
a tracked nonzero baseline (gravity is included, normally near 1 g). Provisional
trigger >=0.30 g, at least two fresh samples spanning >=20 ms, gaps/age <=40 ms.
One isolated spike is insufficient. Cooldown 500 ms plus two calm samples <=0.10 g
before rearming; sustained high input cannot retrigger. Baseline primes on first
fresh sample, then tracks below trigger with alpha=dt/(500000+dt), frozen at/above
trigger. No tilt gating. Invalid/stale/duplicate/reversed/future data clears pending
and baseline; recovery reprimes. Event count/time and cooldown survive invalidity.
Event pulse belongs to one input sample, count saturates at UINT32_MAX. Invalid
rows can retain diagnostics. Startup/recovery during a transient can miss it.

encoder_yaw_rate_estimate_rad_s=(right_cm_s-left_cm_s)/centre_spacing_cm;
deg_s=rad_s*180/pi. Positive is left/CCW viewed from above, consistent with Buddy 2.
Signed reverse curves and opposite-wheel spins are preserved. Configurable spacing
11.3 cm; provisional expiry >=100 ms. Finite speeds/geometry/output and increasing,
fresh/nonfuture timestamps are required. Invalidity retains numeric diagnostics
but sets valid=false. No additional filtering/integration/delay; Buddy 2's existing
alpha=0.5 speed filter already adds roughly 20 ms low-frequency lag at nominal rate.

## Initial operator procedure (not performed by this checkpoint)

Disconnect motor power to inspect wiring and secure both wheels off ground, clear
of hands, cables and loose objects, with physical cutoff accessible. Only when
independently ready to load firmware, capture USB CSV and confirm stopped boot and
fresh IMU. With a secure fixture, use A then R/G once, confirm timeout/latch-off,
and verify G alone cannot restart. S/space can abort earlier. Wait for wheels to
stop; explicitly D then R/G for the opposite direction. No black line is needed.
Manual mode M is for separate flat-ground tests; L explicitly requires the intended
line geometry. No measurement accuracy follows from a supported-wheel direction
check. No collision/strike test is requested.

## Remaining limitations

- Height estimation and both peak estimators are preliminary; accuracy is unverified.
- Possible-impact detection is not validated collision discrimination. Ramps,
  braking, vibration and handling can resemble impacts; short/subthreshold events
  and magnitude-preserving directional changes can be missed.
- Slow turns may legitimately report UNKNOWN. Current stop/turn thresholds and
  confirmation logic are unchanged; no automatic tuning to one recording occurred.
- Encoder yaw rate is an odometry estimate; professor acceptance for the brief's
  turn-rate requirement remains unresolved. Slip, terrain, tyre deformation and
  command-inferred single-phase encoder direction limit motion/rate estimates.
- Buddy 2 retains 12.2 cm geometry; Buddy 4 uses confirmed/measured 11.3 cm. Team
  reconciliation remains necessary; neither value was changed in this checkpoint.
- Calibration values are preserved, not newly validated by these software tests.
- Cooperative timing, USB detection and best-effort CSV are not hard real-time
  guarantees or lossless recording. Serial stop is not a hardware emergency stop.
- Report writing and micro-T-Kernel integration remain deferred.

## Source checkpoint hygiene

Build directories and generated binaries remain ignored; no files were deleted.
The old local buddy_hashes_before.json is retained but ignored because it contains
machine-specific paths. Do not select ignored binaries/snapshots in GitHub Desktop.
No secrets were detected in the reviewed pending source changes; this is a scoped
check, not a repository-wide credential audit. Existing prior prototype changes
are preserved. No staging, commits or pushes were performed.

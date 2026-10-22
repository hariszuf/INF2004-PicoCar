# Buddy 4 — IMU, Motion and Terrain Monitoring

**Project:** INF2004 Autonomous PicoCar
**Checkpoint:** 22 October 2026
**Status:** All planned prototype modules implemented. Selected hardware checks passed; full validation remains incomplete.

This README records the current settings, instructions, experimental results, limitations, and remaining work so development can continue without the original chat.

## 1. My responsibility

The project brief assigns Buddy 4:

- Calibrate the IMU.
- Detect tilt, humps and peak height.
- Classify motion and detect collision.
- Report turn rate and motion events.

Expected deliverables:

1. IMU processing module.
2. Hump-detection algorithm.
3. Terrain analysis report.

Our implementation includes preliminary methods for these features. Implementing a feature does not mean its accuracy or reliability has been fully validated.

Report writing and final µT-Kernel integration are currently deferred.

## 2. Hardware and wiring

| Item | Confirmed setting |
|---|---|
| Microcontroller | Raspberry Pi Pico W |
| Sensor | **LSM303D**, not LSM303DLHC |
| Gyroscope | None |
| Communication | I²C0, 100 kHz |
| SDA | GP16 — physical pin 21 |
| SCL | GP17 — physical pin 22 |
| Sensor supply | 3.3 V, common GND |
| I²C address | `0x1D` |
| Identification register | `WHO_AM_I`, address `0x0F` |
| Expected identification | `0x49` |
| Sensor output rate | 100 Hz |
| IMU processing period | 20 ms, approximately 50 Hz |
| Accelerometer range | ±2 g |
| Raw conversion | Signed 16-bit, little-endian; **no right shift** |
| Nominal sensitivity | `0.000061 g/count` |
| Wheel centre-to-centre spacing | **11.3 cm**, measured |
| Chassis supports | Two drive wheels and a ball caster |

The board’s printed X/Y arrows did not match the expected measured channels during calibration. Processing preserves the sensor’s reported axes; do not silently swap them.

### Geometry history

Earlier dimensions were misunderstood:

- **9.5 cm** was the gap between the tyres.
- **12.2 cm** was outside-to-outside width.
- **10.85 cm** was a provisional midpoint.
- **11.3 cm** was subsequently measured directly centre-to-centre and is the current Buddy 4 setting.

Buddy 2 retains its original **12.2 cm** geometry setting. Do not change it without agreement with its owner. Its heading calculation and Buddy 4’s yaw estimate therefore use different spacing values.

## 3. Active files

| File | Purpose |
|---|---|
| `buddy4_imu.c/.h` | Sensor configuration/acquisition, acceleration calibration, pitch/roll, terrain states, side events and peak estimators |
| `buddy4_motion.c/.h` | Encoder-based estimated motion classification |
| `buddy4_impact.c/.h` | Observational possible-impact detector |
| `buddy4_encoder_yaw.c/.h` | Encoder-based estimated yaw rate |
| `main.c` | Standalone IMU monitor and CSV output; no motor control |
| `CMakeLists.txt` | Standalone firmware and native IMU test build |
| `pico_sdk_import.cmake` | SDK import helper |
| `tests/` | Original standalone IMU tests |
| `../integration_test/` | Separate Buddy 2+3+4 driving-test application and broader regression tests |

Older terrain, driver, MQTT and standalone experiments were archived outside the active application folders. Consult `archive/buddy4/` and its manifest before deleting historical files.

The original tested all-in-one `IMUProject.c` and historical UF2 were preserved. Archived UF2 files are not current firmware.

### Integration boundaries

- Buddy 2 owns motors and wheel encoders.
- Buddy 3 owns line-following decisions.
- Buddy 4 observes sensor/motion data.
- `integration_test` supplies the test scheduler and controls.
- Existing Buddy 2/3 source and tuning have been preserved.
- The current driving application is **standalone Pico SDK/CMake**, not final µT-Kernel integration.
- Do not compile multiple standalone `main()` files into one executable.

## 4. Current calibration

Acceleration correction is applied independently to each reported sensor axis:

```c
corrected_g = (uncorrected_g - bias_g) * scale;
```

| Axis | Bias to subtract, g | Scale |
|---|---:|---:|
| X | 0.03799 | 1.01301 |
| Y | 0.03790 | 1.03419 |
| Z | 0.08411 | 1.01223 |

Current default mounting/geometry settings:

```c
.accel_bias_g = {0.03799f, 0.03790f, 0.08411f},
.accel_scale = {1.01301f, 1.03419f, 1.01223f},
.mounting_offsets_need_recalibration = false,
.pitch_sign = 1.0f,
.pitch_offset_deg = 0.97f,
.roll_sign = 1.0f,
.roll_offset_deg = -0.25f,
.centre_spacing_cm = 11.3f,
.centre_spacing_confirmed = true,
```

These coefficients are **preliminary six-position estimates**. Edge orientations were imperfectly aligned.

Raw counts and uncorrected acceleration remain available in the standalone CSV. Corrected acceleration drives magnitude, angles and transient processing.

There is no automatic startup zeroing. Starting the car on a slope must not redefine that slope as level.

If the sensor is remounted or its bracket moves, recheck the mounting offsets. A confirmed geometry flag confirms the measured spacing, not the accuracy of the height model.

## 5. Tilt and terrain detection

Angle smoothing alpha: **0.15**.

| Signal | Enter tilted state | Return toward level |
|---|---:|---:|
| Pitch | ±6° | ±3° |
| Roll | ±4° | ±2° |

Current mounting convention:

- Positive pitch: `UPHILL`.
- Negative pitch: `DOWNHILL`.
- Negative roll: `LEFT_WHEEL_UP`.
- Positive roll: `RIGHT_WHEEL_UP`.
- Small roll: `SIDE_LEVEL`.

Labels describe chassis orientation. The IMU cannot independently tell whether one wheel rose or the opposite wheel dropped.

A one-wheel obstacle can create both pitch and roll. The ball caster’s contact position also affects chassis orientation.

## 6. Preliminary wheel-height estimation

Current geometric estimate:

```text
wheel_height_difference_cm
    = centre_spacing_cm × sin(abs(filtered_roll))
```

This estimates a wheel-height difference from chassis tilt. It is **not independently validated hump height**.

Limitations include:

- Combined pitch and roll.
- Sensor mounting alignment.
- Tyre compression and contact geometry.
- Ball-caster support geometry.
- Vehicle acceleration and vibration.
- Wheel slip.
- Imperfect acceleration calibration.

A full-width ramp can raise both wheels equally while producing little roll. Its height cannot be obtained from this roll-only formula.

Do not assume a moving ramp crossing has a known height merely because a 2.5 cm book was used in separate stationary tests.

### Original stability estimator

Requires:

- Fresh samples spanning at least **200 ms**.
- Filtered absolute roll variation no greater than **1°**.
- No sample gap greater than **40 ms**.

It retains the highest conservative stable-window estimate.

This rule worked for held positions but sometimes severely underestimated or failed to confirm moving peaks.

### Experimental rolling-median estimator

Runs alongside the original estimator:

- Fixed window of **11 filtered absolute-roll samples**.
- Strictly increasing timestamps.
- Adjacent gaps no greater than **40 ms**.
- Total accepted window span **190–210 ms**.
- Highest accepted window median retained per side event.
- Fixed-size storage; no dynamic allocation.
- Explicit `median_candidate_valid`.

A false candidate-validity flag means **unconfirmed**, not measured zero height.

Event end and side reversal reset active candidate state. Transfer failure aborts active detection. Completed-event snapshots are retained according to the current API contract.

Median filtering rejects isolated spikes in its input window, but an acceleration disturbance can persist through the upstream angle filter. It does not guarantee rejection of every handling or driving artifact.

## 7. Motion classification

Uses Buddy 2’s reported signed encoder wheel speeds.

States:

- `UNKNOWN`
- `STOPPED`
- `FORWARD`
- `REVERSE`
- `TURNING_LEFT`
- `TURNING_RIGHT`

Provisional defaults:

| Setting | Value |
|---|---:|
| Enter stopped | Maximum absolute wheel speed ≤0.5 cm/s |
| Exit stopped | Maximum absolute wheel speed >0.8 cm/s |
| Enter turn | Absolute right-minus-left speed ≥1.5 cm/s |
| Hold confirmed turn | Signed difference magnitude ≥1.0 cm/s |
| State confirmation | 100 ms |
| Stale expiry | 100 ms |

During a pending transition, the previous confirmed state may persist while observations remain fresh.

Invalid/stale observations produce `UNKNOWN`. Opposing slow wheel speeds can cancel in their mean while failing the turn threshold; `UNKNOWN` is intentional in that case.

Single-phase encoder direction is inferred from commanded direction. Labels are estimated motion, not independent ground truth.

## 8. Possible-impact detector

Separate from the older `bump_event`.

Provisional criteria:

| Setting | Value |
|---|---:|
| Signal | Absolute magnitude deviation from tracked nonzero baseline |
| Trigger | ≥0.30 g |
| Confirmation | At least two fresh samples spanning 20 ms |
| Maximum sample gap | 40 ms |
| Cooldown | 500 ms |
| Rearm | Two calm samples ≤0.10 g, with cooldown satisfied |
| Baseline time constant | 500 ms |

Stationary acceleration includes gravity; the detector does not assume a zero-g resting magnitude.

Outputs include:

- Detector validity.
- One-input-sample `possible_impact` event.
- Cumulative event count.
- Sample/event timestamps.
- Magnitude, baseline and deviation.

It is observational only. No motor commands or safety stops depend on it.

Ramps, braking, vibration and handling may resemble impacts. Synthetic tests passed and selected normal-driving runs produced no false flags. **Actual collision discrimination remains unvalidated.**

Do not deliberately crash the car or strike the sensor to test it.

## 9. Estimated turn rate

Encoder-based formula:

```text
yaw_rate_rad_s = (right_speed_cm_s − left_speed_cm_s)
                / centre_spacing_cm

yaw_rate_deg_s = yaw_rate_rad_s × 180 / pi
```

- Positive: left/counterclockwise viewed from above.
- Negative: right/clockwise.
- Spacing: configurable, default **11.3 cm**.
- Freshness limit: provisional **100 ms**.
- No extra filtering beyond the wheel-speed inputs.
- Invalid/stale input clears validity.

Output names explicitly identify `encoder_yaw_rate_estimate`.

This is **not gyroscope rate or independently measured IMU yaw rate**. Accuracy depends on encoder direction assumptions, speed estimation, geometry and slip.

Professor agreement is still needed on whether this method satisfies the brief’s turn-rate requirement.

## 10. Build instructions

Tools used:

- Pico SDK **2.3.1**
- Arm toolchain **15_2_Rel1**
- Installed CMake **v4.3.4**
- Ninja **v1.13.2**

Commands below use the current Windows installation layout. Adjust paths on another computer.

### Standalone Buddy 4

Run in a VS Code PowerShell terminal:

```powershell
cd app_program/buddy4 # from the repository root

$cm = "$env:USERPROFILE/.pico-sdk/cmake/v4.3.4/bin/cmake.exe"

& $cm -S . -B build -G Ninja "-DCMAKE_MAKE_PROGRAM=$env:USERPROFILE/.pico-sdk/ninja/v1.13.2/ninja.exe" -DPICO_BOARD=pico_w
```

Only if configuration succeeds:

```powershell
& $cm --build build --target buddy4
```

Firmware:

```text
app_program/buddy4/build/buddy4.uf2
```

After source edits, save and rebuild. Do not flash an old archived UF2.

Do not run these standalone commands from the parent `app_program` directory: that selects a different CMake project.

### Integration driving-test firmware

Run from the repository root:

```powershell
$env:PATH = "$env:USERPROFILE\.pico-sdk\toolchain\15_2_Rel1\bin;$env:USERPROFILE\.pico-sdk\ninja\v1.13.2;" + $env:PATH

$cm = "$env:USERPROFILE/.pico-sdk/cmake/v4.3.4/bin/cmake.exe"

& $cm -S app_program/integration_test -B app_program/integration_test/build -G Ninja "-DPICO_SDK_PATH=$env:USERPROFILE/.pico-sdk/sdk/2.3.1" -DPICO_BOARD=pico_w -DPICO_NO_PICOTOOL=0 "-Dpicotool_DIR=$env:USERPROFILE/.pico-sdk/picotool/2.3.1/picotool"
```

Only if configuration succeeds:

```powershell
& $cm --build app_program/integration_test/build --target buddy234_driving_test
```

Firmware:

```text
app_program/integration_test/build/buddy234_driving_test.uf2
```

### Regression suite

From repository root:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File app_program/integration_test/test.ps1
```

The runner covers the currently registered module, scheduler, control and schema tests. Inspect its output; do not assume a previous pass applies after new edits.

The original standalone IMU suite has now been reconciled with the deliberately updated calibration defaults. `tests/test_imu.c` is authoritative; the former integration copy is a compatibility include. Independent expected-value calculations, functional assertions and tolerances are preserved. Production calibration is unchanged. See the verified checkpoint below.

## 11. Flash and serial capture

1. Switch motor power off.
2. Close the serial monitor and disconnect Pico USB.
3. Hold the Pico W’s BOOTSEL button while reconnecting USB.
4. Release BOOTSEL when `RPI-RP2` appears.
5. Copy the newly built UF2 onto that drive.
6. The Pico restarts and the drive disappears.
7. Open its USB COM port in VS Code Serial Monitor at **115200**.
8. Enable DTR if required.

COM4 was used during recorded sessions, but the port can change.

The standalone firmware does not drive motors. The integration firmware supports explicit driving commands.

Capture the full header and `# CONTROL` / `# SIDE_EVENT_END` lines. Long CSV rows wrap visually; that is expected.

Logging is best effort. Some captures contain malformed fragments, particularly around terminal command messages. Reject malformed rows rather than silently shifting their columns.

Historical schema sizes changed as features were added:

| Version | Integration columns |
|---|---:|
| Initial driving integration | 48 |
| Median candidate | 51 |
| Motion classification | 54 |
| Possible-impact detector | 62 |
| Encoder yaw rate | 66 |
| Timed turn controls | **69** |

Standalone CSV currently has **33 columns**.

Always use the header matching that firmware. Filter fresh IMU analysis using `valid=1`; check sample timestamps and validity fields for other modules separately.

## 12. Integration controls

Commands are uppercase and sent one at a time.

| Key | Action |
|---|---|
| `M` | Select manual straight mode; no black line needed |
| `L` | Select line-following mode |
| `A` | Select timed left rotation |
| `D` | Select timed right rotation |
| `R` | Explicitly rearm while stopped and healthy |
| `G` | Start selected mode after rearming |
| `S` or space | Stop and latch off |

Manual target: **+3/+3 cm/s**.

Timed turn targets:

- Left: **−2/+2 cm/s**.
- Right: **+2/−2 cm/s**.
- Timeout: approximately **1 second**, then brake/latch with `TURN_TEST_TIMEOUT`.

Mode selection while stopped revokes rearming. `G` alone after a stop does not restart motion.

Boot remains stopped. USB loss, prolonged invalid IMU data, motor faults and serial stop latch off. Line-loss stopping applies to line-following mode only. Recovery does not restart automatically.

These are temporary test controls. Final autonomous operation follows the black line through Buddy 3 while Buddy 4 observes.

Serial stop is not a hardware emergency stop. Keep the physical motor-power cutoff accessible. The cooperative timeout is not a hard real-time stopping guarantee.

## 13. What has been completed

### Implemented and software-tested

- LSM303D identification, configuration and signed data conversion.
- Configurable acceleration and mounting calibration.
- Pitch/roll and terrain/side hysteresis.
- Side-event lifecycle and original stable peak estimator.
- Experimental moving median estimator.
- Encoder-based motion classification.
- Observational possible-impact detector.
- Encoder yaw-rate estimate.
- Separate standalone and driving-test builds.
- CSV schema checks and module regression tests.
- Manual start/stop/rearm and timed-turn controls.

### Observed on hardware

- Level baseline near zero after calibration.
- Correct left/right stationary tilt labels.
- Unassisted one-wheel ramp crossings detected in both directions.
- Return to side-level after crossings.
- Forward/stopped motion labels during flat driving.
- Left/right turn labels and yaw-rate signs.
- Supported-wheel start/stop and timed turn behaviour.
- `G` alone after stopping did not restart.
- No possible-impact events in the reviewed normal straight/turn runs.

These observations do not prove all fault-stop paths or operating conditions have been physically validated.

## 14. Recorded results for the report

The figures below came from serial recordings reviewed during development. They are recorded observations, not guaranteed performance specifications.

Some early capture sections included repositioning. Where noted, settled portions were used.

### A. Before acceleration calibration

For a known **2.5 cm book**, with corrected 11.3 cm wheel spacing but before acceleration correction:

| Position | Mean estimate |
|---|---:|
| Left wheel raised | 3.00 cm |
| Right wheel raised | 2.67 cm |
| Level | Approximately 0.13 cm |

This showed systematic error and left/right asymmetry.

Earlier apparently better results using 9.5 cm spacing must not be presented as final calibrated accuracy. The geometry was wrong.

### B. After acceleration calibration and mounting-offset update

Level-ground check:

| Measurement | Result |
|---|---:|
| Complete valid samples | 693 |
| Duration | 13.84 s |
| Mean filtered pitch | −0.23° |
| Mean filtered roll | −0.33° |
| Mean corrected acceleration magnitude | 0.992 g |
| Reported errors | 0 |
| Labels | FLAT / SIDE_LEVEL throughout |

A preceding calibration check averaged **1.001 g** over 706 valid samples. These are different recordings; do not combine them as one dataset.

Known 2.5 cm book after calibration:

| Position | Mean estimate |
|---|---:|
| Left wheel raised | 2.74 cm |
| Right wheel raised | 2.63 cm |
| Return to level | 0.03 cm |

Confirmed stable peaks in that recording were approximately **2.81 cm left / 2.78 cm right**.

Calibration improved consistency, but did not establish precise height accuracy.

### C. Flat driving

First reviewed slow flat run:

- Running duration approximately **5.3 s**.
- Encoder distance increased approximately **15.1 cm**.
- Mean reported wheel speeds: **2.89 cm/s left / 2.74 cm/s right**.
- No false terrain, side-tilt or bump flags.
- No new IMU errors or skipped updates within the reviewed recording.
- Operator reported straight movement and normal stopping.

An additional run:

- Running duration approximately **5.6 s**.
- Encoder distance increased approximately **15.9 cm**.
- Mean speeds: **2.86 cm/s left / 2.79 cm/s right**.
- No false tilt/bump detections.
- Error counter stayed constant at 10 during that recording.

Some error counters increased between separate sessions. Constant counters within a run mean no additional errors in that run—not that the entire experiment had no errors.

### D. Why the moving peak estimator changed

With the original 200 ms / 1° stability rule:

| Moving crossing | Confirmed estimate |
|---|---:|
| Early left crossing 1 | 1.19 cm |
| Early left crossing 2 | 1.04 cm |
| Early right crossing | 2.39 cm |
| Later left repeat | 1.00 cm |
| Later right repeat | Unconfirmed |

The later right repeat reached **14.47° filtered roll** and **2.82 cm instantaneous estimate**, but no stable window qualified.

An unconfirmed zero peak is not a zero-height obstacle.

Offline rolling-median exploration produced:

| Recording | Candidate estimate |
|---|---:|
| Early left 1 | 2.35 cm |
| Early left 2 | 2.45 cm |
| Early right | 2.46 cm |
| Later left repeat | 2.19 cm |
| Later right repeat | 2.77 cm |

These are exploratory replay results, not independent height validation. The candidate was subsequently implemented alongside the original filter.

### E. Latest paired hardware crossings with median candidate

Same ramp and direction, per operator confirmation:

| Run | Original stable estimator | Median candidate |
|---|---:|---:|
| Left | 2.192 cm | **2.349 cm** |
| Right | 2.375 cm | **2.523 cm** |

- Median difference: **0.174 cm**, approximately 0.17 cm.
- Left event duration: **2.24 s**.
- Right event duration: **2.62 s**.
- Both returned near level.
- No new IMU errors or skipped updates in either reviewed run.
- Four malformed left rows and two malformed right rows were excluded.

**The actual ramp height was not verified for these moving crossings.** Do not calculate percentage height accuracy against the 2.5 cm book—the book was a separate test.

Further broad replay was deferred at the user’s request. Existing limited replay and hardware evidence should be described honestly.

### F. New motion/impact/yaw straight-driving check

- **619 complete rows**, two incomplete rows excluded.
- All complete rows had valid IMU data.
- Zero reported IMU errors and skipped updates.
- Labels followed STOPPED → FORWARD → STOPPED.
- Mean estimated yaw rate while running: approximately **−0.10°/s**.
- Brief yaw estimates ranged approximately **−3.72 to +4.48°/s**.
- No possible-impact events.
- Operator confirmed straight movement and normal stopping.

This checks behaviour and false flags in one normal run. It is not independent yaw-rate accuracy measurement.

### G. One-second physical turn checks

| Measurement | Left test | Right test |
|---|---:|---:|
| Complete running rows | 50 | 50 |
| TURNING label rows | 24 | 42 |
| UNKNOWN rows | 25 | 6 |
| STOPPED rows | 1 | 2 |
| Mean encoder yaw estimate while running | +10.38°/s | −10.60°/s |

- Operator confirmed actual left/right rotation without help.
- Both timeout summaries appeared approximately 1 second after start.
- No new IMU errors, skipped updates or possible-impact events.
- Final motion state returned to STOPPED.
- Mean rates include startup/slow-speed portions.
- No independent reference angle/time measurement was made.

### H. Slow-turn UNKNOWN investigation

The left turn’s wheel-speed difference briefly fell below the **1.0 cm/s hold threshold**. After losing the label, it needed ≥1.5 cm/s entry evidence continuously for 100 ms.

Opposing low speeds also cancelled in the mean, producing UNKNOWN when neither forward/reverse nor turn evidence qualified.

Logged examples:

| Encoder time, s | Left/right speeds, cm/s | Result |
|---|---|---|
| 106.400167 | −0.807 / +0.402 | UNKNOWN |
| 106.520165 | −1.373 / +0.926 | TURNING_LEFT |
| 106.600158 | −0.741 / +0.209 | UNKNOWN; difference +0.950 |
| 106.980109 | −1.032 / +0.850 | TURNING_LEFT restored |

An investigation retained **697 complete rows**. It identified repeated logged observations:

- Left: 41 distinct encoder timestamps from 50 running rows.
- Right: 39 distinct encoder timestamps from 50 running rows.

Repeated observations must not count as fresh classifier updates.

Observed sample ages were approximately **1.55–21.44 ms**, below the 100 ms stale limit. No logged evidence established timestamp invalidity as the main cause.

Not every internal motor update was logged, so exact replay could not reconstruct every transition. Keep thresholds unchanged rather than tune to one recording.

UNKNOWN is preferable to an unsupported motion claim.

## 15. Report plan — not yet written

When preparing the terrain analysis report, include:

1. **Scope and hardware**
   - Buddy 4 requirements.
   - Confirmed LSM303D identity and lack of gyroscope.
   - Wiring and chassis support arrangement.

2. **Software architecture**
   - Reusable modules and public interfaces.
   - Bounded I²C, validity/timestamps, error recovery.
   - Standalone testing versus final RTOS integration.

3. **Calibration**
   - Six orientations and imperfect alignment.
   - Bias/scale formula and coefficients.
   - Mounting offsets and wheel-spacing correction.
   - Before/after results.

4. **Algorithms**
   - Pitch/roll and hysteresis.
   - Side-event segmentation.
   - Stable-window versus median peak estimator.
   - Motion classification.
   - Possible-impact criteria.
   - Encoder yaw-rate formula and assumptions.

5. **Experiments and evidence**
   - Known-height stationary book tests.
   - Flat driving and starts/stops.
   - One-wheel crossings in both directions.
   - Timed turns.
   - Software regression tests separately from physical tests.

6. **Limitations**
   - No independently validated moving hump height.
   - Possible-impact flag is not confirmed collision classification.
   - Single-phase encoder direction assumptions and slip.
   - Slow-turn UNKNOWN periods.
   - Logging fragments and excluded rows.
   - Geometry discrepancy with Buddy 2.
   - Limited speeds, surfaces, heights and repetitions.

7. **Remaining work and conclusions**
   - State what was demonstrated and what was not.
   - Do not claim production-ready reliability or invent grading accuracy targets.

Preserve serial recordings in the repository’s testing/logs area where appropriate. Keep original captures and a clearly documented cleaned-data procedure.

Do not invent missing raw evidence from summary statistics.

## 16. Remaining checkpoints

### Current cleanup task

- Completed in this checkpoint: reconcile original IMU test expectations with the deliberately updated defaults; retain independent checks.
- Preserve independent functional checks and tolerances.
- Identify one authoritative regression suite.
- Run the full current runner, schema checks and both builds.
- Correct outdated README claims.
- Review the diff for unrelated/generated files or secrets.
- Save a verified Git checkpoint using GitHub Desktop.

The cleanup prompt was provided, but its completion has not yet been confirmed in this handover.

### Validation still incomplete

- Numerical accuracy and repeatability of moving hump-height estimates.
- Detection of broader hump shapes, including full-width obstacles.
- Possible-impact/collision discrimination.
- Reverse-motion labels on hardware.
- Broader speeds, surfaces, curves and repeated trials.
- Independently referenced yaw-rate accuracy.
- Full physical verification of every safety/fault path.
- Professor acceptance of encoder-estimated turn rate and expected height accuracy.

No deliberate crashing or sensor striking is required.

### Deferred

- Final vehicle-controller and µT-Kernel integration.
- Team reconciliation of wheel geometry.
- Terrain analysis report.
- Final demonstration preparation.

## 17. Known issues and development notes

- Two pre-existing Buddy 3 compiler warnings were reported during builds.
- Original IMU test expectations historically used old offsets/flags; current integration test copies were updated. Verify current repository state before claiming this is resolved.
- A right motor stopped during testing because its wire was loose. It worked after reconnection; code/tuning was not changed.
- Earlier sensor error counts rose during handling between recordings. Secure connectors and inspect per-run counter changes.
- Do not raise motor duty to mask a loose connection.
- Do not repeatedly recalibrate to every small baseline variation.
- Do not change thresholds simply to eliminate UNKNOWN labels.
- The final car follows a black line; A/D/M commands belong to test firmware.
- Sensor prototype coding is complete; the full assignment is not fully validated or integrated.

## 18. GitHub Desktop checkpoint instructions

1. Select the correct repository and branch.
2. Review each changed file.
3. Include intended Buddy 4 and integration-test changes only.
4. Exclude generated builds/binaries, credentials and unrelated teammate work.
5. Commit after verification.
6. Push origin when ready.

Suggested commit message:

```text
Add Buddy 4 calibrated terrain and motion monitoring prototype
```

Do not claim a push occurred unless GitHub Desktop or Git confirms it.


## Verified software checkpoint (22 October 2026)

No production algorithms, thresholds, calibration, motor tuning, controls or safety
behavior changed during this verification. Buddy 2/3 remained byte-for-byte unchanged.
No report or micro-T-Kernel integration, flashing, motor operation, staging, commit
or push was performed. Earlier operator observations above are distinct from this
software-only checkpoint and are not independent measurement validation.

`tests/test_imu.c` is the authoritative IMU/calibration/recovery suite. Native CMake
and integration test.ps1 use it directly. integration_test/test_imu_updated.c is
only a compatibility include; test_median.c reuses authoritative fixtures and checks
the internal median windows. No functional checks/tolerances were removed. Literal
expected bias/scale/offset calculations remain independent of implementation values;
explicit sign, bias/scale and geometry assertions are included. All other module,
scheduler and control suites are described in the integration README.

Exact commands run from the repository root:

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


Remaining limitations: preliminary height accuracy is unverified; possible-impact
logging is not validated collision discrimination; slow turns may legitimately
remain UNKNOWN; encoder yaw rate is estimated and professor acceptance remains
unresolved. Single-phase direction is command-inferred; slip, uneven terrain and
tyre deformation limit odometry. Buddy 2 retains 12.2 cm geometry while Buddy 4 uses
11.3 cm; team reconciliation is still needed. Report and micro-T-Kernel integration
remain deferred. Software passes do not establish hardware accuracy or safety.

Pending-source review found no generated binaries/build trees, hardcoded user paths
or matching common credential patterns in the source selection. Local build files
and the retained integration hash snapshot are ignored, not deleted. This scoped
scan is not a full credential audit. Nothing was staged.

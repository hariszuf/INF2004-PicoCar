# Buddy 4 — IMU, Motion and Terrain Monitoring

**Project:** INF2004 Autonomous PicoCar  
**Checkpoint:** 22 October 2026  
**Status:** Planned prototype modules implemented; software checks passed and selected hardware tests completed. Full assignment integration and validation remain incomplete.

This README preserves the current configuration, build instructions, experimental results, limitations and deferred work so development can continue without the original chat.

## 1. Responsibilities and current status

Buddy 4 covers:

- IMU calibration.
- Tilt, terrain and hump detection.
- Preliminary peak-height estimation.
- Motion classification.
- Preliminary possible-impact detection.
- Estimated turn rate and motion events.

Expected deliverables include the IMU processing module, hump-detection algorithm and terrain analysis report.

| Area | Current status |
|---|---|
| Prototype processing modules | Implemented |
| Host regressions and CSV checks | Passed at checkpoint |
| Standalone and integration firmware builds | Passed at checkpoint |
| Selected physical checks | Completed; results below |
| General height accuracy | Not established |
| Collision discrimination | Unvalidated |
| Numerical yaw-rate accuracy | Unvalidated |
| Final µT-Kernel/team integration | KIV |
| Terrain analysis report | KIV |
| Professor clarification | KIV |

No additional prototype features are currently planned. Deferred work must not be described as completed.

## 2. Confirmed hardware

| Item | Setting |
|---|---|
| Microcontroller | Raspberry Pi Pico W |
| Actual sensor | **LSM303D**, not LSM303DLHC |
| Gyroscope | None |
| Communication | I²C0, 100 kHz |
| SDA | GP16, physical pin 21 |
| SCL | GP17, physical pin 22 |
| Supply | 3.3 V with common GND |
| Sensor address | `0x1D` |
| Identification register | `WHO_AM_I`, `0x0F` |
| Expected identification | `0x49` |
| Accelerometer output rate | 100 Hz |
| Processing interval | 20 ms, approximately 50 Hz |
| Range | ±2 g |
| Conversion | Signed 16-bit little-endian; no right shift |
| Nominal sensitivity | `0.000061 g/count` |
| Wheel centre spacing | **11.3 cm**, measured |
| Chassis supports | Two drive wheels and a ball caster |

Processing preserves the sensor’s reported axes. Do not swap axes based only on the board’s printed arrows.

### Wheel-spacing history

- **9.5 cm:** gap between tyres.
- **12.2 cm:** outside-to-outside measurement.
- **10.85 cm:** earlier provisional midpoint.
- **11.3 cm:** subsequently measured centre-to-centre; current Buddy 4 setting.

Buddy 2 retains its original **12.2 cm** setting. Team reconciliation is pending; do not silently change another buddy’s geometry.

## 3. Files and ownership

| File | Purpose |
|---|---|
| `buddy4_imu.c/.h` | Acquisition, calibration, tilt, terrain, side events and peak estimators |
| `buddy4_motion.c/.h` | Encoder-based motion classification |
| `buddy4_impact.c/.h` | Observational possible-impact detector |
| `buddy4_encoder_yaw.c/.h` | Encoder-based yaw-rate estimate |
| `main.c` | Standalone IMU monitor and CSV logging |
| `tests/test_imu.c` | Authoritative IMU/calibration/recovery regression suite |
| `CMakeLists.txt` | Standalone firmware and native IMU tests |
| `pico_sdk_import.cmake` | Pico SDK import |
| [`../integration_test/`](../integration_test/) | Separate driving-test application and broader tests |

Historical experiments are archived under the repository’s `archive/buddy4/` area. Consult its manifest before deleting anything. Historical UF2 files are not current firmware.

### Integration boundaries

- Buddy 2 owns motor and encoder processing.
- Buddy 3 owns line-following decisions.
- Buddy 4 provides observational processing.
- `integration_test` supplies temporary controls and a cooperative scheduler.
- Buddy 2/3 source and tuning were preserved.
- Current test applications use **Pico SDK/CMake**, not final µT-Kernel integration.
- Only one application `main()` belongs in each executable.

## 4. Calibration

Raw counts are preserved. Each axis is converted to g before correction:

```c
corrected_g = (uncorrected_g - bias_g) * scale;
```

| Axis | Bias, g | Scale |
|---|---:|---:|
| X | 0.03799 | 1.01301 |
| Y | 0.03790 | 1.03419 |
| Z | 0.08411 | 1.01223 |

Current configuration:

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

The acceleration coefficients are preliminary six-position estimates; orientations were imperfectly aligned.

Corrected acceleration drives magnitude, tilt and transient processing. Standalone logging preserves raw counts, uncorrected acceleration and corrected acceleration.

There is **no startup auto-zero** or assumption that the car starts level.

Recheck mounting offsets if the sensor or bracket moves. The confirmed-spacing flag confirms the measurement, not the accuracy of the height model.

## 5. Tilt and terrain

Angle smoothing alpha: **0.15**.

| Signal | Enter tilted state | Return toward level |
|---|---:|---:|
| Pitch | ±6° | ±3° |
| Roll | ±4° | ±2° |

Mounting convention:

- Positive pitch: `UPHILL`.
- Negative pitch: `DOWNHILL`.
- Negative roll: `LEFT_WHEEL_UP`.
- Positive roll: `RIGHT_WHEEL_UP`.
- Small roll: `SIDE_LEVEL`.

These labels describe chassis orientation. The sensor cannot independently distinguish one wheel rising from the opposite wheel dropping.

A single-wheel obstacle can produce both pitch and roll. Ball-caster contact also affects chassis orientation.

## 6. Preliminary height estimation

The geometric estimate is:

```text
wheel_height_difference_cm
    = centre_spacing_cm × sin(abs(filtered_roll))
```

It estimates wheel-height difference from chassis roll. It does not directly measure the obstacle surface.

Limitations include combined pitch/roll, mounting alignment, tyre deformation, caster geometry, acceleration, vibration and slip.

A full-width obstacle can raise both wheels equally with little roll; this formula cannot recover its height.

### Original stable estimator

Requires fresh samples spanning at least **200 ms**, roll variation no greater than **1°**, and adjacent gaps no greater than **40 ms**.

It retains the highest accepted stable estimate. Moving peaks can be underestimated or remain unconfirmed.

### Experimental rolling-median estimator

- Uses **11 filtered absolute-roll samples**.
- Requires strictly increasing timestamps.
- Adjacent gaps must be ≤40 ms.
- Accepted window span is **190–210 ms**.
- Retains the highest accepted median during each side event.
- Uses fixed-size storage and explicit validity.

An invalid candidate means **unconfirmed**, not zero height.

Event transitions and transfer failures reset or abort active processing according to the API. Completed-event results remain available according to its snapshot contract.

Filtering does not guarantee rejection of sustained handling or driving disturbances.

## 7. Motion classification

Uses Buddy 2’s signed encoder wheel speeds:

`UNKNOWN`, `STOPPED`, `FORWARD`, `REVERSE`, `TURNING_LEFT`, `TURNING_RIGHT`.

| Setting | Provisional value |
|---|---:|
| Enter stopped | Maximum absolute wheel speed ≤0.5 cm/s |
| Exit stopped | Maximum absolute wheel speed >0.8 cm/s |
| Turn entry | Absolute right-minus-left speed ≥1.5 cm/s |
| Turn holding | Difference magnitude ≥1.0 cm/s in the confirmed direction |
| Confirmation | 100 ms |
| Stale expiry | 100 ms |

Opposing slow wheel speeds can cancel in their mean while failing turn thresholds. `UNKNOWN` is intentional when evidence is insufficient.

Repeated logged timestamps are not independent updates. Invalid/stale observations invalidate classification.

Single-phase encoder direction follows commanded direction, so classification is an estimate rather than independent motion ground truth.

## 8. Possible-impact detector

This detector is separate from the existing `bump_event`.

| Setting | Provisional value |
|---|---:|
| Signal | Absolute magnitude deviation from tracked nonzero baseline |
| Trigger | ≥0.30 g |
| Confirmation | At least two fresh samples spanning 20 ms |
| Maximum gap | 40 ms |
| Cooldown | 500 ms |
| Rearming | Two calm samples ≤0.10 g with cooldown satisfied |
| Baseline time constant | 500 ms |

Stationary acceleration includes gravity.

Outputs include validity, a one-sample event, cumulative count, timestamps, magnitude, baseline and deviation. Invalid data, missing samples and reversed timestamps reset pending detection appropriately.

**No motor command or safety stop depends on this detector.**

Ramps, braking, vibration and handling can resemble impacts. Synthetic tests passed; collision discrimination remains unvalidated. Do not deliberately crash the car or strike the sensor.

## 9. Encoder yaw-rate estimate

```text
yaw_rate_rad_s
    = (right_speed_cm_s − left_speed_cm_s) / spacing_cm

yaw_rate_deg_s
    = yaw_rate_rad_s × 180 / pi
```

- Default spacing: **11.3 cm**.
- Positive: left/counterclockwise viewed from above.
- Negative: right/clockwise.
- Freshness limit: **100 ms**.
- No extra filtering beyond wheel-speed inputs.
- Invalid/stale input clears validity.

This is an **odometry-based estimate**, not gyroscope measurement.

Slip, geometry and command-inferred encoder direction limit accuracy. Professor acceptance against the turn-rate requirement remains pending.

## 10. Build instructions

Tools used at the checkpoint:

- Pico SDK **2.3.1**
- Arm toolchain **15_2_Rel1**
- CMake **4.3.4**
- Ninja **1.13.2**

Adjust installation paths on another computer. Save source changes before building.

### Standalone Buddy 4

From the repository root:

```powershell
$env:PATH = "$env:USERPROFILE\.pico-sdk\toolchain\15_2_Rel1\bin;$env:USERPROFILE\.pico-sdk\ninja\v1.13.2;" + $env:PATH

$cm = "$env:USERPROFILE/.pico-sdk/cmake/v4.3.4/bin/cmake.exe"

& $cm -S app_program/buddy4 -B app_program/buddy4/build -G Ninja "-DPICO_SDK_PATH=$env:USERPROFILE/.pico-sdk/sdk/2.3.1" -DPICO_BOARD=pico_w -DPICO_NO_PICOTOOL=0 "-Dpicotool_DIR=$env:USERPROFILE/.pico-sdk/picotool/2.3.1/picotool"
```

Only if configuration succeeds:

```powershell
& $cm --build app_program/buddy4/build --target buddy4
```

Output:

```text
app_program/buddy4/build/buddy4.uf2
```

### Integration test

From the repository root, with the same tools configured:

```powershell
& $cm -S app_program/integration_test -B app_program/integration_test/build -G Ninja "-DPICO_SDK_PATH=$env:USERPROFILE/.pico-sdk/sdk/2.3.1" -DPICO_BOARD=pico_w -DPICO_NO_PICOTOOL=0 "-Dpicotool_DIR=$env:USERPROFILE/.pico-sdk/picotool/2.3.1/picotool"
```

Only if configuration succeeds:

```powershell
& $cm --build app_program/integration_test/build --target buddy234_driving_test
```

Output:

```text
app_program/integration_test/build/buddy234_driving_test.uf2
```

### Regression runner

From the repository root:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File app_program/integration_test/test.ps1

python app_program/integration_test/test_csv_schema.py
```

The host runner requires its documented native compiler/Python environment. Consult the integration README for prerequisites.

Do not assume a previous test pass applies after new edits.

## 11. Flashing and serial recording

1. Switch motor power off.
2. Close the serial monitor and disconnect Pico USB.
3. Hold BOOTSEL while reconnecting USB.
4. Release when `RPI-RP2` appears.
5. Copy the newly built UF2 to that drive.
6. Open the Pico USB serial port at **115200**.
7. Enable DTR if required.

COM4 was used previously, but the port may change.

Standalone firmware monitors the IMU without driving motors. Integration firmware supports explicit driving commands.

Capture the CSV header and event/control summaries. Visual line wrapping is normal. Exclude malformed fragments without shifting columns.

Current schemas:

| Application | Columns |
|---|---:|
| Standalone | 33 |
| Integration | 69 |

Older captures have different schemas. Always use the matching header and module-specific validity/timestamps.

## 12. Integration controls

Send uppercase commands one at a time.

| Key | Action |
|---|---|
| `M` | Select manual straight mode |
| `L` | Select line following |
| `A` | Select timed left rotation |
| `D` | Select timed right rotation |
| `R` | Rearm while stopped and healthy |
| `G` | Start after rearming |
| `S` or space | Stop and latch off |

Targets:

- Manual: **+3/+3 cm/s**.
- Left rotation: **−2/+2 cm/s**.
- Right rotation: **+2/−2 cm/s**.
- Turn timeout: approximately **1 second**, then brake/latch.

Select mode first, then send **R → G**. Mode selection revokes rearming. `G` alone after stopping does not restart.

Boot and fault recovery remain stopped. USB loss, prolonged invalid IMU data and motor faults latch off. Line-loss stopping applies to line-following mode.

These controls support testing. Final autonomous driving follows Buddy 3’s black-line decisions.

Serial stop is not a hardware emergency stop. Keep the physical power cutoff accessible. Cooperative timing does not guarantee a hard real-time stop.

## 13. Verified software checkpoint

The following passed at the 22 October 2026 checkpoint:

| Check | Result |
|---|---|
| Authoritative IMU/calibration/recovery suite | Pass |
| Native CTest | 1/1 pass |
| Median window/lifecycle tests | Pass |
| Motion classification tests | Pass |
| Possible-impact synthetic tests | Pass |
| Encoder yaw-rate tests | Pass |
| Integration mode/control/safety tests | Pass |
| Scheduler tests | Pass |
| Buddy 2 motion simulation | 39 checks, 0 failures |
| Buddy 3 line simulation | 5 checks, 0 failures |
| CSV schemas and argument counts | Pass |
| Standalone firmware build | Pass |
| Integration firmware build | Pass |

`tests/test_imu.c` is authoritative. The former integration IMU test copy is a compatibility include; median tests reuse authoritative fixtures. Calibration expectations were reconciled without removing functional checks or weakening tolerances.

**No remaining test/build failures were reported.** Two existing Buddy 3 unsigned-comparison warnings remain.

Buddy 2/3 were byte-for-byte unchanged during checkpoint verification. No algorithms, thresholds, calibration, motor tuning, controls or safety behaviour changed in that verification pass.

Software checks do not establish physical accuracy or safety. No flashing, motor operation, staging, commit or push occurred during that software-only checkpoint.

## 14. Hardware observations

Observed during selected tests:

- Level baseline near zero after mounting calibration.
- Correct stationary left/right tilt labels.
- Unassisted single-wheel hump crossings on both sides.
- Return to side-level after crossings.
- Forward/stopped labels during flat driving.
- Correct left/right turn direction and yaw-rate sign.
- Start/stop/rearm and timed turn behaviour.
- `G` alone after stopping did not restart.

These observations do not establish every fault path or operating condition.

### Stationary calibration checks

After acceleration calibration and mounting-offset adjustment:

| Measurement | Result |
|---|---:|
| Complete valid level samples | 693 |
| Duration | 13.84 s |
| Mean filtered pitch | −0.23° |
| Mean filtered roll | −0.33° |
| Mean corrected magnitude | 0.992 g |
| Reported errors | 0 |
| Terrain/side labels | FLAT / SIDE_LEVEL |

Separate known **2.5 cm book** tests:

| Position | Before acceleration correction | After correction |
|---|---:|---:|
| Left wheel raised | 3.00 cm | 2.74 cm |
| Right wheel raised | 2.67 cm | 2.63 cm |

These are stationary tests and must not be substituted for moving-hump ground truth.

### Flat driving

Two reviewed slow runs:

| Measurement | Run 1 | Run 2 |
|---|---:|---:|
| Running duration | ~5.3 s | ~5.6 s |
| Encoder distance increase | ~15.1 cm | ~15.9 cm |
| Mean left speed | 2.89 cm/s | 2.86 cm/s |
| Mean right speed | 2.74 cm/s | 2.79 cm/s |

No false tilt/bump flags or additional IMU errors were observed within those recordings. A constant cumulative error counter does not mean all earlier sessions were error-free.

A later motion/impact/yaw check retained **619 complete rows**, excluding two incomplete rows:

- Valid IMU data in all complete rows.
- Zero reported IMU errors/skipped updates.
- STOPPED → FORWARD → STOPPED.
- Mean running yaw estimate approximately **−0.10°/s**.
- No possible-impact events.

This was a behavioural check, not yaw-rate accuracy validation.

## 15. Latest six moving-hump trials

### Setup

- Same hump and crossing direction.
- Measured peak surface height: **2.2–2.3 cm**.
- Three left-wheel and three right-wheel crossings.
- Manual target: **3.0 cm/s per wheel**.
- Unassisted movement.
- One wheel crosses the hump while the other remains on the lower surface.
- A false estimator-validity flag means unconfirmed.

### Results

| Trial | Stable estimate, cm | Median estimate, cm | Side-event duration |
|---|---:|---:|---:|
| Left 1 | 2.272 | 2.387 | 2.36 s |
| Right 1 | 2.322 | 2.427 | 2.64 s |
| Left 2 | 2.420 | 2.510 | 2.38 s |
| Right 2 | 2.235 | 2.319 | 2.42 s |
| Left 3 | Unconfirmed | 2.500 | 2.46 s |
| Right 3 | 2.273 | 2.339 | 2.52 s |

Summary:

| Measurement | Result |
|---|---:|
| Correct crossing-side labels | 6/6 |
| Median confirmation | 6/6 |
| Stable confirmation | 5/6 |
| Median range | 2.319–2.510 cm |
| Mean left median estimate | 2.466 cm |
| Mean right median estimate | 2.362 cm |
| Overall mean median estimate | 2.414 cm |

All six crossings returned to FLAT / SIDE_LEVEL.

Analysis retained **3,913 complete rows** and excluded **12 malformed numeric fragments**. No IMU errors or skipped updates were reported within these captures.

No possible-impact pulses appeared in the reviewed capture rows. Cumulative impact counts increased between some captures; this does not establish zero false detections throughout the whole session.

### Interpretation

The overall mean estimate was approximately **0.114–0.214 cm above** the measured 2.2–2.3 cm range.

This is limited reference-height testing on **one hump at one target speed**. It does not establish general accuracy across other heights, shapes, surfaces or speeds.

The median estimator confirmed every recorded crossing. The stable estimator missed one moving peak. Neither result proves that all future humps will be detected or measured accurately.

## 16. Timed-turn observations

| Measurement | Left test | Right test |
|---|---:|---:|
| Complete running rows | 50 | 50 |
| TURNING label rows | 24 | 42 |
| UNKNOWN rows | 25 | 6 |
| STOPPED rows | 1 | 2 |
| Mean running yaw estimate | +10.38°/s | −10.60°/s |

Both rotations were physically confirmed by the operator, timed out after approximately one second and ended stopped.

No independent reference angle/time measurement was made, so numerical yaw accuracy remains unknown.

### Why UNKNOWN appeared

The left turn briefly fell below the **1.0 cm/s turn-holding threshold**. Re-entry then required **1.5 cm/s** evidence for **100 ms**.

Opposing low speeds can also cancel in their mean without meeting turn thresholds.

An investigation retained 697 complete rows. Running observations contained:

- Left: 41 distinct encoder timestamps in 50 rows.
- Right: 39 distinct timestamps in 50 rows.

Duplicates were not treated as fresh decisions. Logged sample ages were approximately 1.55–21.44 ms, below the 100 ms stale limit.

Not every internal update was recorded, preventing exact reconstruction of all transitions. Thresholds were preserved; UNKNOWN remains preferable to an unsupported label.

## 17. Remaining work and KIV items

### Current position

Prototype coding and the recorded software checkpoint are complete. No extra feature development or repeated testing is currently scheduled.

Save the verified work using GitHub Desktop if it has not already been committed. Do not claim a commit or push without confirmation.

### KIV

- Final µT-Kernel and team-controller integration.
- Terrain analysis report.
- Professor clarification on expected height accuracy and encoder-based turn rate.
- Further possible-impact/collision validation.
- Team reconciliation of 11.3/12.2 cm geometry.

### Unresolved limitations

- Broad moving-height accuracy is not established.
- Roll-only height does not cover general full-width obstacles.
- Collision discrimination is unvalidated.
- Numerical encoder yaw accuracy is unvalidated.
- Slow turns can legitimately remain UNKNOWN.
- Not every physical safety/fault path has been tested.
- Reverse classification has synthetic coverage but has not been established by the recorded hardware checks.

These are limitations to disclose, not claims that every additional experiment is required by the professor.

## 18. Report notes for later

The report has not yet been written. When resumed, include:

1. **Hardware and scope:** LSM303D identity, no gyroscope, wiring and chassis supports.
2. **Architecture:** module ownership, validity/timestamps, error recovery and standalone versus RTOS integration.
3. **Calibration:** coefficients, imperfect alignment, mounting offsets and corrected geometry.
4. **Algorithms:** tilt hysteresis, side events, stable/median estimators, motion classification, possible impact and encoder yaw.
5. **Evidence:** stationary book tests, flat driving, six measured-hump crossings and timed turns.
6. **Software verification:** host tests and builds, clearly separated from hardware evidence.
7. **Limitations:** small test set, estimate bias, unvalidated collision/yaw accuracy, UNKNOWN periods and geometry discrepancy.
8. **Remaining integration:** state exactly what is deferred.

Preserve original serial captures and document excluded rows. Do not invent missing raw evidence or describe exploratory replay as independent measurement.

## 19. GitHub Desktop checkpoint

1. Select the correct repository and branch.
2. Review the changed files.
3. Include intended Buddy 4 and integration-test changes.
4. Exclude generated builds, binaries, credentials and unrelated teammate work.
5. Commit after verification.
6. Push when ready.

Suggested message:

```text
Checkpoint Buddy 4 prototype, verified tests and hardware observations
```

Commit/push status must be checked separately.

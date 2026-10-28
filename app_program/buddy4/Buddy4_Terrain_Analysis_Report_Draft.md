# Buddy 4 Terrain Analysis Report Draft

Project: INF2004 Autonomous PicoCar  
Evidence checkpoint: 28 October 2026  
Status: Working draft; final integration and validation results remain pending.

## 1. Purpose and scope

Buddy 4 implements IMU acquisition and calibration, tilt and terrain detection, preliminary hump-height estimation, motion classification, possible-impact logging, and estimated turn rate. The implementation provides reusable processing modules and separate standalone and driving-test applications.

The confirmed course arrangement raises one wheel on a ramp while the opposite wheel remains on the lower surface. The professor revised the final ramp to approximately the smaller practice ramp height and permitted disregarding the higher practice ramp. The smaller ramp used in the latest tests was measured by the operator as 1.0 cm. This measured value was used as a reference, not embedded in the estimation algorithm.

## 2. Hardware and processing architecture

The robot uses a Raspberry Pi Pico W and an LSM303D accelerometer/magnetometer. Initial identification established address 0x1D and WHO_AM_I value 0x49. The device has no gyroscope. I2C0 uses GP16 for SDA and GP17 for SCL, with a 3.3 V supply and common ground.

The accelerometer operates at 100 Hz with a +/-2 g range; application processing runs approximately every 20 ms. Signed 16-bit little-endian readings are converted using 0.000061 g/count. Sensor axes are preserved rather than reassigned from printed board arrows.

Buddy 2 supplies motor control and encoder data; Buddy 3 supplies line-following decisions. Buddy 4 exposes processing outputs. The integration_test application combines these modules under a cooperative Pico SDK scheduler. This test application is distinct from the final micro-T-Kernel application, which remains pending. Manual ramp tests bypass line-following steering and therefore do not demonstrate final black-line course operation.

## 3. Calibration

Each acceleration axis is corrected as follows:

    corrected_g = (uncorrected_g - bias_g) * scale

| Axis | Bias, g | Scale |
|---|---:|---:|
| X | 0.03799 | 1.01301 |
| Y | 0.03790 | 1.03419 |
| Z | 0.08411 | 1.01223 |

These are preliminary six-position estimates because orientations were imperfectly aligned. Raw counts and uncorrected values remain available in standalone logging. Corrected acceleration drives angle, magnitude and transient processing.

Current mounting offsets are 0.97 degrees for pitch and 0.61 degrees for roll, with positive signs for both channels. There is no automatic startup zeroing: the system must not redefine a starting slope as level.

The roll offset was previously -0.25 degrees. Two flat recordings averaged +0.913 and +0.800 degrees. The code subtracts the mounting offset, so the proposed correction was:

    -0.25 + (0.913 + 0.800) / 2 = 0.6065 degrees

This was rounded to 0.61 degrees, rebuilt, flashed by the operator, and checked in stationary and moving tests. Supporting-surface levelness was assumed; its uncertainty was not quantified.

Wheel centre spacing is measured as 11.3 cm. Earlier values referred to inner tyre gap, outer width, or a provisional midpoint. Buddy 2 retains 12.2 cm in its geometry configuration; team reconciliation is pending.

## 4. Tilt and hump processing

Pitch and roll are filtered using alpha 0.15. Pitch enters uphill/downhill at +/-6 degrees and returns toward flat at +/-3 degrees. Roll enters a side state at +/-4 degrees and returns toward side-level at +/-2 degrees. Negative roll denotes left-wheel-up; positive roll denotes right-wheel-up.

These labels describe chassis orientation. The accelerometer cannot independently distinguish one wheel rising from the opposite wheel dropping. Caster contact and combined pitch/roll also affect the chassis.

The preliminary geometric model is:

    wheel_height_difference_cm = 11.3 * sin(abs(filtered_roll))

Angles must be converted to radians for the sine calculation. The result estimates wheel-height difference, not an independently observed surface height. It cannot recover general obstacle height when both wheels rise equally.

Two event-peak estimators run alongside each other. The original stable estimator requires at least 200 ms of fresh observations with absolute-roll variation no greater than 1 degree and adjacent gaps no greater than 40 ms. The experimental estimator retains the highest accepted median of 11 filtered absolute-roll samples spanning 190-210 ms, again requiring strictly increasing timestamps and gaps no greater than 40 ms.

Validity flags distinguish an unconfirmed estimate from a measured zero. Placement movement may create historical peaks much larger than the settled position, so held-position analysis uses settled instantaneous readings rather than stored event maxima.

## 5. Experimental results

### 5.1 Reference and procedure

The latest ramp reference height was reported as 1.00 cm. Manual driving targets were 3.0 cm/s per wheel. The ramp was centred in the selected tyre path, the other wheel stayed on flat ground, and valid moving trials were unassisted. Stationary measurements used motor power off and no hand support during recording.

Incomplete CSV fragments were excluded rather than interpreted with shifted columns. Complete logged rows are observations, not necessarily independent module updates. Numerical summaries below refer to the previously reviewed captures; original files should accompany the submission where appropriate.

### 5.2 Before the roll-offset correction

| Trial | Stable peak, cm | Median peak, cm |
|---|---:|---:|
| Left 2 | 0.608 | 0.864 |
| Left 3 | 0.785 | 0.840 |
| Right 1 | 1.091 | 1.245 |
| Right 2 | 1.226 | 1.314 |
| Right 3 | 1.135 | 1.278 |

Left 1 was excluded because the operator reported the ramp was at the tyre edge rather than centred in its path. It is not counted as a detection failure.

The valid median averages were 0.852 cm left and 1.279 cm right. Together with the positive flat roll baseline, this suggested investigating mounting offset before changing height scale or detection thresholds.

### 5.3 Stationary checks after correction

| Position | Complete valid rows | Span, s | Mean roll, degrees | Mean height, cm |
|---|---:|---:|---:|---:|
| Flat | 318 | 6.42 | -0.142 | 0.028 |
| Left held at 1 cm | 373 | 7.48 | -4.977 | 0.980 |
| Right held at 1 cm | 574 | 11.46 | +4.916 | 0.968 |

All complete rows were IMU-valid with zero reported cumulative IMU errors. The left/right held estimates differed by approximately 0.012 cm. The correction substantially reduced the earlier observed asymmetry in this setup.

![Stationary height estimates before and after roll correction](evidence/2026-10-28/charts/stationary_height_comparison.png)

**Figure 1.** Settled instantaneous estimates before and after the mounting correction. Black segments show nominal references: zero for flat and 1 cm for raised-wheel positions. Recorded intervals are separate observations; surface levelness and reference uncertainty were not quantified.

### 5.4 Moving checks after correction

| Side | Complete valid rows | Stable peak, cm | Median peak, cm | Event span, s |
|---|---:|---:|---:|---:|
| Left | 458 | 0.994 | 1.049 | 1.280 |
| Right | 394 | 0.989 | 1.065 | 1.360 |

Both runs identified the correct side, confirmed both estimators, returned to side-level and stopped on the serial command. The median errors relative to nominal 1.00 cm were +0.049 cm and +0.065 cm, or +4.9% and +6.5%. Stable errors were -0.006 cm and -0.011 cm.

These are two checks, not a general accuracy specification or a controlled statistical comparison with the earlier runs. Reference-height uncertainty was not quantified. No IMU errors were reported in the complete rows. No possible-impact pulse appeared in either capture; a pre-existing cumulative impact count does not establish an entirely false-trigger-free session.

![Moving height estimates before and after roll correction](evidence/2026-10-28/charts/moving_height_comparison.png)

**Figure 2.** Recorded moving event peaks: squares show the stable estimator and circles show the median estimator. Black segments indicate the nominal 1 cm reference. The before-correction groups contain two valid left and three right runs; after-correction groups contain only one run per side. Invalid Left 1 is excluded. This is a limited observational comparison, not a controlled accuracy study.

### 5.5 Historical evidence

Before the latest mounting correction, six crossings of a measured 2.2-2.3 cm hump produced median estimates from 2.319 to 2.510 cm, with an overall mean of 2.414 cm. Median confirmation was 6/6 and stable confirmation was 5/6. These results belong to an earlier calibration checkpoint and must not be pooled with the latest 1 cm trials as one accuracy dataset.

## 6. Motion, impact and turn-rate processing

Motion classification uses signed encoder speeds and reports STOPPED, FORWARD, REVERSE, TURNING_LEFT, TURNING_RIGHT or UNKNOWN. Selected hardware checks demonstrated forward/stopped transitions and left/right turn signs. Reverse hardware behaviour has not been established by the recorded checks.

Low opposing speeds can cancel in the mean while failing turn thresholds. UNKNOWN periods during slow turns are therefore possible and were observed. Repeated encoder timestamps must not advance confirmation.

The possible-impact detector compares acceleration magnitude with a tracked nonzero baseline. Provisional criteria are at least 0.30 g deviation, two or more fresh samples spanning 20 ms, gaps no greater than 40 ms, and a 500 ms cooldown plus two calm samples no greater than 0.10 g for rearming. It is separate from bump_event and does not control motors or safety stops. Ramps, braking, vibration and handling may resemble impacts; collision discrimination remains unvalidated.

Encoder yaw is estimated as (right speed - left speed) / 11.3 in rad/s, then converted to degrees/s. Positive denotes left/counterclockwise rotation. Selected timed-turn recordings averaged +10.38 degrees/s left and -10.60 degrees/s right, including startup. No independent reference angle/time measurement was made. Numerical accuracy and professor acceptance remain pending.

Buddy 5 ultrasonic readings are planned as obstacle-proximity context, not proof of contact. Combined collision-context testing awaits that integration. Turn-rate validation itself does not require ultrasonic data.

## 7. Software verification and remaining work

The reported offset-change checkpoint passed host regressions, CSV checks for 33 standalone and 69 integration columns, native CTest 1/1, and both firmware builds. Two pre-existing Buddy 3 warnings remained. These are reported test results, not fresh runs performed while drafting this report. Software checks do not establish hardware accuracy or safety.

Remaining work includes Buddy 5 integration, final micro-T-Kernel conversion and course checks, collision discrimination and independent turn-rate validation, geometry reconciliation, and final report completion.

The agreed plan defers Tribunal 1 implementation until a working integrated baseline is measured. The team must then document field ranges and minimum-bit representations, safely synchronise shared packed data, use compact binary MQTT payloads, and compare measured RAM, relevant message sizes, functionality and performance before/after. Tribunal 1 compliance is not yet established.

## 8. Draft conclusion

The current prototype acquires calibrated acceleration and provides terrain, motion and event outputs. Correcting the fixed mounting roll offset reduced the observed left/right height asymmetry in limited 1 cm checks. Both corrected moving trials confirmed a single-wheel event and produced estimates close to the measured nominal height.

The evidence supports a working preliminary single-side height method for the tested setup. It does not establish general height accuracy, reliable collision classification, numerical yaw-rate accuracy or final integrated course performance. Those claims remain outside the demonstrated results.

## Evidence files and submission preparation

Use the Buddy 4 README as the detailed checkpoint index. Preserve original captures and document cleaning; do not manufacture raw data from these summaries. Replace this draft's pending items only when new evidence exists. Add final course/integration results, Tribunal 1 measurements and any required team report formatting before submission.


### Recording index for the latest height results

The [evidence index](evidence/2026-10-28/README.md) explains row selection and exclusions. The [manifest](evidence/2026-10-28/manifest.json) records original-file SHA-256 hashes, calibration versions, accepted row counts and event summaries. Raw files are preserved unchanged.

| Report observation | Original recording |
|---|---|
| Invalid Left 1, excluded | [Left 1](evidence/2026-10-28/raw/left1_invalid.txt) |
| Moving Left 2 before correction | [Left 2](evidence/2026-10-28/raw/left2_before.txt) |
| Moving Left 3 before correction | [Left 3](evidence/2026-10-28/raw/left3_before.txt) |
| Moving Right 1 before correction | [Right 1](evidence/2026-10-28/raw/right1_before.txt) |
| Moving Right 2 before correction | [Right 2](evidence/2026-10-28/raw/right2_before.txt) |
| Moving Right 3 before correction | [Right 3](evidence/2026-10-28/raw/right3_before.txt) |
| Initial flat baseline before correction | [Flat before](evidence/2026-10-28/raw/flat_before.txt) |
| Left held before correction | [Left held before](evidence/2026-10-28/raw/left_held_before.txt) |
| Right held before correction | [Right held before](evidence/2026-10-28/raw/right_held_before.txt) |
| Return to flat before correction | [Flat return](evidence/2026-10-28/raw/flat_return_before.txt) |
| Flat after correction | [Flat after](evidence/2026-10-28/raw/flat_after.txt) |
| Left held after correction | [Left held after](evidence/2026-10-28/raw/left_held_after.txt) |
| Right held after correction | [Right held after](evidence/2026-10-28/raw/right_held_after.txt) |
| Moving left after correction | [Left moving after](evidence/2026-10-28/raw/left_moving_after.txt) |
| Moving right after correction | [Right moving after](evidence/2026-10-28/raw/right_moving_after.txt) |

Historical larger-hump, book and turn results remain checkpoint summaries in this draft. Their original captures and software test artifacts are not indexed in this latest-height bundle; link them separately before claiming complete evidence traceability for the final submission. The draft still needs final report-rubric review, technical references, pending validation results and Tribunal 1 evidence.

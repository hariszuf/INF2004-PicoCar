# Buddy 4 possible-impact vector baseline, 2 November 2026

Software improvement prepared for controlled hardware validation. Collision
discrimination is **not validated**. No firmware flashing, motor operation,
automatic stopping or Git commit/push/merge was performed.

## Investigation and decision

The original observer used `abs(norm(corrected_xyz) - tracked_magnitude)`.
At rest, gravity contributes approximately 1 g. A perpendicular 0.45 g
disturbance changes the magnitude by `sqrt(1 + 0.45^2) - 1 = 0.0966 g`,
so it cannot reach the unchanged 0.30 g trigger. This is a concrete directional
limitation, independently of tuning. The new regressions exercise that example
in both signs, multiple axes, a diagonal and a tilted mounting.

The operator's maximum old deviations were 0.0361 g for gentle manual contact,
0.0397 g for controlled manual movement and 0.1156 g for motor-driven soft-obstacle
contact, all below 0.30 g. These summaries establish why the old threshold did
not fire; without timestamped corrected XYZ they cannot establish the contact's
direction, duration, new residual or whether this revision would detect it.
Low-amplitude, slow or unsampled contact can remain undetected.

The SDK driving client and isolated Buddy 4 MTK owner already feed the same
corrected XYZ through `FRESH && valid && acceleration_valid`. Tilt suitability
is deliberately not required for non-clipped dynamic acceleration. Nonfinite
and raw-clipped input must remain invalid/indeterminate for impact assessment.
Calibration, mounting offsets, IMU transient/bump detection, terrain, stable and
median height processing and encoder observers are untouched. The standalone
SDK IMU monitor does **not** call `buddy4_impact_update`; use the SDK driving UF2
below for this experiment, not `build-buddy4/buddy4.uf2`.

## Algorithm and interfaces

Maintain a three-component reference `b`. Prime `b = a` on the first usable
sample; that sample cannot trigger. For subsequent usable samples:

```
deviation_g = norm(a - b)              # reference BEFORE this sample's update
if deviation_g < 0.30 g:
    b += dt / (500 ms + dt) * (a - b)  # each axis; otherwise freeze all axes
```

Thresholds and event state machine are unchanged: trigger >=0.30 g, release
<=0.10 g, at least two consecutive qualifying fresh samples spanning >=20 ms,
gaps/age <=40 ms, 500 ms cooldown, and two calm samples to rearm. Invalid input,
duplicate/backward/future/stale timestamps, clock reversal and excessive gaps
clear pending evidence and require baseline priming. Counter saturation, last
event time, pulse/getter semantics and history/cooldown across faults are retained.
Vector-subtraction overflow is rejected as nonfinite too.

The low-pass vector follows slow orientation and gentle motion rather than
assuming a fixed level mounting. It is an approximation containing gravity,
DC offsets and slow acceleration, not true gravity subtraction. A fixed gravity
vector would spuriously react to subsequent tilt. Adjacent-sample vector changes
(discrete jerk) emphasize onset but lose a held disturbance after its first
sample, conflicting with the existing multi-sample confirmation. The tracked
vector uses fixed storage, no gyro, allocation or new integration inputs.

Function signatures, config fields/defaults and output field names are unchanged.
The detector state replaces one scalar baseline with three floats, so rebuild
consumers. `magnitude_g` remains `norm(a)`; `baseline_g` is now `norm(b)` and
`deviation_g` is `norm(a-b)`, **not** `abs(magnitude_g-baseline_g)`. The existing
SDK and MTK CSV schemas/column counts remain unchanged. MTK's existing
`impact_deviation_g` snapshot carries the new residual automatically.

## Sampling and limits

Source configuration remains LSM303D 100 Hz ODR, 50 Hz bandwidth, +/-2 g;
SDK/MTK process at approximately 50 Hz (20 ms). There is no FIFO capture here.
A disturbance must survive into at least two usable observations spanning
20 ms. Short pulses may be missed between polls or rejected as isolated spikes;
50 Hz bandwidth is above the processing Nyquist frequency, so vibration/aliasing
remains a limitation. Poll gaps and dropped/invalid input cannot confirm events.
Changing acquisition timing/bandwidth is deferred until labelled XYZ recordings
show whether missed pulses rather than signal direction dominate.

The 500 ms baseline time constant gives a gradual tilt residual roughly
proportional to rotation rate; it cannot remove arbitrary rotation. Abrupt tilt,
hard acceleration/braking, abrupt ramp edges or sustained vibration above the
same envelope may still qualify. A sustained elevated disturbance freezes the
reference and cannot repeatedly trigger; rearming requires calm relative to that
reference as well as cooldown. A new static orientation after abrupt handling
may therefore need recovery/return toward the old reference before rearming.
Contact during startup or recovery may be absorbed into priming. No accelerometer
algorithm can identify every collision or distinguish physically identical
acceleration traces. Every event is only **POSSIBLE_IMPACT**.

## Regression coverage

The original integration host runner passed before edits. Expanded impact tests
cover stationary gravity/XYZ noise; gradual pitch/roll, ramps and DC drift;
representative gentle forward start/coast/braking/reverse and turning profiles;
signed horizontal onset/deceleration, different mountings and retained vertical
detection; isolated/alternating spikes; sustained disturbances; subthreshold
changes; 5/20/40 ms observation timing and exact confirmation/gap boundaries;
duplicate/stale/future/backward data; nonfinite/overflow; I2C failure/short
transfers at every poll transfer; raw clipping; recovery; pulse/history,
saturated count, calm rearming, cooldown and repeated events.

The production-calibrated fake-I2C fault matrix runs for both vertical and
horizontal disturbances, before and after an event. The actual MTK owner test
also confirms horizontal input through its device bridge and published snapshot,
then checks clipping, retained history, cooldown and repeated events. Synthetic
ordinary-motion envelopes are test inputs, not measured vehicle operating limits.
An explicit abrupt-tilt test documents the remaining ambiguity.

## Verification and artifacts

All checks passed on 2 November 2026:

- Original `integration_test/test.ps1` before edits: PASS, including impact.
- Updated `integration_test/test.ps1`: PASS, including 42 motion checks,
  30 terrain checks, line/drive/scheduler, IMU/calibration/quality, stable/median,
  motion classification, expanded impact and encoder yaw tests.
- Updated `buddy4_mtk/test.ps1`: PASS, including startup pin protection,
  console/peak/quality formatting, actual task/bridge horizontal regression,
  native I2C and real kernel wait regressions, plus the integration host runner.
- CSV checks: SDK standalone 39 columns; SDK driving 79 columns; MTK quality
  CSV 17 fields, actual formatter and quality masking: PASS.
- Saved original magnitude-only `.c/.h` compiled against the expanded impact
  tests: expected FAIL at `horizontal_contact`, missing the event after two
  0.45 g perpendicular samples. Updated implementation: PASS.
- SDK 2.3.1 Pico W integration build: PASS. SDK standalone sanity build: PASS
  (the latter remains an IMU monitor, not the possible-impact test application).
- Isolated Buddy 4-only SMP=0 USB CDC firmware build: PASS.
- `git diff --check`: PASS. Existing Buddy 3 unsigned-comparison warnings and
  MTK newlib `_close/_lseek/_read/_write` stub linker warnings remain.

SDK build command (local Pico CMake/Ninja and Arm toolchain on PATH):
`cmake --build app_program/integration_test/build --parallel 4`.
MTK build command: `powershell -NoProfile -ExecutionPolicy Bypass -File app_program/buddy4_mtk/build.ps1`.

**SDK UF2 for the next controlled contact test:**
`C:/Users/yongk/OneDrive/Documents/GitHub/INF2004-PicoCar/app_program/integration_test/build/buddy234_driving_test.uf2`

- Size: 110,080 bytes.
- SHA-256: `875a98c1e26a70dc7f1a47960f970bfb2cc3d88d044d3fd34e3ed50b7c1662d4`.
- ELF text/data/bss: 58,932 / 0 / 3,620 bytes.

Compatible isolated MTK trial UF2:
`C:/Users/yongk/OneDrive/Documents/GitHub/INF2004-PicoCar/app_program/buddy4_mtk/build/mtk3pico_smp0_usb_cdc.uf2`

- Size: 173,568 bytes.
- SHA-256: `63b907ee14c85faa7bc3fc5202f3b21398afd3feceb51d9968650ef0211a35d4`.
- ELF text/data/bss: 86,120 / 544 / 18,632 bytes.
- Existing console marker remains `B4-MEASUREMENT-QUALITY-I USB-only`; use the
  artifact hash to distinguish this build. No console/schema changes were needed.

## Exact changed files

- `app_program/buddy4/buddy4_impact.c`: tracked vector residual and adaptation.
- `app_program/buddy4/buddy4_impact.h`: vector state and signal semantics.
- `app_program/integration_test/test_impact.c`: vector, ordinary-motion, timing
  and production-calibrated horizontal/vertical fault regressions.
- `app_program/buddy4_mtk/overlay/app_program/buddy4/tests/test_mtk_task.c`:
  horizontal input through the actual MTK owner and published snapshot.
- `app_program/buddy4/README.md`: current algorithm and checkpoint link.
- `app_program/buddy4/IMPACT_VECTOR_CHECKPOINT.md`: investigation, results,
  artifacts, limitations and operator procedure.
- `app_program/buddy4_mtk/base_hashes.json`: only the Buddy 4 README fingerprint
  refreshed for the documented edit; all team-source fingerprints unchanged.

The working tree was clean before this task. No Buddy 2/3 source, calibration,
mounting offsets, terrain/height algorithm, integration application, kernel,
driver, motor behaviour or acquisition configuration was modified. Generated
builds and the original-algorithm comparison live in existing ignored build areas.

## Controlled hardware procedure (operator performed)

1. Use the SDK **integration** UF2 identified in the verification section, after
   checking its hash. Retain the prior firmware for comparison. Start with the
   car stationary for at least 2 seconds so the reference settles. Do not handle
   the sensor or change its bracket/calibration during comparisons.
2. Record the complete CSV at its normal cadence, including corrected XYZ,
   raw counts, timestamps, validity/quality/clipping, errors, IMU skipped slots,
   impact magnitude/baseline/deviation, pulse, count and last-event timestamp.
   Label each trial's actual action/contact time independently (operator notes
   or video). Use event-count deltas as well as pulses; missing USB rows can hide
   a one-sample pulse. A cumulative count may predate the trial.
3. Collect stationary noise, ordinary forward/reverse starts, straight driving,
   gentle braking, turns and both directions of normal one-side ramp crossing
   first. Repeat each condition, including starts on a ramp. Check for unwanted
   possible-impact counts and invalid/clipped periods before testing contact.
4. Repeat the prior gentle manual and low-speed soft-obstacle conditions under
   the same controlled setup, ideally at least five trials per condition. Use a
   soft compliant obstacle and the existing operator-controlled stop immediately
   after contact; no automatic motor stop was added. Avoid deliberate hard
   crashes or striking the sensor. This session does not flash or operate motors.
5. Leave at least 2 seconds of calm between trials. Compare labelled contacts
   with events within the relevant sampling window, and non-contact trials with
   unwanted counts. Report hits/misses and false positives by condition rather
   than declaring collision detection validated from a single successful trial.
6. For a miss, inspect whether the vector residual reached 0.30 g in consecutive
   fresh usable samples spanning 20 ms, or whether duration, clipping, poll gaps,
   baseline priming or cooldown explains it. Inspect XYZ directly; total-magnitude
   deviation alone cannot answer this. Preserve misses and false-positive traces
   before considering any further threshold or timing change.

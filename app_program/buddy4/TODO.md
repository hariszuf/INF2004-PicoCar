# Buddy 4 main checklist

Updated: 30 October 2026. This is the authoritative Buddy 4 remaining-work list;
use the READMEs for implementation/build details. Completed means the stated
checkpoint exists, not that all assessed requirements or conditions are validated.
Final team MTK integration is paused while teammates finish their migrations.

**How to use:** tick a task only when its stated checkpoint is met. Checked
software/build tasks do not mean physical validation is complete. Unchecked
tasks are grouped by what you can do now, what needs the robot, what needs
teammates, and what is deferred. Status, dependencies and evidence are retained
under each checkbox.

Collision detection remains required; the existing detector reports preliminary
**possible-impact** observations, not confirmed collisions. Separate shock
detection, dedicated vibration analysis and both-wheel/full-width height
estimation are outside the agreed scope. The course scope is a one-side ramp;
the higher practice ramp may be disregarded under the professor's announcement.

## Current measurement-validity fault (30 October 2026)

- [x] Reproduce successful-transfer endpoint XYZ in production buddy4_poll:
  -35.67/-36.14 degrees, 6.66 cm, 3.6249 g and misleading stable/median peaks.
- [x] Separate acquisition from clipping/finite acceleration and gravity-like tilt;
  abort unsuitable pending/current events, retain completed results, preserve
  non-clipped impact behaviour and expose clipped-impact uncertainty.
- [x] Add raw/quality/configuration diagnostics, regression coverage and distinct
  USB-only diagnostic firmware; run host/schema checks and affected builds.
- [ ] Hardware confirmation: ONE 25-second level/right-tilt/level/left-tilt/level
  capture with motor power disconnected. Do not repeat the full ramp batch.
  Hardware raw values and root cause remain unknown; do not tune mounting offsets
  from the suspicious derived values or treat fresh/zero errors as tilt validity.

[Fault evidence, quality contract, build paths and capture interpretation](MEASUREMENT_QUALITY_CHECKPOINT.md).
This current fault must be confirmed before relying on further height trials;
older test/checklist entries retain their original scope and uncertainty.

## 1. Completed checkpoints—with clear limits

- [x] **Prototype modules and SDK applications**
  - **Status:** Complete as prototypes: LSM303D identification (0x1D/0x49, GP16/17, no gyro), acquisition/recovery, preliminary calibration, tilt/side labels, one-side height, stable/median peaks, encoder motion/yaw and possible-impact detector; SDK standalone/driving tests exist.
  - **Needs:** Final hardware and team qualification below.
  - **Evidence / finish condition:** Existing source, tests, schemas and selected SDK recordings establish this checkpoint only; general accuracy remains open.

- [x] **Isolated MTK bridge and DIAG-E**
  - **Status:** Bridge, protected snapshots, motor-pin protection, 20 ms release, timeout/IRQ checks and USB-only logging exist. One clean Buddy 4-only recording: 40 displayed blocks, zero cumulative I2C errors/skips, 20 ms displayed poll intervals.
  - **Needs:** Longer runs, recovery and team coexistence remain open.
  - **Evidence / finish condition:** [DIAG-E capture and limits](../buddy4_mtk/evidence/2026-10-28/README.md) close this bounded checkpoint; later handling/tilting does not validate height or collisions.

- [x] **Tilt/peak readable output (F through H)**
  - **Status:** H built; NOW and saved B4 RESULT distinguish current state from preliminary completed side-event peaks. Every saved result is explicitly labelled observational tilt, not a confirmed hump crossing. Hardware tests are not established by supplied recordings.
  - **Needs:** Robot and labelled H captures (§3).
  - **Evidence / finish condition:** Build is a software checkpoint only; H needs its own hardware recordings. A 1 Hz latest-event snapshot is not a lossless event queue.


## 2. Do now without robot

- [x] **Review collision-detector software/tests**
  - **Status:** Software checkpoint complete, 29 October 2026: existing tests run first, MTK interrupted-confirmation defect reproduced and corrected, expanded regressions and affected firmware build pass. Physical collision validation remains pending.
  - **Needs:** Existing corrected-acceleration detector; physical event meaning/discrimination still needs independent labels.
  - **Evidence / finish condition:** Software evidence and limits below. Existing detector, calibration and provisional thresholds retained; no separate detector added. Synthetic results cannot close physical collision validation.

- [x] **Review encoder motion/turn-rate software and input contract**
  - **Status:** Software checkpoint complete, 29 October 2026: existing tests ran first; classifier, yaw estimator, isolated mailbox and consumer expiry passed expanded synthetic regressions. No runtime defect was reproduced, so algorithms, thresholds and the 11.3 cm geometry are unchanged.
  - **Needs:** A native owner must provide coherent fresh signed wheel speeds with real acquisition time and validity; Buddy 2's current public status lacks the latter two fields. No adapter or live task connection was added.
  - **Evidence / finish condition:** [Motion/yaw software checkpoint and producer contract](MOTION_YAW_SOFTWARE_CHECKPOINT.md). This closes software/interface review only; numerical yaw accuracy, actual-motion validation, the 11.3 versus 12.2 cm geometry decision and professor acceptance remain open.

- [x] **Prepare Tribunal 1 field/range/bit table**
  - **Status:** Preparation complete: [Buddy 4 Tribunal 1 field/range/bit representation table](TRIBUNAL1_FIELD_REPRESENTATION.md). Analysis only; no runtime type, layout or telemetry change. Tribunal implementation/compliance remains deferred (§5).
  - **Needs:** Obtain and cross-check the original Change Requirement 1 document, which was not found in the current tree or history; resolve the listed Buddy 1/2/dashboard decisions.
  - **Evidence / finish condition:** The linked inventory records source/current types, enforced versus proposed ranges, precision/signedness/invalid codes, calculated logical bits, overflow/wrap behaviour, ownership/concurrency, padding limits and pending before/after measurements.

- [ ] **Review existing evidence**
  - **Status:** Raw captures, charts and a report draft exist; review can proceed without writing the final report.
  - **Needs:** Linked evidence and firmware/configuration labels.
  - **Evidence / finish condition:** Traceable inventory separating SDK, synthetic and MTK results; preserve raw data, exclusions and reference uncertainty. Exclude misaligned Left 1 from valid height results; do not pool differing calibrations or count handling as hump proof.


### Tilt/hump/peak-height software evidence, 29 October 2026

- The complete isolated MTK runner passed before edits. The audit confirmed the
  existing pitch/roll signs, calibration order, terrain/side hysteresis and the
  one-side wheel-height calculation (`11.3 cm * sin(abs(filtered roll))`). Stable
  and rolling-median peaks retain their separate validity, require sufficient
  in-event evidence and become saved results only when a side event ends.
- Existing fault tests cover unavailable samples and failed/short I2C transfers.
  A new lifecycle regression covers duplicate and reversed timestamps, missing
  data during an event, invalid completion after interruption, and isolation of a
  new event from an older, larger peak. An MTK snapshot regression confirms that
  a current acquisition failure invalidates current diagnostics without erasing
  the last valid saved result or its completed-event count.
- No calculation, calibration, geometry or threshold defect was reproduced. The
  required gap was in copied readable output: a saved `B4 RESULT` line described
  a completed side-tilt but did not itself say that it was not an independently
  confirmed hump crossing. H adds `OBSERVATIONAL TILT, NOT CONFIRMED HUMP
  CROSSING` to every saved result; output/retention tests enforce that wording and
  the separation between `B4 NOW` diagnostics and saved results.
- The focused IMU/median suite and final `buddy4_mtk/test.ps1` passed, including
  startup/motor-pin protection, console/output checks, native task/device/I2C/
  kernel-wait tests and all integration regressions. `buddy4_mtk/build.ps1`
  rebuilt SMP=0 / USB CDC firmware at
  `../buddy4_mtk/build/mtk3pico_smp0_usb_cdc.uf2`; ELF text/data/bss are
  84,328 / 544 / 18,512 bytes. Embedded marker:
  `B4-TILT-PEAK-POSSIBLE-IMPACT-H USB-only`. Existing two Buddy 3
  unsigned-comparison warnings and four MTK newlib file-I/O stub warnings remain.
- This closes software behaviour only. Height accuracy/repeatability, real hump
  discrimination, mounting/calibration stability, both-side response and physical
  missing-data recovery remain pending labelled robot tests in section 3.


### Collision-detector software evidence, 29 October 2026

- Reviewed `buddy4_impact.c/.h`, SDK `integration_test/main.c` and isolated MTK
  `buddy4_task.c`. Both clients gate corrected acceleration on a fresh, valid IMU
  poll. SDK standalone `buddy4/main.c` does not call this detector; its
  `transient_event` remains a separate observation.
- Existing integration regressions passed before detector/task edits. The first
  MTK runner stopped at its base guard because only the Buddy 4 README fingerprint
  was stale. Refreshed that documentation hash after review; all eight runtime
  base fingerprints matched and remain unchanged. The existing MTK suites then
  passed before the new regression was added.
- Reproduced a false possible-impact confirmation in the actual MTK task loop:
  baseline at 5,000,000 us, first elevated sample at 5,020,000 us, failed release
  wait, then an elevated sample at 5,060,000 us. Pending evidence survived the
  failure and the recovery sample triggered. The release/operating-time failure
  branch now feeds invalid input to the existing detector, preserving event
  history/cooldown. Regressions exercise forced wait release and clock-read
  failure, recovery priming, retained counts/timestamps and cooldown. No defect
  was found in the detector core itself; `buddy4_impact.c/.h` are unchanged.
- Expanded `integration_test/test_impact.c` covers stationary gravity, slow gravity
  rotation and baseline drift, modest synthetic normal-motion variation,
  isolated spikes, positive/negative and tilted qualifying transients, exact
  confirmation/gap/age/cooldown boundaries, duplicate/stale/reversed/future time,
  invalid/nonfinite/missing input, recovery and rearming. It also checks repeated
  getters, one-input-sample pulses, retained event times and saturating counts.
  Twenty-four acquisition fault/recovery cases reuse the authoritative fake-I2C
  fixtures and production calibration: data-not-ready, duplicate/reversed samples,
  not-ready state and failed/short transfers at each of four SDK poll transfers,
  both before the first event and with retained event history.
- Final `buddy4_mtk/test.ps1` passed: startup/motor-pin protection, USB console and
  tilt/peak output, native task/device/I2C/kernel-wait tests and the full existing
  integration host runner (including calibration, tilt/median peaks, motion/yaw,
  scheduler/drive safety, read-only Buddy 2/3 simulations and 33/73-column schemas).
  The task suite also passed after refining its wait fault to forced release
  (`E_RLWAI`). The G monitor output/retention tests then passed, including current
  validity versus saved history, count-triggered reporting after a missed pulse,
  latest-timestamp retention, multi-event gap warning and read-only snapshots.
  `buddy4_mtk/build.ps1` rebuilt SMP=0 / USB CDC firmware successfully:
  `../buddy4_mtk/build/mtk3pico_smp0_usb_cdc.uf2`; ELF text/data/bss are
  84,280 / 544 / 18,512 bytes. Firmware marker:
  `B4-TILT-PEAK-POSSIBLE-IMPACT-G USB-only`. Existing two Buddy 3 unsigned-comparison warnings
  and four MTK newlib file-I/O stub warnings remain.
- Calibration, tilt/peak algorithms, USB-only logging, motor-pin protection and
  Buddy 4-only mode are unchanged. Teammate/live team sources were not edited.
  No flashing, motor operation, commit or push was performed.
- Limits: the magnitude-only signal cannot identify contact or its cause and can
  miss direction changes with unchanged magnitude. Startup/recovery primes from
  its first sample, so a transient already present then can be missed. Slow
  baseline changes are intentionally tracked. Event history is a saturating
  count plus latest timestamp, not a queue; repeated reads can show the same
  pulse, and slow logging can miss pulses. The G MTK readable monitor exposes
  current detector validity plus saved count/latest timestamp and reports count
  changes without depending on pulse capture; it still cannot recover intermediate
  timestamps, waveforms, peak deviation/direction or missed pulses. Synthetic normal motion/fault timing does not establish
  real false-positive/miss rates, bus recovery or physical collision validity.

## 3. Needs robot

- [ ] **Mounting, calibration, tilt and run/recovery checks**
  - **Status:** Preliminary six-position coefficients/fixed offsets exist; course-board bias unresolved. F hardware validation pending.
  - **Needs:** Rigid mounting, verified surface/reference positions and robot operator; repeat relevant checks in final MTK.
  - **Evidence / finish condition:** Labelled H captures of flat/uphill/downhill, both sides, return to level, longer cadence/error/skip monitoring and missing-data recovery. Recheck offsets if mounting moved; do not zero against an unverified board.

- [ ] **Measured hump/peak-height tests**
  - **Status:** Limited earlier one-side results; general accuracy and repeatability unestablished.
  - **Needs:** Measured smaller ramp centred under each tyre; driving firmware for unassisted crossings.
  - **Evidence / finish condition:** Repeated left/right crossings with reference height, stable/median validity, estimates, errors, failures and exclusions; justify chosen estimator. Test nearby heights if available; do not hardcode 1 cm. Handling/manual previews do not close unassisted validation. Disclose caster/tyre, acceleration, combined pitch/roll and board-bias limits.

- [ ] **Physical collision validation**
  - **Status:** Preliminary possible-impact detection only; collision discrimination unvalidated.
  - **Needs:** Agreed event meaning and non-destructive procedure with independent labels; Buddy 5 only if ultrasonic context is chosen.
  - **Evidence / finish condition:** Labelled normal motion, vibration, braking, handling and ramps plus agreed collision scenarios; measure false positives/misses and document limits. Keep bump_event, possible-impact events and confirmed-collision claims distinct. Do not crash the car or strike the sensor; no automatic motor action without explicit team requirement/review.


Use the [friend tilt/peak instructions](TILT_PEAK_TEST.md) and
[results template](TILT_PEAK_RESULTS_TEMPLATE.txt). These separate held/manual
previews from unassisted crossing evidence.

## 4. Needs teammates

- [ ] **Encoder motion/turn-rate integration and physical validation**
  - **Status:** MTK motion/yaw unavailable without a coherent fresh signed-wheel-speed producer; accuracy and professor acceptance remain open.
  - **Needs:** Buddy 2 data/adapter, robot and independent angle/time reference; reconcile Buddy 4's measured 11.3 cm spacing with recorded Buddy 2 value 12.2 cm. Buddy 5 is not required.
  - **Evidence / finish condition:** STOPPED/FORWARD/REVERSE/LEFT/RIGHT checks, slow turns, duplicates/stale handling and referenced yaw errors; retain conservative UNKNOWN. Record agreed geometry and professor acceptance/expected accuracy for encoder-derived rate without a gyro.

- [ ] **Telemetry and optional collision context**
  - **Status:** Reporting/shared semantics pending; ultrasonic fusion unimplemented.
  - **Needs:** Buddy 1 for MQTT; team for units, timestamps, validity, event retention and fault ownership. Buddy 5 only if ultrasonic collision context is chosen.
  - **Evidence / finish condition:** Decoded valid terrain/height/motion/impact/yaw outputs with explicit units and estimated/unconfirmed labels; agreed event-loss semantics. If chosen, show fresh proximity adds context; proximity alone does not prove contact.

- [ ] **Final team MTK integration and course validation**
  - **Status:** Paused; isolated trial is not final team validation and lacks SDK M/L/R/G/S/A/D driving controls.
  - **Needs:** Teammate migrations, Buddy 2/3 adapters, qualified common application and robot/course.
  - **Evidence / finish condition:** Review/merge overlay without overwriting teammates; qualify active tasks, motor operation, telemetry/USB load and ultrasonic timing if active. Preserve bounded USB-only logging and document loss of early/panic UART. Record black-line/one-side-ramp start/stop/fault/recovery tests; rerun relevant regressions/schemas/builds and retain a working source/firmware checkpoint. Do not push to professor repository.


## 5. Tribunal 1 implementation

Implementation remains deferred until a working integrated baseline is measured.
Field-table preparation in §2 can proceed now; packing Buddy 4 alone cannot
establish team-wide compliance.

- [ ] **Buddy 4-owned representation changes**
  - **Status:** Local layout/encoding design and host-test planning can proceed; production implementation deferred.
  - **Needs:** Field/range/bit table and saved integrated baseline before changes.
  - **Evidence / finish condition:** Appropriate bit-fields or masks/shifts, range/overflow and encode/decode checks, preserved numerical precision/validity and safe concurrent packed access across tasks/ISRs.

- [ ] **Shared formats and before/after comparisons**
  - **Status:** Team implementation and compliance pending.
  - **Needs:** Shared fault bitmask, task/ISR/message contracts, Buddy 1 and dashboard/decoder owners; functioning integrated measurement baseline.
  - **Evidence / finish condition:** Agreed compact binary MQTT fields/decoder, measured RAM and relevant message sizes before/after, correctness/timing comparisons and original change-document cross-check for Week 13. CSV debug text alone is insufficient.


## 6. Report and final submission—report deferred

- [ ] **Final report and acceptance decisions**
  - **Status:** Working draft exists; report writing and professor clarification remain deferred.
  - **Needs:** Final evidence, agreed collision interpretation, height accuracy expectations, encoder-rate acceptance and Tribunal comparisons.
  - **Evidence / finish condition:** Update draft when resumed: MTK architecture, USB/UART timeout cause, bounded DIAG-E results, calibration/height repeatability, motion/yaw/collision limits, figures/citations and Tribunal table/comparisons. Cross-check actual briefing/rubric; do not invent unavailable accuracy statistics.

- [ ] **Handover and submission**
  - **Status:** Pending final team qualification.
  - **Needs:** Final reviewed source/docs/evidence and team ownership/demo agreement.
  - **Evidence / finish condition:** Clean-clone build/test evidence, exact build/flash/serial instructions, preserved raw captures/configurations and intended submission without generated builds or credentials. Any commit/push is a later task, not authorized by this documentation edit.


**Evidence and resume links**

- [Buddy 4 configuration, algorithms and earlier results](README.md)
- [Current isolated MTK trial and build instructions](../buddy4_mtk/README.md)
- [Clean DIAG-E capture and limits](../buddy4_mtk/evidence/2026-10-28/README.md)
- [Earlier SDK captures, analysis rules and charts](evidence/2026-10-28/README.md)
- [Existing SDK driving-test project](../integration_test/README.md)
- [Terrain analysis report draft](Buddy4_Terrain_Analysis_Report_Draft.md)
- [Tribunal 1 field/range/bit representation preparation](TRIBUNAL1_FIELD_REPRESENTATION.md)

When resuming, inspect the current Git diff and teammate updates, then choose
an unfinished task whose dependencies are available. Documentation preparation,
a successful build or one clean recording does not complete hardware validation.

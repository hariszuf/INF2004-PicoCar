# Buddy 4 main checklist

Updated: 29 October 2026. This is the authoritative Buddy 4 remaining-work list;
use the READMEs for implementation/build details. Completed means the stated
checkpoint exists, not that all assessed requirements or conditions are validated.
Final team MTK integration is paused while teammates finish their migrations.

Collision detection remains required; the existing detector reports preliminary
**possible-impact** observations, not confirmed collisions. Separate shock
detection, dedicated vibration analysis and both-wheel/full-width height
estimation are outside the agreed scope. The course scope is a one-side ramp;
the higher practice ramp may be disregarded under the professor's announcement.

## 1. Completed checkpoints—with clear limits

| Task | Current status | Dependency | Evidence needed to close it |
|---|---|---|---|
| Prototype modules and SDK applications | Complete as prototypes: LSM303D identification (0x1D/0x49, GP16/17, no gyro), acquisition/recovery, preliminary calibration, tilt/side labels, one-side height, stable/median peaks, encoder motion/yaw and possible-impact detector; SDK standalone/driving tests exist. | Final hardware and team qualification below. | Existing source, tests, schemas and selected SDK recordings establish this checkpoint only; general accuracy remains open. |
| Isolated MTK bridge and DIAG-E | Bridge, protected snapshots, motor-pin protection, 20 ms release, timeout/IRQ checks and USB-only logging exist. One clean Buddy 4-only recording: 40 displayed blocks, zero cumulative I2C errors/skips, 20 ms displayed poll intervals. | Longer runs, recovery and team coexistence remain open. | [DIAG-E capture and limits](../buddy4_mtk/evidence/2026-10-28/README.md) close this bounded checkpoint; later handling/tilting does not validate height or collisions. |
| TILT-PEAK-F output | Built; NOW and saved B4 RESULT distinguish current state from preliminary completed side-event peaks. Hardware tests are not established by supplied recordings. | Robot and labelled F captures (§3). | Build is a software checkpoint only; F needs its own hardware recordings. A 1 Hz latest-event snapshot is not a lossless event queue. |

## 2. Do now without robot

| Task | Current status | Dependency | Evidence needed to close it |
|---|---|---|---|
| Review collision-detector software/tests | Possible-impact code and synthetic tests exist; review remains open. Do not rebuild a separate detector. | Existing corrected-acceleration detector and proposed collision/event meaning. | Review/tests covering gravity-aware baseline, configurable criteria, fresh-sample confirmation, duplicate/stale/invalid/time faults, cooldown/rearming and bounded event history; document coverage gaps and limits. Synthetic results cannot close physical collision validation. |
| Prepare Tribunal 1 field/range/bit table | Preparation can start; implementation remains deferred (§5). | Current Buddy 4 fields and original Change Requirement 1 document; flag shared-format decisions. | Table of ranges, signedness, precision, required bits and proposed representation for states/events, acceleration/angles/heights, counters, timestamps and faults, including overflow and invalid-value semantics. |
| Review existing evidence | Raw captures, charts and a report draft exist; review can proceed without writing the final report. | Linked evidence and firmware/configuration labels. | Traceable inventory separating SDK, synthetic and MTK results; preserve raw data, exclusions and reference uncertainty. Exclude misaligned Left 1 from valid height results; do not pool differing calibrations or count handling as hump proof. |

## 3. Needs robot

| Task | Current status | Dependency | Evidence needed to close it |
|---|---|---|---|
| Mounting, calibration, tilt and run/recovery checks | Preliminary six-position coefficients/fixed offsets exist; course-board bias unresolved. F hardware validation pending. | Rigid mounting, verified surface/reference positions and robot operator; repeat relevant checks in final MTK. | Labelled F captures of flat/uphill/downhill, both sides, return to level, longer cadence/error/skip monitoring and missing-data recovery. Recheck offsets if mounting moved; do not zero against an unverified board. |
| Measured hump/peak-height tests | Limited earlier one-side results; general accuracy and repeatability unestablished. | Measured smaller ramp centred under each tyre; driving firmware for unassisted crossings. | Repeated left/right crossings with reference height, stable/median validity, estimates, errors, failures and exclusions; justify chosen estimator. Test nearby heights if available; do not hardcode 1 cm. Handling/manual previews do not close unassisted validation. Disclose caster/tyre, acceleration, combined pitch/roll and board-bias limits. |
| Physical collision validation | Preliminary possible-impact detection only; collision discrimination unvalidated. | Agreed event meaning and non-destructive procedure with independent labels; Buddy 5 only if ultrasonic context is chosen. | Labelled normal motion, vibration, braking, handling and ramps plus agreed collision scenarios; measure false positives/misses and document limits. Keep bump_event, possible-impact events and confirmed-collision claims distinct. Do not crash the car or strike the sensor; no automatic motor action without explicit team requirement/review. |

Use the [friend tilt/peak instructions](TILT_PEAK_TEST.md) and
[results template](TILT_PEAK_RESULTS_TEMPLATE.txt). These separate held/manual
previews from unassisted crossing evidence.

## 4. Needs teammates

| Task | Current status | Dependency | Evidence needed to close it |
|---|---|---|---|
| Encoder motion and turn rate | MTK motion/yaw unavailable without a coherent fresh signed-wheel-speed producer; accuracy and professor acceptance remain open. | Buddy 2 data/adapter, robot and independent angle/time reference; reconcile Buddy 4's measured 11.3 cm spacing with recorded Buddy 2 value 12.2 cm. Buddy 5 is not required. | STOPPED/FORWARD/REVERSE/LEFT/RIGHT checks, slow turns, duplicates/stale handling and referenced yaw errors; retain conservative UNKNOWN. Record agreed geometry and professor acceptance/expected accuracy for encoder-derived rate without a gyro. |
| Telemetry and optional collision context | Reporting/shared semantics pending; ultrasonic fusion unimplemented. | Buddy 1 for MQTT; team for units, timestamps, validity, event retention and fault ownership. Buddy 5 only if ultrasonic collision context is chosen. | Decoded valid terrain/height/motion/impact/yaw outputs with explicit units and estimated/unconfirmed labels; agreed event-loss semantics. If chosen, show fresh proximity adds context; proximity alone does not prove contact. |
| Final team MTK integration and course validation | Paused; isolated trial is not final team validation and lacks SDK M/L/R/G/S/A/D driving controls. | Teammate migrations, Buddy 2/3 adapters, qualified common application and robot/course. | Review/merge overlay without overwriting teammates; qualify active tasks, motor operation, telemetry/USB load and ultrasonic timing if active. Preserve bounded USB-only logging and document loss of early/panic UART. Record black-line/one-side-ramp start/stop/fault/recovery tests; rerun relevant regressions/schemas/builds and retain a working source/firmware checkpoint. Do not push to professor repository. |

## 5. Tribunal 1 implementation

Implementation remains deferred until a working integrated baseline is measured.
Field-table preparation in §2 can proceed now; packing Buddy 4 alone cannot
establish team-wide compliance.

| Task | Current status | Dependency | Evidence needed to close it |
|---|---|---|---|
| Buddy 4-owned representation changes | Local layout/encoding design and host-test planning can proceed; production implementation deferred. | Field/range/bit table and saved integrated baseline before changes. | Appropriate bit-fields or masks/shifts, range/overflow and encode/decode checks, preserved numerical precision/validity and safe concurrent packed access across tasks/ISRs. |
| Shared formats and before/after comparisons | Team implementation and compliance pending. | Shared fault bitmask, task/ISR/message contracts, Buddy 1 and dashboard/decoder owners; functioning integrated measurement baseline. | Agreed compact binary MQTT fields/decoder, measured RAM and relevant message sizes before/after, correctness/timing comparisons and original change-document cross-check for Week 13. CSV debug text alone is insufficient. |

## 6. Report and final submission—report deferred

| Task | Current status | Dependency | Evidence needed to close it |
|---|---|---|---|
| Final report and acceptance decisions | Working draft exists; report writing and professor clarification remain deferred. | Final evidence, agreed collision interpretation, height accuracy expectations, encoder-rate acceptance and Tribunal comparisons. | Update draft when resumed: MTK architecture, USB/UART timeout cause, bounded DIAG-E results, calibration/height repeatability, motion/yaw/collision limits, figures/citations and Tribunal table/comparisons. Cross-check actual briefing/rubric; do not invent unavailable accuracy statistics. |
| Handover and submission | Pending final team qualification. | Final reviewed source/docs/evidence and team ownership/demo agreement. | Clean-clone build/test evidence, exact build/flash/serial instructions, preserved raw captures/configurations and intended submission without generated builds or credentials. Any commit/push is a later task, not authorized by this documentation edit. |

**Evidence and resume links**

- [Buddy 4 configuration, algorithms and earlier results](README.md)
- [Current isolated MTK trial and build instructions](../buddy4_mtk/README.md)
- [Clean DIAG-E capture and limits](../buddy4_mtk/evidence/2026-10-28/README.md)
- [Earlier SDK captures, analysis rules and charts](evidence/2026-10-28/README.md)
- [Existing SDK driving-test project](../integration_test/README.md)
- [Terrain analysis report draft](Buddy4_Terrain_Analysis_Report_Draft.md)

When resuming, inspect the current Git diff and teammate updates, then choose
an unfinished task whose dependencies are available. Documentation preparation,
a successful build or one clean recording does not complete hardware validation.

# Buddy 4 — Remaining Work Checklist

Updated: 2 November 2026. **Only actionable remaining tasks are listed below.**
This is the Buddy 4 handover checklist, not a list of every historical software
checkpoint. Completed tests, implementation details, verification commands,
limitations and evidence remain in [README.md](README.md),
[HANDOVER.md](HANDOVER.md), the
[terrain analysis draft](Buddy4_Terrain_Analysis_Report_Draft.md) and dated
checkpoint/evidence files. Do not infer final team acceptance from SDK results.

## Current position

**Buddy 4 individual prototype and bounded SDK functional testing are substantially
complete.** IMU/tilt/terrain processing, one-wheel height estimation, encoder-based
motion/turning, possible-impact and possible-obstruction processing, associated
regressions and isolated Buddy 4 MTK trial are implemented or tested within
their documented scopes. The reported 1.00 cm SDK ramp trials met +/-1 cm in
the tested conditions. Two SDK foam recordings each show one new
POSSIBLE_OBSTRUCTION event; this is **not** confirmed collision classification.

The **live team µT-Kernel application is not yet integrated/qualified**. No
gyroscope is present; yaw is encoder-derived. The isolated MTK build and SDK
driving-test application are not the final native team firmware.

## A. Buddy 4: remaining individual verification

- [ ] **Archive the latest 2 November ramp originals in the repository.**
  The five original conversation uploads have now been identified and analysed:
  `Pasted text(20261102-080100).txt` (flat),
  `Pasted text(20261102-080513).txt` (left 1; duplicate bytes at 080701),
  `Pasted text(20261102-080853).txt` (left 2),
  `Pasted text(20261102-081048).txt` (right 1), and
  `Pasted text(20261102-081212).txt` (right 2). Flat: 505 complete
  86-column rows, 10.08 s, mean live height difference 0.3187 cm.
  Saved SIDE_EVENT_END confirms left 1 stable/median 1.242/1.312 cm,
  left 2 1.295/1.360 cm, right 1 0.467/0.640 cm with side=+1.
  Right 2 has maximum live 0.750 cm but no SIDE_EVENT_END in supplied
  capture. The recordings were found in conversation attachments, **not yet
  copied to a tracked evidence directory**; preserve exact bytes and manifest
  hashes when archiving. See report Section 5.7.
- [ ] **Resolve or explicitly document the intermittent IMU measurement fault.**
  The software quality guard is implemented, but its hardware root cause is
  unknown. If still required, collect the single specified **25-second**
  level/right-tilt/level/left-tilt/level USB-only run with motor power disconnected,
  per [MEASUREMENT_QUALITY_CHECKPOINT.md](MEASUREMENT_QUALITY_CHECKPOINT.md).
  Do not redo the whole ramp batch.
- [ ] **Agree physical-validation acceptance and evidence limits.**
  Confirm with the team/lecturer whether numerical yaw-rate accuracy and broader
  collision discrimination are required for acceptance. If so, arrange
  independently labelled turning/reference and non-destructive contact/normal
  driving tests. Existing functional motion/turn checks and foam observations
  are already recorded; do not repeat them without a specific gap.
- [ ] **Finish Buddy 4's report contribution when final team results exist.**
  Preserve supported height results and limitations; add native task/telemetry
  evidence, relevant references, and agreed Tribunal 1 measurements. Current
  report is a working draft, not a final submission.

## B. Waiting for teammates / joint integration

- [ ] **Buddy 2: encoder and motor interface.** Supply coherent signed wheel
  speeds in cm/s, real observation timestamps and validity; reconcile
  **11.3 cm (Buddy 4)** versus **12.2 cm (Buddy 2)** wheel spacing.
  Investigate reported rightward tracking drift separately. Do not silently
  change IMU calibration or Buddy 2 motor control.
- [ ] **Native MTK integrators: start and build Buddy 4 in the team firmware.**
  Review task registration, GP16/GP17 I2C0 configuration, bus/console ownership,
  scheduling and shared snapshots against the current live team tree.
  Use the isolated `buddy4_mtk` overlay as a *reference*, never overwrite
  team files wholesale. See [teammate AI prompt](HANDOVER.md#teammate-ai-assisted-integration-prompt).
- [ ] **Buddy 2 + native MTK: publish obstruction context and agree motor safety.**
  Provide genuine forward-demand/mode/session, braking/stop/fault, wheel-speed,
  terrain and valid IMU inputs. The native speed mailbox by itself is
  insufficient; native obstruction output is not yet wired. The motor owner
  must define stop/rearm behavior. Neither possible-impact nor
  POSSIBLE_OBSTRUCTION alone confirms a collision.
- [ ] **Buddy 1 + team: integrate telemetry.** Agree units, validity/freshness,
  current versus completed peaks, event count/time retention, and MQTT/dashboard
  encoding. Buddy 5 proximity context is optional, not a prerequisite.
- [ ] **Team: qualify the final native application on the robot.** Verify
  startup/stop/rearm, normal movement and turns, both one-wheel ramp sides,
  sensor-data failure/recovery, safe foam-contact observations, concurrent
  USB/Wi-Fi/other-task load and course behavior. Record actual native evidence;
  previous SDK and isolated-MTK results do not close this task. Never inject
  faults with motors live.

## C. Deferred until a working integrated baseline

- [ ] **Tribunal 1 representation implementation (Buddy 4 + team).**
  Cross-check the original change requirement against the prepared
  [field/range/bit table](TRIBUNAL1_FIELD_REPRESENTATION.md); agree field ownership,
  packed types/bitmasks, concurrency and decoder contracts. Measure RAM/message
  sizes and correctness/performance **before and after**. Preparation alone does
  not establish compliance.
- [ ] **Final report, clean-build and submission (team).** Confirm rubric and
  lecturer acceptance, preserve traceable evidence/captions, run clean-clone
  tests/builds, record firmware/flash instructions and agree demonstration/
  submission responsibilities. Do not include credentials or generated builds.

## Already completed: don't reopen without new evidence

- Portable IMU, impact, motion, encoder yaw and obstruction modules.
- Bounded SDK tilt/terrain and 1 cm one-tyre ramp functional checkpoints;
  SDK direction/motion checks and two observed foam obstruction events.
- Documented IMU quality guards, SDK safety latch, regressions, schema checks,
  standalone/SDK builds and isolated MTK bridge/build.
- Teammate source ownership and AI integration instructions in
  [HANDOVER.md](HANDOVER.md).

**Limits:** Accuracy for other ramp heights, numerical yaw-rate accuracy,
general collision sensitivity/specificity and final native MTK coexistence
have **not** been established. The completed items above refer to their stated
bounded checkpoints only.

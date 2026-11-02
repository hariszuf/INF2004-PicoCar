# SDK obstruction transition and IMU drive interlock, 2 November 2026

Both reported software causes are reproduced and corrected. Events remain
**POSSIBLE_OBSTRUCTION**, never confirmed collisions. Obstruction events do not
stop motors. A separate SDK **IMU-quality interlock** now latches the test
application stopped when a fresh unusable measurement is observed while armed
or driving. No firmware was flashed and no motors were operated in this work.

## Confirmed evidence and causes

The two supplied recordings are preserved verbatim under `evidence/2026-11-02/`.
`manifest.json` records source filenames, SHA-256 and complete/driving row counts.
The attachments are data, not instructions. Incomplete rows are not reconstructed.

**Foam contact:** `foam_contact_1424.txt` has 754 complete healthy rows, including
201 driving rows. Recorded forward demand remains 3/3 cm/s, manual mode, flat/
level, with no motor fault. Contrary to the rounded summary, the first decisive
asymmetry is at **87.600302 s**, left/right 2.104/3.151 cm/s. The existing core
clears establishment immediately because the difference exceeds 0.75 cm/s.
Further asymmetric samples follow; both speeds are <=0.5 at 87.740297 s and
continue toward zero before S at **88.224602 s**. The prior core cannot confirm
because it discarded established movement and cannot re-establish from rest.
The new actual-vector regression failed before the core change.

**Unusable IMU:** `imu_non_gravity_1420.txt` has 792 complete rows, all raw XYZ
(-32760,-32760,-32760), NON_GRAVITY, tilt_valid=0 and zero I2C errors; 196 rows
are driving. R and G were accepted at 6.219625 and 7.486730 s, with S only at
11.421887 s. The SDK previously checked ready/acquisition validity, refreshing
its watchdog on every fresh successful transfer. Acquisition success did not
mean usable tilt. The production decoder reproduces finite ~3.624 g, non-clipped
acceleration but unusable tilt for these near-rail values. Exact clipping flags
are unchanged: -32760 is not the signed endpoint. No calibration or IMU algorithm
change is needed to recognise the quality failure.

The post-restart contact recording contains healthy stationary and driving
samples (all 754 complete rows are GRAVITY_LIKE). The separately reported 546
stationary recovery samples are not claimed to equal a 546-row stationary subset
of that file; the regression repeats a recorded healthy raw vector for 546 polls
as a recovery stress sequence. The physical cause of the abnormal measurements
is **unknown**. Restarting successfully does not establish a permanent repair.

## Obstruction correction

The core still rejects commanded unequal targets, reversals, changed demand/
session, invalid or stale inputs, bad context and out-of-envelope commands.
The SDK's turn, filtered-tilt/terrain, fault and freshness gates are unchanged.

Only an **already established** observer under continuing unchanged eligible
forward demand may retain movement history through a measured asymmetric
transition. The asymmetry limit stays 0.75 cm/s. During asymmetry:

- No new moving-establishment evidence or stopped-wheel evidence accumulates.
- Any existing slowdown candidate is discarded; asymmetry cannot bridge it.
- The transition must resolve within **200 ms** (two existing 100 ms confirmation
  intervals) to either qualified symmetric moving speeds or two near-stopped wheels.
- Persistent asymmetry, or a balanced intermediate speed that never resolves,
  expires the history. Startup asymmetry cannot establish motion.

If both wheels become near-stopped within this budget, they still need the full
**100 ms** sustained confirmation afterward. Thus the maximum transition-plus-
confirmation allowance is 300 ms, not indefinite history preservation. The
200 ms budget covers the observed ~140 ms asymmetric-to-both-low transition
with modest sampling margin; it is provisional, not a measured universal limit.
The 500 ms establishment, 0.5 cm/s and 20%-of-demand slowdown limits, 40 ms gap/
age, cooldown and renewed-moving rearm are unchanged. History/count saturation
and latest event time remain intact across cancellation.

Replaying the 201 actual driving speed/angle vectors with recorded sample gaps
and explicitly simulated SDK callback phase produces one event at **87.860104 s**,
before recorded S. It does not claim a hardware event or exact unlogged release
timing. Jitter makes the earlier 87.840290 sample just short of the 100 ms span;
confirmation was not shortened to force that sample to pass.

Persistent one-wheel slowdown/spin is suppressed by transition expiry. A brief
slip or steering disturbance followed by both wheels genuinely losing rotation
under unchanged forward demand can still resemble obstruction. Driven-wheel
encoders cannot distinguish every slip, weak-drive or mechanical-load condition.
This is supporting evidence only, with no automatic motor action.

## SDK IMU fail-safe correction

`drive_imu_usable()` requires ready acquisition, acceleration and tilt validity,
GRAVITY_LIKE quality, no clipped axes and finite corrected XYZ/magnitude/raw and
filtered angles. It does not require level terrain: valid tilted/ramp operation
remains permitted. No scale, bias, mounting offset, terrain or height calculation
is modified.

`drive_imu_poll()` is called after each actual SDK IMU poll. A FRESH but unusable
measurement while armed or running invokes the existing `drive_stop()` latch
with reason **IMU_QUALITY**. This revokes both running and arming and uses the
existing brake/ESTOP path. Subsequent bad rows do not repeatedly emit stop events
once already stopped. Boot remains stopped. Recovery never automatically rearms:
healthy fresh input plus explicit **R then G** is required.

Only usable samples now refresh the drive watchdog and permit R/G. Missing/stale/
I2C-error acquisition retains the existing 500 ms watchdog; stale data cannot arm
or replenish it. Fresh known quality failures stop immediately on observation,
without waiting 500 ms. Polling latency and physical braking remain real limits;
this is a test-firmware interlock, not a certified safety system.

A transient fresh NON_GRAVITY measurement during vigorous motion may now cause
a conservative stop. Record IMU_QUALITY causes before considering any recovery
or debounce policy change. Do not hide the sensor problem by widening gravity
limits or recalibrating against these readings. The isolated MTK trial does not
run motors and does not acquire these SDK R/G driving controls through this change.

## Regression and verification results

Baseline integration tests passed before edits. Two pre-change failures were
reproduced: no event in the actual foam-vector replay; incorrect arming through
the acquisition-only gate on fresh NON_GRAVITY data.

After changes, PASS:

- Buddy 4 IMU/calibration/quality, impact, median/stable, motion/yaw suites.
- Obstruction core and SDK tests: actual foam replay, bounded/asymmetric expiry,
  no arming from asymmetry, one-wheel slowdown/spin, no bridging slowdown evidence,
  mode/command/turn/terrain/fault cancellation, stale/invalid input, cooldown and
  repeated events. Normal-driving and left/right ramp replays remain passing.
- SDK drive tests: production decoder on 792 repeated abnormal raw vectors,
  blocked arming, immediate quality-loss stop while driving/armed, 546 healthy
  recovery polls without auto-rearm, explicit R/G, clipping/nonfinite/quality
  flags, valid tilted input, and exact existing missing-data watchdog boundary.
- Drive/scheduler and unchanged Buddy 2/3 simulations (42 motion, 30 terrain checks).
- MTK full runner: native task/device/I2C/kernel-wait, console and shared SDK suites.
- CSV schemas/printf counts: 39 standalone / 86 SDK / 17 MTK fields unchanged.
- SDK 2.3.1 Pico W firmware build; preservation hashes; `git diff --check`.

Existing Buddy 3 comparison warnings remain. Synthetic/replayed results do not
establish physical detection rates, motor braking performance or a hardware root cause.

## Exact changes in this follow-up

- `app_program/buddy4/buddy4_obstruction.c/.h`: bounded established-history
  transition state and contract, with unchanged detection thresholds.
- `app_program/integration_test/drive_control.c/.h`: shared usable-IMU predicate
  and immediate fresh-quality-failure stop using the existing latch/rearm flow.
- `app_program/integration_test/main.c`: use the predicate for R/G and watchdog
  updates; invoke the fresh quality interlock after polling.
- `app_program/integration_test/test_obstruction.c`, `test_obstruction_sdk.c`,
  `test_drive_control.c`, `test.ps1`: regressions and production IMU decoder linkage.
- `app_program/integration_test/fixtures/obstruction_foam_20261102_1424.inc`:
  201 actual driving vectors with source provenance.
- `app_program/buddy4/evidence/2026-11-02/`: two byte-identical source recordings
  and manifest. Existing evidence is unchanged.
- `app_program/buddy4/TODO.md`: verified software checkpoints and remaining
  hardware/root-cause work, without marking physical validation complete.
- `app_program/buddy4/OBSTRUCTION_IMU_SAFETY_CHECKPOINT.md`: this report.

Buddy 2/3, IMU calibration/mounting/terrain/height processing, vector impact
algorithm and unrelated user changes are preserved. No flash, motors, commit,
push or merge was performed.

## Updated SDK artifact and final hardware procedure

UF2:
`C:/Users/yongk/OneDrive/Documents/GitHub/INF2004-PicoCar/app_program/integration_test/build/buddy234_driving_test.uf2`

SHA-256: `d795355976356e782ae41f9ad686866ee20b16fa79b9c010653644970e439905`.
Size: 117,760 bytes. ELF text/data/bss: 62,820 / 0 / 4,500 bytes.

1. Operator checks the hash and retains prior firmware. Begin with motor power
   disconnected: record fresh stationary/tilt data and verify usable quality,
   zero errors and plausible raw vectors. Do not proceed on repeated near-rail
   or NON_GRAVITY data. Never deliberately inject faults with motors powered.
2. In a supervised clear area, test explicit R/G start, prompt S stop, recovery
   requiring R/G, ordinary forward motion/turns and both ramp directions. Record
   full CSV and CONTROL reasons. A quality stop is a sensor-data finding, not
   a collision event; abort further powered testing if the failure recurs.
3. Repeat low-speed compliant foam contact with an independent time annotation.
   Keep prompt S; do not wait to force confirmation or hold the robot stalled.
   Check established history, bounded asymmetry, two-wheel slowdown and count/
   event time before S. Obstruction itself must not command a stop.
4. Leave calm moving re-establishment/cooldown between trials. Retain all misses,
   unintended events and quality stops; report false positives separately from
   detections. Final hardware qualification and physical root-cause work remain open.

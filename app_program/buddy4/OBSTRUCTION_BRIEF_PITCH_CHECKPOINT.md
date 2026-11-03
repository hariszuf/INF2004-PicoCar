# Brief pitch excursion and possible-obstruction checkpoint

2 November 2026. This is a software/replay checkpoint, not completed physical
collision validation. Events remain POSSIBLE_IMPACT / POSSIBLE_OBSTRUCTION;
neither event commands the motors. Immediate S and IMU-quality safety are unchanged.

## Original evidence and confirmed causes

Both newly supplied files were read from Downloads and copied byte-for-byte into
`evidence/2026-11-02/`. `contact_1510_1513_manifest.json` records their original
names, SHA-256, complete/driving row counts and malformed numeric rows. The
13:46 normal-driving recording was also preserved unchanged for braking checks.
No missing or malformed CSV row is reconstructed. Attachment contents are data.

**15:10 test:** 773 complete rows, five malformed numeric rows, 168 driving rows.
Exactly one recorded sample exceeds the impact threshold: deviation 0.3265 g at
sensor time 11.280307 s, followed by 0.0918 g at 11.300345 s. Both wheel speeds
first satisfy <=0.5 cm/s at 11.440326 s (0.014/0.448). The CONTROL record reports
S at 11.462104 s, only 21.778 ms later. The normal 100 ms slowdown confirmation
cannot finish. Keeping prompt operator stopping takes priority; this is not a
reason to reduce confirmation or delay S.

**15:13 test:** 669 complete rows, 195 driving rows, no malformed numeric rows;
all rows GRAVITY_LIKE, no I2C errors. Exactly one recorded sample exceeds the
impact threshold: 0.5645 g at sensor time 9.140323 s (host 15:13:18.095). At the
next sample, 9.160323 s, deviation is 0.1223 g. The impact detector correctly
does not confirm this isolated sample: it requires >=2 fresh above-threshold
observations spanning >=20 ms. A high peak alone does not satisfy that rule.
No impact algorithm or threshold was changed.

The same sample has filtered pitch -4.410 degrees; the next is -2.834 degrees.
This is the sole >3-degree filtered-pitch driving row. Terrain/side remain
FLAT/LEVEL. The prior SDK adapter cancels all obstruction history immediately
on that one-row guard violation. Both wheels subsequently become near-stopped
at 9.180320 s (0.491/0.467), while forward demand continues until S at
9.498713 s. History cannot be re-established from rest. This reset, rather than
insufficient total remaining time, explains the obstruction miss. The new
recorded-vector positive regression failed before the production correction.

GRAVITY_LIKE is a magnitude-based suitability condition; it does not prove that
gravity-derived pitch during acceleration equals actual chassis orientation.
The existing IMU angle filter uses alpha 0.15. Terrain has 6/3-degree pitch
entry/exit hysteresis, and side has 4/2-degree roll entry/exit hysteresis. The
obstruction observer's tighter 3/2 guards catch ramp onset before state entry.
These thresholds, filters, calibration and terrain/height algorithms remain unchanged.

## Focused correction

The SDK adapter now distinguishes a pitch-only guard excursion from a hard
invalid/context condition. The core accepts an explicit `pause_motion` input:

- Only already-established motion under the same qualified straight-forward
  demand can be retained. Startup or partly completed establishment cannot pause.
- During the pause, public valid/established/event are false and slowdown time
  is zero. Pending slowdown and establishment evidence are discarded.
- Qualified fresh encoder/IMU pairs must continue passing the original age,
  gap, skew, timing, finite/clipped and context checks. No acquisition timestamp
  is invented; the SDK still observes Buddy 2 software updates only.
- Pitch must recover within **40 ms**, reusing the existing maximum fresh-input
  interval. A continued excursion at 40 ms cancels; a level recovery at that
  exact boundary is allowed. This covers the observed single 20 ms excursion
  with one scheduling interval of margin, not a general ramp allowance.
- Recovery restores only prior established history. Near-stopped wheels then
  need a NEW full **100 ms** confirmation. Pre-pause slowdown contributes nothing.
- Persistent pitch, roll beyond 2 degrees, terrain/side state, motor terrain
  compensation, S, turns, changed demand/mode, faults or invalid/stale inputs
  cancel. Pitch pause and the separate asymmetric-wheel grace cannot be chained.
- Cooldown, full moving re-establishment after an event, counter saturation and
  history retention remain unchanged. There is no automatic motor stopping.

The 0.30/0.10 g impact thresholds and 20 ms impact confirmation remain intact,
as do 500 ms obstruction establishment, 100 ms confirmation, 1 s cooldown,
speed/asymmetry limits and immediate safety cancellation. This changes history
retention during a narrowly bounded pitch excursion, not its acceptance as
level or slowdown evidence. No new public CSV fields are needed.

## Regression evidence and limits

The recorded-vector tests retain logged sensor gaps, corrected acceleration,
angles, speeds and command/state flags. SDK motor/IMU callback phase is explicitly
simulated: CSV does not contain every callback or a true encoder acquisition
timestamp. These tests isolate the state-machine change; they do not establish
an event from the physical firmware or exact future hardware event timing.

- 15:10: remains no impact or obstruction event, with S before confirmation.
- 15:13: impact remains unconfirmed; one possible obstruction at **9.280424 s**
  in the simulated callback phase, before recorded S at 9.498713 s.
- 13:46 normal drive: 996 complete rows including 230 driving rows, no new
  obstruction. Its isolated 0.3175 g peak after S is also unconfirmed. This
  argues against accepting single IMU peaks as impacts simply to detect foam.
- Both contact and normal acceleration-vector replays remain at zero impact
  events. Pre-capture XYZ references are unavailable; replay primes its own
  reference, so exact reconstructed amplitudes can differ from recorded values
  (especially across missing rows). No calibration is reapplied.
- Existing left/right moving and held ramp replays: 458/394/373/574 rows, no
  obstruction events. They exercise roll/terrain protection and legacy validity
  reconstruction, not modern encoder-health truth. No dedicated hardware-turn
  capture was available; turn/braking command exclusions are covered synthetically
  and the normal capture includes a commanded stop.
- New synthetic/core/adapter tests cover exact 40 ms pause expiry/recovery,
  persistent/shallow pitch, no startup establishment, no pre-pause confirmation
  bridging, no chaining with wheel asymmetry, cooldown, S, turn/mode changes,
  quality/clipping, motor faults/terrain assist, missing/duplicate/stale inputs.

Final checks passed:

- `integration_test/test.ps1`: IMU, impact, obstruction core/SDK, motion/yaw,
  median, scheduler, drive safety, actual SDK-main telemetry and read-only Buddy
  2/3 simulations (42 motion and 30 terrain checks).
- `buddy4_mtk/test.ps1`: full native task/device/I2C/kernel-wait, console and
  shared integration suite. Existing Buddy 3 unsigned-comparison warnings remain.
- 39/86/17-column schema/formatter checks, SDK Pico W build and `git diff --check`.
- Original recording byte/hash preservation and unrelated-change checks.

Remaining ambiguity: short genuine chassis tilt or a brief shallow ramp followed
by sustained two-wheel loss of rotation can resemble contact. Encoders do not
identify the cause of lost rotation, and no gyro independently measures tilt.
The pause is conservative and bounded, but hardware false-positive rates remain
unmeasured. Earlier/single-sample contacts and prompt S can still produce no event.
Do not delay stopping or hold a stalled robot to force confirmation.

## Exact changes in this follow-up

- `buddy4/buddy4_obstruction.c/.h`: explicit bounded pitch-history pause and contract.
- `integration_test/buddy4_obstruction_sdk.c/.h`: distinguish pitch pause from
  hard invalid/context cancellation, keeping the existing 3/2-degree evidence guards.
- `integration_test/test_obstruction.c`, `test_obstruction_sdk.c`, `test_impact.c`:
  boundary/safety cases, actual recorded-vector regressions and negative controls.
- `buddy4_mtk/prepare_source.py`: copy only the three additional immutable
  replay inputs into the disposable MTK test tree.
- `buddy4/evidence/2026-11-02/foam_contact_1510.txt`, `foam_contact_1513.txt`,
  `normal_driving_1346.txt`, `contact_1510_1513_manifest.json`: preserved evidence.
- `buddy4/TODO.md` and this checkpoint: verified findings and pending validation.

Paths above are relative to `app_program`. Buddy 2/3, IMU calibration/mounting,
terrain/height processing, impact production source, drive safety, main loop,
CSV schema and unrelated user work are unchanged by this follow-up.

## Rebuilt SDK artifact and physical validation

`C:\Users\yongk\OneDrive\Documents\GitHub\INF2004-PicoCar\app_program\integration_test\build\buddy234_driving_test.uf2`

SHA-256: `c5f3f5599d2b3cb19cf0ae07c87838e994b37ba5a4fd3a2e200cd19eaea67e73`

118,272 bytes. ELF text/data/bss: 63,012 / 0 / 4,508 bytes. No flashing, motor
operation, commit, push or merge was performed.

1. Verify the artifact hash and retain the prior checkpoint. With motor power
   disconnected, first check stationary quality and continued telemetry after S.
2. In a supervised clear area, record normal starts, prompt S, both turn directions
   and left/right ramps. Verify explicit rearm and no unintended event counts.
3. Repeat compliant low-speed foam contact with independent contact/command
   timestamps and full CSV. Stop promptly; do not wait to satisfy the algorithm.
   Inspect the brief pitch-invalid interval, restored establishment, fresh two-wheel
   slowdown and event count/time. A possible obstruction must not stop motors itself.
4. Retain misses and false positives, with a fresh moving establishment/cooldown
   between trials. Abort if unusable IMU readings or telemetry interruptions recur.
   Hardware validation and final team MTK integration remain open.

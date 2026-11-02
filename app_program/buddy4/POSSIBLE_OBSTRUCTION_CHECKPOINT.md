# Buddy 4 possible-obstruction SDK checkpoint, 2 November 2026

Implemented a separate, conservative **POSSIBLE_OBSTRUCTION** observer for the
SDK manual straight-driving experiment. It reports unexpected loss of wheel
motion under continuing demand, not a confirmed collision, and issues no motor
commands. The existing vector-based IMU possible-impact detector is unchanged.
Physical false-positive/miss rates and contact discrimination are unvalidated.

This supersedes the implementation decision in `ENCODER_ASSISTED_CONTACT_REVIEW.md`
using the new pre-stop wheel-speed evidence. That review remains historical and
its ambiguity/timing limitations still apply. Earlier changed files are preserved.

## Evidence, independent reproduction and scope

The operator reports approximately 3.18/3.09 cm/s at 114.86 s, 0.12/0.12 at
114.96 s and 0.004/0.004 at 115.06 s, before S at about 115.09 s, with confirmed
physical foam contact. Normal driving retained approximately 3 cm/s before S.
The contact instant was not independently timestamped. IMU residual peaked at
0.2045 g, so the unchanged 0.30 g IMU trigger could not qualify it.

The new wheel evidence supports a narrow two-wheel blockage observation before
the commanded brake. It does not establish what caused the lost rotation or
whether the complete IMU/timing conditions below held in the actual recording.

Before writing either observer or adapter, `test_obstruction.c` was compiled and
run independently of detector code. Its fixture preserves the three reported
speed points and S time, while explicitly marking inserted 20 ms points as
synthetic assumptions. The final continuous replay qualifies at 115.02 s before
S, given synthetic steady prior motion and usable context. This is **not a replay
of a recovered complete hardware log**. A separate test feeds only sparse anchors;
their 100 ms gaps invalidate continuity and cannot produce an event.

The first runtime adapter is deliberately restricted to `DRIVE_MANUAL`, equal
forward targets in the 2-4 cm/s envelope around the measured 3 cm/s experiment.
Line following, forward curves, spin tests, reverse motion and contact from rest
are outside this observer's coverage. The IMU observer retains its prior coverage.

## Observer criteria and provisional justification

`buddy4_obstruction.c/.h` is portable C with no SDK, RTOS, motor or Buddy 2/3
dependency. Fixed thresholds are intentionally limited to this trial:

| Criterion | Value | Reason/limit |
| --- | --- | --- |
| Stable equal forward demand | 2-4 cm/s | Narrow envelope around the reported 3 cm/s; excludes speeds close to count quantisation. |
| Established measured motion | Each wheel 70-130% of demand for 500 ms | Excludes startup and ordinary tracking error; 500 ms is a conservative provisional warmup, not a measured optimum. |
| Measured left/right difference | <=0.75 cm/s throughout usable assessment | Excludes pronounced asymmetric/one-wheel behaviour; approximately the scale of one raw 20 ms count, not a calibrated turn detector. |
| Both wheels near stopped | Each <=0.5 cm/s AND <=20% of its demand | Reuses the existing near-stop scale; much smaller than normal ~3 cm/s and above observed 0.12/0.004. Does not classify modest load-related slowing. |
| Consecutive near-stop evidence | >=100 ms with observation gaps <=40 ms | The reported 0.12 -> 0.004 interval spans 100 ms. At nominal 20 ms this requires six distinct observations, rejecting isolated low samples. |
| Observation age/gap | <=40 ms | No confirmation across missing observations. |
| Repeated-event control | 1 second cooldown AND a new 500 ms established-motion window | Holding a blockage cannot repeatedly trigger. Faults/stops cannot bypass cooldown. |

Demand/session changes, ineligible context, asymmetric/reverse speeds, invalid
input, nonfinite values, duplicate/backward/future/stale times, clock reversal or
large gaps clear pending and established evidence. Recovery needs a new full
moving window. A moderate symmetric speed reduction does not itself trigger;
the earlier establishment can remain valid until cancelled, expired or an event
occurs. Event count saturates at UINT32_MAX and the latest event time survives
cancellation/faults. The pulse belongs to the qualifying observation; repeated
getters neither consume it nor recount it. Expiry/cancel/next update clears it.

These are engineering starting points supported by the observed separation
between moving and nearly stationary wheels, not validated classification
thresholds. No IMU amplitude threshold was lowered or required for this event.

## SDK observation qualification: no invented acquisition timestamp

Buddy 2's public snapshot has no encoder acquisition timestamp, sequence or
speed-validity/encoder-health flag. The adapter does **not** add or claim these.

`buddy4_obstruction_sdk_note_update()` is called once immediately after the
SDK main loop actually executes `motion_update()` and obtains its coherent
status. It records the before/after controller-update times and the command
context governing that update. `possible_obstruction_observation_us` is the
**end of this software update/status observation**, not the encoder acquisition
time, an interrupt/pulse timestamp or a physical contact timestamp. Counts are
read inside that bounded call; filtered speeds also contain earlier history.

Software qualification requires a previous consumed update, increasing start
times separated by 10-30 ms, execution/status duration <=5 ms, no newly skipped
motor slots, and observation age <=40 ms. These budgets avoid relying on Buddy
2's clamped speed-estimation dt after long delays. Repeated status reads or
duplicate callbacks cannot establish new evidence. The first interval is
unqualified. Even identical counts may be a legitimate new zero-speed update.

The SDK owns/cooperatively schedules its motor update, so this is defensible
software freshness qualification for a **possible** wheel obstruction. It is
not hardware validation: broken/stuck encoder inputs, wiring faults or a shared
failure can still masquerade as stopped wheels. `valid=1` means usable under
this software contract, never independently proven encoder health.

The adapter also requires a new usable IMU timestamp for each consumed motor
observation, IMU age <=40 ms and IMU/update-observation skew <=30 ms. It requires
ready/fresh acquisition, finite non-clipped acceleration and suitable tilt;
current raw and filtered pitch must be within +/-3 degrees and roll within
+/-2 degrees, with flat/level flags. Buddy 2 must report FLAT, speed scale near
1 and terrain assist near zero. Raw-angle checks suppress some ramp onsets
before smoothed terrain flags react. Any unusable context immediately cancels
evidence and requires re-establishment; a false/invalid field does not mean clear.

`buddy4_obstruction_sdk_poll()` runs on every SDK loop **after** serial-command
and safety processing, including loops without new sensor releases. S/space,
USB loss, stopped state, turns, faults, changed targets or a different drive
session therefore cancel before same-loop confirmation. A G cannot reuse a
pre-command motor observation. No stop processing or motor API is changed.
Persistent Buddy 4 IMU state and the new observer/output storage are static to
leave room for the expanded CSV argument frame and nested calls on the SDK main
stack. IMU initialisation/processing is unchanged. No heap allocation is used.

## Remaining ambiguity and conservative exclusions

- A severe ramp/friction load, mechanically jammed wheel, weak drive or encoder
  failure can produce the same speed trace as obstruction. Existing controls
  may brake or limit effort without exposing their actual duty/setpoints. The
  SDK adapter knows requested manual commands, not independently measured torque.
- Tilt/terrain gates reduce false positives but cannot prove absence of a ramp.
  Low-angle ramp loading or onset before detectable tilt can still resemble an
  obstruction. Confirming a collision requires independent evidence.
- Linear acceleration/vibration may violate the raw/filtered angle gates on
  genuinely flat driving, resetting establishment or hiding real contact. The
  reported IMU peak alone cannot prove these gates would pass the foam trial.
  Check `valid` and `established` in the next hardware recording first.
- A chassis blocked while drive wheels slip/spin may not produce a wheel-speed
  loss. One-wheel contacts, startup blockage and contacts during excluded modes
  are intentionally missed by this observer. Below 2 cm/s is not qualified.
- Filter/sampling/confirmation delays remain. If the operator stops before
  confirmation, the result must remain unconfirmed. Never delay S to obtain
  an event. Counts/events may be missed in USB rows; cumulative history is kept.

## Future team MTK integration

The core accepts a coherent speed pair, explicit validity/eligibility, demand
identity/targets and a monotonic producer observation time. Its owner must be
one task, using one clock domain. An eventual native producer must supply
genuinely new encoder observations and their timing/validity plus accepted
command/brake/fault/mode context; it must call cancel as soon as that context
changes. A native controller that owns the update may document an equivalent
bounded-observation contract, but cannot present it as sensor acquisition time.
Do not periodically copy `motion_get_status()` and invent fresh acquisition times.

The isolated Buddy 4 MTK application's existing speed mailbox still lacks a live
producer and command context. It is **not wired to this observer**. No team/MTK
motor code or Buddy 2/3 interface is changed. SDK Arm compilation and portable
host tests establish build compatibility, not completed MTK runtime integration.

## CSV and exact files changed in this follow-up

SDK driving CSV appends seven fields after the original 79 (now **86**); the
original fields retain their order:

`possible_obstruction_valid`, `possible_obstruction_established`,
`possible_obstruction_event`, `possible_obstruction_count`,
`possible_obstruction_observation_us`, `possible_obstruction_event_us`,
`possible_obstruction_slow_ms`.

`established` becomes false after an event; `slow_ms` reports the current
candidate span and resets with evidence. Validity/establishment are diagnostic
eligibility, not physical contact status. Existing 39-column standalone and
17-field MTK CSV outputs remain unchanged. Parse by the current header, not a
hardcoded previous field count. The added header comment identifies the SDK
observation-time semantics and possible-only classification.

- New `app_program/buddy4/buddy4_obstruction.c` and `.h`: portable observer.
- New `app_program/integration_test/buddy4_obstruction_sdk.c` and `.h`: SDK
  observation/context adapter.
- New `app_program/integration_test/test_obstruction.c`: independent fixture,
  portable observer and fault/history regressions.
- New `app_program/integration_test/test_obstruction_sdk.c`: adapter timing,
  command ordering, context, quality and recovery tests.
- Modified `app_program/integration_test/main.c`: read-only observation hooks
  after motor updates and command processing, diagnostic CSV fields.
- Modified `app_program/integration_test/CMakeLists.txt`, `test.ps1` and
  `test_csv_schema.py`: link observer/adapter, run tests and verify 86 columns.
- New `app_program/buddy4/POSSIBLE_OBSTRUCTION_CHECKPOINT.md`: this report.

The earlier eight changed/untracked files are preserved byte-for-byte, including
both vector-impact files, earlier reviews and README/fingerprint edits. Buddy
2/3 C/H sources are checked against pre-task hashes and remain unchanged. IMU
calibration, mounting offsets, terrain/height algorithms and motor behaviour are
untouched. No flashing, motor operation, commit, push or merge occurs.

## Verification and firmware

- Existing integration runner before implementation: PASS.
- Independent synthetic slowdown fixture compiled/executed before observer code:
  PASS (input reproduction only).
- Updated integration runner: PASS. Includes existing IMU/calibration/quality,
  stable/median, vector impact, motion/yaw, scheduler/drive, 42 Buddy 2 motion and
  30 terrain checks and Buddy 3 simulation, plus the two new obstruction suites.
- New tests cover steady normal driving, commanded stops, sustained two-wheel
  slowdown, one-wheel/asymmetric slowdown, partial reductions/isolated low
  samples, ramps/unequal speeds, turns/reverse, faults/mode changes, invalid/
  nonfinite/duplicate/stale/future/backward data, missing/irregular/long/skipped
  updates, source skew, clipping, recovery/cooldown, repeated events and saturation.
  A same-loop stop on the would-be confirming observation suppresses it.
- SDK CSV schemas: PASS, 39/86 columns with actual printf argument counts;
  original field-order checks retained.
- Existing MTK full runner: PASS, including 17-field quality CSV, native
  task/device/I2C/kernel-wait tests and the updated integration runner.
- Native CMake IMU CTest: PASS, 1/1.
- SDK 2.3.1 Pico W integration firmware: PASS. The two existing Buddy 3
  unsigned-comparison warnings remain in host simulations; no new SDK build
  warning was introduced. Hardware execution/timing is not established by builds.
- Preservation hashes and `git diff --check`: PASS.

Exact updated SDK UF2:
`C:/Users/yongk/OneDrive/Documents/GitHub/INF2004-PicoCar/app_program/integration_test/build/buddy234_driving_test.uf2`

Size: 115,200 bytes. SHA-256:
`e6aa23bd9293a541024a56541899a9bb0b7215f2e833002616af2b4881a784f4`.
ELF text/data/bss: 61,628 / 0 / 4,492 bytes. Build command with the installed
Pico toolchain/Ninja/CMake on PATH:
`cmake --build app_program/integration_test/build --parallel 4`.

The standalone IMU firmware and isolated MTK firmware do not contain the SDK
obstruction adapter and were not rebuilt for this change.

## Safe controlled hardware validation (operator performed)

1. Check the UF2 hash, retain prior firmware and load the SDK integration image
   only when ready for a supervised test. This session does not flash or drive.
   Leave the sensor mounting, calibration and usual operator stop unchanged.
2. Capture complete 86-column CSV and CONTROL records. On stationary flat ground,
   `established` should be false. Use manual mode and the existing R/G sequence
   only in a clear controlled area; verify normal straight movement eventually
   shows `valid=1, established=1` for at least 500 ms before interpreting contacts.
   If it never establishes, inspect tilt/quality/timing/skips rather than lowering
   obstruction thresholds or calling the absence of an event a successful test.
3. First repeat normal starts/steady drive/stops, permitted turns/mode changes
   and normal ramp crossings, including sensor-invalid cases only when naturally
   encountered. Expect no new obstruction counts during commanded stops, turns,
   ramps and unsupported modes. Invalid assessment is unknown, not no contact.
4. Repeat the previous low-speed compliant foam contact in a supervised setup,
   preferably at least five trials with several seconds of steady clear travel
   beforehand. Keep S available and stop promptly; do not hold the robot pushing
   the obstacle or delay S to satisfy confirmation. Avoid hard impacts and never
   deliberately lock powered wheels. Do not test induced faults with motors live.
5. Record synchronised video or an independent contact annotation, plus command
   stop time, both wheel speeds, obstruction eligibility/count/time/slow span,
   IMU quality/tilt and scheduling diagnostics. Compare count deltas, not only
   pulses, and distinguish events predating S from deceleration afterward.
6. Require a new established-motion interval between trials; cooldown alone
   cannot rearm. Retain non-detections and unwanted events. Report hit/miss and
   false-positive rates separately for conditions before extending beyond this
   manual/flat envelope or claiming physical validation.

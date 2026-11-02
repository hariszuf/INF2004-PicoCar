# Buddy 4 encoder-assisted contact review, 2 November 2026

Decision: retain the current IMU-only possible-impact observer unchanged. Existing
SDK interfaces could support a limited **unexpected wheel slowdown** observation,
but the available hardware evidence does not justify implementing a new fused
possible-contact event or choosing its thresholds/windows. The isolated Buddy 4
MTK trial additionally lacks a live encoder/command producer. Collision inference
from IMU and driven-wheel encoders alone remains ambiguous.

This follow-up adds only this report. The seven pre-existing changed/untracked
files from the vector-baseline work are preserved byte-for-byte. No calibration,
terrain/height processing, Buddy 2/3 source, runtime code, test, CSV schema or build
configuration is changed. No flashing, motor operation or commit/push/merge occurs.

## Hardware evidence and what it establishes

| Operator-labelled condition | Maximum vector residual | New possible-impact events |
| --- | ---: | --- |
| Normal motor-driven movement | 0.2420 g | None |
| Motor-driven soft-obstacle contact, followed immediately by S | 0.2045 g | None |

The software reason for no IMU event is established: neither reported maximum
reaches the unchanged 0.30 g trigger, so the consecutive-sample confirmation
cannot start from these observations. Lowering a scalar peak threshold enough
to include 0.2045 g would also include the larger normal-driving peak. Maxima
alone do not establish the durations, shape, validity, cooldown state or whether
a temporal rule could separate the trials.

The physical reason the contact was weak is **not established**. Compliance,
slow speed, sampling phase and prompt operator braking are plausible contributors;
none is proven by these summaries. There are no timestamped wheel traces in the
supplied evidence establishing a reduction before S, or even that the wheels
slowed rather than slipped while the chassis contacted the obstacle.

At the SDK manual target of 3 cm/s, an illustrative stop over 20/40/100 ms has
mean deceleration of only 0.1530/0.0765/0.0306 g (`0.03 / dt / 9.80665`). These
are kinematic examples, not measured contact peaks or detector predictions.
Contact does not necessarily create a >=0.30 g residual.

## Inspected interfaces and consequences

| Source | Available information | Consequence for fusion |
| --- | --- | --- |
| `buddy4_impact.c/.h` | Corrected-XYZ vector residual, validity, sample/event time, count; 0.30/0.10 g trigger/release, 20 ms confirmation, 40 ms gap/age, 500 ms cooldown/reference tau | Preserve as an independent IMU observation; lowering its threshold is not justified by the new maxima. |
| `buddy4_motion.c/.h` | Estimated motion label from a fresh signed speed pair; 100 ms confirmation and freshness | Labels describe wheel motion, not command intent or chassis ground speed. The previous confirmed label persists during a fresh pending transition. Do not differentiate these labels to estimate rapid deceleration. |
| Buddy 2 `motion_control.h`, `motion_get_status()` | Atomic snapshot of both counts, filtered signed speeds, motor state/fault, terrain mode/scale/assist | No speed-validity flag, acquisition timestamp/sequence, active command type, effective per-wheel setpoint, applied PWM or pending brake/latch state. EXECUTING does not prove both wheels are being driven forward. |
| Buddy 2 `motion_encoder.h/.c` | Single-phase counts with sign assigned from the last driven motor direction | Counts measure wheel rotation, not ground displacement or independent direction. Missing pulses and genuinely stationary wheels are not distinguished by this interface. |
| Buddy 2 `motion_motor.h` | Set/brake/latch operations | No read-only actual-drive feedback. Buddy 4 must not invoke motor operations to obtain evidence. |
| SDK `integration_test/main.c`, `drive_control.c/.h` | Known motor-update calls, manual/line/turn mode, running state, requested wheel targets, stop reason and CONTROL records | Enough for an SDK-only observational adapter under controlled steady manual drive, with explicit time/command association. These requested targets are not Buddy 2's terrain-scaled, acceleration-limited, actually applied setpoints. Selected targets can remain nonzero while stopped. |
| MTK `buddy4_task.c/.h` and `MOTION_YAW_SOFTWARE_CHECKPOINT.md` | Speed mailbox accepts coherent speeds, valid flag and acquisition time; duplicate observations do not advance evidence | No live producer calls it outside tests in the isolated trial. Mailbox has no command/braking context. A periodic read of retained Buddy 2 status with a new invented timestamp would not satisfy the contract. |
| Buddy 5 `ultrasonic.h`, `obstacle_state.h` (read only) | Range/status and obstacle/clear hysteresis | Possible independent obstacle context, not contact proof. A future producer must carry real acquisition time/validity; the raw result struct alone has no timestamp. This SDK driving test does not currently consume it. |

Buddy 2's existing stall fault means commanded effort with sufficiently low wheel
speed for a sustained period, not a collision classification. Its private logic
checks applied duty or PID integral saturation. The SDK integration overrides
the duty threshold to 0.6 and the duration to 1500 ms (base defaults 0.25 and
500 ms). Its existing protective stop is unchanged. It cannot be treated as
instantaneous evidence of a brief contact, and Buddy 4 must not duplicate or
relabel it as a collision.

## Timing and the immediate S press

Motor and IMU updates nominally run every 20 ms; line decisions run every 5 ms.
Buddy 2 computes count-difference speed and filters it with alpha=0.5. Counts are
approximately 0.0161 cm each, giving about 0.805-0.807 cm/s per count at 20 ms
(about 3.72 counts/window at 3 cm/s). Differentiating filtered speed over a
single interval is therefore both delayed and sensitive to count quantisation.
Longer count windows reduce this uncertainty while delaying detection.

For an ideal steady filtered speed of 3 cm/s followed by completely zero raw
speed, successive filtered observations are 1.5, 0.75, 0.375 and 0.1875 cm/s.
The 0.5 cm/s stop-entry limit is crossed at the third nominal update, 60 ms;
the motion classifier then needs another 100 ms to confirm STOPPED, roughly
160 ms in total. Sampling phase, partial count windows and scheduling can add
delay. These are analytical examples, not target timing measurements or a
necessary delay for every alternative speed-drop detector.

In the SDK loop, motor update/status sampling occurs before line/IMU processing;
serial-command processing then occurs before the CSV row. `drive_command('S')`
calls `drive_stop()`, which immediately latches/brakes using
`motion_emergency_stop()`. Buddy 2 records ESTOP in its next `motion_update()`.
Consequently a row can combine post-S `running=false` with pre-S encoder/IMU data
and an older EXECUTING motor state. The CONTROL record is printed after command
processing, not a sensor-time acquisition record. Row order and selected nonzero
targets cannot substitute for a timestamped command/brake history.

A correct unexpected-slowdown observer must cancel pending evidence when a stop
is requested/applied and must not confirm using later braking deceleration.
If S precedes sufficient pre-stop evidence, the correct result is unconfirmed;
no software rule can recover a counterfactual uncommanded slowdown afterward.
Do not delay the operator's stop merely to fill a detector confirmation window.

Buddy 2 also clamps the speed-estimation dt to 10-40 ms. After longer scheduler
gaps, the denominator may not represent the actual count interval. A fusion
adapter must reject skipped/late observations beyond its validated timing budget,
not simply mark every repeated status read fresh and valid.

## Ambiguities and exclusions

- A wheel-speed deficit can mean obstacle blockage, increased ramp/friction
  load, weak drive, a wheel jam or missing encoder pulses. Both wheels slowing
  together does not uniquely identify contact. A one-wheel rule is also easily
  confused by turns or one-side ramps.
- A stopped chassis with spinning/slipping drive wheels can continue reporting
  normal encoder speeds. Once the contact transient is over, its IMU may look
  stationary. Driven-wheel odometry plus an accelerometer cannot resolve this
  ground-speed ambiguity reliably.
- Normal acceleration produces expected command-versus-speed error. Commanded
  slowing, turning, reversal guards, terrain scaling and motor faults can also
  reduce measured speed without contact.
- Terrain/side flags are gravity-based, smoothed with alpha=0.15 and hysteresis.
  They can lag a ramp edge; a currently FLAT flag does not prove absence of ramp
  loading. Linear acceleration also disturbs the tilt estimate. Invalid tilt
  must be unknown, never assumed flat. Excluding ramps reduces coverage of real
  contact on a ramp, and cannot guarantee zero false positives at ramp onset.

## Recommended staged approach, not implemented

First examine complete labelled SDK CSV recordings. They already contain most
of the necessary exploratory signals: sample times, counts, filtered wheel
speeds, running/mode/requested targets, motor state/fault, terrain/quality,
residual and scheduling diagnostics, plus CONTROL stop records. Use an external
contact/stop-time annotation (for example synchronised video) and explicitly
account for the mixed-time row described above. The supplied maxima alone are
not enough to set a wheel-drop, IMU or confirmation threshold.

If recordings demonstrate a reproducible **pre-stop** loss of wheel motion that
normal driving/ramp trials do not reproduce, the smallest credible next runtime
change is a separate Buddy 4 observer, leaving the IMU-only observer intact:

1. Arm only after established wheel movement under stable, straight forward
   demand. Gate out startup/acceleration, turns/reversal, stops/braking, motor
   faults, invalid or changing terrain and a validated terrain-transition holdoff.
2. Consume coherent new encoder/count observations and finite non-clipped fresh
   IMU observations with bounded gaps and source-time skew. In SDK manual mode,
   stamp the actual update interval and associate the command history at that
   time; do not timestamp a repeated getter as a new acquisition. Do not use the
   delayed motion label as the deceleration signal.
3. Test sustained count-rate reduction/command deficit relative to the preceding
   steady interval, with temporally consistent IMU evidence. Determine window,
   drop/error and IMU criteria from labelled recordings and quantisation bounds.
   Retaining the 0.30 g IMU threshold as a mandatory fused gate would still miss
   this reported contact; any different fused IMU criterion needs evidence.
4. Cancel immediately on stop, changed demand, faults or unusable/skewed samples.
   Require distinct new observations throughout confirmation. Keep separate
   validity, count/time, cooldown and recovery semantics. Label the result
   **UNEXPECTED_WHEEL_SLOWDOWN** or a qualified possible obstruction/contact,
   never confirmed collision; do not issue motor commands.

This is a candidate architecture, not a tuned or validated detector. If there is
no observable pre-stop speed deficit, this approach should be rejected for the
scenario rather than given arbitrary thresholds to manufacture an event.

For team/MTK integration, the minimum producer contract additionally needs
source timestamp/sequence and validity for each coherent encoder pair/count
interval, plus contemporaneous accepted forward targets and stop/brake intent
with times. The motor owner/controller must supply this context; the isolated
Buddy 4 mailbox cannot invent it. Effective setpoints and applied duty/latch
telemetry would improve exclusion of intentional control actions and diagnosis,
but are not independent proof of contact. No Buddy 2 changes are made here.

Buddy 5 proximity is optional for a conservative wheel-motion anomaly, useful
for corroborating an obstacle hypothesis if its timestamps, status and field of
view are suitable. It is not necessary just to compare speeds with known SDK
manual commands, and it cannot by itself establish touching. If the requirement
is positive physical contact even with wheel slip, an independent contact
measurement such as a bumper switch is the minimum direct evidence; external
ground-motion measurement is another way to address slip, not contact proof.

## Checks and next physical evidence

Both existing runners were executed before adding this report:

- `powershell -NoProfile -ExecutionPolicy Bypass -File app_program/integration_test/test.ps1`:
  PASS. Includes 42 motion and 30 terrain checks, line/drive/scheduler, calibrated
  IMU/quality, median/stable, motion, impact and yaw tests; SDK CSV schemas pass
  at 39 standalone and 79 integration columns.
- `powershell -NoProfile -ExecutionPolicy Bypass -File app_program/buddy4_mtk/test.ps1`:
  PASS. Includes startup protection, console/peak/quality formatting (17-field
  MTK CSV), native task/device/I2C/kernel-wait tests and the integration runner.
- Two existing Buddy 3 unsigned-comparison warnings remain. These simulations
  do not establish physical contact discrimination.

No runtime implementation or new synthetic detector tests are added; no affected
firmware build exists, and no UF2 is rebuilt. The prior SDK integration UF2 remains
`app_program/integration_test/build/buddy234_driving_test.uf2`; it contains the
existing IMU-only vector detector, not encoder-assisted contact detection.

Next evidence: repeated labelled normal starts, steady straight drive, commanded
stops without contact, turns, ramps and compliant low-speed contacts, retaining
several seconds before/after each trial. Preserve the immediate operator stop.
Align count increments, real command/stop times, IMU quality/residual and scheduler
gaps. Determine whether both wheels slow before the stop, slip, or remain
indeterminate; measure misses and false positives on separate trials. Only then
choose criteria and add regressions for those recorded behaviours, timing/fault
boundaries, command cancellation, ramps/turns and recovery/repeated events.

# Buddy 4 encoder motion and turn-rate software checkpoint

Checkpoint date: 29 October 2026. This is software and interface evidence only.
No robot measurements or full-team micro T-Kernel integration are claimed.

## Processing behaviour

`buddy4_motion.c/.h` classifies one coherent pair of signed wheel speeds as
`STOPPED`, `FORWARD`, `REVERSE`, `TURNING_LEFT`, `TURNING_RIGHT` or conservative
`UNKNOWN`. The existing provisional hysteresis, 100 ms confirmation and 100 ms
freshness limits are unchanged. Slow opposing speeds whose mean cancels but whose
difference does not reach the turn threshold remain `UNKNOWN`; zero motion is not
invented from missing data.

`buddy4_encoder_yaw.c/.h` estimates differential-drive yaw rate as

`(right_cm_s - left_cm_s) / 11.3 cm`.

The result is radians per second and is also converted to degrees per second.
Positive is a left/counter-clockwise turn viewed from above. The 11.3 cm Buddy 4
wheel-centre spacing is preserved. Buddy 2 currently defines 12.2 cm in
`motion_config.h`; that unresolved physical-geometry difference must be agreed
from measurement and ownership, not silently reconciled in either module.

Both processors reject invalid/nonfinite speeds, future or already stale samples,
duplicate or reversed acquisition timestamps, and reversed consumer time. Motion
requires a new full confirmation interval after invalidation. Yaw is an
instantaneous estimate and accepts the next newer fresh valid observation.
Numeric yaw fields may retain diagnostic values after invalidation; consumers
must check validity.

## Producer contract

The producer must supply one `buddy4_task_speed_t` per genuinely new encoder
observation through `buddy4_task_submit_speed()`:

- `left_cm_s` and `right_cm_s` are one coherent, atomic pair in signed cm/s.
- Positive means forward and negative means reverse for each wheel. Buddy 2's
  single-phase encoder implementation derives sign from commanded direction, so
  these values are estimates and cannot independently prove actual wheel direction
  during slip, external motion or a direction mismatch.
- `sample_us` is the acquisition time, not submission time. It uses the same
  monotonic operating-time domain and microsecond representation returned by
  `buddy4_task_time_us()`. Wall time, Pico SDK epochs and millisecond values are
  not interchangeable.
- `valid` is true only when both speeds and their common acquisition time are
  usable. Invalid input must be submitted as invalid; it must not be replaced by
  fabricated zero speeds.
- Submit every genuinely new observation. Buddy 2 currently updates motion every
  20 ms, which is an appropriate intended producer cadence. The interval must stay
  below the unchanged 100 ms freshness limit, and repeated reads of the same
  observation must retain the same timestamp. Repeated mailbox observations do
  not advance confirmation.
- The task API is task-context only. Its mutex copies the whole pair atomically;
  it must not be called from an ISR. The Buddy 4 owner task alone mutates the
  classifier and yaw estimator.

## Interface sufficiency and dependency

The Buddy 4 mailbox is sufficient for the required coherent pair, validity and
64-bit acquisition timestamp; no additional Buddy 4 adapter is justified.

Buddy 2's current read-only `motion_get_status()` supplies an atomic pair of
filtered signed speeds, but its public status has no encoder acquisition timestamp
or explicit speed-validity field. Therefore a full-team producer cannot yet meet
this contract merely by periodically copying that status and inventing a timestamp.
The owning Buddy 2/team integration must expose or carry the real observation time
and validity with the coherent speed pair. This checkpoint does not modify Buddy 2
or connect live encoder/motor tasks.

## Software evidence and limits

The existing host suites exercise straight forward/reverse travel, stopped state,
forward and reverse curves, opposite-wheel spins, slow ambiguous cancellation,
hysteresis and exact confirmation boundaries. They also cover signed yaw units and
signs, configurable geometry, duplicate/reversed/future/stale times, invalid and
nonfinite speeds, overflow-safe arithmetic, expiry and recovery.

The isolated task regression additionally verifies that a duplicate mailbox value
cannot complete confirmation, nonfinite input invalidates motion/yaw, fresh input
recovers only under each processor's existing rules, and `buddy4_task_get()` expires
both outputs at 100 ms even without another owner-task poll.

The supported `buddy4_mtk/test.ps1` runner passed before edits and again after the
test/documentation changes. The final run passed startup and motor-pin checks,
console checks, native task/device/I2C/kernel-wait suites, Buddy 2 host simulations,
Buddy 3 regressions, Buddy 4 IMU/median/impact/motion/yaw tests and both output-schema
checks. The two pre-existing Buddy 3 unsigned-comparison warnings remain. No runtime
source changed in this checkpoint, so no firmware rebuild was required.

These synthetic checks do not establish numerical yaw accuracy, encoder sign under
real motion, wheel slip, timing on the target, correct physical wheel spacing or
professor acceptance of encoder-derived turn rate without a gyroscope.

# Buddy 4 Tribunal 1 field/range/bit representation preparation

**Prepared:** 29 October 2026

**Scope:** analysis and proposed boundary encodings only. No runtime structure,
algorithm, calibration, threshold, IPC, telemetry or firmware layout has been
changed.

## 1. Source status and rules used

The original Change Requirement 1 document was not found in the current working
tree, repository history, or filename/text search. The repository summaries say
that it requires minimum-bit application data, suitable bit fields or explicit
packing, a shared fault bitmask, compact binary MQTT fields, and before/after RAM,
message-size, correctness and timing evidence. Those summaries are context, not a
substitute for the original document. This table must be cross-checked against the
supplied document before implementation or a compliance claim.

Read-only sources inspected:

- `buddy4_imu.h/.c`, `buddy4_motion.h/.c`, `buddy4_encoder_yaw.h/.c`, and
  `buddy4_impact.h/.c` for processing types, validation, ranges and lifecycle.
- `../buddy4_mtk/overlay/app_program/buddy4/buddy4_task.h/.c` for the isolated
  MTK input and published snapshot. The sensor task owns mutable processing;
  `buddy4_task_get()` copies the published snapshot under a priority-inheritance
  mutex and then expires only the caller's copy. Accessors are task-context only.
- `../buddy1/telemetry.h/.c` and `../buddy1/mqtt_telemetry_bridge.h` for Buddy 1's
  current by-value, non-blocking telemetry queue and MQTT handoff.
- `../shared/robot_messages.h` for a separate 20-byte shared event envelope.
- `../shared/terrain_link.h` and Buddy 2's `motion_control.h` for the current
  terrain handoff and team-owned motion/fault domains.

“Actual range” below means a range enforced by current code or its fixed source
type. “Default-configuration numerical range” is narrower and is not a general
contract because callers may supply other finite calibration values. “Proposed
operating range” is a candidate contract that must be accepted and enforced at an
encoding boundary before its bit calculation is valid.

Internal filtering, trigonometry, calibration, peak estimation, impact detection
and yaw estimation should remain `float`. A compact telemetry or stored-event
encoding must be derived after processing, with explicit range checks; it must not
reduce internal processing precision.

## 2. Buddy 4-owned proposals

### 2.1 States and result codes

| Field/group | Current source and type | Actual enforced range | Proposed encoding and calculation | Invalid/overflow behaviour | Ownership/concurrency |
|---|---|---|---|---|---|
| Terrain state | `buddy4_output_t.terrain`, `buddy4_task_snapshot_t.terrain`: `int` | `classify_angle()` produces -1 downhill, 0 flat, +1 uphill | 2 bits: four codes are enough for downhill, flat, uphill and invalid/reserved; `ceil(log2(3 + 1)) = 2` | Invalid code is distinct from flat. Reject any source value outside -1..+1. | Buddy 4 owns classification. Buddy 2/dashboard must agree code meanings at an interface. |
| Side state | `buddy4_output_t.side`, event `side`, MTK `side`: `int` | -1 left wheel up, 0 level, +1 right wheel up | 2 bits by the same `3 + 1` calculation | Invalid code is distinct from level. Reject other source values. | Buddy 4 owns classification; dashboard labels are shared. |
| Encoder motion estimate | `buddy4_motion_state_t` enum; MTK `motion_state`: `int` | Values 0..5: UNKNOWN, STOPPED, FORWARD, REVERSE, TURNING_LEFT, TURNING_RIGHT | 3 bits because `ceil(log2(6)) = 3`; codes 6..7 reserved | `motion_valid=false` means UNKNOWN/unusable; decoder rejects 6..7. Do not merge UNKNOWN with STOPPED. | Buddy 4 owns estimator; Buddy 2 owns the speed producer and must agree timestamp/freshness. |
| IMU poll result | `buddy4_result_t` enum; MTK `poll_result`: `int` | 0..3: FRESH, STALE, IO_ERROR, NOT_READY | 2 bits because `ceil(log2(4)) = 2` | All four codes used; corrupt code must be caught by enclosing version/length/integrity rules. | Buddy 4-owned diagnostic. This is not a substitute for validity or a shared fault code. |

Use explicit unsigned masks/shifts for any packed representation. C enum width is
implementation-defined, and a C bit-field's allocation order, container width,
padding and byte order are implementation-defined; neither is a wire contract.

### 2.2 Numeric measurements

| Field/group | Current source and type | Actual enforced/default range | Proposed operating range and precision | Proposed bits and encoding | Invalid/overflow behaviour |
|---|---|---|---|---|---|
| Raw acceleration X/Y/Z | `buddy4_output_t.raw[3]`, MTK `raw[3]`: `int16_t` | Exactly -32768..32767 sensor counts | Preserve exact count when raw diagnostics are required | 16-bit signed two's complement per axis; already minimal for all 65,536 patterns | No spare invalid code; pair with IMU validity. Never saturate raw counts. |
| Uncorrected acceleration | `uncorrected_accel_g[3]`: `float`; raw × 0.000061 g | Exactly -1.998848..+1.998787 g from the signed raw count | Usually omit from compact operational payload; if required, -2.100..+2.100 g at 0.001 g | 13-bit signed milli-g supports -4096..+4095; valid -2100..+2100, reserve -4096 invalid. `ceil(log2(4201 valid values + 1 invalid)) = 13` | Out-of-envelope becomes invalid plus overflow/range flag; do not clamp silently. |
| Corrected acceleration X/Y/Z | `accel_g[3]`, MTK `acceleration_g[3]`: `float` | Configuration checks only finite bias and finite positive scale, so no finite general range is enforced. With current defaults and raw extrema: X -2.06334..+1.98631 g, Y -2.10638..+2.02793 g, Z -2.10843..+1.93809 g | Candidate -2.500..+2.500 g at 0.001 g; this includes the complete default numerical envelope, not every allowed configuration | 13-bit signed milli-g; 5,001 values plus invalid = 5,002, so `ceil(log2(5002)) = 13`; reserve -4096 invalid | Encoder validates finite and envelope. Outside range: invalid + range flag. Internal float unchanged. Revisit if calibration or full-scale changes. |
| Magnitude and possible-impact deviation | `magnitude_g`, `buddy4_impact_output_t.magnitude_g/deviation_g`, MTK `magnitude_g/impact_deviation_g`: `float` | Nonnegative for finite inputs; no finite general bound under configurable calibration. Current default component extrema imply a numerical maximum about 3.625 g. | 0.000..4.000 g at 0.001 g | 12-bit unsigned milli-g: values 0..4000; reserve 4095 invalid. 4,002 required codes fit in 4,096 | 4001..4094 reserved; >4 g or nonfinite is invalid + range flag, not silent saturation. Detector thresholds remain float and unchanged. |
| Pitch/roll, raw and filtered | `pitch_deg`, `roll_deg`, filtered forms, MTK `pitch_deg/roll_deg`: `float` | `atan2` contributes -90..+90 degrees, but arbitrary finite configured offsets mean no finite general bound. Current defaults give pitch -90.97..+89.03 and roll -90.61..+89.39 degrees; the EMA stays within finite observed inputs | Candidate -90.00..+90.00 degrees at 0.01 degree for externally useful vehicle attitude. Default offset edge values slightly beyond this envelope require invalid/range handling or an agreed wider envelope | 15-bit signed centidegrees: -9000..+9000 plus invalid = 18,002 codes; `ceil(log2(18002)) = 15`; reserve -16384 invalid | Do not clamp. The team must decide whether the envelope should be widened to include default mathematical edge cases before implementation. Internal floats remain unchanged. |
| Wheel-height difference | `wheel_height_cm`, peak/median heights, MTK `height_cm`, event heights: `float` | Configuration enforces finite spacing >0 but no upper bound; therefore no general finite height bound. With confirmed 11.3 cm spacing and intended absolute roll ≤90 degrees, 0..11.3 cm | Candidate 0.00..11.30 cm at 0.01 cm | 11-bit unsigned centi-cm: 0..1130 plus invalid = 1,132 codes; `ceil(log2(1132)) = 11`; reserve 2047 invalid | 1131..2046 reserved. Invalid geometry, nonfinite, negative or >11.30 becomes invalid + range flag. Revisit if team adopts Buddy 2's recorded 12.2 cm geometry. |
| Encoder yaw-rate estimate | `buddy4_encoder_yaw_output_t` rad/s and deg/s floats; MTK `encoder_yaw_deg_s` | Only finite wheel speeds and finite positive spacing are enforced, so no finite general bound. Buddy 2 currently limits each commanded speed to ±40 cm/s; at 11.3 cm spacing the corresponding differential bound is about ±405.64 deg/s | Candidate -500.0..+500.0 deg/s at 0.1 deg/s | 14-bit signed deci-deg/s: -5000..+5000 plus invalid = 10,002 codes; `ceil(log2(10002)) = 14`; reserve -8192 invalid | Outside range/nonfinite invalid + range flag. Keep the rad/s internal value; transmit one agreed unit, not both. Accuracy remains physically unvalidated. |

The precision values above are representation proposals, not accuracy claims.
They preserve substantially finer resolution than the current displayed 0.01
degree/centimetre output where applicable, but sensor noise and physical accuracy
must still be measured.

### 2.3 Validity, event and diagnostic flags

Current `bool` members are separate C objects whose size and surrounding padding
depend on the ABI. A proposed 16-bit logical status mask is:

| Bit | Meaning | Current source |
|---:|---|---|
| 0 | fresh IMU sample valid | `buddy4_output_t.valid` / snapshot `valid` |
| 1 | MTK timing fields valid | `timing_valid` |
| 2 | height geometry confirmed | `height_geometry_confirmed` (not presently copied into the MTK snapshot) |
| 3 | transient/bump pulse | `transient_event` / `bump_event` |
| 4 | side event ended pulse | `event_ended` / `side_event_ended` |
| 5 | saved stable peak valid | `peak_confirmed` / `last_event.stable_valid` |
| 6 | saved median candidate valid | `median_candidate_valid` / `last_event.median_valid` |
| 7 | possible-impact detector current data valid | `impact_valid` |
| 8 | possible-impact one-sample pulse | `possible_impact` |
| 9 | motion estimate valid | `motion_valid` |
| 10 | yaw estimate valid | `yaw_valid` |
| 11 | saved device error present | `saved_error_present` |
| 12 | saved error was a read | `saved_error_read` |
| 13 | saved error timestamp valid | `saved_error_time_valid` |
| 14 | numeric value outside agreed encoding envelope | proposed boundary encoder diagnostic |
| 15 | at least one transmitted counter saturated | proposed boundary encoder diagnostic |

This is 16 defined logical bits in a fixed 16-bit unsigned mask. Pulses must be
distinguished from retained state: a slow consumer may miss bits 3, 4 and 8.
Counts and latest-event timestamps are retained separately. A versioned decoder
must define how future meanings are introduced; no flag bit is currently spare.

Configuration and internal lifecycle booleans need separate treatment. The
configuration's `mounting_offsets_need_recalibration` and
`centre_spacing_confirmed` should be sent as infrequent version/configuration
metadata if the dashboard needs them; the latter currently feeds
`height_geometry_confirmed`. Internal `ready`, `has_sample`, `filter_valid`,
`bump_armed`, `had_bump`, impact priming/arming and motion/yaw clock-history flags
control algorithms and should remain Buddy 4 processing state rather than being
added to every telemetry sample.

### 2.4 Events, counts, sequences and time

| Field/group | Current source/type and actual behaviour | Proposed contract and bit calculation | Overflow/invalid behaviour | Ownership/dependency |
|---|---|---|---|---|
| Side-event result | Core `side_event_t`; MTK `buddy4_task_event_t` retains side, 64-bit start/end, two validity flags and two float heights | Use the state/angle/height encodings above. A full event candidate is side 2 + start/end time + 2 validity + stable roll 15 + stable height 11 + median roll 15 + median height 11. With 27-bit times this is 110 logical bits before an event count/framing; with 32-bit times it is 120. The current MTK snapshot does not retain the two roll values, so it cannot populate the full candidate without a later reviewed layout change | Invalid numeric codes must accompany cleared validity. Latest-only retention means intermediate details may be lost | Buddy 4 owns event production. Dashboard must agree latest-only versus queue semantics. |
| Possible-impact result | `event_count:uint32_t` saturates at `UINT32_MAX`; `event_us:uint64_t` retains latest time; pulse lasts one input sample | Candidate telemetry: 16-bit saturating per-session count + chosen timestamp + 12-bit latest deviation if required. At 27-bit time this is 55 logical bits; at 32-bit time, 60 bits, before framing/status | Wire count 0..65534, 65535 = saturated. Preserve “possible impact”, never “collision”. Count jumps disclose lost intermediate events; only latest timestamp exists | Buddy 4 owns observation. Physical meaning and dashboard wording require team agreement. |
| Processing/error counters | Core `error_count:uint32_t` increments and currently wraps; MTK `io_errors` copies it. `skipped_slots:uint32_t` saturates. `completed_events:uint32_t` increments and wraps | Candidate 16-bit operational counters: 0..65534, 65535 saturated, because the proposed per-session reporting bound is ≤65,534. This is a policy bound, not an existing enforced bound | Encoder saturates and sets a counter-overflow flag; never wrap on the wire. Internal behaviour stays unchanged until separately reviewed | Buddy 4-owned counters; reset/session semantics shared with dashboard. |
| MTK/sample sequence | MTK `sequence:uint32_t` and SDK logger sequence increment modulo 2^32 | Candidate 16-bit on-wire sequence modulo 65,536. All 16 bits are data; validity is separate | Receiver performs modulo loss/reorder arithmetic. Internal 32-bit sequence stays unchanged | Packet-loss rules require Buddy 1/dashboard agreement. |
| Acquisition/event timestamps | Core and MTK use `uint64_t` microseconds. MTK operating clock has 10 ms resolution and converts to microseconds; SDK clocks have finer resolution | No minimum can be final without maximum session duration. Candidate for an agreed ≤24 h run: unsigned milliseconds 0..86,400,000 needs `ceil(log2(86,400,001)) = 27` bits. A byte-friendly shared alternative is 32-bit milliseconds, wrapping after about 49.71 days | Validity flag carries invalidity; define modulo wrap comparison. Event ordering across reboot needs boot/session ID or external epoch | Epoch, resolution, wrap and reboot semantics are shared decisions. Do not mix SDK microseconds and MTK milliseconds without conversion. |
| Durations/ages | MTK `poll_duration_ms`, `poll_interval_ms`, `sample_age_ms`: `uint32_t`; `UINT32_MAX` means unavailable. `elapsed_ms()` also returns it for reversed/too-large intervals | Candidate 0..4094 ms plus 4095 invalid, unsigned 12-bit milliseconds (`4096` total codes) for these short diagnostics | Values above 4094 encode invalid/overflow plus range flag, not 4095 as a real duration | Buddy 4 diagnostic; confirm whether these belong in production telemetry. |
| Saved error time/register | `saved_error_us:uint64_t`; `saved_error_register:uint8_t` masked with `0x7f`; read/time flags separate | Chosen timestamp encoding + 7-bit register address + flags | Invalid time via flag; register is exactly 0..127 | Buddy 4/device diagnostic. |
| Timeout trace | `saved_timeout_trace[32]` of `unsigned int` (32-bit on target) | Keep as a local diagnostic record, not a periodic compact payload: it is 1,024 logical value bits before metadata | Existing sentinel values such as `UINT32_MAX` retain their documented meanings | Buddy 4/device/kernel diagnostic; any remote transfer needs a separate versioned diagnostic message. |

### 2.5 Fault representation

Current `device_error` and `saved_error_code` are target `int`/`ER` values; their
meaning includes kernel/device codes and cannot truthfully be reduced to a small
raw field without a mapping. Keep the raw signed code locally for diagnosis.

A candidate normalized 3-bit telemetry fault class is: 0 none, 1 not-ready,
2 stale/no-new-data, 3 bus/short-transfer, 4 timeout, 5 clock/timestamp, 6 sensor
verification/configuration, 7 other. Three bits are sufficient for these eight
defined classes (`ceil(log2(8)) = 3`), but this is a lossy classification and
requires Buddy 1/dashboard agreement. If raw error transport is required, use an
additional signed fixed-width field with an explicitly versioned error namespace;
do not cast implementation-specific `ER` values into the 3-bit class.

### 2.6 Internal estimator state that should remain internal

`side_event_t` includes an 11-sample float/time median window, `median_count`
(`uint8_t`, algorithmically limited to 0..11 and therefore four logical count
bits), window-active state, window timestamps and float min/max working values.
The impact detector similarly retains float baseline/configuration and pending/
calm sample counts; motion and yaw retain confirmation and clock history. These
are working-state requirements, not proposed periodic payload fields. Packing or
quantizing them could change numerical or temporal behaviour and requires separate
algorithm evidence. Tribunal telemetry packing should initially encode outputs
and retained summaries at the publication boundary, leaving this internal state
unchanged.

## 3. Shared decisions requiring Buddy 1, Buddy 2 or dashboard agreement

1. **Original requirement wording.** Obtain and cross-check Change Requirement 1,
   including whether “minimum bits” applies to internal processing state, IPC,
   MQTT payloads, or all three.
2. **Message framing.** Choose version, type, length, byte order, reserved-bit rule,
   integrity/error handling and whether Buddy 4 uses one periodic frame plus event
   frames. Explicit masks/shifts over byte arrays are preferred for a wire format.
3. **Telemetry envelope.** Buddy 1 currently copies a generic
   `telemetry_message_t` containing source/type/flags, two `int32_t` values and a
   `uint32_t` sequence. It offers only 64 generic value bits per message and no
   timestamp. The MQTT bridge copies the same fields; no Buddy 4-specific binary
   schema or dashboard decoder exists. Decide whether to version/extend this
   envelope or send multiple typed records.
4. **Time contract.** Agree epoch (boot-relative versus external), 1 ms versus
   another precision, maximum uninterrupted session, wrap comparison, reboot/session
   identification and event ordering. The 27-bit proposal is valid only for the
   explicitly bounded 24-hour session.
5. **Numeric envelopes.** Approve acceleration, angle, height and yaw ranges and
   resolution using hardware evidence. In particular, reconcile Buddy 4's 11.3 cm
   wheel-centre spacing with Buddy 2's recorded 12.2 cm value.
6. **Motion producer.** Buddy 2 must provide one coherent signed left/right speed
   observation and timestamp. Its current command bound is ±40 cm/s, but measured
   speed and encoder fault ranges need qualification.
7. **Validity and faults.** Agree the 16-bit status mask, normalized fault classes,
   ownership of a shared fault bitmask, and whether raw target error codes are ever
   transported.
8. **Event retention.** Agree latest-only versus queued side/possible-impact events,
   counter reset/session semantics, saturation reporting and dashboard behaviour
   when counts jump. A possible-impact event remains observational.
9. **Precision versus accuracy.** Representation resolution must not be presented
   as sensor, height, yaw or collision-classification accuracy.

## 4. Logical payload bits versus C object/RAM size

The tables count logical information bits. They do not predict `sizeof` a C
structure, queue allocation, stack use, ELF RAM, MQTT framing or network bytes.
Byte alignment, member alignment, tail padding, enum width, `bool` size and ABI
rules can make an unpacked C structure much larger. C bit-fields may also insert
implementation-defined padding and are unsuitable as a portable serialized form.

For comparison only:

- Buddy 1's current field order normally makes `telemetry_message_t` 16 bytes on
  the RP2040 ABI (1 + 1 + 2 + 4 + 4 + 4, naturally aligned), but there is no
  `_Static_assert` for it. Its source allocation formula would reserve 256 bytes
  for an eight-message queue when `sizeof(UW)=4` and the message is 16 bytes, and
  another 256 bytes for the MQTT outbox when enabled. These are source-derived
  expectations, not measurements of an integrated image.
- `robot_event_t` is separately asserted as 20 bytes; it includes a 32-bit
  millisecond timestamp, two 32-bit values, sequence, source/type and flags.
- A candidate periodic Buddy 4 logical set consisting of states/result/status/
  normalized fault (28 bits), corrected XYZ + pitch + roll + height + yaw
  (94 bits), 16-bit sequence and timestamp is 165 bits with a 27-bit timestamp or
  170 bits with a 32-bit timestamp. Byte padding alone rounds these to 21 or 22
  bytes before version/type/length/integrity or transport framing. This is an
  analysis bundle, not an approved message.

## 5. Measurements and implementation evidence still pending

No before/after result exists because Tribunal packing has not been implemented
and the final integrated baseline is not available. Before changing layouts:

1. Save a working integrated baseline and record compiler, flags, target ABI and
   enabled tasks/features.
2. Measure target `sizeof` and relevant member offsets for Buddy 4 processing,
   snapshot/event types, Buddy 1 telemetry/MQTT handoff types and queue storage.
3. Record ELF `.data`/`.bss`, task stacks/high-water marks, heap/pools, queue RAM,
   serialized application payload bytes and complete MQTT/network bytes.
4. Measure producer encode time, mutex hold/copy time, queue latency/drop counts,
   consumer decode time, sampling jitter/skips and USB/network interaction.
5. Implement explicit encode/decode range and reserved-code tests, round-trip and
   boundary tests, malformed-payload tests, wrap/saturation tests, event-loss tests
   and concurrency tests.
6. Repeat functional regressions and physical qualification. Compare like-for-like
   baseline and changed builds; do not attribute unrelated integration changes to
   representation packing.

Packing a snapshot does not make multiword access atomic. The current MTK design's
single writer plus mutex-protected whole-snapshot copy is the safe ownership model
to preserve. An ISR must not take that mutex. If a packed representation is shared
across tasks or an ISR, publish a complete immutable copy under the existing task
lock or use a separately reviewed lock-free protocol; `volatile` alone is not
synchronization.

## 6. Preparation status

This document completes the Buddy 4 field/range/bit **preparation checkpoint**.
It does not implement Tribunal 1, approve a team telemetry schema, establish
minimum bits for still-unbounded requirements, measure before/after savings, or
demonstrate compliance. Those items remain pending the original document, team
decisions and an integrated baseline.

# Actual 13:46 SDK obstruction capture analysis, 2 November 2026

The raw-angle eligibility gates, not repeated CSV observations, are sufficient
to explain why this recording never establishes. No additional runtime defect
was demonstrated. Production code, thresholds and firmware are unchanged.

## Input and parsing

Operator attachment: `C:/Users/yongk/Downloads/Pasted text(20261102-054705).txt`.
SHA-256: `5c9d7af7990b32e1c35c713bc45879dc38cecd4567baee71dc444c889acbc333`.
The attachment is treated only as recorded data, not as instructions.

Strip the serial monitor's `HH:MM:SS:mmm -> ` prefix; accept only complete
86-column numeric-sequence rows using the existing SDK header. This retains
996 complete rows, including 230 with `running=1`, from sequence 465 to 696.
Driving rows span firmware output times 33.002179 to 37.621904 s (4.619725 s).
The START record is at 32.989691 s and SERIAL_STOP at 37.641930 s.

The capture begins with a truncated row and contains a malformed joined line;
driving sequences 468/469 are not recoverable as complete rows. They are neither
invented nor interpreted as physical missing sensor updates. The 60 ms gap
between retained rows at that point is a recording gap, not evidence of a
motor/IMU freshness fault by itself.

## Exact eligibility findings

| Check on complete driving rows | Result |
| --- | --- |
| Logged obstruction valid | 108 true, 122 false |
| Logged established / events / count | All zero |
| `abs(raw pitch)>3 deg` | 60 rows; every one invalid |
| `abs(raw roll)>2 deg` | 69 rows; every one invalid |
| Either raw-angle gate (21-row overlap) | 108 rows; every one invalid |
| Filtered pitch/roll outside the adapter limits | Zero |
| Wheel-speed difference >0.75 cm/s | One row, sequence 470: 0.933/1.686, difference 0.753 cm/s; invalid |
| Outside the 2.1-3.9 cm/s moving band on either wheel | 26 rows, principally startup; 13 also invalid for another gate |
| FLAT / SIDE_LEVEL / GRAVITY_LIKE | All driving rows |
| Motor state / fault | EXECUTING / NONE on every driving row |
| Terrain speed scale / assist | 1.000 / 0.000 on every driving row |
| Motor / IMU skipped counters | Zero throughout driving |
| Logged IMU or encoder snapshot age >40 ms | Zero |
| Logged IMU/encoder snapshot skew >30 ms | Zero |
| Maximum accumulated motor update runtime | 358 us, below the 5 ms adapter limit |

Raw pitch ranges from -7.788 to +8.994 degrees and raw roll from -6.354 to
+2.709 degrees. Filtered pitch stays within -1.540 to +1.171 degrees and filtered
roll within -1.805 to -0.253 degrees. FLAT and GRAVITY_LIKE therefore do **not**
imply passing the adapter's separate raw-angle checks.

The longest uninterrupted retained sequence passing the raw gates AND the moving
speed band spans only **139,991 us**, from IMU timestamps 36.740309 to 36.880300 s
(eight rows). The 500,000 us establishment criterion cannot finish. The recorded
speed/angle fixture reproduces this without any timing-fault injection.

In `buddy4_obstruction_sdk_poll()`, a raw-angle failure makes `usable=false`.
The `if (!usable)` branch calls `buddy4_obstruction_cancel()` and clears adapter
pending state. Core `clear_evidence()` clears `establishing`, `established`,
pending slowdown and demand identity. On the next usable moving observation,
`moving_since_us` starts again. This is the exact reset mechanism for those
108 measured raw-angle violations. It is an intentional existing protection,
not a duplicate-timestamp error.

The one speed-asymmetry rejection clears evidence inside the core's speed gate.
For the remaining 13 invalid rows, the current raw angles/symmetry pass but every
row immediately follows a raw-angle rejection: sequences 510, 538, 543, 548,
564, 573, 583, 591, 616, 632, 647, 667 and 681. Twelve retain the preceding
row's motor timestamp. They show previously cancelled assessment before another
eligible motor observation has been accepted, not proof of a new timing fault.
Sequence 591 has an intervening motor observation; the preceding cached IMU
at sequence 590 already had roll -2.415 degrees. Cancellation using that cached
context before the next good IMU is consistent with the SDK order. The CSV does
not record every intervening callback, so this last ordering is an inference,
not a claimed exact reconstruction of unlogged execution.

## Timing contract distinctions

| Situation | Existing behaviour / required interpretation |
| --- | --- |
| Waiting for new IMU with a qualified pending motor snapshot | Equal fresh IMU timestamp leaves it pending. No evidence is added. Existing age/skew/eligibility checks and getter expiry still apply. |
| Repeated CSV rows or getter polls | The last accepted observation timestamp can repeat. No new core update is submitted; repetition itself neither advances nor resets establishment. |
| Duplicate source callback | A repeated/backward motor callback is not a new observation. Timing qualification rejects it. This is different from reading the same output twice. |
| Genuinely stale/invalid input | Unusable context, invalid/nonfinite/clipped input, source age/skew violation, bad cadence/runtime/skips, reversed source time or expired core evidence cancels. |
| Fresh qualified motor update with new usable IMU | Consumed once with its original SDK update-observation time. Continuous eligible moving updates may complete 500 ms. No acquisition time is invented. |
| A second motor callback while an earlier pairing remains pending | Current contract rejects continuity. This remains conservative; the supplied capture does not establish it as the cause of the failed 500 ms window. |

`possible_obstruction_observation_us` repeats across 115 pairs of adjacent
driving rows. In 97 raw-angle-rejection rows it is unchanged from the preceding
row: cancellation retains the historical accepted timestamp. The live encoder
snapshot timestamp takes 194 distinct values across the 230 rows. An unchanged
observer timestamp is therefore not evidence that Buddy 2 stopped updating.

## Deterministic regression and decision

The new fixture contains the actual six angle/speed values from all 230 complete
driving rows, with source sequence numbers and attachment hash. The SDK adapter
test drives the real `scheduler.h` at 20 ms, using SDK motor/IMU/serial/poll order,
synthetic bounded loop/I2C/CSV durations and repeated output/getter observations.
These synthetic timings are explicit because the hardware CSV does not contain
every callback. It does not claim an exact replay of hidden execution.

- Recorded angles/speeds: never establishes, no events; many output timestamps
  repeat as in the recording.
- Counterfactual test input with only raw angles set within the existing gates:
  same speeds/scheduling establishes and stays established after warmup. No
  production threshold or filtered angle is changed.
- Repeated output/getter observations leave the evidence time and establishment
  unchanged. Existing phase-wait, stale/missing-source, duplicate-source, stop,
  fault, ramp/quality, recovery/cooldown and repeated-event regressions still pass.

The observer can establish during continuously eligible synthetic straight
driving. This real straight-driving capture is **not continuously eligible under
the present raw-angle policy**. The instantaneous accelerometer-derived angles
vary substantially while filtered orientation stays level. The recording alone
does not separate vibration/linear acceleration from actual rapid chassis tilt.

Removing/widening raw gates, replacing them with filtered angles or accumulating
warmup across their failures would change false-positive/ramp protections. Those
are detector-design changes requiring labelled normal-driving AND ramp/contact
comparisons, not a demonstrated timing bug fix. No such change is made merely to
force establishment. The unresolved blocker is an eligibility policy that rejects
ordinary driving, plus accelerometer-only tilt ambiguity, rather than an exposed
encoder acquisition timestamp that Buddy 2 can supply here.

## Files, verification and firmware

Only these files change in this follow-up:

- `app_program/integration_test/test_obstruction_sdk.c`: recorded-vector and
  cooperative-scheduling/duplicate-output characterization regression.
- `app_program/integration_test/fixtures/obstruction_drive_20261102_1346.inc`:
  actual angle/speed fixture with provenance.
- `app_program/buddy4/OBSTRUCTION_1346_CAPTURE_ANALYSIS.md`: this report.

Results: baseline integration runner PASS; expanded integration runner PASS;
existing MTK full runner PASS, including native task/device/I2C/kernel-wait and
updated adapter tests. CSV schemas remain 39/86/17 fields and pass. Existing
Buddy 3 comparison warnings remain. Preservation and whitespace checks PASS.

No production code or build configuration changed, so no firmware rebuild is
needed and **no updated UF2 is produced**. Existing SDK UF2 remains:
`C:/Users/yongk/OneDrive/Documents/GitHub/INF2004-PicoCar/app_program/integration_test/build/buddy234_driving_test.uf2`

Its unchanged SHA-256 is
`b0979206cb87d6d0e8d3cd9d6b830b51a2c8a408381bf12b909c4c78842f004a`.
This existing image still contains the raw-angle eligibility policy analysed
above. Buddy 2/3 sources, calibration, terrain/height algorithms and unrelated
pre-existing changes are preserved. No flashing, motors, commit or push.

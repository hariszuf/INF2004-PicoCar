# SDK obstruction establishment timing fix, 2 November 2026

A demonstrated adapter defect could repeatedly reset the unchanged 500 ms
establishment window during normal driving. The SDK adapter is corrected;
thresholds, core obstruction processing and the IMU impact observer are unchanged.

## Root cause and evidence limits

The previous adapter consumed each pending motor observation on its first main
loop poll. It required an IMU timestamp strictly newer than the one already used.
When the next IMU poll had not happened yet, the previous IMU could still be
fresh and valid, but its equal timestamp caused `.valid=false` to be sent to the
core. That cleared establishment/pending evidence and consumed the motor snapshot.
The subsequent new IMU sample could not repair the lost snapshot. Ordinary poll
ordering was therefore treated as a data fault.

The original test helper always supplied a new IMU before polling the adapter,
so it missed this phase dependency. A new regression supplies 20 ms motor
updates at 3.18/3.09 cm/s for four seconds, with alternating IMU arrival before
and shortly after the first main-loop adapter poll. Against the pre-fix adapter,
it failed at a valid-input assertion: the delayed cycle cancelled assessment.
Repeating this pattern resets the moving window before it can span 500 ms.
After the fix, the same sequence establishes and remains established after warmup.

This reproduces a software mechanism consistent with the reported intermittent
validity and absent establishment. The supplied hardware summary is not a full
timestamped recording, so it does not prove this was the only active reset gate.
Legitimate resets remain possible from angles, speed asymmetry, tracking error,
skips or age/skew limits even with zero I2C errors and no motor stall.

## Focused correction

An equal, still-fresh IMU timestamp now leaves the motor snapshot pending. The
adapter waits for the next genuinely new IMU sample without submitting duplicate
evidence or advancing the observation time, establishment window or slow span.
When the new usable IMU arrives, that motor snapshot is consumed exactly once
using its original SDK update-observation time, not a retimestamped acquisition.

All eligibility and age/skew checks still run on every poll. Stops, mode/target
changes, faults, unsuitable terrain/tilt, invalid/clipped input and stale sources
still cancel. Bad motor timing and reversed IMU timestamps cancel immediately.
Known nonfinite wheel speeds are now checked before waiting too. If another
motor callback arrives before the pending pairing is consumed, the existing
timing qualification rejects continuity. Waiting cannot bridge missing updates.

No timing allowance, speed/angle threshold, 500 ms establishment, 100 ms slowdown
confirmation or 1 second cooldown was loosened. No motor command is introduced.
CSV remains 86 columns with unchanged field names and timestamp semantics.

## Changed files in this follow-up

- `app_program/integration_test/buddy4_obstruction_sdk.c`: wait for a new IMU
  rather than invalidate on a still-fresh equal timestamp; keep genuine faults
  immediately cancelling and reject nonfinite speeds before waiting.
- `app_program/integration_test/buddy4_obstruction_sdk.h`: document bounded
  pending-pairing behaviour and lack of evidence advancement while waiting.
- `app_program/integration_test/test_obstruction_sdk.c`: phased four-second
  driving regression, repeated waiting polls, deferred confirmation, S during
  waiting, absent source progress, unconsumed pairing and nonfinite speed checks.
  The former equal-timestamp fault case now tests truly reversed IMU time.
- `app_program/buddy4/OBSTRUCTION_ESTABLISHMENT_FIX.md`: this report.

Other pre-existing changes are preserved byte-for-byte. Buddy 2/3 C/H sources,
IMU calibration/terrain/height processing, main scheduling and core obstruction/
impact algorithms are unchanged. No flashing, motor operation, commit or push.

## Verification and artifact

- Existing integration host runner before modification: PASS.
- New phased regression against pre-fix adapter: expected FAIL.
- Updated integration host runner: PASS, including new pairing regressions and
  all existing obstruction/core, IMU/impact, motion/yaw, drive/scheduler and
  Buddy 2/3 simulation tests. Existing Buddy 3 comparison warnings remain.
- CSV schema/printf checks: PASS, standalone 39 and SDK integration 86 columns.
- Existing MTK full runner: PASS, including unchanged 17-field quality CSV,
  native task/device/I2C/kernel-wait and updated integration regressions.
- SDK 2.3.1 Pico W integration firmware: PASS.
- Preservation hashes and whitespace checks: PASS.

Exact rebuilt SDK UF2:
`C:/Users/yongk/OneDrive/Documents/GitHub/INF2004-PicoCar/app_program/integration_test/build/buddy234_driving_test.uf2`

Size: 115,712 bytes. SHA-256:
`b0979206cb87d6d0e8d3cd9d6b830b51a2c8a408381bf12b909c4c78842f004a`.
ELF text/data/bss: 61,724 / 0 / 4,492 bytes.

Next operator check: repeat normal four-second manual straight driving on flat
ground, recording the complete CSV. Under continuous eligible paired observations,
establishment should appear after 500 ms. If it still resets, retain the complete
rows including raw/filtered angles, speed pair, source times and scheduling
diagnostics to identify the remaining gate. Do not lower thresholds from summary
statistics. Then repeat normal stops/turns/ramps before compliant contact tests,
keeping the immediate S stop. Hardware obstruction/collision discrimination
remains unvalidated; this fix only establishes the software timing behaviour.

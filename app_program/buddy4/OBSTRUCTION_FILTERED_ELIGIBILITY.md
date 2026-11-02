# Filtered-orientation obstruction eligibility, 2 November 2026

The SDK possible-obstruction adapter now uses valid **filtered** pitch/roll
within the existing +/-3/+/-2 degree limits, together with FLAT/SIDE_LEVEL and
absence of motor terrain compensation. Raw angles must remain finite but their
instantaneous amplitude no longer vetoes eligibility. The core obstruction and
vector-IMU impact algorithms are unchanged. This is a supported eligibility-policy
correction for the observed driving envelope, not validated collision detection.

## Investigation before implementation

The existing integration runner passed before edits. The 13:46 fixture retains
all 230 complete driving rows. Raw angles violate +/-3/+/-2 in 108 rows, while
filtered pitch stays -1.540..+1.171 degrees and filtered roll -1.805..-0.253.
The old raw-amplitude veto repeatedly reset the 500 ms moving window. An updated
normal-driving regression was run against the old policy and failed at the
establishment assertion before production code was changed.

The IMU already filters orientation with alpha=0.15. Existing hysteresis enters
pitch terrain at +/-6 degrees and returns at +/-3, and enters wheel-side tilt
at +/-4 degrees and returns at +/-2. Those processing constants and the mounting
correction are untouched. The adapter keeps both the filtered return-to-level
limits and the hysteresis state: it can reject tilt before the state enters,
and a latched nonlevel state still rejects even at an angle boundary.

Acquisition validity is not tilt suitability. Existing `tilt_valid` requires
finite non-clipped acceleration with magnitude within 0.8..1.2 g. Bad tilt clears
terrain flags to zero but also makes filtered angles NaN. The adapter still
requires ready/acquisition/acceleration/tilt validity and finite raw/filtered
angles; zero flags alone never mean usable flat ground. Gravity-like magnitude
is necessary, not proof of pure gravity or correct attitude under acceleration.

Four frozen after-correction captures were inspected and replayed before the
production edit. They use the historical 69-column schema and retain their
original calibration; no raw recording is edited or recalibrated.

| Capture | Complete rows | Filtered roll range | Rows rejected by filtered tilt/state guard | Replay obstruction events |
| --- | ---: | ---: | ---: | ---: |
| `left_moving_after.txt` | 458 | -5.747..+0.138 deg | 101 | 0 |
| `right_moving_after.txt` | 394 | -0.530..+5.666 deg | 104 | 0 |
| `left_held_after.txt` | 373 | -5.020..-4.939 deg | 373 | 0 |
| `right_held_after.txt` | 574 | +4.876..+4.965 deg | 574 | 0 |

Source files and hashes are in `evidence/2026-10-28/manifest.json`. Moving records
contain 177 left and 201 right driving rows. Held recordings contain no actual
driving; their tests deliberately simulate continuing forward demand against an
already established observer to challenge the tilt guard. They are not additional
motor-driven evidence. Malformed rows are rejected (2 left-moving, 1 left-held).

The replays retain recorded angles, speeds, state and sample gaps. Callback/I2C
phase is synthetic, not a reconstruction of hidden releases. Legacy captures
lack modern quality flags and motor terrain telemetry; tests derive only finite,
rail and magnitude suitability and leave motor terrain compensation absent so
that the IMU guard itself must suppress tilt. These bounded software checks
support this change but cannot establish general ramp false-positive rates.

## Focused change and preserved protections

The only runtime behaviour change removes `abs(raw pitch)<=3` and
`abs(raw roll)<=2` from the adapter. Filtered limits and terrain conditions retain
their numerical values; no new filter, mounting offset or terrain calculation
is introduced. Continuous eligible moving observations must still span 500 ms.

Unchanged: valid new encoder observation qualification; bounded motor cadence,
duration and skipped slots; 40 ms age/gap, 30 ms source skew; new-IMU pairing and
bounded waiting; command/mode/turn/stop/fault cancellation; both-wheel slowdown
<=0.5 cm/s and <=20% of demand for 100 ms; 1 second cooldown and renewed moving
establishment; counter/history/recovery semantics; manual straight-driving
restriction. No motor action is added. The SDK observation time is still not an
invented encoder acquisition timestamp. CSV remains 86 columns.

The actual 230 normal-driving vectors now establish at **1.182 s of the synthetic
cooperative replay**, with 171 established rows and no events. Startup speeds
delay the first qualifying 500 ms; this timing is not a prediction for hardware.

Tests also cover signed sustained ramp onsets (8 degree pitch / 5 degree roll),
where the alpha=.15 filtered guard crosses before the 100 ms near-stop candidate
can confirm. A filtered-angle violation on the would-be confirming sample aborts
it. Returning flat requires the full moving warmup again. Existing normal-stop,
turn/mode, fault, invalid/clipped/nonfinite, stale/missing/duplicate data, source
timing, cooldown and repeated-event regressions remain active and pass.

## Remaining risks and hardware validation

This deliberately changes early raw-tilt suppression. Filter lag can permit
transient assessment before tilt is recognised. Shallow ramps, low-angle loads
or a brief edge that strongly impedes wheels may resemble obstruction while
filtered angles/states remain eligible. For example, not every sustained pitch
step just above 3 degrees crosses the filtered guard within 100 ms. The tested
8/5 degree onsets and two one-side ramp crossings do not prove universal safety
against ramp false positives. Do not claim arbitrary tilt rejection.

Wheel slip can hide chassis blockage, and weak drive/encoder failure or other
loading can resemble wheel obstruction. Accelerometer filtering does not remove
gravity/linear-acceleration ambiguity. Stops still cancel before confirmation;
never delay S merely to fill the confirmation window. Outcomes remain only
POSSIBLE_OBSTRUCTION, independent of POSSIBLE_IMPACT and confirmed collisions.

Next supervised operator test: record normal manual driving and confirm valid /
established state, then repeat normal stops, left/right turns and both directions
of the normal one-side ramp before soft-contact trials. Capture complete CSV,
CONTROL records and independent action/contact annotations. Keep prompt S and
compare count deltas, eligibility, filtered angles and terrain states. Retain
misses and unwanted events; do not tune to a known ramp height. Physical
obstruction discrimination remains unvalidated.

## Exact files changed in this follow-up

- `app_program/integration_test/buddy4_obstruction_sdk.c`: remove only the raw
  amplitude veto and explain the retained filtered/state eligibility.
- `app_program/integration_test/buddy4_obstruction_sdk.h`: document that policy
  and its filter-lag/shallow-ramp limitation; no API/layout changes.
- `app_program/integration_test/test_obstruction_sdk.c`: actual normal-driving
  establishment regression, four legacy ramp replays, filtered pre-hysteresis
  bounds, onset/confirmation cancellation, return-to-flat re-establishment.
- `app_program/buddy4_mtk/prepare_source.py`: copy only the four immutable text
  captures needed by the shared SDK tests into the disposable trial source tree.
  General evidence copying remains excluded. The first trial test run exposed
  the missing fixture dependency; the rerun passes after this preparation fix.
- `app_program/buddy4/OBSTRUCTION_FILTERED_ELIGIBILITY.md`: this report.

Calibration, IMU/height/terrain code, Buddy 2/3, source recordings, observer core,
IMU impact algorithm and unrelated existing changes are preserved. No flashing,
motor operation, commit, push or merge.

## Verification and SDK artifact

PASS: baseline integration runner; pre-change expected failure of the new normal
replay; updated integration runner including all prior Buddy 4 tests, 42 motion
and 30 terrain checks plus Buddy 3/drive/scheduler; MTK full runner including
native task/device/I2C/kernel wait and the shared SDK/ramp tests; CSV schemas and
printf counts (39 standalone / 86 SDK / 17 MTK); SDK 2.3.1 Pico W build;
preservation hashes and whitespace checks. Existing two Buddy 3 comparison
warnings remain. No new SDK compiler warning was introduced.

Exact updated SDK UF2:
`C:/Users/yongk/OneDrive/Documents/GitHub/INF2004-PicoCar/app_program/integration_test/build/buddy234_driving_test.uf2`

Size: 115,200 bytes. SHA-256:
`2334d718edbfb29743ff53f92422d6b3b616ef5faaad6d0c31aa3385678e990d`.
ELF text/data/bss: 61,684 / 0 / 4,492 bytes.
The standalone IMU and isolated MTK firmware do not use this SDK adapter and
were not rebuilt; native team MTK integration still needs its own qualified
encoder/command producer.

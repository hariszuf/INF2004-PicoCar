# Buddy 4 measurement validity correction

30 October 2026. Software evidence only; no firmware flashed or motors operated.
The working tree was clean before investigation. The known G backup was preserved.

## Reproduced defect and hardware limits

The recording `C:/Users/yongk/Downloads/BUDDY 4 ? FLAT SURFACE COMPARISON.txt`
identifies `B4-TILT-PEAK-POSSIBLE-IMPACT-G USB-only`. Its suspicious styrofoam
and moving sections report -35.67 degrees pitch, -36.14 degrees roll and 6.66 cm
height as fresh/valid with zero errors. Earlier firm-flat readings have a different
baseline; those are not interchangeable mounting/calibration evidence.
Recording SHA-256: `7dc2514023840f926835ef9addaaaa0b291e35c520f749a51959f47c701cbca3`.

Before changing processing, an added fixture injected XYZ=(-32768,-32768,-32768)
through successful fake register transfers into the actual `buddy4_poll()` using
production calibration. Sixty 20 ms polls reproduced exactly the rounded angles,
6.66 cm, magnitude 3.6249 g, valid=1, errors=0, stable=1 and median=1.
This proves that complete fresh acquisition alone was insufficient measurement
validation. The recording contains no raw XYZ; matching derived numbers does not
prove those codes occurred on hardware or identify a sensor, wiring, supply,
configuration or native transfer fault. No new transport defect was reproduced.
No mounting offset, sign, calibration coefficient, geometry or tilt/detector
threshold was tuned to this recording.

## Device and transfer audit

Checked against [ST LSM303D datasheet, DocID023312 Rev 2](https://www.st.com/resource/en/datasheet/lsm303d.pdf):
Table 3, section 6.1, sections 8.6, 8.16-8.18 and 8.23-8.27.
The device uses signed 16-bit two's-complement acceleration, low/high registers
0x28-0x2D; +/-2 g sensitivity is 0.061 mg/count. Multi-byte I2C reads require
subaddress bit 7 (0xA8 for XYZ). WHO_AM_I is 0x49 at address 0x1D with SA0 high.
CTRL1=0x6F selects 100 Hz, BDU and all axes. BDU holds output until both bytes
are read. CTRL2=0xC0 selects +/-2 g, 50 Hz bandwidth and self-test off.
CTRL0=0 disables FIFO; CTRL7=0x02 bypasses acceleration high-pass output and
powers down magnetometry. STATUS_A bit 3 indicates new XYZ; bits 7-4 indicate
overrun, not saturation. Overrun can occur at the existing 50 Hz host polling
rate. No documented accelerometer overflow bit replaces raw endpoint checks.

Repository audit: decoding already subtracts 65536 for signed values without
right-shifting. The core requires exactly one register-pointer byte and exactly
six data bytes. SDK callbacks retain bounded timeouts and repeated-start/STOP.
The native bridge combines pointer+read in T_I2C_EXEC, checks request-object
completion and rejects short/error returns. The native controller decrements
RX count per byte and waits for STOP after the last byte; abort/timeout paths
return errors, not success. Existing actual-driver and kernel-wait models cover
1..6-byte reads, STOP timing, IRQ delay, partial timeout and recovery. These
models do not prove physical bus integrity or correct sensor output.

Configuration writes remain unchanged. Readback now records and verifies identity
plus CTRL0/1/2/3/4/7 (expected 49 and 00/6F/C0/00/00/02). The isolated owner checks
these once per second, before polling, with no scan or busy wait. A mismatch or
failed readback aborts detection and enters the existing configuration retry path.
Readback is a timestamped observation, not continuous assurance; failed readback
may contain partially refreshed bytes, explicitly marked unverified.

## Quality semantics

- `BUDDY4_FRESH` and `valid` mean a complete data-ready acquisition with a newer
  host timestamp. They do not promise physically trustworthy acceleration/tilt.
- `clipped_axes` marks any exact raw -32768 or +32767 endpoint before calibration.
  `CLIPPED` means range-limited/suspected saturation, not a diagnosed cause. No
  arbitrary near-rail margin is invented; adjacent codes are not marked clipped.
- `acceleration_valid` requires finite corrected XYZ/magnitude and no raw clipping.
  It is the input gate for the existing possible-impact observer and transient
  observation. Non-clipped high acceleration remains usable for those observers.
- `tilt_valid` additionally requires corrected magnitude in inclusive [0.8,1.2] g.
  This is a conservative engineering suitability envelope around static 1 g,
  independent of orientation, not an ST accuracy specification or a collision
  threshold. Twenty percent tolerates preliminary calibration and modest noise;
  outside it the gravity-only interpretation is unsuitable. It cannot guarantee
  correct tilt inside the envelope: linear acceleration can imitate gravity.
- Quality is UNAVAILABLE, GRAVITY_LIKE, CLIPPED, NONFINITE or NON_GRAVITY, in that
  order of purpose (clipping takes precedence over derived magnitude checks).
  Neither clipping nor magnitude rejection increments the I2C/config error count.
  Equal consecutive raw samples are never treated as proof of freezing.
- One unsuitable sample aborts the active tilt event without a completion pulse,
  clears both pending estimators and resets angle smoothing. Sustained bad input
  cannot create events. Previously accepted peaks in that interrupted active
  event are discarded; genuine previously completed `ended_event` and task
  results survive. Recovery seeds from the next good orientation and needs a new
  full 200 ms window before stable/median confirmation. Stale-window semantics
  for ordinary missing-data polls remain unchanged.
- Invalid filtered tilt/height are NaN internally and N/A in readable output;
  raw angles remain diagnostic only. Zero terrain/side during invalidity is a
  reset state, not a measured FLAT/SIDE_LEVEL. Consumers must gate on tilt_valid.

## Impact meaning and diagnostics

A real high-acceleration event may clip. A clipped vector loses amplitude and
cannot safely complete the existing impact confirmation. Callers explicitly
invalidate pending/baseline evidence; saved impact count/time/cooldown survive.
No synthetic acceleration, automatic collision or automatic possible-impact
count is invented. Current output says UNKNOWN_CLIPPED and the saturating saved
clipped-sample count remains visible after recovery with the label "impact
assessment indeterminate". A false event pulse on an invalid observer is not
"no impact". Non-clipped detector thresholds, baseline, confirmation, rearming
and cooldown are unchanged, including high acceleration outside the tilt envelope.
The SDK driving client uses the acceleration gate for impact and publishes
invalid terrain on unsuitable fresh tilt without labelling it a bus failure.

The USB-only 1 Hz monitor shows acquisition freshness, tilt usability, quality,
raw XYZ, corrected XYZ/magnitude, STATUS_A, sample time, cumulative clipped and
unsuitable sample counts, and timestamped configuration readback. These are
bounded snapshots/counters, not a lossless waveform/event queue. Identical
stationary raw readings are permitted. Existing motor-pin startup protection,
Buddy 4-only mode and USB console path remain unchanged.

Schema: standalone SDK 39 columns; SDK integration 79. Six new fields follow
`valid`: tilt_valid, acceleration_valid, measurement_quality, clipped_axes,
clipped_samples, unsuitable_samples. Existing field names remain available;
parse by header. Optional MTK CSV is 17 fields (B4 tag plus 16 values), retaining
the original prefix and appending the six quality fields (quality is enum 0..4).
Unusable current MTK integer tilt/height use -2147483647, checked before casting;
completed unconfirmed peaks still use -1. Readable status remains the default.

## Verification and build

The complete `buddy4_mtk/test.ps1` runner covers startup pin ownership, USB-only
console timing model, actual readable/CSV formatting, native task/device/I2C and
real kernel-wait tests, Buddy 2/3 host simulations, drive/scheduler, production
IMU, stable/median, motion, impact, yaw and fixed-column/printf schema checks.
New tests cover production-calibrated endpoint reproduction, each signed rail
and adjacent code on each axis, raw clipping before calibration, six stationary
gravity directions with identical repeated samples, ordinary tilt, magnitude
boundaries/freefall/nonfinite computation, isolated/sustained bad input, pending
and confirmed event interruption, retained completed history, saturation counters,
readback mismatch and recovery, and actual owner-task impact clipping/history.
Non-clipped possible-impact regressions retain their existing expected outcomes.

The initial runner stopped at a pre-existing base fingerprint mismatch. Reviewed
upstream commit 9e4a887: its only pico_rp2040.mk change adds CONSOLE=none. Preserved
that option in the isolated overlay and refreshed only its reviewed fingerprint.
Live teammate build, kernel, device and application files were not edited.
Main README fingerprint is refreshed only for this documented Buddy 4 update.

Results: complete runner PASS (42 Buddy 2 motion checks, 30 terrain checks,
Buddy 3/drive/scheduler and all Buddy 4/native/console/schema suites). Both
SDK CMake builds and the isolated SMP=0 USB CDC build PASS; optional MTK CSV
also compiles with the native Arm toolchain. `git diff --check` passes. Existing
two Buddy 3 unsigned-comparison warnings and four MTK newlib file-I/O stub linker
warnings remain; no new warning was introduced. Isolated ELF text/data/bss:
85,952 / 544 / 18,624 bytes. Diagnostic UF2: 173,056 bytes,
SHA-256 `fae6f99e87410a300fab9f73c438b413c6534770d2ae9a73251b96390f1fbbf2`.

Firmware builds: isolated SMP=0 USB CDC, SDK standalone, SDK integration. The
isolated marker is `B4-MEASUREMENT-QUALITY-I USB-only`.
UF2: `C:/Users/yongk/OneDrive/Documents/GitHub/INF2004-PicoCar/app_program/buddy4_mtk/build/mtk3pico_smp0_usb_cdc.uf2`.
The G backup remains `build/mtk3pico_smp0_usb_cdc_B4-TILT-PEAK-POSSIBLE-IMPACT-G_backup_2026-10-29.uf2`,
SHA-256 `94dee6c332be2f1474d051fec54a2e70433262b4243e4e3e0a9547ddf4899f46`.
No flashing, motor actions, commit, push, team MTK integration, Tribunal change
or report work was performed.

## Changed files

- Shared Buddy 4 processing/contracts: `app_program/buddy4/buddy4_imu.c`,
  `buddy4_imu.h`, `buddy4_impact.h`; SDK logger `app_program/buddy4/main.c`.
- Isolated monitor/owner: `app_program/buddy4_mtk/overlay/app_program/app_main.c`,
  `overlay/app_program/buddy4/buddy4_task.c`, `buddy4_task.h`.
- Existing SDK integration client: `app_program/integration_test/main.c`
  (Buddy 4 quality gates and logging only).
- Regression coverage: `app_program/buddy4/tests/test_imu.c`,
  `app_program/integration_test/test_impact.c`, `test_csv_schema.py`,
  `app_program/buddy4_mtk/test_peak_console.py`, `test_quality_csv.py`, `test.ps1`,
  `overlay/app_program/buddy4/tests/test_mtk_task.c`.
- Isolated build guard maintenance: `app_program/buddy4_mtk/base_hashes.json`,
  `overlay/build_make/pico_rp2040.mk` (reviewed upstream CONSOLE=none only).
- Documentation: this checkpoint, `app_program/buddy4/README.md`, `TODO.md`,
  `app_program/buddy4_mtk/README.md`, `overlay/app_program/buddy4/README.md`,
  `README_MTK.md`, `app_program/integration_test/README.md`.

## One remaining hardware capture

After the operator loads the diagnostic UF2, record ONE continuous 25-second
USB capture with motor power disconnected, same mounting and no calibration
changes: 5 s untouched on a firm level surface, 5 s gently held at a small
right-wheel-up tilt, 5 s level, 5 s small left-wheel-up tilt, 5 s level. Avoid
wire/connector changes during the capture. This needs no driving or ramp batch.

Changing plausible near-1 g raw vectors with stable expected readback separates
an orientation/baseline question from the reproduced endpoint case. Persistent
endpoints/non-gravity magnitude while stationary establish unsuitable hardware
input; unchanged raw through both controlled tilts strengthens sensor/read-path
suspicion, but does not identify which component is responsible. Wrong readback
isolates configuration/identity evidence; failed transfers expose a transport
failure separately. A 1 Hz snapshot may miss brief excursions (counters reveal
that), and correct readback cannot rule out supply/wiring/sensor faults. No crash
or sensor strike is needed. If static endpoints persist despite correct readback,
a later electrical/bus investigation may still be required; this capture cannot
by itself distinguish silicon from bytes delivered by the native read path.

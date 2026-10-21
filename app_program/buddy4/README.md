# Buddy 4 standalone LSM303D monitor

This independent Pico W project covers Buddy 4 only. It has no motor code, networking, or dependency on the Buddy 2/3 combined build. No hardware was flashed during this refactor. No AGENTS.md instructions were found in the repository or its parent paths.

## Files and provenance

- `buddy4_imu.c` / `buddy4_imu.h`: reusable SDK-independent sensor configuration, acquisition and tilt/event processing; public state and bounded-transfer callback API.
- `main.c`: standalone Pico SDK adapter and 50 Hz USB serial test logger on I2C0, SDA GP16 / SCL GP17, 100 kHz bus. Sensor address 0x1D; WHO_AM_I 0x0F must return 0x49. This is LSM303D, not LSM303DLHC.
- `CMakeLists.txt`: independent `buddy4` firmware and optional native test build; `pico_sdk_import.cmake` retained unchanged.
- `tests/test_imu.c`: deterministic fake-I2C and synthetic gravity/event tests.
- `.gitignore`: local builds, binaries, editor files and local credentials excluded.

Superseded files are preserved outside `app_program` in `../../archive/buddy4/pre-refactor` (relative to the repository root: `archive/buddy4/pre-refactor`). See the archive manifest there. The original tested `UpDownHillLeftRightUpWheel/IMUProject.c`, its CMake and existing UF2 are preserved unchanged. The archived UF2 is historical, not the output of this build.

## Preserved behaviour

Signed full 16-bit little-endian acceleration, +/-2 g, 0.000061 g/count, without shifting. Configuration follows the tested source: stop CTRL1, CTRL0=0, CTRL2=0xC0, CTRL3/4=0, CTRL7=2 (magnetometer off), CTRL1=0x6F (100 Hz, block data update, XYZ). ID and CTRL1/2/7 are checked.

Pitch and roll use the reference gravity equations with signs +1, offsets 2.50 and 2.85 degrees, and smoothing alpha 0.15. Pitch hysteresis enters at +/-6 degrees and exits at +/-3; roll enters at +/-4 and exits at +/-2. Negative roll means LEFT_WHEEL_UP; positive means RIGHT_WHEEL_UP. There are no fitted left/right book-test correction factors. Calibration values are configurable; automatic calibration is not implemented in the active module.

Side events track start, side and conservative confirmed peak. A peak requires consecutive fresh samples spanning at least 200 ms, with filtered absolute roll range <=1 degree. The minimum sustained value in the window is used, and the highest confirmed window is retained. An isolated angular spike cannot become a confirmed peak without satisfying that window. Side change or return to level exposes the ended summary for that fresh call and logs it. Raw/filtered tilt and instantaneous height are not spike-free.

The reference magnitude-baseline transient flag is retained (0.18 g trigger, 0.08 g rearm, 500 ms cooldown, baseline alpha 0.05). This is a preliminary acceleration-transient indicator, not validated hump or collision detection. Acceleration, braking, vibration and handling affect gravity-based angles.

## Geometry and feature status

9.5 cm is the INNER tyre gap; 12.2 cm is the OUTER width. Neither is centre spacing. The default `centre_spacing_cm=10.85` is their midpoint, provisional only if both measurements describe the same axle with equal-width tyres. `centre_spacing_confirmed=false` is explicit in configuration and output. Measure the actual wheel-centre spacing and set it in the configuration passed to `buddy4_init`. Confirmation of spacing alone does not validate the height model.

`height = centre_spacing * sin(abs(filtered_roll))` is only a preliminary wheel-height-difference estimate, assuming a rigid chassis, modest pitch and approximately static gravity. Tyre compression, suspension, combined pitch/roll and dynamics can invalidate it. Full-width ramp height cannot be measured from roll alone. Driving validation is pending; no accurate hump height is claimed.

Implemented and host-tested: sensor configuration and signed decoding, configured mounting offsets, pitch/roll terrain states, side-event tracking and stable peak confirmation. Preliminary: wheel height difference and acceleration-transient flag. Missing: validated hump segmentation/height, motion/collision classification and turn-rate reporting. There is no gyroscope; no gyroscope yaw/turn rate is available. Magnetometer heading/turn-rate estimation is not implemented or calibrated.

## Integration and timing contract

Call `buddy4_init`, then `buddy4_configure`, then `buddy4_poll` about every 20,000 us with a monotonic host acquisition timestamp. Configuration validates signs (+/-1), finite offsets and positive finite spacing. The state is caller-owned; no allocation, sleeps, USB operations or Pico dependencies occur in the module. Callback implementations must honor transfer bounds.

Each poll checks STATUS_A once. No data-ready loop: stale returns immediately after the status transaction. A fresh sample uses four transfers maximum (status address/read, acceleration address/read), each bounded to 1 ms by the Pico adapter, plus CPU overhead. Configuration is a separate bounded sequence (15 transfers maximum). Main retries configuration at most once per second, at startup and after runtime transfer failures clear `ready`. Polls return not-ready without I/O until configuration succeeds.

`output.valid` is true only for the latest fresh poll; stale, failure and not-ready calls clear it. Timestamp and measurements retain the last fresh values; always check validity before consuming them. `timestamp_us` is the host acquisition/slot time, not an exact hardware conversion timestamp. `error_count` counts failed configuration attempts or failed poll transactions; stale is not an error. Duplicate/backwards sample timestamps are stale. Stale data restarts peak confirmation; gaps >40 ms also restart angle filters and transient baseline. Existing events/confirmed peaks are retained across stale samples and are not evidence of uninterrupted motion. Transfer failures instead abort/reset the active event and peak, clear completed-event metadata without a completion pulse, reset filter/transient state, and clear `ready`. Reconfiguration resets detection state, including the transient baseline, rearm and cooldown history, while preserving cumulative `error_count`. No event completion is inferred from missing data or failures.

Main skips missed slots rather than catching up, samples regardless of USB connection, and limits USB stdout timeout to 1 ms per SDK write. Logging is best effort and may drop output. This is bounded polling, not a hard real-time scheduling guarantee. USB connection never gates initialization or acquisition.

## Exact VS Code build and flash steps

1. Open **`app_program/buddy4` itself** with File > Open Folder in VS Code. Use the Raspberry Pi Pico extension, Pico W (`pico_w`), SDK **2.3.1**, toolchain **15_2_Rel1**. Do not configure the parent `app_program` project for this firmware.
2. Open Terminal > New Terminal in that folder. On this installed Windows setup run:

```powershell
$buddy4Cmake = "$env:USERPROFILE/.pico-sdk/cmake/v4.3.4/bin/cmake.exe"
& $buddy4Cmake -S . -B build -G Ninja "-DCMAKE_MAKE_PROGRAM=$env:USERPROFILE/.pico-sdk/ninja/v1.13.2/ninja.exe" -DPICO_BOARD=pico_w
& $buddy4Cmake --build build --target buddy4
Get-Item ./build/buddy4.uf2 | Select-Object FullName,Length,LastWriteTime
```

The CMake extension integration selects the installed SDK/toolchain. Firmware output is **`app_program/buddy4/build/buddy4.uf2`**, alongside ELF/BIN/HEX; use that newly built UF2, not an archived one.

3. Manual flash, when ready: disconnect USB, hold BOOTSEL on the Pico W while reconnecting USB, then release it. Copy `build/buddy4.uf2` to the `RPI-RP2` drive in Explorer. The drive disappears and firmware restarts. These are instructions only; no flash was performed here.
4. In VS Code open a serial monitor for the Pico USB COM port at 115200. Start level and stationary, check the `result` and `valid` columns are `0` and `1`, pitch/roll near zero with the configured mounting, then test uphill/downhill and each wheel by hand. Codes: fresh=0, stale=1, I/O error=2, not ready=3. Geometry remains marked unconfirmed. Sampling starts even before the monitor opens.

Native tests from the same terminal (installed MSYS2 GCC):

```powershell
& $buddy4Cmake -S . -B build-host -G Ninja -DBUDDY4_HOST_TESTS=ON -DCMAKE_C_COMPILER=C:/msys64/ucrt64/bin/gcc.exe "-DCMAKE_MAKE_PROGRAM=$env:USERPROFILE/.pico-sdk/ninja/v1.13.2/ninja.exe"
& $buddy4Cmake --build build-host
& "$env:USERPROFILE/.pico-sdk/cmake/v4.3.4/bin/ctest.exe" --test-dir build-host --output-on-failure
```

## Serial CSV format

The monitor emits a 27-column CSV header on each observed USB connection, followed by rows with the same field order for fresh, stale and error results. Fields include sequence and poll/sample times, result/validity/error count, signed raw X/Y/Z counts, acceleration in g and magnitude, raw and filtered pitch/roll, terrain and side labels, transient values, preliminary height, geometry confirmation, event activity and confirmed peak roll/height. For invalid rows, measurements and sample time retain the last fresh sample; do not treat those values as new data. Sequence counts emitted sample rows.

Side-event summaries appear on separate `# SIDE_EVENT_END` lines; exclude lines starting with `#` when importing sample CSV. USB output remains best effort, and a truncated/dropped line should be discarded by the receiver. Height and peak fields keep their preliminary interpretation described above.

## Verification and remaining work

On 2026-10-21: native GCC 15.2.0 build passed with `-Wall -Wextra -Werror` and assertions enabled; CTest 1/1 passed. Tests cover signed limits, register settings/readback and wrong ID, each configuration transfer failure/short transfer, poll transfer failures/short reads, stale/fresh handling and timestamps, configurable calibration/spacing, pitch hysteresis and both signs, side transitions/reversal/return, transient flag, angular-spike peak rejection, exact 200 ms confirmation, excessive variation and gap restart. Pico W standalone build passed with installed SDK 2.3.1 / Arm GCC 15.2.1 and generated UF2. These are synthetic host tests and a compile/link check, not physical validation.

Remaining Buddy 4 work: measure centre spacing; recheck offsets and axes on the final mounting; collect stationary and driving logs; validate peak timing and geometry against known wheel lifts without fitted side factors; characterize noise, vibration and fault recovery; design and validate hump and motion/collision detection separately; establish a defensible turn-rate source and calibration (no gyro present). Integration should consume validity/timestamp/error count without changing Buddy 2/3 control behaviour. MQTT is archived and is not part of this active target.

# SDK telemetry interruption investigation, 2026-11-02

## Evidence and conclusion

The operator reports VS Code Serial Monitor on COM4 at 115200, with the last
CSV row at 14:53:14.531 and S sent at 14:53:16.414. Output therefore ended
approximately 1.883 seconds before S. Physical stopping does not independently
prove when the firmware received S; USB-disconnect and IMU safety paths can
also stop movement. Power cycling previously restored telemetry.

`Pasted text(20261102-065325).txt` was not found at the supplied Downloads
location. This investigation uses the operator's chronology, source inspection
and deterministic host tests, not an analysis of that complete recording.

No SDK application defect causing permanent telemetry loss has been reproduced.
No production code or firmware configuration was changed for this investigation.
The physical interruption remains unresolved. Motor noise, supply disturbance,
USB cable/host/DTR problems, USB-driver failure and a target hang/reset are
possibilities, not established root causes.

## Application stop and reporting contract

- `integration_test/drive_control.c`: S/space calls `drive_stop`, stops line
  following, requests an emergency stop and clears running/armed. It does not
  disable the scheduler, USB or printing. Recovery still requires explicit R/G.
- Buddy 3 `line_follow_stop` calls Buddy 2 `motion_stop`; the SDK then calls
  `motion_emergency_stop`. These stop paths have no USB operations or wait
  loops. The emergency latch brakes immediately. Subsequent motor updates
  publish ERROR/ESTOP and continue returning.
- `integration_test/main.c`: the main loop and due motor, line and sensor jobs
  continue while stopped. CSV is conditional on `usb && sampled`, not running,
  motor state or valid IMU measurements. Every due poll sets `sampled`, including
  I2C failures. A reconnect requests another header and does not rearm movement.
- With the default Pico SDK USB configuration, `stdio_usb_connected()` depends
  on CDC/DTR. A host that drops connection/DTR suppresses CSV and invokes drive
  safety. Reasserting connection permits reporting again; motors stay stopped.

## Installed SDK 2.3.1 USB path inspected

Local sources inspected:

- `src/rp2_common/pico_stdio_usb/stdio_usb.c`
- `src/rp2_common/pico_stdio_usb/include/pico/stdio_usb.h`
- `src/rp2_common/pico_stdio/include/pico/stdio.h`
- `lib/tinyusb/src/class/cdc/cdc_device.c`

The target enables USB stdio, disables UART stdio and sets
`PICO_STDIO_USB_STDOUT_TIMEOUT_US=500`. This is a no-space/progress timeout
inside the output driver, not a total 500 us bound on `printf`. A write can
refresh the progress timestamp; formatting, mutex acquisition and USB task
processing are additional work. The default stdio deadlock mutex timeout is
1000 ms. Slow/non-reading hosts can lose parts of lines without an application
error being reported. The SDK flush loop has no explicit total deadline, but
the inspected TinyUSB implementation returns zero when the endpoint is busy,
not ready or empty. Ordinary host backpressure alone has not demonstrated a
permanent loop in this application.

USB CDC's selected 115200 line coding is not a physical 115200-baud UART path
in this target. No evidence justifies changing baud, stop behavior, safety
limits or collision thresholds as a fix. These source checks cannot exclude
an IRQ/driver/device failure or hardware disturbance.

## Deterministic tests

`integration_test/test_main_telemetry.c` includes the actual SDK `main.c` and
links production control, Buddy 4 processing and the existing read-only Buddy
2/3 simulations. Test-only headers supply GPIO/I2C/USB APIs. No COM port or
hardware is accessed. Every process runs 6.5 seconds of simulated time with
M/R/G, repeated S, a G without rearm, and space. The tests assert 86 CSV fields,
increasing row timestamps/sequence, continued post-stop output, ERROR/ESTOP,
immediate braked latches and no implicit rearm.

| Scenario | Rows received before S | Rows received after S | Result |
| --- | ---: | ---: | --- |
| Connected, healthy | 110 | 216 | Passed |
| Simulated TX sink drops output from 1.5 to 4 s; RX works | 75 | 126 | Passed; 127 writes dropped, receiving resumes |
| Simulated connection/DTR absent from 1.5 to 4 s | 75 | 126 | Passed; new header, stopped on reconnect |
| Post-stop I2C failure from 3 to 4 s | 110 | 216 | Passed; invalid rows continue and retry recovers |

Transport stubs deliberately model finite wait/drop and connection state.
They do not execute TinyUSB, emulate electrical interference or prove physical
USB recovery. These are guards against an application stop/reporting regression;
they do not reproduce the reported hardware failure.

Verification passed: integration runner (including scheduler, drive safety,
IMU, impact, obstruction core/adapter, motion/yaw and Buddy 2/3 simulations),
full Buddy 4 MTK runner, 39/86/17-column schema checks and `git diff --check`.
Existing Buddy 3 unsigned-comparison warnings remain in its read-only simulation.

## Changes in this investigation

- `integration_test/test_main_telemetry.c`: actual-main-loop host tests.
- `integration_test/host_sdk/pico/stdlib.h`, `pico/stdio_usb.h`, and
  `hardware/i2c.h`: test-only declarations/state.
- `integration_test/test.ps1`: build/run those four scenarios.
- This report.

No Buddy 2/3, calibration, terrain/height, observer thresholds, production
SDK/MTK sources or evidence recordings were changed. Pre-existing work was
preserved. No flashing, motor operation, commit, push or merge was performed.

## Next physical diagnostic capture

1. With motor power disconnected, capture idle telemetry through the same
   monitor. Check its receive/pause state and Windows COM-device presence if
   output ends; retain the log and timestamps. Closing/reopening the monitor
   can change DTR and safely latch stopped; do not send R/G for this check.
2. If needed, repeat using one alternate serial reader, with the first reader
   closed. Record whether reconnect yields a new CSV header and whether sequence
   and device time continue or restart. Continuing counters support a transport
   interruption; restarting counters support a reset, without identifying its
   cause. Absence of output alone cannot distinguish target hang from USB loss.
3. Before another powered driving test, capture host USB disconnect/reconnect
   information alongside the complete CSV and command timestamps. If only motor
   operation reproduces it, investigate power integrity, grounding and motor
   interference with suitable physical measurements. Do not infer an electrical
   root cause from temporal correlation alone. Serial S is not a hardware estop.

No firmware rebuild was required because production inputs were unchanged.
The existing artifact remains:

`C:\Users\yongk\OneDrive\Documents\GitHub\INF2004-PicoCar\app_program\integration_test\build\buddy234_driving_test.uf2`

SHA-256: `d795355976356e782ae41f9ad686866ee20b16fa79b9c010653644970e439905`

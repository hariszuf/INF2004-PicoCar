# Buddy 2 micro T Kernel trial

Status: separate trial, not merged into the live team application. Same approach
as `app_program/buddy4_mtk` (Ignatius): the live team files (`app_main.c`, the
build files, the kernel) are **not** edited. `prepare_source.py` copies the
repository into `build/source/` and applies `overlay/` there; only that copy is
built. Built and host-tested 28 Oct 2026; **not yet run on the car**.

The only live change is in Buddy 2's own `motion_control.c`: a third platform
branch (`MOTION_USE_UTKERNEL`) for time and interrupt masking. It is inactive in
the Pico SDK (CMake) builds and the host tests.

## What the trial image contains

Buddy 2 **only**. Buddy 1 and Buddy 5 are left out on purpose, so the serial
output is only Buddy 2's and any problem in this image belongs to Buddy 2.

| Prio | Task | Period | What it does |
|---:|---|---|---|
| 3 | Buddy 2 Motion (`motion_task_mtk.c`) | 20 ms | `motion_update()`: speed PID, moves/turns, stall and e-stop handling |
| 12 | Buddy 2 self-test (`motion_selftest_mtk.c`) | - | scripted test below, prints results |

The self-test counts down 15 s from power-up (it does not wait for the serial
monitor; the header repeats every 5 s), then runs once: forward 30 cm, backward 30 cm, turn left 90, turn right 90,
drive 3 cm/s for 3 s. Serial output (example from the host simulation):

```
----------------------------------------------------------------------------------
 STEP 1/5   Forward 30 cm
----------------------------------------------------------------------------------
   time  state       enc L    enc R   speed L  speed R  distance  heading
   (ms)              (count)  (count) (cm/s)   (cm/s)   (cm)      (deg)
      0  IDLE             0        0      0.0      0.0       0.0      0.0
    500  MOVING         415      423     23.6     23.4       6.8      0.5
   1000  MOVING        1094     1098     21.0     20.5      17.7      0.1
   1800  IDLE          1856     1860      0.1      0.1      29.9      0.0
 RESULT: DONE     encoder says 29.9 cm, 0.0 deg  ->  now measure the real distance
...
==================================================================================
 SUMMARY - BUDDY 2 MOTION SELF-TEST
==================================================================================
  #  step                    result   encoder reading            your measurement
  1  Forward 30 cm           DONE        29.9 cm              ______ cm
  2  Backward 30 cm          DONE       -30.0 cm              ______ cm
  3  Turn left 90 deg        DONE        81.5 deg             ______ deg
  4  Turn right 90 deg       DONE       -81.5 deg             ______ deg
  5  Drive 3 cm/s for 3 s    DONE     L 3.1 / R 3.2 cm/s
----------------------------------------------------------------------------------
  Steps passed      : 5 of 5
  Motion task       : 1260 updates, longest gap 21.0 ms (target 20 ms)
==================================================================================
```

The summary repeats every 15 s, so it is still visible if the monitor was opened late.
Fill in "your measurement" with a ruler / protractor. For turns the encoder
shows about 88 % of the angle on purpose (`MOTION_TURN_SCALE` corrects wheel
slip), so judge turns by the real angle. The longest gap should stay close to
20 ms (at most ~30 ms with the 10 ms kernel tick).

**First run: wheels off the ground** (car on a stand). Then on the floor with
about 1 m of clear space. Power off to abort. A fault (stall, e-stop) stops the
sequence and keeps the motors braked.

## Files

| Path | Purpose |
|---|---|
| `overlay/app_program/buddy2/mtk/motion_mtk_hw.c/.h` | RP2040 timer, DI/EI critical sections, PWM/TIMER reset release, shared GPIO interrupt dispatcher |
| `overlay/app_program/buddy2/mtk/motion_motor_mtk.c` | micro T-Kernel version of `motion_motor.c` (PWM slices 4/5, 15 kHz) |
| `overlay/app_program/buddy2/mtk/motion_encoder_mtk.c` | micro T-Kernel version of `motion_encoder.c` (GP6 left / GP4 right) |
| `overlay/app_program/buddy2/mtk/motion_task_mtk.c/.h` | `buddy2_init()` / `buddy2_start()`, the Motion task |
| `overlay/app_program/buddy2/mtk/motion_selftest_mtk.c/.h` | the self-test task |
| `overlay/app_program/buddy2/mtk/tests/` | host tests on a fake register map |
| `overlay/app_program/app_main.c` | starts Buddy 2 and its self-test only |
| `overlay/build_make/mtkernel_3/app_program/subdir.mk` | explicit source list (the live one compiles every `.c` in every buddy folder, including Pico SDK programs, and fails) |
| `overlay/build_make/pico_rp2040.mk`, `overlay/build_make/makefile` | link `-lm` with nosys stubs (console unchanged: USB + UART mirror) |
| `base_hashes.json` | fingerprints of the 4 live files the overlay replaces |

`prepare_source.py` refuses to build if one of those 4 live files changed (for
example a teammate edited `app_main.c`): compare and update the overlay first,
do not just refresh the fingerprints.

## Why these changes are needed

- The port does not link the Pico SDK runtime, so `hardware/pwm.h`,
  `gpio_set_irq_enabled_with_callback()`, `time_us_64()` and
  `save_and_disable_interrupts()` used by the CMake drivers do not exist.
- The port releases the **PWM block from reset only when `USE_PTMR=1`** (it is 0)
  and never releases the **TIMER** or starts its 1 us tick. Without this the
  motors get no PWM and every microsecond time reads 0. `motion_mtk_hw_init()`
  releases both and starts the tick.
- The RP2040 has **one** GPIO interrupt for all pins. `motion_mtk_hw.c` owns it
  (`tk_def_int(13)`) and dispatches per pin. A later buddy that needs a GPIO
  interrupt (for example a Buddy 5 echo pin) must register through
  `motion_mtk_gpio_irq_register()` instead of defining IRQ 13 again.
- `tm_printf` keeps interrupts off for the whole call, and the UART mirror waits
  for each character (found in the Buddy 4 trial), so a few encoder edges can be
  missed while a line prints. The first trial build used the USB-only console
  (`-DTM_CONSOLE_UART=0`) to avoid this, but on the car it showed no serial output
  at all, so the trial is back on the team's default console (USB + UART mirror),
  the setting that already drove this car. The self-test prints only twice a
  second; expect at most a few tenths of a cm of encoder error from this.
- The kernel's sample I2C driver sets GP8/GP9 (motor M1A/M1B) to I2C with
  pull-ups at boot. `buddy2_init()` switches GP8-GP11 back to PWM, braked, before
  any task runs. For the final team image the I2C driver should be moved to the
  IMU pins GP16/17 (Ignatius's overlay already does this).

## Commands (from the repository root)

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File app_program/buddy2_mtk/test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File app_program/buddy2_mtk/build.ps1
```

(`test.sh` does the same as `test.ps1` from Git Bash / Linux.) Output:

```text
app_program/buddy2_mtk/build/mtk3pico_smp0_usb_cdc.uf2
```

Flash it like the other micro T-Kernel images and open the Pico's USB serial port.
Build outputs are ignored by git; commit only the overlay, scripts and README.

## Verification so far

- Host: 34 driver checks on a fake RP2040 register map (reset release, timer,
  GPIO dispatch, encoder counting/direction, PWM pins and levels, latch,
  `motion_control.c` driving the real drivers, e-stop), plus the Buddy 2 motion
  (42, default and line-following tuning) and terrain (30) suites.
- Firmware builds: SMP=0 USB CDC and SMP=0 UART, no new warnings (the four
  newlib nosys stub warnings are expected).
- **Not run on the car yet.** Hardware results (does it move the right way, do
  counts match, real distance/angle error, longest update gap) still to record.

## Limits

- SMP=0 only. Buddy 2 uses its default tuning (`motion_config.h`), not the slower
  line-following overrides of the CMake `EmbeddedProject`.
- Kernel tick is 10 ms, so the 20 ms Motion period can jitter by up to one tick;
  `motion_update()` measures its real time step.
- Terrain (Buddy 4) and obstacle (Buddy 5) inputs are not connected in this trial.
- The old `buddy2/motion_task.c` skeleton is not used by this trial.

# Buddy 4 micro T Kernel trial

## Current test output: B4-MEASUREMENT-QUALITY-I USB-only

The endpoint-input validity gap is corrected. Fresh acquisition, usable corrected
acceleration and usable tilt/height are distinct. The monitor shows raw XYZ,
corrected acceleration/magnitude, clipping/quality counters and periodic sensor
identity/configuration readback. Clipped impact assessment is indeterminate.
Completed history survives; interrupted tilt evidence cannot finish a peak.

See [the focused measurement-quality checkpoint](../buddy4/MEASUREMENT_QUALITY_CHECKPOINT.md)
for reproduction, semantics, tests/builds, exact UF2 path and the single 25-second
controlled stationary/tilt capture still needed. No hardware root cause is claimed.
The G backup remains untouched. Default output is readable; optional MTK CSV now
has 17 fields with explicit quality flags and masked invalid current tilt values.

## Previous display checkpoint: H

This display-only revision preserves the clean DIAG-E USB-only timing path,
motor-pin protection, calibration and all detection/estimation algorithms.
NOW remains the current snapshot. A newly observed completed side-tilt event
prints B4 RESULT once, with side, event number/end time, stable peak and median
candidate in cm. Unconfirmed values say UNCONFIRMED, not zero. A side-tilt event
can come from handling; it is not independently confirmed as a hump crossing.
H also puts `OBSERVATIONAL TILT, NOT CONFIRMED HUMP CROSSING` on every saved
`B4 RESULT` line so copied results retain that limitation.

The readable monitor now shows current possible-impact detector validity and the
saved cumulative possible-impact count. When that count changes, it prints the
retained latest event timestamp as `B4 POSSIBLE IMPACT`, explicitly labelled as
a saved observational event and **not a confirmed collision**. Reporting is
driven by the retained count, not the detector's one-input-sample pulse, which a
1 Hz monitor will usually miss. If the count jumps by more than one, the monitor
warns that intermediate timestamps were not retained. Current validity and saved
history use separate labels; saved count/time remain visible during invalid data.

The 1 Hz monitor retains only the latest completed event, reports skipped
intermediate event summaries and is not a lossless recorder. For possible impacts,
it retains only a saturating cumulative count and the newest timestamp: it cannot
recover intermediate timestamps, event waveforms, peak deviation/direction or a
one-sample pulse missed between reads. It also cannot establish that contact or a
collision occurred. Start recording
before each trial and wait at least 3 seconds after returning to SIDE_LEVEL.
At that earlier checkpoint the original 11-field CSV was unchanged. This firmware does not drive.

Friend instructions and a plain-text results template:
[Tilt/peak testing](../buddy4/TILT_PEAK_TEST.md),
[copy/paste template](../buddy4/TILT_PEAK_RESULTS_TEMPLATE.txt).
DIAG-E hardware evidence below predates this new output revision; F requires
its own hardware check. Reported peak accuracy remains unvalidated.


**Main Buddy 4 to-do list:** [Completed checkpoints and all remaining work](../buddy4/TODO.md).
This includes MTK/team integration, height/motion validation, collision/possible-impact validation,
Tribunal 1 and report/submission work.


Status: separate trial, not merged into the live team application. Uses the professor-based kernel already in this repository. No changes are pushed to the professor's repository.

## Current checkpoint: B4-DIAG-E (2026-10-28)

### User hardware checkpoint after B4-DIAG-E

The operator supplied a successful E USB-only recording after flashing. All
displayed status blocks have zero cumulative I2C errors, zero skipped slots,
fresh polls and a 20 ms interval. See
[saved recording and limitations](evidence/2026-10-28/README.md).
This is one Buddy 4-only run, with later handling/tilting; final coexistence and
measurement accuracy remain unverified. Historical statements below describe
the investigation before this capture, rather than the latest hardware status.


The reported B4-DIAG-D hardware run now has 20 ms polling and zero displayed
skipped slots, but still times out on READ 0x28. Most errors have STOP state 3,
zero bytes left, eight IRQs, completion around 18,900 us and failure capture
around 19,500 us. Another has RECV state 5 with three bytes unread around
24,000 us. These are user-reported observations, not a hardware run performed
by this change. Neither observation proves completion before the wait deadline.

### Deadline and ordering

`knl_gcb_make_wait` initially stores E_TMOUT in the waiting syscall's result.
`knl_timer_insert` queues expiry at **K + 10 + 10 ms**, where K is the last
serviced kernel tick at wait registration. It does not queue at hardware
request-start + 20,000 us. With ordinary tick delivery, the physical time until
that deadline is the remaining tick phase plus one 10 ms tick: approximately
10..20 ms from registration. Request setup precedes registration. Pending or
delayed SysTicks complicate conversion to hardware time; masked ticks coalesce
rather than necessarily advancing software time once per elapsed period.

At expiry, `knl_wait_release_tmout` removes the task from the flag wait queue
and makes it runnable, leaving E_TMOUT intact. A subsequent I2C STOP IRQ may
then finish all bytes and call `tk_set_flg`, but there is no longer a waiter to
release. It leaves a sticky flag bit and cannot replace the expired syscall's
result. PendSV/task execution may follow still later, especially behind the
higher-priority USB service task. On this board I2C IRQ priority 2 precedes
priority-3 SysTick/PendSV when both are eligible. A pending RX IRQ can service
one byte after a blackout, then SysTick can expire the wait before the next
byte/STOP IRQ is ready. If flag release wins first, the wait timer is removed
and late task resumption alone cannot produce E_TMOUT.

**B4-DIAG-D did not record registration, queued deadline, expiry or actual
flag-release time. Its timestamps cannot identify the exact deadline of each
recorded request.** The completed traces are consistent with expiry preceding
STOP, not evidence of a false timeout. The partial trace requires genuine
unfinished work at cleanup. The E probe below makes those events observable.

The new test compiles the real `timer.c` and trial `wait.c`, including timer
insertion, callback expiry and successful release. Bus/flag delivery, task
dispatch and interrupt blackouts are modeled; timeout results are produced by
the actual kernel wait code, not forced by the test. A request registered
9.9 ms after a serviced tick has a deadline due 10,100 us after request start.
Two deterministic fixtures reproduce the reported shapes:

| Case | Deadline due | Expiry callback | STOP IRQ | Task return / trace | Outcome |
|---|---:|---:|---:|---:|---|
| Completed | 10,100 us | 18,540 us | 18,900 us, 8 IRQs | 19,500 us | E_TMOUT, state 3, no bytes left |
| Partial | 10,100 us | 23,950 us | Not yet seen | 24,000 us | E_TMOUT, state 5, 3 bytes left |

These synthetic times demonstrate a possible ordering, not the unrecorded
exact hardware deadline. Tests also cover different tick phases and completion
before expiry with task resumption delayed to 19,500 us: that case stays E_OK.

### Correction: remove the blocking UART mirror from the trial USB build

The real console path is `tm_printf` -> `tm_putchar` -> `tm_snd_dat`.
`tm_printf` holds DI across the whole call; the original USB configuration also
enables the UART0 mirror. `tm_com.c` polls UART FIFO space for each character
at nominal 115200 baud, **inside that interrupt-disabled call**, even when no
UART reader is attached. USB ring enqueue itself does not wait for the host.
This is a concrete source of long I2C/SysTick service gaps. Back-to-back
diagnostic calls can repeatedly defer the byte-at-a-time I2C state machine;
retained error trace output adds more UART traffic after the first error.

The production correction is just the isolated USB profile's
`-DTM_CONSOLE_UART=0`; USB remains enabled and readable output is retained.
The profile stamp changes to prevent stale objects. **This trial's USB image
therefore no longer provides an early/panic UART mirror.** The UART-only profile
and live team console configuration are unchanged. No timeout value, timeout
return handling, sampling frequency, filtering or freshness tolerance changes.

`test_console.py` compiles the real formatter, nested DI/EI and console sink
code, adapting only headers and UART registers to a 32-byte FIFO/baud model.
For the same existing IRQ trace line it verifies identical 133 USB bytes:
modeled longest masked interval 11,571 us with UART, 133 us without UART.
Those are modeled transport/CPU costs, not physical performance measurements.
Together with the real wait/driver tests this establishes a reproducible
failure mechanism. It does not prove that this was the only cause in the
hardware recording or that all hardware timeouts are fixed.

### E wait probe and verification

The isolated `overlay/kernel/tkernel/wait.c` adds observation hooks at wait
registration, timeout callback entry and successful release. Queue operations,
deadline calculation, results and dispatch rules are unchanged. The original
live file is fingerprinted in `base_hashes.json`; no live kernel file is edited.
The probe filters by I2C completion flag ID and TTW_FLG, and never prints in an
IRQ or kernel critical section. Existing sixteen failure values stay in order;
the trace now has 32 words, retained through recovery:

| Index | Added meaning |
|---|---|
| 16..18 | Wait registration hardware offset (us), kernel time K (ms), actual queued deadline (ms, low 32 bits). |
| 19..20 | Remaining SysTick counter phase (us, rounded down), SCB_ICSR at registration; bit 26 means SysTick pending. Phase is observational, not a deadline. |
| 21..22 | Timeout callback entry hardware offset (us) and kernel time (ms). |
| 23 | Successful flag-wait release offset (us), before timer/queue removal. |
| 24..25 | Entry to flag signalling from I2C IRQ and return from `tk_wai_flg` in the task (us). The latter precedes pre-cleanup capture. |
| 26..27 | Maximum gap from request start/previous active I2C IRQ to the next IRQ, and first active I2C IRQ offset (us). |
| 28..29 | Flag-signal return code and wait return code (printed signed). |
| 30..31 | Driver state and unread byte count at the actual timeout callback. |

4294967295 means an event was not observed in unsigned time/state fields.
All hardware offsets use TIMERAWL modulo subtraction. The printed `resume`
field is task return from the wait, not the exact PendSV instruction time.
Probe overhead is unmeasured; only SMP=0 is qualified by these host/build checks.

The requested `test.ps1` passed: startup protection, expanded task/driver/wait
tests, both console configurations, existing drive/line-follow and Buddy 4
processing suites, and unchanged 33/69-column CSV checks. The existing two
Buddy 3 unsigned-comparison warnings remain. `build.ps1` passed for SMP=0 / USB
CDC with the existing four newlib file-I/O stub warnings. ELF sections: text
82976, data 544, bss 18504 bytes. No flash, motor operation, commit or push.

Exact new UF2, relative to repository root:

```text
app_program/buddy4_mtk/build/mtk3pico_smp0_usb_cdc.uf2
```

Hardware confirmation remains pending. After manually flashing, use one
15-second stationary capture with motor power disconnected, showing
`B4-DIAG-E USB-only`, error/skip counters and every trace line if a failure
occurs. Registration/deadline, expiry, successful release, signal and task
return now distinguish expiry-before-completion from an on-time release with
late resumption. The partial case includes the byte count **at expiry**, not
only at cleanup. A clean capture is evidence for that interval, not lifetime
qualification. Calibration, sensor axes, tilt/height processing, thresholds,
GP8..11 low startup and Buddy 4-only mode are preserved. Historical tilt counts
and records stay hidden in readable output; internal data/optional CSV remain.

## Previous checkpoint: B4-DIAG-D (2026-10-28)

The recurring hardware E_TMOUT on READ 0x28 is **not yet verified fixed**.
The prior combined-read IRQ handover change still failed on hardware. This
checkpoint fixes two independently reproduced driver defects and builds the
instrumented firmware needed to distinguish the remaining failure paths.
The initial uncommitted overlay changes were preserved. All new edits and
generated files are inside this trial; live team/professor sources are unchanged.

### Native path and evidence

The paths below refer to the live sources read for the investigation; only the
driver/application copies under `overlay/` are modified.

| Stage | Actual path / behavior |
|---|---|
| Release | `buddy4_task.c`: 20 ms kernel cyclic handler sets one release flag; sensor task consumes with ORW/BITCLR. It performs configuration/status/data requests synchronously. |
| Device call | `buddy4_device.c` stages the register prefix, then `tk_swri_dev(TDN_I2C_EXEC)` enters [`deviceio.c`](../../kernel/tkernel/deviceio.c): `tk_wri_dev` -> `knl_request` -> driver `execfn`. |
| Driver execution | [`msdrvif.c`](../../device/common/drvif/msdrvif.c) locks the device and calls [`i2c.c`](../../device/i2c/i2c.c) `dev_i2c_writefn` -> `write_atr` -> `dev_i2c_llctl` -> `i2c_trans`. The I/O runs in the calling sensor task, with no separate I2C worker. `req->error` returns through `tk_wai_dev`/`tk_swri_dev`; the outer TMO_FEVR does not replace the driver's 10 ms wait. |
| Controller IRQ | `tk_def_int(TA_HLNG)` selects [`interrupt.c`](../../kernel/sysdepend/cpu/core/armv6m/interrupt.c) `knl_hll_inthdr` -> `i2c_inthdr`. Combined write/read advances in that IRQ, then publishes completion at STOP/error. `ClearInt` clears NVIC pending state, not controller latches. |
| Completion / timeout | [`eventflag.c`](../../kernel/tkernel/eventflag.c) `tk_set_flg` publishes outputs and releases the waiter; [`wait.h`](../../kernel/tkernel/wait.h) `knl_wait_release` removes its timer. `tk_wai_flg` tests an already-set bit atomically or queues the task with E_TMOUT as the initial result. BITCLR consumes only the requested bit. |
| Timer / dispatch | [`timer.c`](../../kernel/tkernel/timer.c) inserts the relative wait at current kernel time + 10 ms + TIMER_PERIOD (10 ms). SysTick expires it via `knl_wait_release_tmout`, or services the cyclic release. [`cpu_status.h`](../../kernel/sysdepend/cpu/core/armv6m/cpu_status.h) END_CRITICAL_SECTION requests dispatch only with entry PRIMASK clear; [`cpu_cntl.c`](../../kernel/sysdepend/cpu/core/armv6m/cpu_cntl.c) requests PendSV, and `dispatch.S` switches tasks after the IRQ. |

Two failing regressions were observed before their respective corrections:

1. Completion was signalled inside the driver's DI lock. `tk_set_flg` made the
   waiter ready and removed its timer, but its critical-section exit saw
   PRIMASK=1 and did not pend dispatch. The driver's EI and HLL IRQ return do
   not request dispatch themselves. The blocked high-priority task therefore
   needed an unrelated interrupt. The single-core correction masks the
   controller and publishes STOP/error under DI, restores the original IRQ
   mask, then sets the flag. Task execution resumes only after IRQ return.
   SMP notification ordering is preserved; SMP is outside this trial's checks.
2. A pending NVIC IRQ after timeout cleanup can have INTR_STAT=0 because the
   controller causes have been masked. The old state machine treated that as
   a new E_IO and signalled completion. It now acknowledges and ignores that
   empty IRQ without altering transfer/error/flag state.

These are concrete dispatch/recovery corrections, **not proof of the cause of
the once-per-second hardware timeout**. In particular, a completion that already
released its waiter cannot subsequently expire that removed timer. Late IRQ
completion after a timeout still returns E_TMOUT; no success is fabricated.
No bus polling fallback, timeout increase, sampling change, error suppression
or wider freshness tolerance was added.

STOP tests use raw latches separately from interrupt masks, RX FIFO levels,
read-to-clear registers, deferred IRQs and nested PRIMASK restoration. They
exercise STOP already latched while masked, STOP during the last FIFO read,
and STOP after it. The existing selective latch clearing and STOP ordering
pass those cases; no further STOP sequencing change is justified by them.
Register semantics were checked against Raspberry Pi's
[RP2040 I2C register definitions](https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2040/hardware_regs/include/hardware/regs/i2c.h).

### Retained failure trace

The original eight B4-DIAG-C values remain in the same order, captured before
cleanup. B4-DIAG-D adds a second readable `IRQ trace` line:

| Index | Meaning |
|---|---|
| 0..7 | State, read bytes left, write bytes left, RAW_INTR_STAT, INTR_MASK, TX_ABRT_SOURCE, STATUS, RXFLR. States: SEND=4, RECV=5, WAIT_STOP=6, STOP=3. |
| 8 | Hardware microseconds elapsed from request start to failure capture. |
| 9 | Offset of the last IRQ with an active masked cause from request start. |
| 10..11 | Count of those IRQs and the last INTR_STAT bits. |
| 12 | Offset when the completion/error IRQ published its final state; 4294967295 means no such IRQ. |
| 13 | Driver state at entry to the last IRQ with an active cause. |
| 14..15 | I2C NVIC enable and pending bits at failure capture (0 or 1). |

Index 9 is also 4294967295 when no active-cause IRQ ran. Offsets use unsigned
subtraction of RP2040 TIMERAWL, including a tested 32-bit wrap. They measure
software observations, **not physical bus waveforms**. The first trace registers
and hardware timer are read only; no latch-clearing register is read to capture
the trace. Successful requests/reconfiguration retain all sixteen saved values.
Readable `Transfer time` remains the coarse kernel-clock duration of the full
poll, including scheduling. It must not be interpreted as a bus measurement.

### Verification and next hardware check

The requested `test.ps1` host regressions passed, including unchanged existing
IMU/median/impact/motion/yaw, drive/line-follow and 33/69-column CSV checks.
New native driver checks cover all combined lengths 1..6 with three STOP
timings, direct reads/writes, pre-wait completion, delayed IRQ delivery,
dispatch without another interrupt, empty pending IRQs, aborts in five transfer
positions, timeout with no IRQ, timeout mid-read, final RX+STOP pending at the
deadline, completion after expiry, and successful recovery with stale flags
and hardware latches preserved. The task/bridge test checks propagation and
retention of all sixteen trace fields. No existing tolerance changed.

The requested `build.ps1` completed for SMP=0 / USB CDC. ELF sections:
text 82272, data 544, bss 17992 bytes. The four existing newlib file-I/O stub
linker warnings remain; the host suites retain their two existing Buddy 3
unsigned-comparison warnings. No flashing or motor operation was performed.

Exact manually flashable file (repository root):

```text
app_program/buddy4_mtk/build/mtk3pico_smp0_usb_cdc.uf2
```

Please make **one 15-second stationary serial capture** after manually flashing
that UF2, with **motor power disconnected**. Keep the current wiring and console
mode; include the BUDDY 4 ONLY banner, B4-DIAG-D marker, NOW/timing/error/skip
lines and both complete trace lines if an error occurs. No tilt or motor command
is needed. This will distinguish an unserviced TX/RX interrupt (remaining bytes,
raw FIFO cause, pending bit and IRQ progress), a STOP-only wait (WAIT_STOP), and
an IRQ completing after wait expiry (STOP plus `done` offset). A long hardware
elapsed value can expose software/IRQ latency beyond the coarse clock, but
cannot identify which code masked interrupts. A bus that makes no progress
without a pending service cause still needs electrical evidence; this capture
cannot prove USB logging or wiring is responsible. If there are no failures,
that establishes only this observation window, not a validated hardware fix.

Calibration, axes, processing, thresholds, 20 ms releases, 10 ms driver timeout,
motor-pin low startup and Buddy 4-only mode are preserved. Readable output
continues to hide historical tilt events/counts; internal records and optional
11-field CSV remain available.

## Why this folder exists

The working Buddy 4 CMake project, driving-test firmware and shared team startup should remain stable while the native task integration is reviewed and hardware-tested. The previously applied native integration was relocated here; the live files were restored to their verified pre-integration versions.

`overlay/` contains the proposed source/build/documentation files, mirrored by their future repository paths. These files are inactive: they are applied only to a generated source copy under this folder's ignored `build/` directory. They are ordinary application and board-driver changes against the professor's port, not a replacement RTOS or custom standalone kernel.

`prepare_source.py` checks the fingerprints in `base_hashes.json`, copies the current team source and applies the overlay inside `build/source/`. If a teammate changes a relevant base file, it stops and requests overlay review instead of silently overwriting team work. Fingerprints use UTF-8 text with normalised line endings and no BOM, so Windows/Linux checkout conventions alone do not cause a mismatch. It recreates only the disposable source copy; do not edit code in `build/source/`.

Edit proposed integration files inside `overlay/`, then run tests/build. Do not update fingerprints blindly after a base change: compare and rebase affected files first.

## Commands from repository root

Requires the existing Python, GNU Make, native host GCC/g++ and Arm toolchain installation. Paths match the documented Windows layout; adjust on another computer.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File app_program/buddy4_mtk/test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File app_program/buddy4_mtk/build.ps1
```

Output:

```text
app_program/buddy4_mtk/build/mtk3pico_smp0_usb_cdc.uf2
```

Build/test scripts never flash or commit. Each preparation discards only `build/source/`, so do not run them concurrently. Build outputs are ignored; only overlay, scripts, manifest and documentation should be committed.

## Proposed integration

- Register a Buddy 4 IMU task alongside existing Buddy 1/5 tasks.
- Use the professor port's kernel mutex/time/device APIs and registered `iica` I2C driver.
- Keep the established calibration, processing algorithms, raw data and geometry.
- Publish protected snapshots; leave encoder motion/yaw invalid until a native Buddy 2 producer supplies fresh observations.
- Include board-specific GP16/17 mapping and native I2C STOP/restart completion fixes with host regressions.
- Keep the ordinary CMake standalone and driving tests available.

See [detailed proposed native integration notes](overlay/app_program/buddy4/README_MTK.md). Paths in those notes refer to the generated trial source tree and later merge layout, not active changes in the live application.

## Existing verification and limitations

Before relocation, the identical overlay source passed native task/device tests, register/IRQ driver regressions, existing host/CSV suites, native USB team firmware build and both CMake firmware builds in a staging copy. After relocation, source preparation produced an identical original 17-file overlay in the isolated tree and the native task/driver host tests passed there. The relocated build.ps1 wrapper has now successfully rebuilt the readable-display trial UF2. The trial test.ps1 host suites and existing 33/69-column CSV checks passed. Display checks covered decimal signs/units, invalid-data masking and historical peak validity; the optional CSV branch also compiled with the native Arm toolchain. Hardware operation remains unverified.

The optional Buddy-1-only compatibility build was not run. Four bare-metal newlib file-I/O stub linker warnings remain, along with the existing two Buddy 3 warnings in its regression/CMake builds. Physical I2C, scheduling, stack use and coexistence with ultrasonic timing remain unverified.

This trial does not migrate Buddy 2/3 controls, validate collisions/yaw accuracy, resolve course-board height bias, implement Tribunal 1 or finish final course integration. The RTOS application does not implement the driving-test M/L/R/G/S commands. Keep motor power off for initial qualification.

Merge into live team files only after review and the required tests. Until then, use the original working firmware for the existing driving tests.

## Reading the serial monitor

The trial now defaults to a readable status block once per second. Pitch and
roll use degrees with two decimal places; wheel height difference uses cm.
Terrain and wheel side appear as words. Invalid/stale IMU samples show current
tilt/height as N/A. Historical tilt counts/records and retained peak results are
hidden from this view; internal records and optional CSV are preserved.
The one-second display is a snapshot, not a lossless event recorder.

To record the original CSV instead, set `BUDDY4_CONSOLE_CSV` to `1` near the top
of `overlay/app_program/app_main.c`, then rebuild and flash the trial. Set it
back to `0` for readable output. There is no live serial mode-switch command.
CSV keeps its existing 11 fields and milli-unit definitions. Other buddies may
also print messages; filter `B4,` lines for CSV analysis. No sensing, control,
calibration, safety or other Buddy implementation is changed by this display.

## Startup pin correction after unexpected left-wheel movement

The first hardware attempt reported unsolicited left-wheel movement. Inspection
found that the reference board startup still selected GP8/GP9 as pulled-up I2C,
although the device-driver overlay had already moved I2C0 to GP16/GP17. This is
a concrete motor-pin conflict and a plausible cause; electrical causation has
not been independently measured. Do not use the earlier trial UF2.

The additional board-startup overlay now preloads GP8..GP11 low, disables their
pulls, selects SIO and enables those four outputs before later device setup.
I2C startup also uses GP16/GP17. Reference GP16 external-LED setup is removed;
GPIO diagnostic scans are forbidden at compile time. The live team BSP is
unchanged. This is an IMU-only trial and has no driving command support.

Motor power must remain disconnected for flashing and initial IMU checks.
Software register checks do not prove electrical levels during ROM boot,
reset, power sequencing or hardware faults, and cannot certify no movement.

## Buddy 4-only timing diagnosis (current default)

`overlay/app_program/app_main.c` now defaults `BUDDY4_TRIAL_IMU_ONLY` to 1.
Buddy 1 and Buddy 5 init/start calls are compiled out; their sources are
unchanged. Professor kernel/device/USB services remain. The startup banner
must say BUDDY 4 ONLY. Set the macro to 0 and rebuild to restore the combined
trial. This does not modify the working SDK integration firmware.

Readable output adds latest poll result, reading age, poll-start interval and
time inside the IMU polling call (status/data transfers plus any scheduling
delay). These use the kernel's coarse 10 ms clock, so zero does not mean no
transfer time. Timing valid=0 means duration unavailable; age 4294967295 means
unavailable/reversed. A fresh poll can still be too old when read by the monitor.
The original 11-field CSV is unchanged. These diagnostics are not accuracy proof.

The kernel adds one timer period to relative delays in timer.c; the earlier
absolute-release loop called tk_dly_tsk for the remaining interval, contributing
oversleep. The current cyclic release replaced that loop. USB printing also
masks interrupts in the supplied port.
Neither observation alone proves the source of all skipped slots or I2C errors.
Keep calibration, freshness limits and scheduling unchanged for this comparison.
See the current checkpoint above for the focused stationary capture.

Readable label NOW describes the current snapshot. Historical tilt events/counts
were removed from this view; event logic, saved records and CSV are unchanged.

## Sampling release correction

The supplied kernel deliberately adds TIMER_PERIOD (10 ms) to relative task
delays. The previous loop also discarded a release exactly equal to the polling
completion time. Together these can turn 20 ms polling into 40 ms starts.
The trial now uses the professor kernel's 20 ms cyclic handler to set a single
event-flag bit. The IMU task blocks on that flag rather than delaying relative to
completion. Exact-boundary releases remain eligible; overruns coalesce, missed
releases are counted and old releases are not replayed in bursts. The cyclic
handler performs no sensor work or logging. Kernel sources and I2C driver are
unchanged by this cadence correction. The 20 ms kernel-measured polling duration
still includes scheduling; this does not prove a 20 ms physical bus transfer.

New lifecycle tests cover flag/cyclic creation/start failures and cleanup;
scheduler tests cover exact boundaries, duplicate releases, coalesced wakes,
overruns and skipped-counter saturation. Hardware cadence must be checked after
flashing. Freshness thresholds and all processing/calibration remain unchanged.
Readable output now omits the repeated saved-event block/count; internal event
records and original CSV are retained. Buddy 4-only mode remains the default.

## Saved IMU failure diagnostics

Readable mode latches the latest IMU error's device return code, last accessed
register, read/write operation and coarse observation time since boot. Successful
polls/reconfiguration do not erase it; the next error replaces it. TIMEOUT maps
to E_TMOUT (-50); E_IO (-57) is labelled BUS_ERROR_OR_SHORT_TRANSFER because the
bridge uses it for short transfers too. Code zero with an IMU error indicates a
sensor verification failure (such as register/identity mismatch), not proof of
a bus fault. Observation time is when the task records the failed poll/config
attempt, not an oscilloscope timestamp. No saved line is shown before an error.
The current NOW status remains separate. No timeout, calibration, detection,
sampling or motor settings were changed by these diagnostics. CSV is unchanged.
Host checks cover short-transfer capture, timeout capture and retention after
successful recovery. Hardware error cause remains to be identified.

## Combined-read interrupt handover correction

Hardware saved-error logs identified repeated E_TMOUT (-50) during READ 0x28
(the six-byte accelerometer output). The driver previously signalled the task
after the register-prefix write, masked interrupts and waited for that task to
start the read phase. The trial now queues the first READ/RESTART directly in
the same IRQ after the prefix. It signals completion only after physical STOP
or a bus error; no task handover is required between write and read. The 10 ms
timeout value is unchanged and now covers the whole combined request.

Register/IRQ regressions exercise combined lengths 1 through 6, a single wait
and completion, RESTART/last-byte STOP ordering, abort and timeout cleanup.
This removes a concrete scheduling gap, but hardware confirmation is still
required; a timeout code alone does not prove its electrical or scheduling cause.
No calibration, filtering, freshness limits, cyclic cadence or motor behavior
was changed. Continue the stationary motor-power-disconnected check.

## B4-DIAG-C: unresolved timeout trace

The hardware recording after the combined-read change still timed out reading
0x28. That change is not a validated fix. B4-DIAG-C identifies the flashed build
on each status block and retains controller state at the failed wait, before
cleanup: state, remaining RX/TX byte counts, raw interrupt bits, interrupt mask,
abort-source bits, controller status and RX FIFO level. Trace capture reads
diagnostic registers only and preserves it through successful recovery. Tests
cover pre-cleanup capture and retention. This diagnostic build does not change
the numerical timeout or claim the hardware issue is fixed. Copy the full
Timeout trace line in a stationary recording with motor power disconnected.

## Collision detection and possible-impact validation

Required collision-detection work is tracked in the
[main Buddy 4 checklist](../buddy4/TODO.md#3-needs-robot).
The existing preliminary possible-impact detector is retained; collision
discrimination remains unvalidated. Validate against labelled normal motion,
vibration, braking, handling and ramp crossings, measure false positives and
document limits. Separate shock detection and dedicated vibration measurement
are outside the current feature plan.

Keep bump_event, possible-impact events and confirmed-collision claims distinct.
The detector remains observational, with no automatic motor action. Do not
deliberately crash the car or strike the sensor.

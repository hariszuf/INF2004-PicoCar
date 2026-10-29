# Buddy 1: remove the preliminary scan dependency

Branch: `fix/buddy1-direct-wifi-join`, based on diagnostic commit `02bf883`.

## Finding and limits

On 29 October 2026, the user confirmed that the USB-disabled diagnostic image
on the power bank repeatedly produced exactly one flash per group, with no
MQTT messages. In that image, one flash means `scan_complete` remains false.
The ongoing LED groups indicate repeated diagnostic execution, unlike the
earlier USB-enabled run that reportedly became permanently dark.

The probable immediate blocker in this run is the application's **unbounded
dependency on completion of a preliminary diagnostic scan**. The old radio
task only issued the first `cyw43_wifi_join()` after detecting scan completion.
Its join retry could not help because no join had started. This explains why
the MQTT layer could remain inactive even though its own retry tests passed.

This does not establish why scan completion was not observed on power-bank
power, or explain every earlier result. No physical pass is claimed for the
updated firmware. The reported persistent one-flash pattern, source inspection,
and host regression support this targeted change; they do not prove the radio
or supply behaves correctly under all conditions.

## Small runtime change

Only `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c` changes relative to the
diagnostic branch:

- Lines 522-529: MQTT builds do not request the preliminary diagnostic scan.
  Non-MQTT qualification builds retain it.
- Line 546: the diagnostic scan-active flag stays false when no scan ran.
  The code does not falsely mark a skipped scan as completed.
- Lines 619-637: MQTT starts its first join directly after radio setup, using
  the existing SSID/password. The existing five-second failed-join retry,
  thirty-second pending-join timeout and late-success protection remain.
- Lines 454-476: the LED no longer waits on the skipped scan. Link/IP/MQTT
  patterns remain 2/3/4, with solid indicating MQTT at the last owner poll.

The installed Pico SDK follows the same direct-join approach in
`src/rp2_common/pico_cyw43_arch/cyw43_arch.c:93`: its asynchronous connection
helper calls `cyw43_wifi_join()` without requiring a separate diagnostic scan.
Joining the configured SSID still performs the radio's necessary AP discovery.

No MQTT adapter, kernel, USB service, credentials, topic, broker address,
teammate implementation or motor behavior changes in this fix. No blind radio
reset, static IP or extra task is added. Radio initialization failure still
requires separate diagnosis.

## Software checks

`tools/test_wifi_startup.py` now exercises the production setup and join blocks.
The old MQTT source fails the new no-preliminary-scan assertion; the corrected
source passes. Both MQTT and non-MQTT checks cover:

- setup failure, and absence of preliminary scan calls in MQTT builds;
- joining without a scan-complete event, including stale scan-state flags;
- no overlapping join or duplicate join when a late scan event arrives;
- cooldown, initial request rejection, terminal error and pending timeout;
- preserving a late successful link; qualification scan/one-shot behavior.

The LED test is updated to expect joining rather than the removed scan stage.
All 11 network scenarios still pass, including real lwIP MQTT handshake,
subscription and heartbeat; real Buddy 1 telemetry forwarding with exact
payload checks; fragmented START and normal STOP through the parser and
consumer with simulated kernel queues; wrong-topic and invalid-command
rejection. The tests invoke no hardware or live motor command.

Run from the repository root with host GCC available:

```powershell
python tools/test_wifi_startup.py
python tools/buddy1_startup_tests/run_network_tests.py --sdk C:/Users/muhdh/.pico-sdk/sdk/2.3.1
python tools/buddy1_startup_tests/test_led_diagnostic.py
```

## Firmware and first test

**Test `buddy1-direct-join-no-usb.uf2` first.** This retains the USB-disabled
profile from the last observed one-flash failure, so the comparison changes
the scan/join startup logic rather than re-enabling USB at the same time.

Artifacts are in:

```text
C:\Users\muhdh\.codex\.chatgpt-projects\g-p-6abcb00776a081918f2ebf953585b093\buddy1-investigation\firmware
```

Primary UF2 SHA256:
`5a0c6c55fb29d9a15bd1d60fb41f2ef9f508bbb21d6e56914e097a23335b37d1`.

1. Keep the same hotspot, broker and subscriber running. Confirm the laptop's
   hotspot IP remains `172.20.10.2`. The dashboard must use that broker too;
   its earlier connection to a separate localhost broker was corrected.
2. Disconnect board power. Hold Pico W **BOOTSEL** while connecting its USB
   cable to the laptop. Release when **RPI-RP2** appears.
3. Copy **buddy1-direct-join-no-usb.uf2** to RPI-RP2. Allow it to restart and
   observe laptop-USB startup without COM5. COM5 is intentionally unavailable
   in this image. Confirm fresh heartbeat and telemetry on the subscriber.
4. Leave the car-side plug in the exact same socket. Disconnect only the
   laptop end for ten seconds, then plug it into the **existing power bank**.
   Do not hold BOOTSEL; use no wall charger and change no other wiring.
5. Observe for three minutes. A successful run requires a **new** broker
   connection identifying `picocar-vehicle1`, plus advancing heartbeat and
   telemetry sequences. An old connection, dashboard Online alone, or the
   subscriber's own PINGRESP is insufficient.
6. If successful, repeat three complete power-bank cold starts. Record the
   firmware name/hash, power-on time, LED pattern, new car connection time/IP,
   first heartbeat and telemetry progression for each run.

| LED on updated image | Meaning at last checkpoint |
|---|---|
| 2 flashes per group | Wi-Fi joining / link down |
| 3 flashes per group | Link up, interface or IPv4 not usable yet |
| 4 flashes per group | IPv4 usable, MQTT not connected |
| Solid | MQTT connected at last poll; confirm continuing messages |
| 5 flashes | Initialization error, if radio LED access remains possible |
| Dark/frozen | Ambiguous: execution, radio/LED I/O or power may be involved |

Groups still repeat every four seconds. The one-flash scan code is removed.
If a single-flash pattern persists, record the exact flashed filename and the
timing before drawing conclusions. No physical command test is needed during
this startup comparison; commands were exercised in software without motors.

The same source also builds `buddy1-direct-join-usb.uf2` with USB CDC restored.
Keep that for a later comparison after the USB-disabled result is known. A
successful no-USB run alone cannot establish that USB-enabled standalone
startup is fixed, especially given the earlier dark-LED observation.

Rollback through BOOTSEL using `buddy1-no-usb-diagnostic.uf2` (previous comparison)
or `existing-before-investigation.uf2` (preserved original). No images were
flashed by the assistant.

## Build reproduction

Builds use the separate investigation source copy, preserving repository build
outputs. Arm GCC 15.2.1 and Pico SDK 2.3.1 are installed. From its `build_make`
directory:

```powershell
$env:PATH = 'C:\Users\muhdh\.pico-sdk\toolchain\15_2_Rel1\bin;C:\raylib\w64devkit\bin;' + $env:PATH
make SMP=0 CONSOLE=uart WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WIFI_MQTT=1 PICO_SDK_PATH=C:/Users/muhdh/.pico-sdk/sdk/2.3.1 'TEMP_SRCS=../app_program/app_main.c $(wildcard ../app_program/buddy1/*.c) $(wildcard ../app_program/buddy5/*.c)' -j8
```

For the later USB-enabled comparison, replace only `CONSOLE=uart` with
`CONSOLE=usb_cdc`. Both source selections match the current `app_main.c`:
Buddy 1 and Buddy 5. Neither build modifies teammates' sources.

Logs are `repo/build-direct-join-no-usb.log`, `repo/build-direct-join-usb.log`,
`direct-join-regression-before.log` and `direct-join-regression-after.log` in
the investigation folder. Firmware hashes are in `firmware/direct-join-build.json`.

# Buddy 1 final review — 29 October 2026

## Decision

**Ready for staged communication-only integration; not yet a complete Vehicle
Controller interface or an unattended network-recovery sign-off.** Three
user-performed power-bank cold boots recovered MQTT, telemetry and five-second
status heartbeats. Command delivery to a controller, application acknowledgements,
hotspot recovery and combined sensor/task loading remain gates.

No motors were activated, no commands were published to hardware during this
final review, and no USB wall charger was used. Teammate runtime code is
unchanged. The minimum-bit tribunal exercise is **out of scope**, as clarified
by the lecturer through the user. Historical references calling it deferred do
not create an implementation requirement. Keep the existing readable payloads.

## Source and firmware identity

Review branch: `review/buddy1-finalization`, based on runtime commit
`0f07d6df448da5ca5f581cf4cb89f229c51ba475`.

Working image: `buddy1-clock-fix-no-usb.uf2`.

SHA256: `ed1ceeaeb06e8bca56ce19031d9796f7db68eada4899388c2552d95f47f3b893`.

The final review adds documentation, evidence and regression coverage only.
Its runtime remains the successfully tested `0f07d6d` source. A fresh build
from an isolated Git export reproduced the working UF2 **byte for byte**.
The saved ELF/map and original image remain preserved outside tracked source:

`C:/Users/muhdh/.codex/.chatgpt-projects/g-p-6abcb00776a081918f2ebf953585b093/buddy1-investigation/known-good/2026-10-29-clock-fix/`

Local Wi-Fi/MQTT configuration remains ignored and is not committed. The UF2
is also ignored by repository policy; its exact hash and evidence are tracked
under `testing/buddy1/finalization/2026-10-29/`. Nothing is pushed or merged.

## Troubleshooting change inventory

The audit compares `b4e9a2b` with `0f07d6d`; baseline MQTT/IPC behavior must
not be misattributed to this fix.

| Change | Classification and disposition |
|---|---|
| `02bf883`: failed initial Wi-Fi join retry, five-second cooldown, thirty-second pending timeout | Retain. Prevents a failed first join becoming a permanent application dead end; does not implement rejoin after a previously successful link drops. |
| `02bf883`: interface/link/IP and connection-attempt diagnostics in `mqtt_utk.c` | Retain for review. No protocol, broker, QoS or five-second heartbeat change. UART logging consumes caller time; measure impact under combined load. |
| `02bf883`: owner-task Pico W LED diagnostics and `mqtt_utk_is_connected()` accessor | Retain for bring-up. Owner-only radio GPIO calls; guard avoids I/O before radio readiness. LED is a last-observed state, not proof of continuing traffic. |
| `23ce50f`: direct configured-SSID join in MQTT builds | Retain. Removes dependency on a preliminary diagnostic scan-complete event. Non-MQTT qualification builds retain their scan path. |
| `0f07d6d`: RP2040 WATCHDOG.TICK initialization after clk_ref setup | Essential correction. Supplies the 1 MHz TIMER reference on cold boot. Does not enable watchdog resets or zero the time count. |
| Existing five-second status, three-second MQTT retry, QoS 0 telemetry, IPC bridges | Predate troubleshooting; inspected, not newly implemented. Async TCP/MQTT failure may take longer than the nominal retry interval. |
| UART-only versus USB CDC comparison UF2s | Build-profile experiments, not proof of a USB enumeration bug. Only the corrected single-core UART profile has the recorded three cold-boot passes. |
| Dashboard broker mismatch | Two brokers existed; the dashboard previously used localhost while the car used the LAN broker. Launch environment was corrected to the LAN broker. The already-untracked dashboard source was not changed or committed. |
| Temporary snapshots, host tests and investigation reports | Evidence, not additional runtime features. Earlier reports are dated historical states, superseded by this review and the final results. |

Source changes outside Buddy 1 are the shared RP2040 clock/register correction;
they affect other TIMER users such as Buddy 5 and require sensor timing checks.
Buddy 2/3/4/5 implementations, app_main startup selection, USB service and
kernel scheduler were not rewritten by troubleshooting.

The final review carries forward the investigation's newer thirteen-scenario
network test, seven-heartbeat test, radio-owner failure characterization and
test documentation. Generated test files are ignored; the clock test writes
under `tools/buddy1_startup_tests/build-clock/`.

## Probable root cause and confidence

**The missing TIMER reference initialization is the probable primary cause of
the observed power-bank timing/startup defect.** This conclusion has strong
source, reproducibility and before/after hardware support, with specific limits:

| Confirmed evidence | What it does and does not establish |
|---|---|
| Before the fix, board clock startup set clk_ref to 12 MHz but never programmed WATCHDOG.TICK. `sys_now()` divided TIMER counts as if they were microseconds. | Confirmed software initialization omission; fake-register regression fails before correction and passes afterward. |
| Earlier status arrived approximately every 213 seconds, with roughly 211/212 telemetry producer iterations per heartbeat. A later run had live telemetry but status older than 18 minutes. | Confirms sparse/stale status, not a dashboard-only display error. Does not directly measure the hardware divider. |
| Corrected source sets ENABLE=1 and CYCLES=12 before TIMER-based services. SDK and boot ROM use this tick reference. | Explains how a post-BOOTSEL warm restart can differ from a full power removal. The board register was not read during a failing boot. |
| Same direct-join UART profile plus the timer correction passed three subsequent user-performed power-bank cold boots. | Strong practical support for the correction. Not an isolated register experiment, statistical reliability guarantee or diagnosis of every earlier dark LED. |
| Independent MQTT log and dashboard snapshots agree on advancing status; each boot has a new TCP source port. | Confirms live recovery rather than cached dashboard messages or an old connection. Exact CONNECT/CONNACK and physical power-on times were not captured in these subscriber logs. |

Do not claim the cold board's exact divider value, a measured 42.7x timer error,
USB CDC starvation, power-bank undervoltage, packet-loss rate or a failed DHCP
allocation as established hardware causes. The earlier scan gating and broker
mismatch were distinct software/configuration issues, not proof that all
failures had a single cause.

Primary references: [RP2040 datasheet, timer and watchdog tick](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf),
[SDK clock/tick initialization](https://github.com/raspberrypi/pico-sdk/blob/2.3.1/src/rp2_common/pico_runtime_init/runtime_init_clocks.c),
[BOOTSEL versus normal flash boot](https://github.com/raspberrypi/pico-bootrom-rp2040/blob/master/bootrom/bootrom_main.c).

## Available validation rerun

Arm GCC 15.2.1, Pico SDK 2.3.1, host GCC, Python; single-core profile.
Logs and machine-readable commands/results are in the evidence directory.

| Check | Result and scope |
|---|---|
| Fresh UART firmware build, all objects rebuilt | PASS; text/data/BSS 326796/2616/47128 bytes; UF2 matches saved working hash exactly. |
| Fresh USB CDC firmware build, all objects rebuilt | PASS; text/data/BSS 337144/2640/53216 bytes. Hardware standalone behavior of this new USB variant is unverified. |
| Clock startup fake registers | PASS across reset/inherited tick values; ordering, timer count preservation and watchdog-reset control checked. |
| Network runner | Thirteen scenarios complete: twelve behavior cases pass; one characterization confirms that an initial DHCP UDP-PCB allocation failure does not automatically recover. Seven five-second heartbeats pass under a mocked clock. |
| Buddy command/telemetry path | Actual Buddy 1 sources and lwIP exercised with simulated kernel queues: telemetry payload, fragmented START, STOP, wrong topic and invalid command. No live radio, real scheduler or motors. |
| USB boundary runner | Four cases pass: absent host, blocked host, event burst, initialization failure. Bounded drops/sleep behavior, not physical USB IRQ fairness. |
| Initial join control | PASS for MQTT and qualification configurations. |
| LED helper | PASS for state selection, initialization guard and transition-only writes. |
| Radio-owner runner | Six cases complete: four behavior passes, two characterizations of missing initialization retry and missing application rejoin after link loss. These are not successful recovery tests. |
| Existing Buddy 5 host tests | PASS for obstacle-state hysteresis/fault handling and independent distance formula. No ultrasonic hardware timing or real distance accuracy established. |

No full all-buddies application, SMP=1 profile, motor timing, Wi-Fi outage or
physical START/STOP consumer test was performed during this finalization.
The selected application remains Buddy 1 + Buddy 5. Unfiltered standalone demo
sources are not a valid combined application target.

## Recorded hardware evidence

| Check | Supported result |
|---|---|
| Normal publishing, 15:05:12–15:05:51 +08:00 | PASS: telemetry 276–315, heartbeat 52–59 at exactly five-second captured intervals. |
| Cold boot 1 | PASS: new port 59555; fresh telemetry and heartbeat 5 → 6 → 8 in API samples; intermediate status messages exist in subscriber log. |
| Cold boot 2 | PASS: new port 53525; heartbeat reset then advancing; telemetry resumed. |
| Cold boot 3 | PASS: new port 63041; heartbeat 0/1/2/3/4 at 15:12:42/:47/:52/:57/15:13:02; telemetry resumed. |

User performed/reported the power cycles; the assistant independently observed
traffic, freshness and socket changes. Power source, flashed bytes and power-on
instants were not independently instrumented. No claim of loss-free telemetry:
startup bursts and sequence gaps were observed. The first stale API sample
during some reconnects contains cached previous-run data and is not counted
as a post-connection failure. All preserved original firmware remains intact.

## Findings that constrain integration

1. **Controller handoff is missing.** `command.c` consumes and logs commands.
   Adding another consumer to that same queue would race with the logger.
   Choose one controller consumer/handoff, with explicit command acceptance
   and completion evidence. Broker delivery or dashboard “publish queued” is
   not application execution.
2. **Recovery is not signed off.** Initial radio failure parks the owner;
   initial DHCP allocation error can remain OFF; post-success application
   rejoin is absent. Driver auto-rejoin is unverified. Test broker/hotspot loss
   before claiming unattended reliability; do not add blind radio reset loops.
3. **Drops and freshness need an integration policy.** IPC is non-blocking,
   finite queues drop, and telemetry is removed before MQTT publish success.
   A failed status attempt advances its attempt timestamp. No retained status
   or last will is configured. Dashboard accepts any `/status` payload as
   freshness and ignores telemetry for that indicator. Keep these limitations
   explicit; avoid masking failures by extending its timeout.
4. **Single ownership is essential.** One radio/lwIP owner already exists.
   Buddy 2 must own actuator/encoder updates; the controller arbitrates decisions.
   Network callbacks must not become motor drivers or call blocking sensor code.
5. **Pins and native driver adapters are blockers for a combined image.**
   UART uses GP0/GP1, while Buddy 3 uses GP0 as a line input. BSP external LED
   default GP16 conflicts with Buddy 4 I2C SDA GP16. The current SDK-compat IRQ
   registration hooks are weak no-op stubs; do not assume linking SDK encoder
   callbacks makes them work under this kernel. Review existing Buddy 2/4
   native trials before changing shared BSP or IRQ handling.

Detailed gates: [hardware checklist](BUDDY1_HARDWARE_CHECKLIST.md).
Next work: [prioritized integration plan](PICOCAR_INTEGRATION_PLAN.md).

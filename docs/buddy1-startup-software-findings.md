# Buddy 1: source and software investigation, 28 October 2026

**Standalone operation remains unresolved.** The retry firmware did not restore
observable MQTT during the recorded power-bank run. The user reported that the
extra hotspot device remained connected, but no fresh car TCP connection or
numbered heartbeat was observed. Association evidence does not establish DHCP.

This follow-up used source inspection and host software tests only. No board
was flashed, no broker traffic or motor command was sent, and no electrical
setup was changed during this follow-up. Wall-charger tests are excluded.

## 1. DHCP without USB enumeration

There is no USB-enumeration condition in the inspected DHCP path. In Pico SDK
2.3.1, `lib/cyw43-driver/src/cyw43_lwip.c:213-226` brings up the network interface
and starts DHCP; its link-up callback at line 293 updates lwIP. With no link,
DHCP waits in INIT. Link-up starts discovery. The radio owner's regular
`sys_check_timeouts()` call (`cyw43_utk.c:519`) advances DHCP timers.

The tests ran the real lwIP DHCP implementation with synthetic Ethernet
OFFER/ACK packets and verified the assigned IP, gateway, DNS and lease state.
Both an immediate server response and one delayed by 45 simulated seconds
succeeded without USB code. MQTT made no connection attempt before an IP was
available. This verifies the protocol path, **not DHCP on the physical board**.

`cyw43_utk.c:397-428` latches its diagnostic DHCP result after 30 seconds.
A later lease can leave that report saying timeout: the diagnostic stops
updating, while lwIP continues running. MQTT checks the live interface address,
so this stale report does not itself block the numeric-IP broker connection.

## 2. Reaching mqtt_client_connect()

The radio task calls `mqtt_utk_poll()` at `cyw43_utk.c:529`, independently of the
USB-gated status-printing task in `app_program/buddy1/network.c:39`.
`mqtt_utk.c:440` requires interface-up, link-up and nonzero IPv4. It does not
require USB, the diagnostic DHCP-complete flag, or DNS test completion.

Once those conditions hold, client allocation at `mqtt_utk.c:474` resets the
attempt timestamp. The initial `mqtt_client_connect()` at line 570 therefore
occurs three seconds after successful allocation. The synthetic peer test
observed this call, completed TCP and MQTT handshakes, and received a heartbeat.
Thus this source can reach CONNECT without USB; reaching it on power-bank
hardware remains unverified.

## 3. Initial MQTT failure recovery

The full production MQTT adapter and real lwIP were exercised together.
Immediate failure clears `connecting` at `mqtt_utk.c:581`. Asynchronous
TCP/MQTT outcomes invoke `connection_callback()` at line 203, which clears it
at line 212. The application retries after a three-second cooldown once the
attempt has ended; it deliberately does not overlap a pending attempt.

| Injected condition | Observed software behavior |
|---|---|
| Immediate connect-call error | Retry after 3 seconds |
| First client allocation fails | Allocation retries after 3 seconds; CONNECT 3 seconds later |
| No broker ARP response | Next attempt 33.75 seconds after the first |
| ARP works, TCP SYN unanswered | Next attempt 21.25 seconds after the first |
| TCP explicitly refused | Retry 3 seconds after refusal |
| TCP established, no MQTT CONNACK | Next attempt 103 seconds after the first |
| MQTT CONNACK rejects client as unavailable | Retry 3 seconds after rejection |
| Successful TCP/MQTT exchange | Connected state and heartbeat publication |

These are deterministic simulated timings with SDK 2.3.1, the repository's
lwIP configuration and a 10 ms test polling interval, not hardware guarantees.
TCP connection setup uses TCP's own timers. The MQTT CONNECT timeout starts
only after TCP connects (`SDK lib/lwip/src/apps/mqtt/mqtt.c:1070-1073`). All
timeout-based recovery depends on the radio task continuing to service lwIP.
No stuck initial-MQTT retry state was reproduced under these conditions.

## 4. USB initialization and scheduling

`kernel/sysdepend/pico_rp2040/devinit.c:101,107` starts the USB and radio
services separately. USB is priority 3; radio is priority 4, both on core 0
in the tested SMP=0 profile. Higher-priority USB activity can therefore affect
radio scheduling even though enumeration is not a network prerequisite.

The real USB console implementation was compiled against a fake kernel and
TinyUSB boundary. With 10,000 output bytes and no host it retained 4,095 bytes,
dropped 5,905 and requested its normal 10 ms sleep. The same held with an
enumerated host accepting no data. A finite 100-event burst drained and then
slept. A failed TinyUSB initialization exited the service task.

Relevant source is `lib/libtm/sysdepend/pico_rp2040/usb/usb_console.c:160,175,208,283`.
This rules out an unconditional host wait or full-ring wait at that boundary.
The tests replace TinyUSB initialization and do not emulate RP2040 interrupts.
`usb_sdk_compat.c:47-59` wakes the task on every USB interrupt; a sustained event
stream or accumulated wakeups could delay lower-priority work. No such storm
has been measured. Enumeration workaround waits are disabled in the project's
`usb/pico/config_autogen.h:17-18`; this is source evidence, not a hardware check.

## 5. Controlled firmware changes and remaining failure paths

The existing five-second initial Wi-Fi join retry remains appropriate and its
regression checks pass for MQTT=0 and MQTT=1. It handles rejected, timed-out
and terminally failed initial joins without overlapping attempts or disturbing
a late successful link. It does not implement recovery after a previously
successful Wi-Fi association is lost.

Two earlier gates remain in the production radio task:

- Radio setup or scan-start error reaches the permanent sleep at
  `cyw43_utk.c:484-510`. In this case neither DHCP nor MQTT polling runs.
- A scan that never finishes prevents the first join at
  `cyw43_utk.c:563-584`. Polling runs, but there is no join request.

These are real source failure paths, but neither has been demonstrated on the
power bank. Removing the preliminary scan only for MQTT builds is a possible
small follow-up if scan failure is established; it would not repair DHCP,
USB interrupt starvation or radio initialization failure. Adding a second MQTT
retry loop is not justified by the passing protocol tests.

No further runtime patch or new firmware is introduced by this follow-up.
The separate, previously prepared LED diagnostic remains an optional later
hardware observation tool, not a reliability fix. Its radio writes can alter
timing and its frozen state only represents the last successful poll.

## Reproduction and limits

Added `tools/buddy1_startup_tests/` contains the two runners, C harnesses and
README. Run from the repository root:

```powershell
python tools/buddy1_startup_tests/run_network_tests.py --sdk C:/Users/muhdh/.pico-sdk/sdk/2.3.1
python tools/buddy1_startup_tests/run_usb_tests.py
python tools/test_wifi_startup.py
```

Results: 10 network scenarios, 4 USB scenarios, and both existing Wi-Fi join
configurations pass. Network tests use the actual repository MQTT source,
not the workspace-only LED variant. They use host GCC, simulated time and
packets; 64-bit host heap alignment is the only lwIP option override. Console
and Buddy IPC are stubbed; command ingestion is forbidden by an assertion.
No board, sockets or live broker are involved.

Only test/documentation files were added or updated in this follow-up, so no
new firmware build was needed. The earlier retry and LED images already built
with Arm GCC 15.2.1 / SDK 2.3.1 in a separate tree. Those builds do not establish
standalone behavior. Existing firmware artifacts and teammate subsystems remain
untouched. A reliable physical startup claim still requires observing a new
board connection after a standalone cold boot.

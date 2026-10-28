# Buddy 1 standalone startup investigation — 28 October 2026

The power-bank failure remains **unresolved**. The user flashed the retry
firmware; it worked on laptop USB, but no fresh car connection or numbered
heartbeat was observed in the subsequent power-bank run. The user reports the
extra hotspot device stays connected; its DHCP address remains unverified.
The assistant sent no motor command. See
[the software follow-up](buddy1-startup-software-findings.md) for source findings
and host tests. Further electrical comparisons are paused; wall chargers are
excluded at the user's request.

## Findings from the local source

Baseline: `b4e9a2b`. The local `dashboard/` directory was already untracked.

- `app_program/buddy1/network.c:39` waits for USB enumeration, but this task
  only reports status. It does not start or poll the radio or MQTT.
- `kernel/sysdepend/pico_rp2040/devinit.c` starts the USB and radio services
  independently. The USB task has priority 3 and the radio task priority 4.
  The USB producer uses a finite ring and drops excess output; it does not
  wait for a host. No host-dependent wait was found on the inspected network
  startup path. This does not rule out a hardware or lower-level USB fault.
- `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c` initializes the radio, scans,
  and joins after scan completion. Before this patch, an immediate join error,
  terminal link error, or 30-second join timeout had no application retry.
  The driver also has internal retries for certain authentication events;
  those do not replace the missing application retry for all failures.
- Radio/bus or scan-request failure still parks the radio service. A scan
  that never completes still prevents joining. These are separate unresolved
  possibilities; adding blind hardware resets without evidence was avoided.
- `lib/libnet/lwip/mqtt_utk.c` polls independently of the status-report task.
  It requires interface up, link up, and nonzero local IPv4, then attempts
  `172.20.10.2:1883` as `picocar-vehicle1`. It retries failed MQTT connections
  with a 3-second interval once the asynchronous attempt finishes. A TCP
  attempt can take longer than three seconds to fail. DNS test completion
  is not a runtime gate for this numeric-IP broker connection.
- DHCP's 30-second completion flag is diagnostic. lwIP timeouts continue
  running afterward, and MQTT checks the live address, not that flag. A later
  lease can therefore permit MQTT even if the DHCP report says timed out.
- The local `dashboard/app.py:33` timestamps every received message, and
  `:78` treats a `/status` timestamp younger than 15 seconds as a heartbeat.
  A manual publication to `/status` can make it appear Online. The later
  retry-run capture contained `network-test` on `/status`; publisher identity
  was unavailable. The earlier chat's test payload/topic remain unknown.
  The dashboard was not edited.
- `app_program/app_main.c` starts Buddy 1 and Buddy 5. Buddy 1's command
  consumer currently logs commands; this patch does not add motor operations.

## Focused changes

- `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c:46,458,567`: MQTT builds retry
  a failed initial join five seconds after failure. In-progress joins are not
  overlapped, and a late successful link prevents retry. Scan/join qualification
  profiles retain their one-shot behaviour. This is not a full reconnection
  policy after a previously successful association is lost.
- `lib/libnet/lwip/mqtt_utk.c:414,562`: existing five-second debug output now
  includes interface/link/IP; each broker attempt logs client ID, endpoint
  and uptime. These logs do not depend on opening COM5 to advance networking.
  USB buffering is finite, so attaching a terminal later is not a guaranteed
  complete startup capture. Under power-bank-only operation use broker logs.
- `tools/test_wifi_startup.py`: compiles the production join-control block with
  a fake clock/radio. It tests immediate failure, cooldown, pending attempt,
  timeout, terminal failure, success, late success, and non-MQTT behaviour.

## Validation and build

The regression check passes with `TM_WIFI_MQTT=0` and `=1`:

```powershell
python tools/test_wifi_startup.py
```

A native Windows build linked and produced UF2 using installed Arm GCC
15.2.1 and Pico SDK **2.3.1**. This differs from the README's SDK 2.2.0
qualification baseline; existing local object dependencies also used 2.3.1.
No physical behaviour is inferred from a successful build.

The unfiltered application build fails on unrelated Buddy 2/3 standalone
source dependencies (`pico/stdlib.h`, `motion_control.h`). `BUDDY1_ONLY=1`
alone also excludes Buddy 5 while `app_main.c` calls it. The successful build
therefore explicitly selects exactly the application sources currently used:

```powershell
$env:PATH = 'C:\Users\muhdh\.pico-sdk\toolchain\15_2_Rel1\bin;C:\raylib\w64devkit\bin;' + $env:PATH
Set-Location build_make
make SMP=0 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WIFI_MQTT=1 PICO_SDK_PATH=C:/Users/muhdh/.pico-sdk/sdk/2.3.1 'TEMP_SRCS=../app_program/app_main.c $(wildcard ../app_program/buddy1/*.c) $(wildcard ../app_program/buddy5/*.c)' -j8
```

Builds were run in the separate investigation copy, preserving the original
repository's firmware artifacts. This command in the original repository
would overwrite that profile's output, so save it elsewhere first.

## Optional later test without COM5

The current plan is source inspection and software tests. The following is a
reference for a later user-controlled test, not a request to change power now.

1. Keep the broker, dashboard, and laptop on the same iPhone hotspot. Confirm
   the laptop's current Wi-Fi IPv4 is **172.20.10.2** with `ipconfig`; a changed
   address requires matching firmware broker configuration. Ensure the existing
   broker listener and firewall still accept LAN connections. A local dashboard
   connection by itself does not verify this.
2. Keep the existing broker's verbose console or file log visible. If necessary,
   enable `connection_messages true` and `log_type all` in its existing config,
   retaining its current listener/authentication settings. Start logging before
   the test. Do not launch a competing broker on port 1883. Official reference:
   https://mosquitto.org/man/mosquitto-conf-5.html
3. Start a separate subscriber; `-R` suppresses retained messages delivered on
   subscription. Use a unique observer client ID, never `picocar-vehicle1`:

   ```powershell
   & 'C:\Program Files\mosquitto\mosquitto_sub.exe' -h 172.20.10.2 -p 1883 -i buddy1-startup-observer -t 'picocar/vehicle1/#' -v -R
   ```

   Official option reference: https://mosquitto.org/man/mosquitto_sub-1.html
4. First test the existing firmware. Power the board completely off between
   runs; use the same cable and board USB socket for laptop USB (COM5 closed)
   and the existing power bank only. Do not use a wall charger. Record power-on
   time and observe
   each run for at least 120 seconds. Keep motors disabled as in the current
   firmware. Do not send START or publish anything to `/status` or `/telemetry`.
5. Look for a **new**, timestamped broker entry such as `New client connected
   from <board-IP>:<port> as picocar-vehicle1`, followed by status/telemetry
   publications from that client. An old connection timing out is not a fresh
   connection. Observe increasing `heartbeat seq=N` about every five seconds
   and live telemetry about once per second. Subscriber messages alone do not
   identify the publisher; correlate them with the broker log.
6. Repeat with the separate retry test UF2 only when ready to flash it. Retain
   the preserved original UF2 for rollback. Also test cold boot with the hotspot
   initially unavailable, enabling it after about 45 seconds; this specifically
   exercises initial join recovery. Do not change firmware and power source
   together in a comparison.

| Observation | What it establishes / next evidence |
|---|---|
| iPhone connected-device count rises | Supporting association evidence only; does not prove a DHCP lease, unique device identity, or MQTT. |
| New broker TCP connection from board IP only | IP communication reaches the laptop; check for MQTT CONNECT/client acceptance next. |
| New accepted `picocar-vehicle1` connection and its publications | Fresh MQTT startup works for that run; record source IP and timestamps. |
| No new broker connection | Association, DHCP, routing, firewall, or boot may still be failing; broker silence alone cannot separate them. |
| Laptop USB works with COM5 closed; power bank fails | Host/power-dependent startup remains a hypothesis; the diagnostic-task USB wait is still not proof. |
| New connection followed by failure without unplugging | Record disconnect/reconnect timing; investigate runtime recovery separately. |

For the no-connection case, a Wireshark capture on the laptop's hotspot Wi-Fi
interface with `arp || bootp || tcp.port == 1883` may provide positive evidence:
a DHCP ACK addressed to the board proves a lease, an ARP request from a nonzero
board IP proves that it uses that address, and SYN/CONNECT/CONNACK distinguish
TCP from MQTT. Identify the board by its MAC or known baseline address first.
An ordinary laptop capture may not see other clients' unicast DHCP exchanges;
missing packets do not establish DHCP failure. `arp -a` is a supporting cache,
not the iPhone's DHCP lease table. A static IP should not be guessed.

If still unresolved, obtain owner-task LED stage diagnostics or an independent
UART capture in a subsequent focused change. The Pico W LED is radio-driven,
so it cannot by itself diagnose a radio initialization failure. No LED pin or
teammate subsystem was changed here.

Save one row per run: firmware filename/hash, power source, power-on time,
hotspot count before/after, new board connection time/IP, first heartbeat,
last heartbeat, and relevant broker lines. Those observations are needed before
claiming that power-bank startup is fixed.

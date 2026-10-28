# Windows: Buddy 1 standalone startup diagnostic

Branch: `diagnostic/buddy1-powerbank-startup`.

**This is a diagnostic build, not a confirmed power-bank fix.** Source review
has not established a probable hardware root cause. The earlier initial-join
retry handles a real missing recovery path, but the recorded power-bank run
still failed. This branch retains that retry and adds an owner-task LED stage
indicator so the next change can target the observed failure.

The build starts the existing Buddy 1 and Buddy 5 application only. No teammate
source, motor behavior, Wi-Fi credentials, broker endpoint or topic is changed.
The consumer in `app_program/buddy1/command.c` still only logs commands.
No hardware test or firmware flash was performed by the assistant.

## Source and findings

- `app_program/buddy1/network.c:39`: USB wait affects status printing only.
- `kernel/sysdepend/pico_rp2040/devinit.c:101,107`: USB and radio start separately.
- `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c`: owns radio polling, DHCP timers
  and MQTT polling. Setup/scan errors can prevent startup; their occurrence
  on the physical power-bank boot is unknown. Its new `startup_led_poll()`
  runs in that same task and never drives a motor pin.
- `lib/libnet/lwip/mqtt_utk.c`: live interface/link/IPv4 gate, existing retries,
  and an owner-only connection-state getter for the indicator. MQTT retry
  failures were not reproduced in software.
- `lib/libnet/lwip/mqtt_utk.h`: declares that getter.

The indicator uses **CYW43 GPIO 0**, the small LED on the Pico W module.
It does not use RP2040 GPIO 25, which is the radio chip select on Pico W.
LED I/O occurs on transitions, with an initialized-radio guard. It does add
radio transactions and can alter timing: a successful diagnostic run alone
does not prove the original firmware or a later non-diagnostic fix works.

## 1. Locate and preserve the images

In PowerShell:

```powershell
$bundle = 'C:\Users\muhdh\.codex\.chatgpt-projects\g-p-6abcb00776a081918f2ebf953585b093\buddy1-investigation\firmware'
Get-FileHash "$bundle\buddy1-powerbank-diagnostic.uf2" -Algorithm SHA256
```

The matching hash and build information are in `powerbank-diagnostic-build.json`
in that folder. The preserved rollback image is `existing-before-investigation.uf2`.
Keep both. Firmware contains the existing Wi-Fi configuration; keep it private.

## 2. Prepare the laptop before flashing

1. Enable the same iPhone hotspot and connect the laptop to it.
2. Run `ipconfig`. Confirm the laptop's hotspot-facing IPv4 is `172.20.10.2`,
   the broker address compiled into this image. If it changed, stop this test
   and update the firmware configuration/build to match the actual address.
3. Keep the existing Mosquitto broker and dashboard running. Do not start a
   second broker. Keep its verbose connection log visible, ideally with
   `connection_messages true` and `log_type all` in its existing configuration.
4. Close COM5 and any application that automatically opens it.
5. Start this independent observer in another PowerShell window:

   ```powershell
   & 'C:\Program Files\mosquitto\mosquitto_sub.exe' -h 172.20.10.2 -p 1883 -i picocar-powerbank-observer -t 'picocar/vehicle1/#' -v -R -d
   ```

   Its own CONNACK/SUBACK and PINGRESP confirm only the observer's connection.
   Never use `picocar-vehicle1` as the observer's client ID. `-R` excludes retained
   messages. Do not publish a network-test message to `/status` or `/telemetry`:
   that can make the dashboard appear Online without a working car.

## 3. Flash using laptop USB

1. Disconnect the board from the power bank and every other power source.
2. Hold **BOOTSEL on the Pico W module**, then connect the board USB port to
   the laptop using the existing data cable. Release BOOTSEL once Windows
   shows the **RPI-RP2** drive. This is a storage drive, not COM5.
3. In File Explorer, copy `buddy1-powerbank-diagnostic.uf2` onto RPI-RP2.
   The drive should disappear as the board restarts. Copying the file is the
   flash operation; do not open COM5 afterward.
4. Leave it on laptop USB for up to three minutes. Look for a new broker
   connection identifying **picocar-vehicle1**, a solid Pico W LED, fresh
   `heartbeat seq=N` messages about every five seconds and changing telemetry
   about once a second. Record the broker connection time and source IP.
5. If this baseline fails, record the LED pattern and broker output before
   proceeding. A failed laptop baseline cannot isolate the power-bank issue.

## 4. Cold-start using the existing power bank

1. Leave the laptop broker, observer and hotspot running. Keep COM5 closed.
2. Disconnect all power from the board for ten seconds. Do not hold BOOTSEL.
3. Connect the same board USB port and cable to the **existing power bank**.
   Use no wall charger. Record the power-on time, for example with
   `Get-Date -Format o` in PowerShell.
4. Watch the small Pico W LED for 15-20 seconds to count complete groups, then
   observe the broker/subscriber for three minutes. Record pattern changes.
5. Look for a **new** broker connection after power-on identifying
   `picocar-vehicle1`, followed by increasing heartbeat and telemetry sequences.
   An old connection timing out, hotspot count, dashboard Online alone, or the
   observer's PINGRESP does not establish a fresh car connection.
6. If successful, repeat three complete power-bank cold starts. Record each
   run independently. This tests automatic startup without ever opening COM5.

| Pico W LED, groups repeating every four seconds | Last observed stage |
|---|---|
| 1 flash | Preliminary scan has not completed |
| 2 flashes | Scan completed; Wi-Fi link down / joining |
| 3 flashes | Link up, but interface or IPv4 not yet usable |
| 4 flashes | Interface and IPv4 usable; MQTT not connected |
| 5 flashes | Scan-start/setup error after the STA interface initialized |
| Solid | MQTT was connected at the last radio-task poll |
| Dark, or frozen state | Ambiguous; boot, radio initialization, LED I/O or task progress may be involved |

A frozen solid LED is not proof of ongoing MQTT. Use fresh broker messages.
Three flashes do not by themselves prove DHCP server failure; four flashes
do not prove the laptop broker received a connection attempt.

## 5. Command check and what software already verified

Host tests exercise real lwIP MQTT and the unchanged Buddy 1 code: subscription
to `/command`, fragmented START and normal STOP through the real parser and
consumer with simulated queues, invalid-command/wrong-topic rejection, and
telemetry forwarding with exact topic/payload checks. No motor code is linked.

An optional physical command check can be made on laptop USB **after** verifying
automatic MQTT startup with COM5 closed: open COM5 using the existing terminal
settings, then send only STOP from PowerShell:

```powershell
& 'C:\Program Files\mosquitto\mosquitto_pub.exe' -h 172.20.10.2 -p 1883 -t 'picocar/vehicle1/command' -m 'STOP'
```

The board should log `[MQTT] RX ... payload=STOP` and `[Command] type=STOP ...`.
Seeing your own STOP in the subscriber does not prove the board consumed it.
There is currently no command acknowledgment topic, so broker/subscriber output
alone cannot prove command-consumer execution on power-bank power. This optional
serial check is separate from the standalone startup test; close COM5 afterward.

## Rollback and next evidence

Repeat the BOOTSEL procedure and copy `existing-before-investigation.uf2` to
RPI-RP2 to restore the preserved firmware. Do not change teammate sources.

Record: firmware hash; power source; power-on time; LED pattern over time;
new car connection time/IP; first and last heartbeat; telemetry progression.
The missing observation is the first stalled stage on a power-bank cold boot.
That evidence determines a minimal targeted fix; the current branch does not
claim to make standalone startup reliable.

## Rebuild and software checks

Builds were performed in the separate investigation source copy to preserve
the original repository's existing firmware artifacts. Arm GCC 15.2.1 and
Pico SDK 2.3.1 were used. From that copy's `build_make` directory:

```powershell
$env:PATH = 'C:\Users\muhdh\.pico-sdk\toolchain\15_2_Rel1\bin;C:\raylib\w64devkit\bin;' + $env:PATH
make -B SMP=0 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WIFI_MQTT=1 PICO_SDK_PATH=C:/Users/muhdh/.pico-sdk/sdk/2.3.1 'TEMP_SRCS=../app_program/app_main.c $(wildcard ../app_program/buddy1/*.c) $(wildcard ../app_program/buddy5/*.c)' -j8
```

The explicit source list matches `app_main.c` and avoids unrelated standalone
Buddy 2/3 test sources that fail the unfiltered application build. It changes no
teammate implementation. SDK 2.3.1 differs from the README's 2.2.0 baseline.

From the branch repository root, with host GCC available:

```powershell
python tools/test_wifi_startup.py
python tools/buddy1_startup_tests/run_network_tests.py --sdk C:/Users/muhdh/.pico-sdk/sdk/2.3.1
python tools/buddy1_startup_tests/run_usb_tests.py
python tools/buddy1_startup_tests/test_led_diagnostic.py
```

Passing software checks and a firmware build do not constitute physical tests.

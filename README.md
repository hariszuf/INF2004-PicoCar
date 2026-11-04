# INF2004 PicoCar

Autonomous robotic car project running on Raspberry Pi Pico W with
µT-Kernel 3.0.

Course reference: the professor's
[INF2004_ProjectTemplate](https://github.com/sirfonzie/INF2004_ProjectTemplate).
The comparison below follows its README; the setup commands here follow
PicoCar's checked-in makefiles and application code.

## Current RTOS configuration

- Board: Raspberry Pi Pico W / Cytron Robo-Pico
- MCU: RP2040
- RTOS: µT-Kernel 3.0
- Mode: Single-core (`SMP=0`)
- Debug console: USB CDC
- Pico SDK: 2.2.0

## Differences from the course template

| Area | Course template (documented behavior) | This PicoCar repository |
|---|---|---|
| Bare `make` | `SMP=1 CONSOLE=usb_cdc WIFI=cyw43` | `SMP=0 CONSOLE=uart WIFI=none`; pass the USB/Wi-Fi flags explicitly. |
| Application | `subsystem1/`–`subsystem5/` examples | The driving build selects the vehicle controller and all five Buddy subsystems through `picocar_integration.mk`. |
| Shared definitions | `app_program/app.h` | Shared files belong in `app_program/shared/`; see `TEAM.md` for interfaces. |

Use the commands below when building PicoCar. Build options apply to each invocation;
a subsequent bare `make` returns to the local defaults above.

## Future implementation

- Add a `TEST=1` firmware profile that runs subsystem self-tests at startup.
- Add a `CRASH_REPORT` build option to control UART crash reporting.
- Align networking configuration with the course template's `NET=lwip`,
  `MQTT=1` and `wifi_secrets.h` approach.

These are planned improvements; use the setup and networking commands below
for the current implementation.

## Setup

### 1. Install the build tools

You need Git, GNU Make, the Arm GNU toolchain (`arm-none-eabi-gcc` and
`arm-none-eabi-size`), and a host C++ compiler (`g++`). The SDK supplies driver
sources; install the Arm compiler separately. The inherited RTOS port records
GCC **13.2.1** as its qualification baseline, rather than a required exact
version. This project's documented SDK version is **2.2.0**.

The makefiles also use utilities such as `sh`, `find`, `xargs`, `rm`,
and `mkdir`. On Windows, use the native PowerShell setup in the
[hotspot walkthrough](#test-the-dashboard-and-car-on-a-windows-hotspot).
It uses w64devkit for those tools and the host C++ compiler. The general
Ubuntu and macOS examples below use Bash.

For Ubuntu/Debian, including Ubuntu under WSL:

```bash
sudo apt update
sudo apt install git gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
```

On macOS, install the basic tools with `xcode-select --install`, then install
the [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
for your host and the `arm-none-eabi` target. A manually downloaded toolchain
also needs its `bin` directory on `PATH`; the Ubuntu package above sets that
up automatically. For WSL, use Linux tools rather than Windows executables.

Check the tools before continuing:

```bash
git --version
make --version
arm-none-eabi-gcc --version
arm-none-eabi-size --version
g++ --version
```

This project uses plain GNU Make. CMake and `picotool` are not required;
the build compiles its own ELF-to-UF2 converter with the host `g++`.

### 2. Clone PicoCar

```bash
git clone https://github.com/hariszuf/INF2004-PicoCar.git
cd INF2004-PicoCar
```

### 3. Install Pico SDK 2.2.0

For the project's USB CDC profile, initialise TinyUSB. Also initialise the
CYW43 driver and lwIP if you intend to use Wi-Fi:

```bash
mkdir -p "$HOME/pico"
git clone --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git "$HOME/pico/pico-sdk"
git -C "$HOME/pico/pico-sdk" submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
export PICO_SDK_PATH="$HOME/pico/pico-sdk"
```

Set `PICO_SDK_PATH` in each new terminal, or add the export to `~/.bashrc`
(`~/.zshrc` on macOS). It must point to the SDK root, not its `src` or `lib`
directory. Under WSL, use the Linux path. Prefer paths without spaces because
the makefiles use SDK paths in dependency lists.

Verify the source files needed for USB and the optional IP networking build:

```bash
ls "$PICO_SDK_PATH/lib/tinyusb/src/tusb.c"
ls "$PICO_SDK_PATH/lib/cyw43-driver/src/cyw43_ll.c"
ls "$PICO_SDK_PATH/lib/lwip/src/core/init.c"
```

### 4. Build the firmware

From the PicoCar repository root:

```bash
cd build_make
make SMP=0 CONSOLE=usb_cdc PICO_SDK_PATH="$PICO_SDK_PATH" -j8
```

The output is `build_make/mtk3pico_smp0_usb_cdc.uf2` relative to the repository
root, alongside the `.elf` and `.map` files. Only flash the UF2 file.

Other build profiles, run from `build_make/`:

| Profile | Command | UF2 output |
|---|---|---|
| Single-core, UART (Makefile default; no SDK needed) | `make SMP=0 CONSOLE=uart -j8` | `mtk3pico_smp0_uart.uf2` |
| Dual-core, USB CDC | `make SMP=1 CONSOLE=usb_cdc PICO_SDK_PATH="$PICO_SDK_PATH" -j8` | `mtk3pico_smp1_usb_cdc.uf2` |

The PicoCar working profile is single-core; upstream SMP qualification does
not establish that every PicoCar subsystem has been tested with two cores.
The build automatically invalidates old objects when profile flags change.
After editing application code, repeat the same build command with all the
desired flags and flash the resulting UF2 again.

### 5. Flash the Pico W

1. Hold **BOOTSEL** while connecting the Pico W with a USB data cable.
2. Release BOOTSEL when the **RPI-RP2** drive appears.
3. Copy `mtk3pico_smp0_usb_cdc.uf2` from `build_make/` onto that drive.
4. The board reboots automatically. Open its USB serial port in a serial monitor.

On Windows, select the new COM port shown in Device Manager and open it at
**115200 baud** in PuTTY or VS Code Serial Monitor. On Linux, the port usually
appears as `/dev/ttyACM0`; on macOS, look for `/dev/tty.usbmodem*`. For a WSL
build, run `explorer.exe .` from `build_make/` to open the output folder,
copy the UF2 to the board in Explorer, then use a Windows serial monitor.
For the UART profile, use a 3.3 V USB-serial adapter: Pico GP0 (TX) to adapter
RX, GP1 (RX) to adapter TX, and common GND, at **115200 baud, 8N1**.

For the integrated vehicle, use the driving build in
[Configure and flash the Pico W](#configure-and-flash-the-pico-w). It selects
the vehicle controller entry point and the Buddy tasks.

## Working on a subsystem

Application sources live in `app_program/`. The local build automatically
includes `.c` files directly in that directory and **one directory level
below it**, including buddy and shared folders. Deeper nested sources need
an explicit build change.

- Keep each subsystem's implementation, public header and application drivers
  in its `app_program/buddyN/` folder. `device/` contains kernel drivers.
- Integrate `buddyN_init()` and `buddyN_start()` in `app_main.c`: initialise
  all shared resources before starting any subsystem tasks.
- Record ownership, GPIOs, priorities and shared interfaces in [TEAM.md](TEAM.md).
  Keep GP23/GP24/GP25/GP29 reserved for the Pico W radio and GP0/GP1 for the
  current UART mirror; check Robo-Pico wiring before assigning other pins.
- Define cases and record actual test results in [testing/](testing/README.md).
  Kernel resource limits are configured in `config/config.h`.

## Optional: Wi-Fi, DHCP and DNS

From the repository root, create your local Wi-Fi configuration:

```bash
cp config/wifi_credentials.example.h config/wifi_credentials.h
```

Edit `config/wifi_credentials.h`: set `UTK_WIFI_SSID` and `UTK_WIFI_PASSWORD`,
then change `UTK_WIFI_CREDENTIALS_SET` to `1`. This local file is ignored by
Git; keep credentials out of committed files.

Build from `build_make/` with the SDK environment variable set:

```bash
make SMP=0 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 PICO_SDK_PATH="$PICO_SDK_PATH" -j8
```

Flash `build_make/mtk3pico_smp0_usb_cdc_wifi_dns.uf2`. DHCP assigns the network
address, so this profile does not need `network_config.h`. Static addressing
and UDP/TCP echo profiles need additional local configuration; see the
[networking walkthrough](docs/GETTING_STARTED.md#7-networking-development-profile).
Upstream treats networking as a development profile. Record PicoCar hardware
results using the [testing instructions](testing/README.md).


## Buddy 1 — Wi-Fi and MQTT

Buddy 1 joins Wi-Fi, connects to Mosquitto, and publishes the five-second
vehicle heartbeat and sensor reports. It also receives START/STOP through
the command queue. The dashboard reports that a command was queued for
publication. Controller acknowledgement and physical execution are separate
from MQTT delivery.

| Topic | Direction | Content |
|---|---|---|
| `picocar/vehicle1/status` | Pico W → broker | Vehicle heartbeat |
| `picocar/vehicle1/telemetry` | Pico W → broker | Startup probe and sensor reports |
| `picocar/vehicle1/command` | Dashboard → broker → Pico W | Startup acknowledgement, START or STOP |

<a id="first-run-on-windows"></a>

### Test the dashboard and car on a Windows hotspot

Use the `Full-Implemented-wif-microt` branch for both firmware and dashboard.
The dashboard acknowledges a telemetry probe at startup. The car queues its
first sensor report after that acknowledgement and can then move automatically
once its other readiness checks pass.

**What you need**

- A Raspberry Pi Pico W on the car, a USB data cable, a Windows laptop, and a
  trusted 2.4 GHz hotspot using WPA2-Personal.
- [Python 3.10 or newer](https://www.python.org/downloads/windows/) on Windows
  with `python` on `PATH`, and [Eclipse Mosquitto](https://mosquitto.org/download/)
  with the broker and command-line clients. The launcher installs the Python
  packages from [dashboard/requirements.txt](dashboard/requirements.txt).
- To build a UF2 on Windows, install [Git for Windows](https://git-scm.com/download/win),
  extract [w64devkit](https://github.com/skeeto/w64devkit/releases), and
  install the [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
  for Windows with the `arm-none-eabi` target. The build uses w64devkit's
  GNU Make, shell utilities, and host `g++` to build the UF2 converter.
  Arm GCC compiles the Pico W firmware. This MQTT profile was built with
  Arm GCC 15.2.1 and [Pico SDK 2.3.1](https://github.com/raspberrypi/pico-sdk/tree/2.3.1).

**Terminals**

| Terminal | Shell | When to use it |
|---|---|---|
| Build | Windows PowerShell | Build or rebuild the UF2. Close it after the build. |
| Dashboard | Windows PowerShell | Start Mosquitto and the dashboard with one launcher. Keep it open during testing. |
| MQTT monitor | A second Windows PowerShell | Optional. Watch the probe, acknowledgement, heartbeat and sensor packets. |

Only the dashboard terminal needs to stay open for a normal run. If you
start the broker manually, it needs its own terminal. See the
[manual dashboard guide](dashboard/README.md).

#### 1. Connect to the hotspot and find the broker address

Enable the phone hotspot and connect the Windows laptop to it. Use the same
hotspot SSID for the Pico W in the next step. In Windows PowerShell, run
`ipconfig` and find the **IPv4 Address** of the adapter on the network the
Pico will join. If Windows hosts the hotspot, use the IPv4 address of its
hotspot adapter. This is the address of the laptop running Mosquitto.

The examples below use `172.20.10.2`. Substitute your own laptop address
everywhere it appears. If the hotspot assigns the laptop a different address
later, update the broker IP in the header, rebuild and reflash, and pass the
new address to the launcher.

#### 2. Configure and build the firmware

In Windows PowerShell, add the tool folders to the current session's
`PATH`. Replace the example paths with your installed `bin` folders.
Repeat this in each new PowerShell session used for building.

```powershell
$armBin = 'C:\Tools\arm-gnu-toolchain\bin'
$w64Bin = 'C:\Tools\w64devkit\bin'
$env:Path = "$armBin;$w64Bin;$env:Path"
git --version
make --version
arm-none-eabi-gcc --version
g++ --version
```

Clone the integration branch from a Windows folder where you want the
checkout. If you already have the repo, switch to
`Full-Implemented-wif-microt` and pull the latest changes instead.

```powershell
git clone --branch Full-Implemented-wif-microt https://github.com/hariszuf/INF2004-PicoCar.git
Set-Location INF2004-PicoCar
```

From the repo root, create any missing local config files.

```powershell
if (!(Test-Path config/wifi_credentials.h)) {
    Copy-Item config/wifi_credentials.example.h config/wifi_credentials.h
}
if (!(Test-Path config/mqtt_config.h)) {
    Copy-Item config/mqtt_config.example.h config/mqtt_config.h
}
```

| Local file | Set these values |
|---|---|
| `config/wifi_credentials.h` | Set `UTK_WIFI_CREDENTIALS_SET` to `1`. Set `UTK_WIFI_SSID` and `UTK_WIFI_PASSWORD` to your hotspot credentials. |
| `config/mqtt_config.h` | Set `MQTT_BROKER_IP` to your laptop's hotspot-facing address. Keep `MQTT_BROKER_PORT` at `1883U` and the default vehicle ID and topics for a single car. |

Git ignores both local headers. Keep your credentials local. Rebuild and
reflash after changing either header.

Use a Pico SDK folder with no spaces in its path. If you already have SDK
2.3.1, set `$sdkPath` to its root and skip the clone. Use forward slashes in
the SDK path so GNU Make can use it in build dependencies.

```powershell
$sdkPath = 'C:/Tools/pico-sdk-2.3.1'
New-Item -ItemType Directory -Force (Split-Path $sdkPath) | Out-Null
git clone --branch 2.3.1 https://github.com/raspberrypi/pico-sdk.git $sdkPath
git -C $sdkPath submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
$env:PICO_SDK_PATH = $sdkPath
```

From the repository root in the same build terminal:

```powershell
make -C build_make -f picocar_integration.mk TELEMETRY_MQTT=1 HEADLESS=1 "PICO_SDK_PATH=$env:PICO_SDK_PATH" -j8
```

With motor power off, hold BOOTSEL while connecting USB and copy the
resulting `build_make/picocar_integration_headless_mqtt.uf2` to the
`RPI-RP2` drive in Windows Explorer. The headless build is intended for
power-bank operation. See the [driving firmware guide](app_program/buddy5/vehicle/README.md)
for other build profiles.

#### 3. Start the broker and dashboard

In Windows PowerShell, open the repository root from the same checkout and
run the launcher with your laptop's address.

```powershell
.\Start-PicoCarDashboard.bat -VehicleBrokerAddress 172.20.10.2
```

The launcher creates `dashboard/.venv`, installs the pinned dependencies,
starts Mosquitto or reuses an existing listener, and starts the dashboard.
Keep the terminal open. If the launcher starts Mosquitto, it also stops it
when the launcher exits. Allow inbound TCP `1883` on the trusted test network
if Windows Firewall blocks the broker. The launcher permits anonymous MQTT
access for bench testing on that network.

Open [the dashboard](http://127.0.0.1:8000/). For a laptop-only UI check,
omit `-VehicleBrokerAddress`. Sensor cards need a publishing Pico W to
display live data.

If a broker is already running, you can point the dashboard directly at
that broker. This is also useful when testing the startup gate with the
broker and dashboard in separate terminals. Create the dashboard environment
if the launcher has never run in this checkout:

```powershell
python -m venv dashboard/.venv
& .\dashboard\.venv\Scripts\python.exe -m pip install -r dashboard/requirements.txt
```

Run the dashboard in its own PowerShell terminal, using the address of the
existing broker:

```powershell
$env:MQTT_BROKER_HOST = '172.20.10.2'
$env:MQTT_BROKER_PORT = '1883'
& .\dashboard\.venv\Scripts\python.exe -m uvicorn app:app --app-dir dashboard --host 127.0.0.1 --port 8000
```

#### 4. Verify the startup gate and telemetry

Keep motor power off for the dashboard check. If you want to observe the
startup exchange, start this subscriber before powering the Pico W:

```powershell
& "$env:ProgramFiles\mosquitto\mosquitto_sub.exe" -h 172.20.10.2 -p 1883 -v -t 'picocar/vehicle1/#'
```

Look for a `startup_probe=<token>` on `/telemetry` and a matching
`dashboard_ready=<token>` on `/command`. A fast exchange can finish before
the subscriber is attached. After acknowledgement, the dashboard should
show a connected broker, a fresh heartbeat, and sensor values that update
over time. Use **Diagnostics → Export log (.txt)** to save browser-observed
payloads.

To check the gate itself, keep Mosquitto running separately, leave the
dashboard backend stopped, and boot the Pico. The subscriber should see
repeated probes. Start the dashboard backend and check for the
acknowledgement and regular sensor reports. For a driving test, put the car
on the intended course before applying motor power because it can start
automatically after the startup checks pass.

| If you see this | Check this |
|---|---|
| No packets in the subscriber | Confirm the hotspot credentials, laptop IPv4 address, broker listener on port `1883`, and Windows Firewall access on the test network. |
| Repeated startup probes and no acknowledgement | Stop any older dashboard server. Start the dashboard from the same checkout and point it at the same broker as the subscriber. If using a broker bridge, it must carry telemetry toward the dashboard and the command topic back toward the Pico. |
| Acknowledgement and sensor data, but no movement | The MQTT gate has passed. Check motor power and the controller's existing readiness checks, including its forward range reading. |
| Port `8000` already in use | Stop the old dashboard server or pass `-DashboardPort 8001` to the launcher and open that port. |

## Remaining work

- Driving tests covered heartbeat cadence, Wi-Fi recovery, and telemetry
  delivery. Results were inconsistent across runs. Diagnose the gaps and
  measure queue drops during bursts and reconnects.
- Verify the expanded encoder, IMU diagnostic and saved-event packets on
  hardware and compare them with the dashboard.
- Agree on controller acknowledgement and safety feedback before wiring
  remote START/STOP to vehicle execution.
- Define telemetry with the subsystem owners for decoded barcode results and
  raw ultrasonic distance. The dashboard marks these as Future implementation.

## Troubleshooting

| Symptom | Check |
|---|---|
| Compiler, `make`, or shell utility not found | Check `PATH` inside the shell used for the build; the Arm compiler and host `g++` are separate tools. |
| `TinyUSB not found` | Set `PICO_SDK_PATH` explicitly and initialise the SDK's `lib/tinyusb` submodule. |
| CYW43 or lwIP not found | Initialise `lib/cyw43-driver` and `lib/lwip` in that SDK checkout. |
| Wi-Fi placeholder configuration error | Fill in the local credentials header and set `UTK_WIFI_CREDENTIALS_SET=1`. |
| No RPI-RP2 drive | Reconnect while holding BOOTSEL and check that the USB cable supports data. |
| No serial output | Check that you flashed the intended console profile and selected its current port after reboot. |

## Project documentation and references

- [Team architecture and subsystem interfaces](TEAM.md)
- [Testing workflow and evidence](testing/README.md)
- [Inherited setup and networking guide](docs/GETTING_STARTED.md) — upstream examples use its repository name and demo; use the PicoCar clone and application steps above.
- [RP2040 port status and hardware constraints](docs/PORT_RP2040.md)
- **Professor's course template:** [sirfonzie/INF2004_ProjectTemplate](https://github.com/sirfonzie/INF2004_ProjectTemplate). Reference for course setup and subsystem organisation; its build options differ as documented above.
- **Underlying RTOS port:** [sirfonzie/mtk3smp-rp2040](https://github.com/sirfonzie/mtk3smp-rp2040). Refer to its [setup guide](https://github.com/sirfonzie/mtk3smp-rp2040/blob/main/docs/GETTING_STARTED.md) for upstream build details and its [port notes](https://github.com/sirfonzie/mtk3smp-rp2040/blob/main/docs/PORT_RP2040.md) for validation scope.
- [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk/tree/2.2.0)

See [LICENSE](LICENSE) and [LICENSES](LICENSES/) for licensing information.

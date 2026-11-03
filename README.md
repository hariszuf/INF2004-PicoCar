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

The makefiles also use Unix utilities such as `sh`, `find`, `xargs`, `rm`,
and `mkdir`. On Windows, follow the course template's **WSL (Ubuntu)** setup
and run the build commands inside its Ubuntu terminal. PowerShell alone does
not supply the required build environment. The commands below use Bash;
keep the SDK and compilers in the same environment as the build.

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
| `picocar/vehicle1/telemetry` | Pico W → broker | Named sensor fields, sequence and flags |
| `picocar/vehicle1/command` | Dashboard → broker → Pico W | START or STOP |

### First run on Windows

Install [Python 3.10 or newer](https://www.python.org/downloads/windows/)
and [Eclipse Mosquitto](https://mosquitto.org/download/). Connect the laptop
and Pico W to the same trusted Wi-Fi network. Use `ipconfig` to find the
laptop's Wi-Fi IPv4 address. From the repository root, run the launcher
with that address.

```powershell
.\Start-PicoCarDashboard.ps1 -VehicleBrokerAddress 172.20.10.2
```

Replace the example address with your laptop's address. The launcher creates
`dashboard/.venv`, installs the dashboard dependencies on first run or when
they change, starts Mosquitto or reuses the existing listener, and starts
FastAPI. A broker started by the script stops when the script exits. The
broker allows anonymous access on the supplied address for bench testing.
For a laptop-only test, omit `-VehicleBrokerAddress`.

Open [the dashboard](http://127.0.0.1:8000/). Keep the terminal open during
the test. If Windows prompts for firewall access, allow it on the trusted
test network so the Pico W can reach port `1883`.

### Configure and flash the Pico W

If the car already has the MQTT build for this Wi-Fi network and broker,
boot it and check the heartbeat. For a new configuration, copy the example
headers once from the repository root.

```powershell
Copy-Item config/wifi_credentials.example.h config/wifi_credentials.h
Copy-Item config/mqtt_config.example.h config/mqtt_config.h
```

Set the SSID and password in `wifi_credentials.h` and set
`UTK_WIFI_CREDENTIALS_SET` to `1`. Set `MQTT_BROKER_IP` to the same laptop
IPv4 address passed to the launcher. Keep `MQTT_BROKER_PORT` at `1883` for
the default setup. Git ignores the local headers.

Follow [Setup](#setup) for the Arm toolchain, GNU Make and Pico SDK, then
build from the repository root in the configured Bash environment.

```bash
cd build_make
make -f picocar_integration.mk TELEMETRY_MQTT=1 HEADLESS=1 PICO_SDK_PATH="$PICO_SDK_PATH" -j8
```

With motor power off, hold BOOTSEL while connecting USB and copy
`build_make/picocar_integration_headless_mqtt.uf2` to `RPI-RP2`. This build
starts the driving controller automatically. See the
[driving firmware guide](app_program/buddy5/vehicle/README.md) for build
variants and packet fields.

### Verify telemetry

The dashboard should show **MQTT broker: Connected** and a fresh vehicle
heartbeat after boot. Sensor views show the latest readings and their ages.
In **Diagnostics**, use **Export log (.txt)** to save payloads observed by
the browser. To watch the MQTT stream directly, run this in another terminal.

```powershell
& 'C:\Program Files\mosquitto\mosquitto_sub.exe' -h 127.0.0.1 -p 1883 -v -t 'picocar/vehicle1/status' -t 'picocar/vehicle1/telemetry'
```

For manual broker and dashboard startup, see the
[dashboard guide](dashboard/README.md).

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

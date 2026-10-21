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
| Application | `subsystem1/`–`subsystem5/` examples | `buddy1/`–`buddy5/`; only Buddy 1 is currently started by `app_main.c`. |
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

The current `app_program/app_main.c` initialises and starts Buddy 1, then
prints `[APP] All registered buddies started`. Other buddy startup calls
have not yet been integrated. The producer/consumer demo described in the
inherited upstream guide is not the current PicoCar application.

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


## Buddy 1 — MQTT Communication

Buddy 1 provides external Wi-Fi/MQTT communication for the
PicoCar using lwIP and µT-Kernel 3.0.

Buddy 1's MQTT development profile has also been built and tested on Windows using Pico SDK 2.3.1 and Arm GNU Toolchain 15.2.1.

### Implemented functionality

- Wi-Fi association, DHCP and DNS
- MQTT connection to a Mosquitto broker
- Periodic MQTT status heartbeat (5 seconds)
- MQTT START/STOP command reception
- Non-blocking command submission through µT-Kernel IPC
- Telemetry forwarding from µT-Kernel message buffers to MQTT
- Separate networking and application telemetry paths

Current integration has been tested using a Raspberry Pi Pico W
and Mosquitto running on a Windows laptop.

Network reconnection, message loss under load, full-team
integration and compact binary MQTT payloads require further testing
or implementation.

### MQTT configuration

Copy the example configuration from the repository root.

Windows PowerShell:

```powershell
Copy-Item .\config\mqtt_config.example.h .\config\mqtt_config.h
```

Linux / WSL:

```bash
cp config/mqtt_config.example.h config/mqtt_config.h
```

Edit `config/mqtt_config.h` and set `MQTT_BROKER_IP` to the
IPv4 address of the computer running Mosquitto.

The local `mqtt_config.h` is excluded from Git. Do not commit
Wi-Fi passwords or other secrets.

### MQTT topics

| Topic | Direction | Purpose |
|---|---|---|
| `picocar/vehicle1/status` | Pico W → broker | MQTT connection heartbeat |
| `picocar/vehicle1/telemetry` | Pico W → broker | µT-Kernel telemetry messages |
| `picocar/vehicle1/command` | Broker → Pico W | Remote commands |

### Build

The verified Buddy 1 MQTT development profile uses:

- `SMP=0`
- `CONSOLE=usb_cdc`
- `WIFI=cyw43`
- `WIFI_JOIN=1`
- `WIFI_NETIF=1`
- `WIFI_DHCP=1`
- `WIFI_DNS=1`
- `WIFI_MQTT=1`
- `BUDDY1_ONLY=1`

From `build_make/`:

```bash
make BUDDY1_ONLY=1 SMP=0 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_DNS=1 WIFI_MQTT=1 PICO_SDK_PATH=/path/to/pico-sdk -j8
```

Replace `/path/to/pico-sdk` with the installed SDK path.

The generated firmware is:

`mtk3pico_smp0_usb_cdc_wifi_mqtt.uf2`

The isolated Buddy 1 build does not validate compilation or
runtime integration of all five subsystems.

### Local MQTT broker

For development, Mosquitto can listen on TCP port 1883.

Example development configuration:

```text
listener 1883 0.0.0.0
allow_anonymous true
```

This configuration is intended only for a trusted, isolated
development network. It does not provide authentication or
encrypted communication.

The Pico W and the broker computer must have network
connectivity to one another.

### Windows testing

Replace `BROKER_IP` with the actual broker IPv4 address.

Subscribe to Pico W status:

```powershell
mosquitto_sub -h BROKER_IP -p 1883 -t "picocar/vehicle1/status" -v
```

Subscribe to telemetry:

```powershell
mosquitto_sub -h BROKER_IP -p 1883 -t "picocar/vehicle1/telemetry" -v
```

Publish START:

```powershell
mosquitto_pub -h BROKER_IP -p 1883 -t "picocar/vehicle1/command" -m "START"
```

Publish STOP:

```powershell
mosquitto_pub -h BROKER_IP -p 1883 -t "picocar/vehicle1/command" -m "STOP"
```

If Mosquitto is not available on PATH, use its full executable
path, such as `C:\Program Files\mosquitto\mosquitto_pub.exe`.

The Pico W serial monitor should display the received command
and the corresponding µT-Kernel command consumer output.

### Verified development results

- Wi-Fi association, DHCP and DNS successful
- MQTT broker connection and subscription successful
- MQTT heartbeat received by the Windows subscriber
- START and STOP received through µT-Kernel command IPC
- µT-Kernel heartbeat telemetry received on the MQTT telemetry topic

These results establish communication and IPC functionality only.
They do not establish successful motor operation, robot navigation,
automatic recovery or full-system integration.

### Remaining work

- Integrate telemetry from all five subsystems
- Connect remote commands to the Vehicle Controller
- Test communication loss and reconnection
- Measure telemetry queue overflow and message loss
- Implement the compact binary payload change requirement
- Complete full-system integration and hardware validation
```

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

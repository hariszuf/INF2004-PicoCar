# INF2004 PicoCar

Autonomous robotic car project running on Raspberry Pi Pico W with
µT-Kernel 3.0.

## Current RTOS configuration

- Board: Raspberry Pi Pico W / Cytron Robo-Pico
- MCU: RP2040
- RTOS: µT-Kernel 3.0
- Mode: Single-core (`SMP=0`)
- Debug console: USB CDC
- Pico SDK: 2.2.0

## Setup

### 1. Install the build tools

You need Git, GNU Make, the Arm GNU toolchain (`arm-none-eabi-gcc` and
`arm-none-eabi-size`), and a host C++ compiler (`g++`). The upstream toolchain
baseline is GCC **13.2.1**; this project uses **Pico SDK 2.2.0** for USB CDC
and Wi-Fi support.

The makefiles also use Unix utilities such as `sh`, `find`, `xargs`, `rm`,
and `mkdir`. On Windows, use a WSL Linux environment or an MSYS2 environment
with these tools and both compilers available on `PATH`. PowerShell alone
does not supply the required build environment. The commands below use Bash;
keep the SDK and compilers in the same environment as the build.

For Ubuntu/Debian, including Ubuntu under WSL:

```bash
sudo apt update
sudo apt install git gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
```

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

Set `PICO_SDK_PATH` in each new terminal, or add the export to your Bash
profile. It must point to the SDK root, not its `src` or `lib` directory.
For MSYS2, use a shell-compatible path such as `C:/pico/pico-sdk`; for WSL,
use the Linux path. Prefer paths without spaces because the makefiles use
SDK paths in dependency lists.

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

### 5. Flash the Pico W

1. Hold **BOOTSEL** while connecting the Pico W with a USB data cable.
2. Release BOOTSEL when the **RPI-RP2** drive appears.
3. Copy `mtk3pico_smp0_usb_cdc.uf2` from `build_make/` onto that drive.
4. The board reboots automatically. Open its USB serial port in a serial monitor.

On Windows, select the new COM port shown in Device Manager; on Linux, it
usually appears as `/dev/ttyACM0`. For a WSL build, use Windows Explorer to
copy the UF2 to the board and a Windows serial monitor to view the console.
For the UART profile, use a 3.3 V USB-serial adapter: Pico GP0 (TX) to adapter
RX, GP1 (RX) to adapter TX, and common GND, at **115200 baud, 8N1**.

The current `app_program/app_main.c` initialises and starts Buddy 1, then
prints `[APP] All registered buddies started`. Other buddy startup calls
have not yet been integrated. The producer/consumer demo described in the
inherited upstream guide is not the current PicoCar application.

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
- **Professor's repository / upstream RTOS port:** [sirfonzie/mtk3smp-rp2040](https://github.com/sirfonzie/mtk3smp-rp2040). PicoCar builds on this µT-Kernel 3.0 RP2040 port; refer to its [setup guide](https://github.com/sirfonzie/mtk3smp-rp2040/blob/main/docs/GETTING_STARTED.md) for upstream build details and its [port notes](https://github.com/sirfonzie/mtk3smp-rp2040/blob/main/docs/PORT_RP2040.md) for validation scope.
- [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk/tree/2.2.0)

See [LICENSE](LICENSE) and [LICENSES](LICENSES/) for licensing information.

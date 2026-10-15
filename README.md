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

## Build

From `build_make/`:

```powershell
make SMP=0 CONSOLE=usb_cdc PICO_SDK_PATH="C:\path\to\pico-sdk" -j8
# Buddy4 GY-511 C + VS Code + MQTT (Raspberry Pi Pico W)

This is an **independent project**. It does not use µT-Kernel. Build with Pico SDK v2.x using the VS Code Raspberry Pi Pico extension. It assumes that I2C `0x1D` is an ST LSM303D. The `WHO_AM_I` register at `0x0F` **must return `0x49`** before the program will read accelerometer data. If it doesn't, please share the serial output rather than changing arbitrary registers.

## Wiring

| GY-511 | Pico W |
| --- | --- |
| SDA | GP16, physical pin 21 |
| SCL | GP17, physical pin 22 |
| GND | GND |
| 3.3V on module | Pico 3V3 OUT (physical pin 36) |

Check the actual breakout's power labels. Use 3.3 V logic; do not add 5 V to Pico I/O. Leave the car's motors disconnected or its wheels lifted during initial testing.

## Stage 1: IMU only, no MQTT yet

1. Extract the zip and open **the extracted folder containing `CMakeLists.txt`** in VS Code.
2. In Raspberry Pi Pico extension select **Pico W (`pico_w`)**, not `pico` or `pico2_w`.
3. Choose Configure CMake and Compile Project, with MQTT disabled (default).
4. Upload the freshly generated `build/buddy4.uf2`. Open the correct USB Serial port at 115200.
5. Keep the car flat and still during the 2-second calibration. Tilt forward and backward by hand.

Expected output (example):

```
Buddy4 / GY-511: Pico SDK sensor test
SDA=GP16, SCL=GP17, I2C0, address=0x1D
LSM303D identified. Keep car LEVEL and STILL for 2 seconds.
Calibrated: pitch baseline 1.00 deg.
X=+0.01 Y=+0.01 Z=+1.00 g | pitch=+0.2 deg | FLAT
...
```

If the module +Y axis points forward, edit `FORWARD_AXIS_IS_X` to `0` in `terrain.c`. If the slope labels are reversed, set `DIRECTION_SIGN` to `-1.0f`. `POSSIBLE_HUMP` reports a brief acceleration spike: it is **not** a confirmed hump; handling, braking or vibrations can also trigger it.

## Stage 2: Enable MQTT only after Stage 1 passes

**DO NOT store real passwords in a public repository.** Create a local `local_config.h` by copying `local_config.example.h`, then put in the correct Wi-Fi SSID/password, MQTT broker's IPv4 address and topic. The example is set up for **plain MQTT on port 1883, intended only for a trusted local network**, not an Internet-facing broker. TLS is not configured.

To enable MQTT, configure the build using Pico extension's CMake settings or run in a Pico SDK-enabled terminal:

```powershell
cmake -S . -B build -G Ninja -DPICO_BOARD=pico_w -DBUDDY4_ENABLE_MQTT=ON
cmake --build build -v
```

Use a new empty build folder or delete `build` if it was configured for a different board. If turning MQTT OFF again, reconfigure with `-DBUDDY4_ENABLE_MQTT=OFF`.

MQTT publishes to `inf2004/buddy4/terrain` by default, roughly once per second, with JSON such as:

```json
{"terrain":"UPHILL","pitch_deg":13.2,"ax_g":0.22,"ay_g":0.01,"az_g":0.98}
```

To view on your computer with a local Mosquitto broker:

```powershell
mosquitto_sub -h YOUR_BROKER_IP -p 1883 -t "inf2004/buddy4/terrain" -v
```

The Pico W and MQTT broker must be able to reach each other over Wi-Fi. If the broker requires TLS, client certs or a cloud-specific protocol, this starter code needs a different connection setup. The on-board Wi-Fi client assumes WPA2-personal for now.

## If linking or UF2 generation gets stuck again

Previous screenshots showed a newer `buddy4.elf` but an older `buddy4.uf2`. That suggests you should check the **post-link** commands rather than changing the IMU logic. Stop all old builds, close duplicate Ninja tasks, and run the real build with `-v` to see which sub-command is running.

If the ELF timestamp is newer and ELF is valid, you can manually create a UF2 with the installed SDK picotool:

```powershell
& "$env:USERPROFILE\.pico-sdk\picotool\2.3.1\picotool\picotool.exe" uf2 convert ".\build\buddy4.elf" ".\build\buddy4_manual.uf2" --family rp2040 --platform rp2040
```

Only flash `buddy4_manual.uf2` if conversion completed and it has a **current timestamp**. Repeating old `buddy4.uf2` uploads will not test current code.

## Validated vs not validated

The standalone terrain math is covered by host-side tests. Full Pico SDK linking and live Pico W / GY-511 / Wi-Fi / MQTT behavior cannot be tested in this environment; use the steps above and share error output if needed. For the sensor variant, verify `WHO_AM_I = 0x49`.

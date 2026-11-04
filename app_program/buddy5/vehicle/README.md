# Full-test vehicle integration

`main.c` owns the autonomous vehicle controller and its μT-Kernel task. It
receives Buddy 5 obstacle requests, calls Buddy 2 motion primitives through
`vehicle_nav.c`, and starts or resumes Buddy 3 line following. The GPIO interrupt
dispatcher and Pico SDK compatibility headers live here because they are
shared by the encoder, ultrasonic echo, motor, and IR drivers.
The driving profile starts the native I²C driver for Buddy 4 on GP16/GP17.

The default driving profile uses Buddy 1's local telemetry queue for USB
diagnostics. To add Wi-Fi and MQTT, create the ignored local
`config/wifi_credentials.h` and `config/mqtt_config.h` from their examples.
Set the Wi-Fi credentials, enable the credentials guard, and set the broker
IP. From `build_make/`, using a Pico SDK with TinyUSB, CYW43 and lwIP initialized:

```sh
make -f picocar_integration.mk TELEMETRY_MQTT=1 PICO_SDK_PATH="$PICO_SDK_PATH" -j8
```

Flash `picocar_integration_mqtt.uf2`. With `HEADLESS=1`, the output is
`picocar_integration_headless_mqtt.uf2`. Both are driving images with automatic
start. Keep motor power off for the first MQTT bench test.

```sh
mosquitto_sub -h BROKER_IP -v -t 'picocar/vehicle1/status' -t 'picocar/vehicle1/telemetry'
```

Expect the Buddy 1 heartbeat plus these sensor reports about once per second.
Sequences group reports within a source. The telemetry payload contains
`src`, `type`, `seq`, two named values, and `flags`. For example:

```text
src=4 type=7 seq=16 pitch_cdeg=501 roll_cdeg=96 flags=1
```

| Source / type | Value 1 | Value 2 |
| --- | --- | --- |
| 2 / 3 | Signed left wheel speed (mm/s) | Signed right wheel speed (mm/s) |
| 2 / 4 | Signed left encoder ticks | Signed right encoder ticks |
| 2 / 10 | Signed distance since odometry reset (mm) | Motion fault enum |
| 3 / 5 | Raw IR bitmask, including the barcode sensor bit | Line-follow state enum |
| 4 / 7 | Pitch (centidegrees) | Roll (centidegrees) |
| 4 / 11 | IMU sample age (ms), or -1 if unavailable | IMU errors |
| 4 / 12 | Clipped samples | Unsuitable tilt samples |
| 4 / 13 | Terrain state | Side state |
| 4 / 14 | Estimated motion state | Encoder yaw rate (centidegrees/s) |
| 4 / 15 | Wheel-height difference (mm) | Measurement quality |
| 4 / 16 | Saved stable peak height (mm) | Saved median peak height (mm) |
| 4 / 17 | Saved event side | Event end uptime (ms) |
| 4 / 18 | Possible-impact count | Last possible-impact uptime (ms) |
| 5 / 50 | Obstacle state | Obstacle fault |
| 5 / 51 | Width (mm) | Upper depth estimate (mm) |
| 5 / 52 | Bypass side | Signed bypass offset (mm) |
| 5 / 53 | Lower depth estimate (mm) | Upper depth estimate (mm) |
| 5 / 54 | Front clearance (mm), or -1 for unavailable/out of range | Commanded servo angle (degrees) |
| 5 / 56 | Obstacle flags mask | Reserved |
| 6 / 57 | Controller line reacquisitions since boot | Reserved |

IMU flags bit 0 marks usable tilt for types 7, 13 and 15, fresh acquisition
for type 11, usable motion for type 14, a stable peak for type 16, a saved
event for type 17, and current impact detector validity for type 18. Bit 1
marks usable yaw for type 14 and a saved median peak for type 16.
Obstacle flags bits 0 and 1
mark width and depth validity, bit 4 identifies the driving profile, and
envelope bit 5 marks the driver's out-of-range clearance sentinel. The raw IR
bitmask does not contain a decoded barcode. MQTT START/STOP still reaches
Buddy 1's logging consumer. The driving controller uses its existing controls.


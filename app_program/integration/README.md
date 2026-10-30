# PicoCar μT-Kernel integration

This is the single integrated driving application on this branch. Build it
with `build_make/picocar_integration.mk`; only `shared/main.c` defines the
μT-Kernel entry point. The build references each buddy's original source
where possible, so there are no second copies to maintain.

| Folder | Owner and status |
| --- | --- |
| `buddy1/` | Local telemetry used for USB diagnostics; Wi-Fi/MQTT disabled |
| `buddy2/` | Motion controller and encoder sources used directly |
| `buddy3/` | IR line following sources used directly |
| `buddy4/` | μT-Kernel IMU integration pending; separate prototype retained |
| `buddy5/` | Ultrasonic, servo, scan and profile sources owned here |
| `shared/` | Vehicle navigation, task wiring and hardware compatibility |

`app_program/integration_test/` is a separate Buddy 2/3/4 Pico SDK/CMake
prototype with its own tests. It is not part of this μT-Kernel build.


# PicoCar μT-Kernel integration

This is the single integrated driving application on this branch. Build it
with `build_make/picocar_integration.mk`; only `buddy5/vehicle/main.c` defines the
μT-Kernel entry point. The build references each buddy's original source
where possible, so there are no second copies to maintain.

| Folder | Owner and status |
| --- | --- |
| `buddy1/` | Local telemetry used for USB diagnostics; Wi-Fi/MQTT disabled |
| `buddy2/` | Motion controller and encoder sources used directly |
| `buddy3/` | IR line following sources used directly |
| `buddy4/` | IMU sources, `rtos/` task, and retained trials |
| `buddy5/` | Ultrasonic, servo, scan, profile, and `vehicle/` coordinator |

`app_program/buddy4/sdk_integration_test/` is a separate Buddy 2/3/4 Pico SDK/CMake
prototype with its own tests. It is not part of this μT-Kernel build.


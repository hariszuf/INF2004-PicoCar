# PicoCar μT-Kernel integration

This is the single integrated driving application on this branch. Build it
with `build_make/picocar_integration.mk`. The μT-Kernel entry point in
`app_main.c` initialises resources, starts tasks, and sleeps. The Vehicle
Controller owns runtime driving decisions and motor updates.

| Folder | Owner and status |
| --- | --- |
| `buddy1/` | Telemetry and command tasks for the optional MQTT profile |
| `buddy2/` | Motion controller and encoder sources used directly |
| `buddy3/` | IR line following sources used directly |
| `buddy4/` | IMU sources, `rtos/` task, and retained trials |
| `buddy5/` | Ultrasonic, servo, integrated scanner, and navigation/platform adapters |
| `vehicle/` | Vehicle Controller task, automatic start, commands, and scanner feedback |

`app_program/buddy4/sdk_integration_test/` is a separate Buddy 2/3/4 Pico SDK/CMake
prototype with its own tests. It is not part of this μT-Kernel build.


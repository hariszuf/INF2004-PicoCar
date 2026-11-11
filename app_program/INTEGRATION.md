# PicoCar μT-Kernel integration

This is the single integrated driving application on this branch. Build it
with `build_make/picocar_integration.mk`. The entry point in `app_main.c`
initialises resources and starts tasks. The Vehicle Controller coordinates driving
and publishes execution status.

| Folder | Owner and status |
| --- | --- |
| `buddy1/` | Telemetry and MQTT commands |
| `buddy2/` | Motion controller and encoder sources used directly |
| `buddy3/` | IR line following sources used directly |
| `buddy4/` | IMU sources, `rtos/` task, and retained trials |
| `buddy5/` | Ultrasonic, servo, integrated scanner, and navigation adapters |
| `vehicle/` | Vehicle Controller task and execution state |

`app_program/buddy4/sdk_integration_test/` is a separate Buddy 2/3/4 Pico SDK/CMake
prototype with its own tests. It is not part of this μT-Kernel build.


Power-on starts a run after the existing readiness gates. A STOP or recoverable
stop holds the car stationary until dashboard START or USB R requests another
run. The controller waits for stopped wheels and fresh forward data before
retrying. A latched motor stall or motor reset failure requires a power cycle.
MQTT START received before dashboard acknowledgement and the 10-second
connection delay is rejected. MQTT STOP depends on delivery to the vehicle. Headless builds without MQTT
use the default junction action because they do not start a barcode task.

The dashboard reads execution status from source 5, type 60. Its first value
packs state in bits 0–3 and the last START result in bits 4–7. The second value
holds the stop reason in bits 0–7. Old firmware can also report automatic retry
counts in the high bits. START result 4 means the MQTT startup gate was closed.
Stop reason 13 means a junction stopped without a barcode action. Execution
reports are independent of command publication and do not identify a specific
browser request.

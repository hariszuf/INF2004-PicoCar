# Buddy4 build and code review

## Standalone IMU logic test (not Pico UF2 firmware)

From the `buddy4` folder on a computer with CMake and a C compiler:

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

For the robot firmware, `buddy4.c` depends on µT-Kernel headers `<tk/tkernel.h>` and `<tm/tmonitor.h>`, plus `../../device/include/dev_i2c.h`. Build from the complete INF2004-PicoCar repository's firmware build system and link both `buddy4.c` and `imu_detect.c`. Importing `buddy4` alone with Raspberry Pi Pico extension is **not** sufficient.

## Review items to verify against complete repository and hardware

- `write_reg` uses `tk_swri_dev(..., MPU_ADDR, ...)` while `read_regs` uses `TDN_I2C_EXEC`. Confirm the µT-Kernel I2C driver contract: direct register writes may require an I2C EXEC transaction too.
- Confirm the I2C device name `iica` and pin mux match the full BSP configuration and hardware wiring.
- Validate MPU6050 identity (WHO_AM_I == 0x68), AD0 wiring and sensor orientation.
- `buddy4.c` stores only the latest 20 ms sample. A separate consumer could miss `hump_event`.
- The code's slope/hump thresholds are unverified on physical hardware.
- If a read fails once or twice, the existing status still looks valid; consider explicitly reporting transient failures.
- On startup failure, mutex/device resources are not cleaned up.
- `buddy4_start()` needs to be called after `buddy4_init()`, and the main firmware must call both.
- The README's older host test command contains the wrong path for the ZIP layout. The CMake test above works with the included source tree.

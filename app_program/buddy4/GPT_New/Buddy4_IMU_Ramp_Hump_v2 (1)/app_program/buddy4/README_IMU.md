# Buddy 4: IMU ramp and hump detection

This package replaces the initial Buddy 4 implementation. It is **MPU6050-specific** (I2C 7-bit address 0x68, AD0 grounded), and has not been tested on the physical PicoCar. Do **not** commit a claim of working hardware detection before testing.

## Code
- `buddy4.c / buddy4.h`: MPU6050 I2C driver and periodic µT-Kernel task (50 Hz), status mutex, API.
- `imu_detect.c / imu_detect.h`: independent pitch estimation and terrain state classifier.
- `tests/test_imu_detect.c`: deterministic desktop smoke test (`cc -std=c11 -Wall -Wextra -Werror app_program/buddy4/imu_detect.c tests/test_imu_detect.c -o /tmp/imu_test && /tmp/imu_test`).

## Sensor wiring must be resolved first
Current repo's `kernel/sysdepend/pico_rp2040/hw_setting.c` configures I2C0 on **GP8 SDA / GP9 SCL**. Those pins were previously reserved for the left motor. Do not attach a sensor to GP16/17 without changing the BSP pin mux first. GP16/17 is an alternative for I2C0, **but it needs agreement with the team and a BSP edit**. Sensor uses 3.3V logic. Check breakout voltage tolerance and I2C pull-ups. MPU6050 part number and board mounting orientation still need physical confirmation.

## How to integrate (after pinout and sensor verification)
1. Place all four `.c` / `.h` files in `app_program/buddy4/`. The Makefile recursively includes buddy source files, so no source list change is necessary.
2. In `app_program/app_main.c`, add `#include "buddy4/buddy4.h"`.
3. During Phase 1, after Buddy1 init succeeds: `error_code = buddy4_init(); if (error_code < E_OK) return 1;`.
4. During Phase 2, after Buddy1 start succeeds: `error_code = buddy4_start(); if (error_code < E_OK) return 1;`.
5. Keep robot stationary and level for ~2 seconds after startup. Readings are invalid until calibrated.
6. Motion control can call `buddy4_get_latest(&status)` and check `status.result.terrain`, `status.result.pitch_mdeg`, `status.result.hump_event`, and `status.sequence`.

**Important:** `hump_event` is a one-frame (~20ms) pulse in the latest snapshot. A slower consumer could miss it. For robust inter-task delivery, add a queue or monotonically increasing hump event counter after the team agrees on the IPC contract. Do not automatically slow motors merely from the prototype classifier. Integrate and validate with Buddy2 first.

## Detection logic
- 50 Hz samples; stationary 100-sample initial calibration to remove pitch offset and Y gyro bias.
- Fixed-point complementary filter: integrate gyro Y and correct slowly using accelerometer X/Z incline (assuming +X forward, +Z up).
- Uphill: approximately +8 degrees or more for 6 samples; downhill: -8 degrees or less for 6 samples; back to flat when within 5 degrees (hysteresis).
- Hump event: acceleration magnitude deviates by >=0.28g for 2 successive samples; cooldown 0.5 second.
- Print state changes, not raw readings each sample.
- After 3 I2C read failures, invalidate status and recalibrate once reads recover.

These are prototype thresholds, not values established by tests. A bump can trigger when the robot jolts; a smooth hump can fail to trigger. Ramp pitch alone cannot determine ramp length, and accelerometer spikes cannot accurately determine hump height without substantially more information. Calibration on the actual track is mandatory.

## Check mounting signs
With the robot nose lifted (stationary), `pitch_mdeg` should turn **positive**; nose lowered should turn **negative**. If incorrect, adjust the sensor axis in `accel_pitch_mdeg()` and `IMU_PITCH_GYRO_SIGN`/`IMU_PITCH_ACCEL_SIGN` macros in `imu_detect.h`. The assumed gyro Y sign can differ with chip mounting. Test physically before tuning thresholds.

## Checks performed
`tests/test_imu_detect.c` passes with the host compiler for level calibration, rising slope, level, descending slope, and a sharp hump. **Full Pico cross-compilation and hardware testing have not been performed** in this environment. MPU6050 identification, kernel device registration, I2C pin assignment and timing must be verified on-board.

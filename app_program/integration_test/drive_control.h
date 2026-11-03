#ifndef TEST_DRIVE_CONTROL_H
#define TEST_DRIVE_CONTROL_H
#include <stdbool.h>
#include <stdint.h>
#include "motion_control.h"
#include "buddy4_imu.h"
#define MANUAL_SPEED_CM_S 3.0f
/* Integration-only settings; leave Buddy 2 tuning untouched. */
#ifndef TURN_TEST_SPEED_CM_S
#define TURN_TEST_SPEED_CM_S 2.0f
#endif
#define TURN_TEST_LIMIT_US 1000000u
#define INVALID_LIMIT_US 500000u
typedef enum { DRIVE_MANUAL = 0, DRIVE_LINE = 1, DRIVE_TURN_LEFT = 2, DRIVE_TURN_RIGHT = 3 } drive_mode_t;
typedef struct {
    bool running, armed;
    drive_mode_t mode;
    const char *reason;
    motion_fault_t stop_fault;
    bool event_pending;
    uint64_t started_us;
    float selected_left_cm_s, selected_right_cm_s;
} drive_control_t;
void drive_init(drive_control_t *c);
void drive_stop(drive_control_t *c, const char *reason, motion_fault_t fault);
/* has_fresh/last_fresh refer to the last USABLE IMU measurement, not merely
 * a successful acquisition. The existing 500 ms missing-data watchdog remains. */
void drive_safety(drive_control_t *c, bool usb, bool has_fresh,
                  uint64_t now, uint64_t last_fresh);
void drive_command(drive_control_t *c, int key, bool usb, bool imu_healthy, uint64_t now_us);
void drive_line_update(drive_control_t *c);
/* SDK test-firmware permission, separate from acquisition freshness. A usable
 * sample is finite, non-clipped and tilt-suitable; zero terrain flags alone
 * never mean healthy. A fresh bad measurement latches stopped immediately.
 * Missing/I2C-error polls retain the existing bounded invalid-data watchdog.
 * Recovery cannot rearm; the operator must explicitly use R then G. */
bool drive_imu_usable(const buddy4_imu_t *imu);
bool drive_imu_poll(drive_control_t *c,const buddy4_imu_t *imu,buddy4_result_t result);
#endif

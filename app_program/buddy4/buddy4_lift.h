#ifndef BUDDY4_LIFT_H
#define BUDDY4_LIFT_H
#include <stdbool.h>
#include <stdint.h>

/* Robot lifted / overturned observer (replaces the old IMU collision observer).
 * Gravity-based: the angle between the measured gravity vector and the robot's
 * up axis (sensor Z times up_sign when the car is level), plus a sustained deviation of the
 * acceleration magnitude from 1 g (the push of being picked up).
 * Once triggered the state LATCHES until the robot has been level and still for
 * clear_us, so a brief hand shake cannot restart the motors. */
typedef enum {
    BUDDY4_LIFT_NONE = 0,
    BUDDY4_LIFT_TILTED = 1,     /* tilted beyond tilt_enter_deg (picked up / tipped) */
    BUDDY4_LIFT_OVERTURNED = 2, /* upside-down or on its side: tilt >= overturn_deg */
    BUDDY4_LIFT_MOTION = 3      /* sustained non-gravity acceleration while level */
} buddy4_lift_reason_t;

typedef struct {
    /* Sign of the sensor Z reading when the car sits level: +1 if sensor +Z points up,
     * -1 if the board is mounted with +Z pointing down. Measured on this car: level = -1. */
    float up_sign;
    float tilt_enter_deg, tilt_exit_deg, overturn_deg;
    float accel_enter_g, accel_exit_g;
    uint32_t tilt_confirm_us, accel_confirm_us, clear_us, max_gap_us, invalid_release_us;
} buddy4_lift_config_t;

typedef struct {
    bool valid;     /* A fresh, finite, non-clipped sample was seen within the last 250 ms. */
    bool lifted;    /* Latched lifted/overturned state: motors must stay stopped. */
    uint8_t reason; /* buddy4_lift_reason_t of the latest trigger (0 when none). */
    uint32_t count; /* Number of lift events since boot. */
    uint64_t since_us; /* Sample time of the latest trigger. */
    float tilt_deg;      /* Angle of gravity from the up axis, 0 = level, 180 = upside-down. */
    float deviation_g;   /* | ||a|| - 1 g |. */
} buddy4_lift_output_t;

typedef struct {
    buddy4_lift_config_t config;
    buddy4_lift_output_t output;
    bool configured, has_time;
    uint64_t last_time_us, tilt_since_us, accel_since_us, calm_since_us, invalid_since_us;
    bool tilt_pending, accel_pending, calm_pending, invalid_pending;
} buddy4_lift_t;

buddy4_lift_config_t buddy4_lift_default_config(void);
bool buddy4_lift_init(buddy4_lift_t *d, buddy4_lift_config_t config);
/* One call per poll. valid must mean NEW finite, non-clipped corrected data.
 * Invalid input clears pending evidence but keeps a latched lift; the latch is
 * only released after invalid_release_us of continuous invalid input (an IMU
 * failure must not block driving forever, matching the rest of the car). */
void buddy4_lift_update(buddy4_lift_t *d, float x_g, float y_g, float z_g,
                        bool valid, uint64_t sample_us, uint64_t now_us);
const char *buddy4_lift_reason_name(int reason);
#endif

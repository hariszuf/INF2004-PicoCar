#ifndef BUDDY4_LIFT_H
#define BUDDY4_LIFT_H
#include "../barrc_types.h"
#include <stdbool.h>
#include <stdint.h>

/* Robot lifted / overturned observer (replaces the old IMU collision observer).
 * Gravity-based: the angle between the measured gravity vector and the robot's
 * up axis (sensor Z times up_sign when the car is level), plus a sustained
 * deviation of the
 * acceleration magnitude from 1 g (the push of being picked up).
 * Once triggered the state LATCHES until the robot has been level and still for
 * clear_us, so a brief hand shake cannot restart the motors. */
typedef enum
{
    BUDDY4_LIFT_NONE = 0,
    /* tilted beyond tilt_enter_deg (picked up / tipped) */
    BUDDY4_LIFT_TILTED = 1,
    /* upside-down or on its side: tilt >= overturn_deg */
    BUDDY4_LIFT_OVERTURNED = 2,
    /* sustained non-gravity acceleration while level */
    BUDDY4_LIFT_MOTION = 3
} buddy4_lift_reason_t;

typedef struct
{
    /* Sign of the sensor Z reading when the car sits level: +1 if sensor +Z
     * points up,
     * -1 if the board is mounted with +Z pointing down. Measured on this car:
     * level = -1. */
    float32_t up_sign;
    float32_t tilt_enter_deg;
    float32_t tilt_exit_deg;
    float32_t overturn_deg;
    float32_t accel_enter_g;
    float32_t accel_exit_g;
    uint32_t tilt_confirm_us;
    uint32_t accel_confirm_us;
    uint32_t clear_us;
    uint32_t max_gap_us;
    uint32_t invalid_release_us;
} buddy4_lift_config_t;

typedef struct
{
    /* A fresh, finite, non-clipped sample was seen within the last 250 ms. */
    bool valid;
    /* Latched lifted/overturned state: motors must stay stopped. */
    bool lifted;
    /* buddy4_lift_reason_t of the latest trigger (0 when none). */
    uint8_t reason;
    uint32_t count; /* Number of lift events since boot. */
    uint64_t since_us; /* Sample time of the latest trigger. */
    /* Angle of gravity from the up axis, 0 = level, 180 = upside-down. */
    float32_t tilt_deg;
    float32_t deviation_g;   /* | ||a|| - 1 g |. */
} buddy4_lift_output_t;

typedef struct
{
    buddy4_lift_config_t config;
    buddy4_lift_output_t output;
    bool configured;
    bool has_time;
    uint64_t last_time_us;
    uint64_t tilt_since_us;
    uint64_t accel_since_us;
    uint64_t calm_since_us;
    uint64_t invalid_since_us;
    bool tilt_pending;
    bool accel_pending;
    bool calm_pending;
    bool invalid_pending;
} buddy4_lift_t;

buddy4_lift_config_t buddy4_lift_default_config (void);
bool buddy4_lift_init (buddy4_lift_t * d, buddy4_lift_config_t config);
/* One call per poll. valid must mean NEW finite, non-clipped corrected data.
 * Invalid input clears pending evidence but keeps a latched lift; the latch is
 * only released after invalid_release_us of continuous invalid input (an IMU
 * failure must not block driving forever, matching the rest of the car). */
void buddy4_lift_update (buddy4_lift_t * d, float32_t x_g, float32_t y_g,
                         float32_t z_g,
                         bool valid, uint64_t sample_us, uint64_t now_us);
const char * buddy4_lift_reason_name (int32_t reason);
#endif

/*** end of file ***/

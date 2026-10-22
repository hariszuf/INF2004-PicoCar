#ifndef BUDDY4_ENCODER_YAW_H
#define BUDDY4_ENCODER_YAW_H
#include <stdbool.h>
#include <stdint.h>
/* Encoder-only estimate; independent of SDK, IMU processing, and motor control. */
typedef struct {
    float centre_spacing_cm;
    uint32_t stale_us;
} buddy4_encoder_yaw_config_t;
typedef struct {
    bool valid;
    uint64_t sample_us;
    float encoder_yaw_rate_estimate_rad_s;
    float encoder_yaw_rate_estimate_deg_s;
} buddy4_encoder_yaw_output_t;
typedef struct {
    buddy4_encoder_yaw_config_t config;
    buddy4_encoder_yaw_output_t output;
    bool configured, has_sample_time, has_clock;
    uint64_t last_sample_us, clock_us;
} buddy4_encoder_yaw_t;
/* Confirmed default centre spacing 11.3 cm; provisional freshness limit 100 ms. */
buddy4_encoder_yaw_config_t buddy4_encoder_yaw_default_config(void);
bool buddy4_encoder_yaw_init(buddy4_encoder_yaw_t *d, buddy4_encoder_yaw_config_t cfg);
/* New signed speed observation (cm/s), timestamped at acquisition. Positive output
 * is left/CCW viewed from above. No additional filtering or temporal integration.
 * Invalid input clears validity immediately; last accepted numeric values/time
 * remain diagnostic only. Recovery accepts the next newer, fresh valid sample. */
void buddy4_encoder_yaw_update(buddy4_encoder_yaw_t *d, float left_cm_s,
    float right_cm_s, bool valid, uint64_t sample_us, uint64_t now_us);
/* Check age even when there are no incoming samples. */
buddy4_encoder_yaw_output_t buddy4_encoder_yaw_get(buddy4_encoder_yaw_t *d,
                                                 uint64_t now_us);
#endif

#ifndef BUDDY4_IMPACT_H
#define BUDDY4_IMPACT_H
#include <stdbool.h>
#include <stdint.h>
/* Preliminary possible-impact observation, not collision detection or safety. */
typedef struct {
    float trigger_g, release_g;
    uint32_t confirm_us, max_gap_us, cooldown_us, baseline_tau_us;
} buddy4_impact_config_t;
typedef struct {
    bool valid, possible_impact;
    uint32_t event_count;
    uint64_t sample_us, event_us;
    float magnitude_g, baseline_g, deviation_g;
} buddy4_impact_output_t;
typedef struct {
    buddy4_impact_config_t config;
    buddy4_impact_output_t output;
    bool configured, primed, has_time, has_clock, armed;
    uint8_t pending_samples, calm_samples;
    float baseline_g;
    uint64_t last_time_us, clock_us, pending_since_us;
} buddy4_impact_t;
buddy4_impact_config_t buddy4_impact_default_config(void);
bool buddy4_impact_init(buddy4_impact_t *d, buddy4_impact_config_t config);
/* One call per poll. valid must mean NEW finite, non-clipped corrected data.
 * Use IMU acceleration_valid, not tilt_valid: dynamic non-clipped input remains
 * useful. Clipping makes amplitude/event assessment unknown, never proof of
 * collision or no impact. Callers must expose clipping separately from this
 * observer's false event pulse. Existing event count/time remain history.
 * Invalid/time faults clear baseline and pending evidence, preserving event history
 * and cooldown. First recovery sample primes baseline and cannot trigger.
 * possible_impact pulses for exactly the triggering input sample. */
void buddy4_impact_update(buddy4_impact_t *d, float x_g, float y_g, float z_g,
                         bool valid, uint64_t sample_us, uint64_t now_us);
/* Age check for clients without new polls; does not repeat detection/counting. */
buddy4_impact_output_t buddy4_impact_get(buddy4_impact_t *d, uint64_t now_us);
#endif

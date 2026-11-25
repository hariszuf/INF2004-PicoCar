#ifndef BUDDY4_IMPACT_H
#define BUDDY4_IMPACT_H
#include "../barrc_types.h"
#include <stdbool.h>
#include <stdint.h>
/* Preliminary possible-impact observation, not collision detection or safety.
 */
typedef struct
{
    /* Thresholds apply to ||corrected XYZ - tracked XYZ baseline||, in g. */
    float32_t trigger_g;
    float32_t release_g;
    uint32_t confirm_us;
    uint32_t max_gap_us;
    uint32_t cooldown_us;
    uint32_t baseline_tau_us;
} buddy4_impact_config_t;
typedef struct
{
    bool valid;
    bool possible_impact;
    uint32_t event_count;
    uint64_t sample_us;
    uint64_t event_us;
    float32_t magnitude_g; /* ||corrected XYZ||, still includes gravity. */
    /* Norm of the vector reference used for this sample. */
    float32_t baseline_g;
    /* ||corrected XYZ - reference||; NOT |magnitude-baseline|. */
    float32_t deviation_g;
} buddy4_impact_output_t;
typedef struct
{
    buddy4_impact_config_t config;
    buddy4_impact_output_t output;
    bool configured;
    bool primed;
    bool has_time;
    bool has_clock;
    bool armed;
    uint8_t pending_samples;
    uint8_t calm_samples;
    float32_t baseline_xyz_g[3];
    uint64_t last_time_us;
    uint64_t clock_us;
    uint64_t pending_since_us;
} buddy4_impact_t;
buddy4_impact_config_t buddy4_impact_default_config (void);
bool buddy4_impact_init (buddy4_impact_t * d, buddy4_impact_config_t config);
/* One call per poll. valid must mean NEW finite, non-clipped corrected data.
 * Vector baseline follows subthreshold changes (default tau 500 ms) and freezes
 * at/above trigger. Not gyro-based gravity removal: abrupt tilt, hard braking
 * or
 * sustained vibration can resemble contact. No confirmed collision or motor
 * action.
 * Use IMU acceleration_valid, not tilt_valid: dynamic non-clipped input remains
 * useful. Clipping makes amplitude/event assessment unknown, never proof of
 * collision or no impact. Callers must expose clipping separately from this
 * observer's false event pulse. Existing event count/time remain history.
 * Invalid/time faults clear baseline and pending evidence, preserving event
 * history
 * and cooldown. First recovery sample primes baseline and cannot trigger.
 * possible_impact pulses for exactly the triggering input sample. */
void buddy4_impact_update (buddy4_impact_t * d, float32_t x_g, float32_t y_g,
                           float32_t z_g,
                           bool valid, uint64_t sample_us, uint64_t now_us);
/* Age check for clients without new polls; does not repeat detection/counting.
 */
buddy4_impact_output_t buddy4_impact_get (buddy4_impact_t * d, uint64_t now_us);
#endif

/*** end of file ***/

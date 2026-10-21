#include "terrain.h"
#include <math.h>

#define FORWARD_AXIS_IS_X 1
#define DIRECTION_SIGN 1.0f // Change to -1.0f if uphill/downhill are reversed.
#define SLOPE_ENTER_DEG 8.0f
#define SLOPE_EXIT_DEG 5.0f
#define HUMP_THRESHOLD_G 0.25f
#define HUMP_COOLDOWN_SAMPLES 25 // 25 x 20 ms = 0.5 seconds.

float terrain_magnitude(const accel_g_t *a) {
    return sqrtf(a->x*a->x + a->y*a->y + a->z*a->z);
}

float terrain_pitch_degrees(const accel_g_t *a) {
#if FORWARD_AXIS_IS_X
    float forward = a->x, other = a->y;
#else
    float forward = a->y, other = a->x;
#endif
    return atan2f(forward, sqrtf(other*other + a->z*a->z)) * 57.2957795f;
}

void terrain_init(terrain_detector_t *d, float pitch_baseline, float mag_baseline) {
    d->baseline_pitch = pitch_baseline;
    d->baseline_magnitude = mag_baseline;
    d->filtered_pitch = 0.0f;
    d->cooldown = 0;
    d->hump_active = false;
}

terrain_t terrain_process(terrain_detector_t *d, const accel_g_t *a, float *pitch_deg) {
    float instant = DIRECTION_SIGN * (terrain_pitch_degrees(a) - d->baseline_pitch);
    d->filtered_pitch = 0.8f*d->filtered_pitch + 0.2f*instant;
    *pitch_deg = d->filtered_pitch;
    float disturbance = fabsf(terrain_magnitude(a) - d->baseline_magnitude);

    if (d->cooldown) --d->cooldown;
    bool spike = disturbance >= HUMP_THRESHOLD_G;
    if (spike && !d->hump_active && d->cooldown == 0) {
        d->cooldown = HUMP_COOLDOWN_SAMPLES;
        d->hump_active = true;
        return TERRAIN_POSSIBLE_HUMP;
    }
    d->hump_active = spike;

    if (d->filtered_pitch >= SLOPE_ENTER_DEG) return TERRAIN_UPHILL;
    if (d->filtered_pitch <= -SLOPE_ENTER_DEG) return TERRAIN_DOWNHILL;
    // Until slopes are fully calibrated, use FLAT below the entry threshold.
    return TERRAIN_FLAT;
}

const char *terrain_name(terrain_t t) {
    switch (t) {
        case TERRAIN_FLAT: return "FLAT";
        case TERRAIN_UPHILL: return "UPHILL";
        case TERRAIN_DOWNHILL: return "DOWNHILL";
        case TERRAIN_POSSIBLE_HUMP: return "POSSIBLE_HUMP";
        default: return "UNKNOWN";
    }
}

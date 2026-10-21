#ifndef BUDDY4_TERRAIN_H
#define BUDDY4_TERRAIN_H
#include "lsm303d.h"
#include <stdbool.h>

typedef enum { TERRAIN_FLAT, TERRAIN_UPHILL, TERRAIN_DOWNHILL,
               TERRAIN_POSSIBLE_HUMP } terrain_t;

typedef struct {
    float baseline_pitch;
    float baseline_magnitude;
    float filtered_pitch;
    unsigned cooldown;
    bool hump_active;
} terrain_detector_t;

// The forward axis assumes the module +X points toward the front of the car.
// To use +Y, set FORWARD_AXIS_IS_X to 0 in terrain.c.
float terrain_pitch_degrees(const accel_g_t *a);
void terrain_init(terrain_detector_t *detector, float pitch_baseline, float mag_baseline);
terrain_t terrain_process(terrain_detector_t *detector, const accel_g_t *a,
                          float *pitch_deg);
const char *terrain_name(terrain_t t);
float terrain_magnitude(const accel_g_t *a);
#endif

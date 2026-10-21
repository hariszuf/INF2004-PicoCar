#include "terrain.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void) {
    const accel_g_t flat = {0, 0, 1};
    const accel_g_t uphill = {0.342f, 0, 0.94f};
    const accel_g_t downhill = {-0.342f, 0, 0.94f};
    const accel_g_t spike = {0, 0, 1.4f};
    terrain_detector_t detector;
    terrain_init(&detector, terrain_pitch_degrees(&flat), terrain_magnitude(&flat));
    float pitch;
    assert(terrain_process(&detector, &flat, &pitch) == TERRAIN_FLAT);
    for (int i = 0; i < 20; ++i) terrain_process(&detector, &uphill, &pitch);
    assert(terrain_process(&detector, &uphill, &pitch) == TERRAIN_UPHILL);
    for (int i = 0; i < 25; ++i) terrain_process(&detector, &flat, &pitch);
    assert(terrain_process(&detector, &flat, &pitch) == TERRAIN_FLAT);
    for (int i = 0; i < 20; ++i) terrain_process(&detector, &downhill, &pitch);
    assert(terrain_process(&detector, &downhill, &pitch) == TERRAIN_DOWNHILL);
    assert(terrain_process(&detector, &spike, &pitch) == TERRAIN_POSSIBLE_HUMP);
    puts("Host-side terrain tests passed.");
    return 0;
}

#include <assert.h>
#include <stdio.h>
#include "../app_program/buddy4/imu_detect.h"

static imu_raw_t level = {.ax=0, .ay=0, .az=16384};
static imu_raw_t ascent = {.ax=4200, .ay=0, .az=15830};
static imu_raw_t descent = {.ax=-4200, .ay=0, .az=15830};

static imu_result_t repeat(imu_detector_t *d, imu_raw_t r, int n)
{
    imu_result_t result = {0};
    while(n--) result = imu_detector_step(d, &r);
    return result;
}
int main(void)
{
    imu_detector_t d;
    imu_result_t out;
    imu_raw_t bump = {.ax=0,.ay=0,.az=24576};
    imu_detector_reset(&d);
    out = repeat(&d, level, 100);
    assert(out.ready && out.terrain == IMU_TERRAIN_FLAT);
    out = repeat(&d, ascent, 100);
    assert(out.terrain == IMU_TERRAIN_UPHILL);
    out = repeat(&d, level, 100);
    assert(out.terrain == IMU_TERRAIN_FLAT);
    out = repeat(&d, descent, 100);
    assert(out.terrain == IMU_TERRAIN_DOWNHILL);
    out = repeat(&d, level, 100);
    assert(out.terrain == IMU_TERRAIN_FLAT);
    out = repeat(&d, bump, 2);
    assert(out.hump_event && out.terrain == IMU_TERRAIN_HUMP);
    out = imu_detector_step(&d, &level);
    assert(!out.hump_event);
    puts("PASS: calibration, uphill, flat, downhill, hump event and clear");
    return 0;
}

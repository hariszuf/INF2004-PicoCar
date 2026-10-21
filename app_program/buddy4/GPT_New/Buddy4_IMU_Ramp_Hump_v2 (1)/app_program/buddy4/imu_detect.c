#include "imu_detect.h"
#include <string.h>

#define CALIBRATION_SAMPLES 100U  /* 2 seconds stationary at 50 Hz */
#define RAMP_ENTER_MDEG 8000      /* 8 degrees */
#define RAMP_EXIT_MDEG 5000       /* 5 degrees: hysteresis */
#define RAMP_STABLE_COUNT 6U      /* 120 ms */
#define HUMP_DISTURBANCE_MG 280  /* 0.28 g magnitude change */
#define HUMP_SAMPLES 2U           /* 40 ms */
#define HUMP_COOLDOWN_SAMPLES 25U /* 500 ms */
#define FILTER_GYRO_WEIGHT 97    /* complementary filter, percent */

static int32_t iabs32(int32_t x) { return x < 0 ? -x : x; }

/* Fast integer approximation to atan(x/z) in millidegrees, suitable for
 * shallow ramps. x and z are acceleration counts. Avoids libm dependency. */
static int32_t accel_pitch_mdeg(const imu_raw_t *r)
{
    int32_t x = (int32_t)r->ax * IMU_PITCH_ACCEL_SIGN;
    int32_t z = r->az;
    int32_t ax = iabs32(x);
    int32_t az = iabs32(z);
    int32_t angle;
    if (ax == 0 && az == 0) return 0;
    /* atan(t) ≈ t*(45 + 15.7*(1-t)) degrees for t in [0,1]. */
    if (ax <= az) {
        int32_t t = (ax * 1000) / (az ? az : 1);
        angle = (t * (60700 - (15700 * t)/1000))/1000;
    } else {
        int32_t t = (az * 1000) / ax;
        angle = 90000 - (t * (60700 - (15700 * t)/1000))/1000;
    }
    return x < 0 ? -angle : angle;
}

/* Integer square root, no floating-point library needed. */
static uint32_t isqrt64(uint64_t x)
{
    uint64_t bit = (uint64_t)1 << 62;
    uint64_t out = 0;
    while (bit > x) bit >>= 2;
    while (bit) {
        if (x >= out + bit) { x -= out + bit; out = (out >> 1) + bit; }
        else out >>= 1;
        bit >>= 2;
    }
    return (uint32_t)out;
}

void imu_detector_reset(imu_detector_t *d)
{
    memset(d, 0, sizeof(*d));
    d->state = IMU_TERRAIN_CALIBRATING;
    d->candidate = IMU_TERRAIN_CALIBRATING;
}

imu_result_t imu_detector_step(imu_detector_t *d, const imu_raw_t *r)
{
    imu_result_t out = {0};
    int32_t raw_pitch;
    int32_t magnitude;
    int32_t pitch;
    imu_terrain_t next;
    uint64_t mag_sq;
    if (!d || !r) { out.terrain = IMU_TERRAIN_ERROR; return out; }
    mag_sq = (uint64_t)((int64_t)r->ax*r->ax + (int64_t)r->ay*r->ay + (int64_t)r->az*r->az);
    magnitude = (int32_t)isqrt64(mag_sq);
    out.accel_disturbance_mg = iabs32((magnitude * 1000)/16384 - 1000);
    raw_pitch = accel_pitch_mdeg(r);

    if (d->calibration_samples < CALIBRATION_SAMPLES) {
        /* Do not calibrate while being moved/shaken. */
        if (out.accel_disturbance_mg > 150 || iabs32(r->gy) > 400) {
            imu_detector_reset(d);
            return out;
        }
        d->pitch_sum += raw_pitch;
        d->gyro_sum += r->gy;
        d->calibration_samples++;
        if (d->calibration_samples == CALIBRATION_SAMPLES) {
            d->pitch_zero_mdeg = (int32_t)(d->pitch_sum / CALIBRATION_SAMPLES);
            d->gyro_bias_raw = (int32_t)(d->gyro_sum / CALIBRATION_SAMPLES);
            d->pitch_mdeg = 0;
            d->state = IMU_TERRAIN_FLAT;
            d->candidate = IMU_TERRAIN_FLAT;
        }
        out.terrain = d->state;
        out.ready = (d->calibration_samples == CALIBRATION_SAMPLES);
        return out;
    }

    /* Gyro integration over 20ms: raw gyro has 131 counts per degree/sec.
     * 20 ms converts to 1000*20/(131*1000) millideg/raw = 20/131. */
    pitch = d->pitch_mdeg + ((int32_t)r->gy - d->gyro_bias_raw) * IMU_PITCH_GYRO_SIGN * 20 / 131;
    /* During violent motion, avoid interpreting linear acceleration as tilt. */
    if (out.accel_disturbance_mg < 180) {
        pitch = (FILTER_GYRO_WEIGHT * pitch +
                 (100-FILTER_GYRO_WEIGHT)*(raw_pitch-d->pitch_zero_mdeg))/100;
    }
    d->pitch_mdeg = pitch;
    out.pitch_mdeg = pitch;
    out.ready = 1;

    if (d->hump_cooldown) d->hump_cooldown--;
    if (out.accel_disturbance_mg >= HUMP_DISTURBANCE_MG) {
        if (d->hump_count < HUMP_SAMPLES) d->hump_count++;
    } else d->hump_count = 0;
    if (d->hump_count >= HUMP_SAMPLES && d->hump_cooldown == 0) {
        out.hump_event = 1;
        d->hump_cooldown = HUMP_COOLDOWN_SAMPLES;
        d->hump_count = 0;
    }

    /* Ramps are classified by sustained pitch, not acceleration spikes. */
    if (pitch >= RAMP_ENTER_MDEG) next = IMU_TERRAIN_UPHILL;
    else if (pitch <= -RAMP_ENTER_MDEG) next = IMU_TERRAIN_DOWNHILL;
    else if (iabs32(pitch) <= RAMP_EXIT_MDEG) next = IMU_TERRAIN_FLAT;
    else next = d->state == IMU_TERRAIN_UPHILL || d->state == IMU_TERRAIN_DOWNHILL
                ? d->state : IMU_TERRAIN_FLAT;

    if (next == d->candidate) {
        if (d->stable_count < RAMP_STABLE_COUNT) d->stable_count++;
    } else { d->candidate = next; d->stable_count = 1; }
    if (d->stable_count >= RAMP_STABLE_COUNT) d->state = next;
    out.terrain = out.hump_event ? IMU_TERRAIN_HUMP : d->state;
    return out;
}

const char *imu_terrain_name(imu_terrain_t t)
{
    switch (t) {
    case IMU_TERRAIN_CALIBRATING: return "CALIBRATING";
    case IMU_TERRAIN_FLAT: return "FLAT";
    case IMU_TERRAIN_UPHILL: return "UPHILL";
    case IMU_TERRAIN_DOWNHILL: return "DOWNHILL";
    case IMU_TERRAIN_HUMP: return "HUMP";
    default: return "ERROR";
    }
}

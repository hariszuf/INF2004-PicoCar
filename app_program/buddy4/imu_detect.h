#ifndef PICOCAR_IMU_DETECT_H
#define PICOCAR_IMU_DETECT_H
#include <stdint.h>

/* Physical mounting assumption: +X points towards robot front, +Z upwards.
 * Gyro Y increases for nose-up rotation. Set signs below if installed otherwise. */
#ifndef IMU_PITCH_ACCEL_SIGN
#define IMU_PITCH_ACCEL_SIGN 1
#endif
#ifndef IMU_PITCH_GYRO_SIGN
#define IMU_PITCH_GYRO_SIGN 1
#endif

typedef enum {
    IMU_TERRAIN_CALIBRATING = 0,
    IMU_TERRAIN_FLAT,
    IMU_TERRAIN_UPHILL,
    IMU_TERRAIN_DOWNHILL,
    IMU_TERRAIN_HUMP,
    IMU_TERRAIN_ERROR
} imu_terrain_t;

typedef struct {
    int16_t ax, ay, az;  /* +/-2g, 16384 counts/g */
    int16_t gx, gy, gz;  /* +/-250 degrees/sec, 131 counts/deg/sec */
} imu_raw_t;

typedef struct {
    int32_t pitch_mdeg;           /* positive: robot nose up */
    int32_t accel_disturbance_mg; /* |acceleration magnitude - 1g| */
    imu_terrain_t terrain;
    uint8_t ready;
    uint8_t hump_event;          /* one-sample rising-edge notification */
} imu_result_t;

typedef struct {
    int32_t pitch_mdeg;
    int32_t pitch_zero_mdeg;
    int32_t gyro_bias_raw;
    int64_t gyro_sum;
    int64_t pitch_sum;
    uint16_t calibration_samples;
    uint8_t stable_count;
    uint8_t hump_count;
    uint8_t hump_cooldown;
    imu_terrain_t state;
    imu_terrain_t candidate;
} imu_detector_t;

void imu_detector_reset(imu_detector_t *d);
/* Call at fixed 20 ms interval; remain stationary and level for calibration. */
imu_result_t imu_detector_step(imu_detector_t *d, const imu_raw_t *r);
const char *imu_terrain_name(imu_terrain_t terrain);
#endif

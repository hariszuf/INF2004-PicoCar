#ifndef BUDDY4_TASK_H
#define BUDDY4_TASK_H
#include <stdbool.h>
#include <stdint.h>
#ifdef BUDDY4_MTK_HOST_TEST
#include "tests/mtk_stubs/device_api.h"
#else
#include <tk/typedef.h>
#endif

/* Public application data: no standard-size_t or Pico SDK dependency.
 * All accessors are task-context only. No ISR may take this mutex. */
typedef struct {
    bool valid;
    uint64_t sample_us;
    float left_cm_s, right_cm_s;
} buddy4_task_speed_t;
typedef struct {
    int side;
    uint64_t start_us, end_us;
    bool stable_valid, median_valid;
    float stable_cm, median_cm;
} buddy4_task_event_t;
typedef struct {
    uint32_t sequence, skipped_slots, io_errors, completed_events;
    uint64_t poll_us, sample_us;
    /* Kernel clock resolution is 10 ms. UINT32_MAX means unavailable age. */
    uint32_t poll_duration_ms, poll_interval_ms, sample_age_ms;
    bool timing_valid;
    bool valid; /* Acquisition freshness only. */
    bool acceleration_valid, tilt_valid;
    int measurement_quality;
    uint8_t clipped_axes, status_a;
    uint32_t clipped_samples, unsuitable_samples;
    bool sensor_config_valid;
    uint8_t who_am_i, sensor_ctrl[6];
    uint64_t config_checked_us;
    int poll_result, device_error;
    /* Latched until next IMU error or reset, even after successful recovery. */
    bool saved_error_present, saved_error_read, saved_error_time_valid;
    int saved_error_code;
    uint8_t saved_error_register;
    uint64_t saved_error_us;
    unsigned int saved_timeout_trace[32];
    int16_t raw[3];
    float acceleration_g[3], uncorrected_g[3], magnitude_g;
    float pitch_deg, roll_deg, height_cm;
    int terrain, side;
    bool bump_event, side_event_ended;
    buddy4_task_event_t last_event;
    bool impact_valid, possible_impact;
    uint32_t impact_count;
    uint64_t impact_time_us;
    float impact_deviation_g;
    int motion_state;
    bool motion_valid, yaw_valid;
    uint64_t encoder_sample_us;
    float encoder_yaw_deg_s;
} buddy4_task_snapshot_t;

ER buddy4_task_init(void);
ER buddy4_task_start(void);
ER buddy4_task_get(buddy4_task_snapshot_t *out);
ER buddy4_task_submit_speed(const buddy4_task_speed_t *speed);
ER buddy4_task_time_us(uint64_t *out);
#endif

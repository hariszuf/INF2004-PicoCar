#ifndef BUDDY4_IMU_H
#define BUDDY4_IMU_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BUDDY4_PERIOD_US 20000u
#define BUDDY4_I2C_TIMEOUT_US 1000u

/* Callbacks return transferred byte count; use bounded hardware transfers.
 * no_stop keeps the bus active for the following repeated-start read. */
typedef int (*buddy4_write_fn)(void *context, uint8_t address,
                               const uint8_t *data, size_t count, bool no_stop);
typedef int (*buddy4_read_fn)(void *context, uint8_t address,
                              uint8_t *data, size_t count, bool no_stop);

typedef struct {
    void *context;
    buddy4_write_fn write;
    buddy4_read_fn read;
} buddy4_bus_t;

typedef enum {
    BUDDY4_FRESH,
    BUDDY4_STALE,
    BUDDY4_IO_ERROR,
    BUDDY4_NOT_READY,
} buddy4_result_t;

typedef struct {
    /* Sensor XYZ: corrected = (uncorrected - bias) * scale. Identity: 0, 1. */
    float accel_bias_g[3];
    float accel_scale[3];
    bool mounting_offsets_need_recalibration;
    float pitch_sign;
    float pitch_offset_deg;
    float roll_sign;
    float roll_offset_deg;
    /* Confirmed default wheel-centre spacing: 11.3 cm. */
    float centre_spacing_cm;
    bool centre_spacing_confirmed;
} buddy4_config_t;

typedef struct {
    int side;
    uint64_t start_us;
    bool peak_confirmed;
    float peak_roll_deg;
    float peak_height_cm;
    /* Experimental moving peak, independent of stability confirmation.
     * Numeric fields are meaningful only when median_candidate_valid is true. */
    bool median_candidate_valid;
    float median_candidate_roll_deg;
    float median_candidate_height_cm;
    uint8_t median_count;
    float median_roll[11];
    uint64_t median_time_us[11];
    bool window_active;
    uint64_t window_start_us;
    uint64_t last_sample_us;
    float window_min_roll;
    float window_max_roll;
    float window_min_height;
} side_event_t;

typedef struct {
    bool valid;             /* True only for the latest successful fresh poll. */
    uint64_t timestamp_us;  /* Host acquisition time of last fresh sample. */
    uint32_t error_count;   /* Configuration/transfer failures; excludes stale. */
    int16_t raw[3];         /* X, Y, Z signed sensor counts. */
    float uncorrected_accel_g[3]; /* Sensor XYZ counts converted to g. */
    float accel_g[3];       /* Corrected sensor XYZ in g; used for processing. */
    float magnitude_g;
    float pitch_deg;
    float roll_deg;
    float filtered_pitch_deg;
    float filtered_roll_deg;
    int terrain;
    int side;
    float transient_g;
    bool transient_event;
    float wheel_height_cm;
    bool height_geometry_confirmed;
    side_event_t event;
    side_event_t ended_event;
    bool event_ended;       /* One fresh-sample pulse; ended_event holds summary. */
} buddy4_output_t;

typedef struct {
    buddy4_bus_t bus;
    buddy4_config_t config;
    buddy4_output_t output;
    bool ready;
    bool has_sample;
    bool filter_valid;
    bool bump_armed;
    bool had_bump;
    float baseline;
    uint64_t last_bump_us;
} buddy4_imu_t;

/* Preliminary acceleration calibration; preserve the configured mounting offsets.
 * Confirmed wheel-centre spacing is 11.3 cm. No startup auto-zero. */
buddy4_config_t buddy4_default_config(void);

/* Reset caller-owned state and store callbacks/configuration; no I/O occurs.
 * imu must be non-NULL and remain available for subsequent calls. */
void buddy4_init(buddy4_imu_t *imu, buddy4_bus_t bus, buddy4_config_t config);

/* Validate configuration, identify the LSM303D and write/read back settings.
 * Resets detection state; returns false and increments error_count on failure.
 * Call after init, using bounded bus callbacks. */
bool buddy4_configure(buddy4_imu_t *imu);

/* Check data-ready once and process at most one sample; never wait for freshness.
 * Supply monotonic host timestamps, normally at BUDDY4_PERIOD_US intervals.
 * Check output.valid: non-fresh calls retain measurements and their timestamp.
 * Stale samples restart both estimator windows, retaining accepted event peaks.
 * Completed-event snapshots survive errors/reconfiguration until the next event ends.
 * Transfer errors abort the event,
 * clear ready and increment errors; reconfigure before polling resumes. */
buddy4_result_t buddy4_poll(buddy4_imu_t *imu, uint64_t timestamp_us);

/* Decode one full signed 16-bit, little-endian acceleration value without shift. */
int16_t buddy4_decode_signed16(uint8_t low, uint8_t high);

/* Map negative/zero/positive pitch state to DOWNHILL/FLAT/UPHILL. */
const char *buddy4_terrain_name(int state);

/* Map negative/zero/positive roll state to LEFT_WHEEL_UP/SIDE_LEVEL/RIGHT_WHEEL_UP. */
const char *buddy4_side_name(int state);

#endif

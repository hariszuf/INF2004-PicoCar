#ifndef BUDDY4_IMU_H
#define BUDDY4_IMU_H

#include "../barrc_types.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BUDDY4_PERIOD_US 20000u
#define BUDDY4_I2C_TIMEOUT_US 1000u

/* Callbacks return transferred byte count; use bounded hardware transfers.
 * no_stop keeps the bus active for the following repeated-start read. */
typedef int32_t (*buddy4_write_fn)(void * context, uint8_t address,
                               const uint8_t * data, size_t count,
                                       bool no_stop);
typedef int32_t (*buddy4_read_fn)(void * context, uint8_t address,
                              uint8_t * data, size_t count, bool no_stop);

typedef struct
{
    void * context;
    buddy4_write_fn write;
    buddy4_read_fn read;
} buddy4_bus_t;

typedef enum
{
    BUDDY4_FRESH,
    BUDDY4_STALE,
    BUDDY4_IO_ERROR,
    BUDDY4_NOT_READY,
} buddy4_result_t;

/* Acquisition success is independent of suitability for gravity-based tilt. */
typedef enum
{
    BUDDY4_QUALITY_UNAVAILABLE,
    BUDDY4_QUALITY_GRAVITY_LIKE,
    BUDDY4_QUALITY_CLIPPED,
    BUDDY4_QUALITY_NONFINITE,
    BUDDY4_QUALITY_NON_GRAVITY,
} buddy4_quality_t;

typedef struct
{
    uint8_t who_am_i;
    uint8_t ctrl[6]; /* CTRL0,1,2,3,4,7, in that order. */
    /* Last verification succeeded; not continuous verification. */
    bool valid : 1;
} buddy4_sensor_config_t;

typedef struct
{
    /* Sensor XYZ: corrected = (uncorrected - bias) * scale. Identity: 0, 1. */
    float32_t accel_bias_g[3];
    float32_t accel_scale[3];
    float32_t pitch_sign;
    float32_t pitch_offset_deg;
    float32_t roll_sign;
    float32_t roll_offset_deg;
    /* Confirmed default wheel-centre spacing: 11.3 cm. */
    float32_t centre_spacing_cm;
    bool mounting_offsets_need_recalibration : 1;
    bool centre_spacing_confirmed : 1;
} buddy4_config_t;

typedef struct
{
    uint64_t start_us;
    float32_t peak_roll_deg;
    float32_t peak_height_cm;
    /* Experimental moving peak, independent of stability confirmation.
     * Numeric fields are meaningful only when median_candidate_valid is true.
     */
    float32_t median_candidate_roll_deg;
    float32_t median_candidate_height_cm;
    float32_t median_roll[11];
    uint64_t median_time_us[11];
    uint64_t window_start_us;
    uint64_t last_sample_us;
    float32_t window_min_roll;
    float32_t window_max_roll;
    float32_t window_min_height;
    int8_t side;            /* -1, 0 or +1 */
    uint8_t median_count;   /* 0..11 */
    bool peak_confirmed : 1;
    bool median_candidate_valid : 1;
    bool window_active : 1;
} side_event_t;

typedef struct
{
    uint8_t clipped_axes;   /* XYZ bit mask: exact signed ADC endpoints. */
    uint8_t status_a;       /* Last status read; overrun is not clipping. */
    uint32_t clipped_samples;
    uint32_t unsuitable_samples; /* Saturating history. */
    uint64_t timestamp_us;  /* Host acquisition time of last fresh sample. */
    /* Configuration/transfer failures; excludes stale. */
    uint32_t error_count;
    int16_t raw[3];         /* X, Y, Z signed sensor counts. */
    float32_t uncorrected_accel_g[3]; /* Sensor XYZ counts converted to g. */
    /* Corrected sensor XYZ in g; used for processing. */
    float32_t accel_g[3];
    float32_t magnitude_g;
    float32_t pitch_deg;
    float32_t roll_deg;
    float32_t filtered_pitch_deg;
    float32_t filtered_roll_deg;
    float32_t transient_g;
    float32_t wheel_height_cm;
    side_event_t event;
    side_event_t ended_event;
    int8_t terrain;         /* -1 (unknown) .. 4, fits in one byte */
    int8_t side;            /* -1, 0 or +1 */
    uint8_t quality : 3;    /* buddy4_quality_t, 5 values */
    /* Acquisition only: latest complete, fresh poll. */
    bool valid : 1;
    /* Finite, non-clipped; usable by impact observer. */
    bool acceleration_valid : 1;
    /* Gravity-like input; still not proof of static tilt. */
    bool tilt_valid : 1;
    bool transient_event : 1;
    bool height_geometry_confirmed : 1;
    /* One fresh-sample pulse; ended_event holds summary. */
    bool event_ended : 1;
} buddy4_output_t;

typedef struct
{
    buddy4_bus_t bus;
    buddy4_config_t config;
    buddy4_output_t output;
    buddy4_sensor_config_t sensor_config;
    float32_t baseline;
    uint64_t last_bump_us;
    bool ready : 1;
    bool has_sample : 1;
    bool filter_valid : 1;
    bool tilt_filter_valid : 1;
    bool bump_armed : 1;
    bool had_bump : 1;
} buddy4_imu_t;

/* Preliminary acceleration calibration; preserve the configured mounting
 * offsets.
 * Confirmed wheel-centre spacing is 11.3 cm. No startup auto-zero. */
buddy4_config_t buddy4_default_config (void);

/* Reset caller-owned state and store callbacks/configuration; no I/O occurs.
 * imu must be non-NULL and remain available for subsequent calls. */
void buddy4_init (buddy4_imu_t * imu, buddy4_bus_t bus, buddy4_config_t config);

/* Validate configuration, identify the LSM303D and write/read back settings.
 * Resets detection state; returns false and increments error_count on failure.
 * Call after init, using bounded bus callbacks. */
bool buddy4_configure (buddy4_imu_t * imu);

/* Bounded readback, no register writes. Failure aborts detection, clears ready,
 * increments configuration/transfer errors. Caller may retry configuration. */
bool buddy4_verify_config (buddy4_imu_t * imu);
const char * buddy4_quality_name (int32_t quality);

/* Check data-ready once and process at most one sample; never wait for
 * freshness.
 * Supply monotonic host timestamps, normally at BUDDY4_PERIOD_US intervals.
 * output.valid means acquisition only; tilt_valid gates tilt/height/peaks;
 * acceleration_valid gates impact input. Questionable fresh samples retain raw
 * diagnostics, abort the active tilt event without completion, reset its
 * filter,
 * and preserve ended_event. Clipping does not increment error_count.
 * Non-fresh calls retain measurements and their timestamp with validity false.
 * Stale samples restart both estimator windows, retaining accepted event peaks.
 * Completed-event snapshots survive errors/reconfiguration until the next event
 * ends.
 * Transfer errors abort the event,
 * clear ready and increment errors; reconfigure before polling resumes. */
buddy4_result_t buddy4_poll (buddy4_imu_t * imu, uint64_t timestamp_us);

/* Decode one full signed 16-bit, little-endian acceleration value without
 * shift. */
int16_t buddy4_decode_signed16 (uint8_t low, uint8_t high);

/* Map negative/zero/positive pitch state to DOWNHILL/FLAT/UPHILL. */
const char * buddy4_terrain_name (int32_t state);

/* Map negative/zero/positive roll state to
 * LEFT_WHEEL_UP/SIDE_LEVEL/RIGHT_WHEEL_UP. */
const char * buddy4_side_name (int32_t state);

#endif

/*** end of file ***/

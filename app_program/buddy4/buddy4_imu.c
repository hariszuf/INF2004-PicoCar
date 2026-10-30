#include "buddy4_imu.h"
#include <math.h>
#include <string.h>

/* Confirmed LSM303D register map and configuration values. */
#define LSM303D_ADDRESS                 0x1D
#define LSM303D_REG_WHO_AM_I             0x0F
#define LSM303D_REG_CTRL0                0x1F
#define LSM303D_REG_CTRL1                0x20
#define LSM303D_REG_CTRL2                0x21
#define LSM303D_REG_CTRL3                0x22
#define LSM303D_REG_CTRL4                0x23
#define LSM303D_REG_CTRL7                0x26
#define LSM303D_REG_STATUS_A             0x27
#define LSM303D_REG_OUT_X_L_A            0x28
#define LSM303D_WHO_AM_I_VALUE           0x49
#define LSM303D_AUTO_INCREMENT           0x80
#define LSM303D_XYZ_DATA_READY           0x08
#define LSM303D_CTRL1_100HZ_BDU_XYZ      0x6F
#define LSM303D_CTRL2_2G_50HZ_BANDWIDTH   0xC0
#define LSM303D_CTRL7_MAG_POWER_DOWN     0x02

#define G_PER_COUNT             0.000061f
#define RAD_TO_DEG              57.2957795131f
#define DEG_TO_RAD              0.0174532925199f

/* Reference angle smoothing; mounting calibration is caller-configurable. */
/* Conservative suitability envelope, not an ST specification or calibration.
 * Static gravity is ~1 g in every orientation. Outside +/-20%, gravity-only
 * height is demonstrably unreliable. Inside is necessary, never sufficient. */
#define TILT_MIN_G              0.8f
#define TILT_MAX_G              1.2f

#define ANGLE_ALPHA             0.15f

/* Provisional tilt thresholds with hysteresis */
#define PITCH_ENTER_DEG         6.0f
#define PITCH_EXIT_DEG          3.0f
#define ROLL_ENTER_DEG          4.0f
#define ROLL_EXIT_DEG           2.0f

/* Stable-window settings for confirmed peak estimates */
#define PEAK_CONFIRM_US         200000
#define PEAK_ALLOWED_CHANGE_DEG 1.0f
#define MAX_SAMPLE_GAP_US       (2 * BUDDY4_PERIOD_US)

/* Preliminary acceleration-transient detector */
#define BUMP_THRESHOLD_G        0.18f
#define BUMP_REARM_G            0.08f
#define BUMP_COOLDOWN_US        500000
#define BASELINE_ALPHA          0.05f

const char *buddy4_quality_name(int quality)
{
    switch (quality) {
    case BUDDY4_QUALITY_GRAVITY_LIKE: return "GRAVITY_LIKE";
    case BUDDY4_QUALITY_CLIPPED: return "CLIPPED";
    case BUDDY4_QUALITY_NONFINITE: return "NONFINITE";
    case BUDDY4_QUALITY_NON_GRAVITY: return "NON_GRAVITY";
    default: return "UNAVAILABLE";
    }
}

static int classify_angle(int previous,
                          float angle,
                          float enter_deg,
                          float exit_deg)
{
    if (angle >= enter_deg) {
        return 1;
    }

    if (angle <= -enter_deg) {
        return -1;
    }

    if (previous == 1 && angle > exit_deg) {
        return 1;
    }

    if (previous == -1 && angle < -exit_deg) {
        return -1;
    }

    return 0;
}

const char *buddy4_terrain_name(int state)
{
    if (state > 0) {
        return "UPHILL";
    }
    if (state < 0) {
        return "DOWNHILL";
    }
    return "FLAT";
}

const char *buddy4_side_name(int state)
{
    /* Mapping established from your stationary tests */
    if (state > 0) {
        return "RIGHT_WHEEL_UP";
    }
    if (state < 0) {
        return "LEFT_WHEEL_UP";
    }
    return "SIDE_LEVEL";
}

static void finish_side_event(side_event_t *event,
                              uint64_t timestamp,
                              const char *reason)
{
    if (event->side == 0) {
        return;
    }

    (void)timestamp;
    (void)reason;
    *event = (side_event_t){0};
}

static void start_peak_window(side_event_t *event,
                              uint64_t timestamp,
                              float absolute_roll,
                              float height_cm)
{
    event->window_active = true;
    event->window_start_us = timestamp;
    event->last_sample_us = timestamp;
    event->window_min_roll = absolute_roll;
    event->window_max_roll = absolute_roll;
    event->window_min_height = height_cm;
}

/* Fixed-size chronological window; insertion sort costs at most 55 comparisons. */
static void update_median(side_event_t *event, uint64_t timestamp,
                          float absolute_roll, float spacing_cm)
{
    unsigned n = event->median_count;
    if (n && (timestamp <= event->median_time_us[n - 1] ||
              timestamp - event->median_time_us[n - 1] > MAX_SAMPLE_GAP_US))
        n = 0;
    if (n == 11) {
        memmove(event->median_roll, event->median_roll + 1, 10 * sizeof(float));
        memmove(event->median_time_us, event->median_time_us + 1, 10 * sizeof(uint64_t));
        n = 10;
    }
    event->median_roll[n] = absolute_roll;
    event->median_time_us[n] = timestamp;
    event->median_count = ++n;
    if (n != 11) return;
    uint64_t span = timestamp - event->median_time_us[0];
    if (span < 190000 || span > 210000) return;
    float sorted[11];
    memcpy(sorted, event->median_roll, sizeof(sorted));
    for (unsigned i = 1; i < 11; ++i) {
        float value = sorted[i];
        unsigned j = i;
        while (j && sorted[j - 1] > value) {
            sorted[j] = sorted[j - 1]; --j;
        }
        sorted[j] = value;
    }
    if (!event->median_candidate_valid || sorted[5] > event->median_candidate_roll_deg) {
        event->median_candidate_roll_deg = sorted[5];
        event->median_candidate_height_cm = spacing_cm * sinf(sorted[5] * DEG_TO_RAD);
    }
    event->median_candidate_valid = true;
}

static void update_side_event(side_event_t *event,
                              int side,
                              uint64_t timestamp,
                              float roll,
                              float height_cm)
{
    if (event->side != 0 && side != event->side) {
        finish_side_event(
            event,
            timestamp,
            side == 0 ? "RETURNED_TO_LEVEL" : "SIDE_CHANGED"
        );
    }

    if (side == 0) {
        return;
    }

    if (event->side == 0) {
        *event = (side_event_t){0};
        event->side = side;
        event->start_us = timestamp;
    }

    float absolute_roll = fabsf(roll);

    /*
     * Require fresh consecutive samples.
     * A long gap restarts the confirmation window.
     */
    if (!event->window_active ||
        timestamp - event->last_sample_us > MAX_SAMPLE_GAP_US) {
        start_peak_window(
            event, timestamp, absolute_roll, height_cm
        );
        return;
    }

    event->last_sample_us = timestamp;

    if (absolute_roll < event->window_min_roll) {
        event->window_min_roll = absolute_roll;
    }

    if (absolute_roll > event->window_max_roll) {
        event->window_max_roll = absolute_roll;
    }

    if (height_cm < event->window_min_height) {
        event->window_min_height = height_cm;
    }

    /*
     * Restart if roll changes too much within the window.
     */
    if (event->window_max_roll - event->window_min_roll >
        PEAK_ALLOWED_CHANGE_DEG) {
        start_peak_window(
            event, timestamp, absolute_roll, height_cm
        );
        return;
    }

    if (timestamp - event->window_start_us >= PEAK_CONFIRM_US) {
        /*
         * Use the minimum sustained value from a stable window.
         * This is a conservative confirmed estimate.
         */
        if (!event->peak_confirmed ||
            event->window_min_roll > event->peak_roll_deg) {
            event->peak_roll_deg = event->window_min_roll;
        }

        if (!event->peak_confirmed ||
            event->window_min_height > event->peak_height_cm) {
            event->peak_height_cm = event->window_min_height;
        }

        event->peak_confirmed = true;
        event->window_active = false;
    }
}

buddy4_config_t buddy4_default_config(void)
{
    return (buddy4_config_t){
        /* Preliminary acceleration calibration */
        .accel_bias_g = {0.03799f, 0.03790f, 0.08411f},
        .accel_scale = {1.01301f, 1.03419f, 1.01223f},
        .mounting_offsets_need_recalibration = false,
        .pitch_sign = 1.0f,
        .pitch_offset_deg = 0.97f,
        .roll_sign = 1.0f,
        /* Provisional mounting offset; pending flat/1 cm ramp hardware verification. */
        .roll_offset_deg = 0.61f,
        .centre_spacing_cm = 11.3f,
        .centre_spacing_confirmed = true,
    };
}

void buddy4_init(buddy4_imu_t *imu, buddy4_bus_t bus, buddy4_config_t config)
{
    memset(imu, 0, sizeof(*imu));
    imu->bus = bus;
    imu->config = config;
    imu->bump_armed = true;
}

int16_t buddy4_decode_signed16(uint8_t low, uint8_t high)
{
    uint16_t word = (uint16_t)low | ((uint16_t)high << 8);
    return (int16_t)((word & 0x8000u) ? (int32_t)word - 65536 : (int32_t)word);
}

static bool read_registers(buddy4_imu_t *imu, uint8_t reg,
                           uint8_t *data, size_t count)
{
    if (count > 1) {
        reg |= LSM303D_AUTO_INCREMENT;
    }

    return imu->bus.write && imu->bus.read &&
           imu->bus.write(imu->bus.context, LSM303D_ADDRESS,
                          &reg, 1, true) == 1 &&
           imu->bus.read(imu->bus.context, LSM303D_ADDRESS,
                         data, count, false) == (int)count;
}

/* Clear per-call flags while retaining the last fresh measurements. */
static void invalidate_output(buddy4_imu_t *imu)
{
    imu->output.valid = false;
    imu->output.acceleration_valid = false;
    imu->output.tilt_valid = false;
    imu->output.quality = BUDDY4_QUALITY_UNAVAILABLE;
    imu->output.transient_event = false;
    imu->output.event_ended = false;
}

/* Abort interrupted detection without producing a completed-event pulse.
 * Keep the last measurements/timestamp and cumulative error count for diagnostics. */
static void reset_detection(buddy4_imu_t *imu)
{
    invalidate_output(imu);
    imu->filter_valid = false;
    imu->tilt_filter_valid = false;
    imu->has_sample = false;
    imu->baseline = 0.0f;
    imu->bump_armed = true;
    imu->had_bump = false;
    imu->last_bump_us = 0;
    imu->output.transient_g = 0.0f;
    imu->output.event = (side_event_t){0};
    /* Retain the last completed event across errors/reconfiguration. */
    imu->output.terrain = 0;
    imu->output.side = 0;
}

bool buddy4_configure(buddy4_imu_t *imu)
{
    imu->ready = false;
    imu->sensor_config.valid = false;
    reset_detection(imu);

    if (!isfinite(imu->config.centre_spacing_cm) ||
        imu->config.centre_spacing_cm <= 0 ||
        !isfinite(imu->config.pitch_offset_deg) ||
        !isfinite(imu->config.roll_offset_deg) ||
        fabsf(imu->config.pitch_sign) != 1 ||
        fabsf(imu->config.roll_sign) != 1) {
        goto fail;
    }

    for (int axis = 0; axis < 3; ++axis) {
        if (!isfinite(imu->config.accel_bias_g[axis]) ||
            !isfinite(imu->config.accel_scale[axis]) ||
            imu->config.accel_scale[axis] <= 0.0f) {
            goto fail;
        }
    }

    if (!read_registers(imu, LSM303D_REG_WHO_AM_I, &imu->sensor_config.who_am_i, 1) ||
        imu->sensor_config.who_am_i != LSM303D_WHO_AM_I_VALUE) {
        goto fail;
    }

    /* Stop sampling, disable FIFO/interrupts, configure scale, then restart. */
    const uint8_t settings[][2] = {
        {LSM303D_REG_CTRL1, 0x00},
        {LSM303D_REG_CTRL0, 0x00},
        {LSM303D_REG_CTRL2, LSM303D_CTRL2_2G_50HZ_BANDWIDTH},
        {LSM303D_REG_CTRL3, 0x00},
        {LSM303D_REG_CTRL4, 0x00},
        {LSM303D_REG_CTRL7, LSM303D_CTRL7_MAG_POWER_DOWN},
        {LSM303D_REG_CTRL1, LSM303D_CTRL1_100HZ_BDU_XYZ},
    };
    for (size_t index = 0; index < sizeof(settings) / sizeof(settings[0]); ++index) {
        if (imu->bus.write(imu->bus.context, LSM303D_ADDRESS,
                           settings[index], 2, false) != 2) {
            goto fail;
        }
    }

    return buddy4_verify_config(imu);

fail:
    ++imu->output.error_count;
    return false;
}

bool buddy4_verify_config(buddy4_imu_t *imu)
{
    buddy4_sensor_config_t *d = &imu->sensor_config;
    d->valid = false;
    const uint8_t registers[] = {0x1f, 0x20, 0x21, 0x22, 0x23, 0x26};
    const uint8_t expected[] = {0, 0x6f, 0xc0, 0, 0, 2};
    bool matches = read_registers(imu, LSM303D_REG_WHO_AM_I, &d->who_am_i, 1);
    if (!matches) goto fail;
    matches = d->who_am_i == LSM303D_WHO_AM_I_VALUE;
    for (unsigned i = 0; i < sizeof(registers); ++i) {
        if (!read_registers(imu, registers[i], &d->ctrl[i], 1)) goto fail;
        if (d->ctrl[i] != expected[i]) matches = false;
    }
    if (!matches) goto fail;
    d->valid = imu->ready = true;
    return true;
fail:
    ++imu->output.error_count;
    imu->ready = false;
    reset_detection(imu);
    return false;
}

buddy4_result_t buddy4_poll(buddy4_imu_t *imu, uint64_t timestamp_us)
{
    buddy4_output_t *output = &imu->output;
    invalidate_output(imu);
    if (!imu->ready) {
        return BUDDY4_NOT_READY;
    }

    uint8_t status;
    uint8_t bytes[6];
    if (!read_registers(imu, LSM303D_REG_STATUS_A, &status, 1)) {
        goto fail;
    }
    output->status_a = status;
    if (!(status & LSM303D_XYZ_DATA_READY)) {
        output->event.window_active = false;
        output->event.median_count = 0;
        return BUDDY4_STALE;
    }
    if (!read_registers(imu, LSM303D_REG_OUT_X_L_A, bytes, sizeof(bytes))) {
        goto fail;
    }

    /* Repeated/backwards timestamps cannot count toward peak stability. */
    if (imu->has_sample && timestamp_us <= output->timestamp_us) {
        output->event.window_active = false;
        output->event.median_count = 0;
        return BUDDY4_STALE;
    }
    bool sample_gap = imu->filter_valid &&
                      timestamp_us - output->timestamp_us > MAX_SAMPLE_GAP_US;
    if (sample_gap) {
        imu->filter_valid = false;
        imu->tilt_filter_valid = false;
    }

    output->clipped_axes = 0;
    for (int axis = 0; axis < 3; ++axis) {
        output->raw[axis] = buddy4_decode_signed16(bytes[2 * axis],
                                                  bytes[2 * axis + 1]);
        if (output->raw[axis] == INT16_MIN || output->raw[axis] == INT16_MAX)
            output->clipped_axes |= (uint8_t)(1u << axis);
        /* Preserve sensor axes and counts; printed arrows do not remap axes. */
        output->uncorrected_accel_g[axis] = output->raw[axis] * G_PER_COUNT;
        output->accel_g[axis] =
            (output->uncorrected_accel_g[axis] - imu->config.accel_bias_g[axis]) *
            imu->config.accel_scale[axis];
    }
    float accel_x = output->accel_g[0];
    float accel_y = output->accel_g[1];
    float accel_z = output->accel_g[2];
    output->magnitude_g = sqrtf(accel_x * accel_x + accel_y * accel_y +
                                accel_z * accel_z);

    output->acceleration_valid = !output->clipped_axes &&
        isfinite(accel_x) && isfinite(accel_y) && isfinite(accel_z) &&
        isfinite(output->magnitude_g);
    output->quality = output->clipped_axes ? BUDDY4_QUALITY_CLIPPED :
        !output->acceleration_valid ? BUDDY4_QUALITY_NONFINITE :
        output->magnitude_g < TILT_MIN_G || output->magnitude_g > TILT_MAX_G ?
            BUDDY4_QUALITY_NON_GRAVITY : BUDDY4_QUALITY_GRAVITY_LIKE;
    output->tilt_valid = output->quality == BUDDY4_QUALITY_GRAVITY_LIKE;
    if (output->clipped_axes && output->clipped_samples != UINT32_MAX)
        ++output->clipped_samples;
    if (!output->tilt_valid && output->unsuitable_samples != UINT32_MAX)
        ++output->unsuitable_samples;

    /* Gravity-based tilt; vehicle acceleration can disturb these estimates. */
    output->pitch_deg = imu->config.pitch_sign *
        atan2f(accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z)) *
        RAD_TO_DEG - imu->config.pitch_offset_deg;
    output->roll_deg = imu->config.roll_sign *
        atan2f(accel_y, sqrtf(accel_x * accel_x + accel_z * accel_z)) *
        RAD_TO_DEG - imu->config.roll_offset_deg;

    /* Clipped acceleration has unknown amplitude: break transient evidence too.
     * Non-clipped dynamic inputs keep the previous transient behaviour. */
    if (!output->acceleration_valid) imu->filter_valid = false;
    output->transient_g = 0;
    if (!output->acceleration_valid) {
        output->transient_event = false;
    } else if (!imu->filter_valid) {
        imu->baseline = output->magnitude_g;
        imu->filter_valid = true;
        imu->bump_armed = true;
    } else {
        output->transient_g = output->magnitude_g - imu->baseline;
        imu->baseline += BASELINE_ALPHA * output->transient_g;

        float transient_amplitude = fabsf(output->transient_g);
        if (transient_amplitude < BUMP_REARM_G) {
            imu->bump_armed = true;
        }
        if (imu->bump_armed && transient_amplitude >= BUMP_THRESHOLD_G &&
            (!imu->had_bump ||
             timestamp_us - imu->last_bump_us >= BUMP_COOLDOWN_US)) {
            output->transient_event = true;
            imu->bump_armed = false;
            imu->had_bump = true;
            imu->last_bump_us = timestamp_us;
        }
    }

    if (!output->tilt_valid) {
        /* Abort, do not complete an interrupted event or publish its peaks.
         * Completed history is separate and survives. Never blend bad tilt
         * into the recovery filter or bridge pending stability/median evidence. */
        imu->tilt_filter_valid = false;
        output->event = (side_event_t){0};
        output->terrain = output->side = 0; /* State invalid, not measured flat. */
        output->filtered_pitch_deg = output->filtered_roll_deg = NAN;
        output->wheel_height_cm = NAN;
    } else {
        if (!imu->tilt_filter_valid) {
            output->filtered_pitch_deg = output->pitch_deg;
            output->filtered_roll_deg = output->roll_deg;
            imu->tilt_filter_valid = true;
        } else {
            output->filtered_pitch_deg += ANGLE_ALPHA *
                (output->pitch_deg - output->filtered_pitch_deg);
            output->filtered_roll_deg += ANGLE_ALPHA *
                (output->roll_deg - output->filtered_roll_deg);
        }
        output->terrain = classify_angle(output->terrain, output->filtered_pitch_deg,
                                         PITCH_ENTER_DEG, PITCH_EXIT_DEG);
        output->side = classify_angle(output->side, output->filtered_roll_deg,
                                      ROLL_ENTER_DEG, ROLL_EXIT_DEG);
        /* Preliminary wheel-height difference; centre spacing remains configurable. */
        output->wheel_height_cm = imu->config.centre_spacing_cm *
            sinf(fabsf(output->filtered_roll_deg) * DEG_TO_RAD);
        output->height_geometry_confirmed = imu->config.centre_spacing_confirmed;

        if (output->event.side && output->event.side != output->side) {
            output->ended_event = output->event;
            output->event_ended = true;
        }
        update_side_event(&output->event, output->side, timestamp_us,
                          output->filtered_roll_deg, output->wheel_height_cm);

        if (output->event.side)
            update_median(&output->event, timestamp_us,
                          fabsf(output->filtered_roll_deg), imu->config.centre_spacing_cm);

    } /* tilt_valid */

    output->timestamp_us = timestamp_us;
    imu->has_sample = true;
    output->valid = true;
    return BUDDY4_FRESH;

fail:
    ++output->error_count;
    imu->ready = false;
    reset_detection(imu);
    return BUDDY4_IO_ERROR;
}

#include "../barrc_types.h"
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
#define TILT_MIN_G              0.7f
#define TILT_MAX_G              1.3f

#define ANGLE_ALPHA             0.30f

/* Provisional tilt thresholds with hysteresis */
#define PITCH_ENTER_DEG         4.0f
#define PITCH_EXIT_DEG          2.0f
#define ROLL_ENTER_DEG          2.5f
#define ROLL_EXIT_DEG           1.2f

/* Stable-window settings for confirmed peak estimates */
#define PEAK_CONFIRM_US         200000
#define PEAK_ALLOWED_CHANGE_DEG 1.0f
#define MAX_SAMPLE_GAP_US       (2 * BUDDY4_PERIOD_US)

/* Preliminary acceleration-transient detector */
#define BUMP_THRESHOLD_G        0.18f
#define BUMP_REARM_G            0.08f
#define BUMP_COOLDOWN_US        500000
#define BASELINE_ALPHA          0.05f

const char * buddy4_quality_name (int32_t quality)
{
    switch (quality)
    {
        case BUDDY4_QUALITY_GRAVITY_LIKE:
        return "GRAVITY_LIKE";
        case BUDDY4_QUALITY_CLIPPED:
        return "CLIPPED";
        case BUDDY4_QUALITY_NONFINITE:
        return "NONFINITE";
        case BUDDY4_QUALITY_NON_GRAVITY:
        return "NON_GRAVITY";
        default:
        return "UNAVAILABLE";
    }
}

static int32_t classify_angle (int32_t previous,
                           float32_t angle,
                           float32_t enter_deg,
                           float32_t exit_deg)
{
    if (angle >= enter_deg)
    {
        return 1;
    }

    if (angle <= -enter_deg)
    {
        return -1;
    }

    if ((1 == previous) && (angle > exit_deg))
    {
        return 1;
    }

    if ((-1 == previous) && (angle < -exit_deg))
    {
        return -1;
    }

    return 0;
}

const char * buddy4_terrain_name (int32_t state)
{
    if (state > 0)
    {
        return "UPHILL";
    }

    if (state < 0)
    {
        return "DOWNHILL";
    }

    return "FLAT";
}

const char * buddy4_side_name (int32_t state)
{
    /* Mapping established from your stationary tests */
    if (state > 0)
    {
        return "RIGHT_WHEEL_UP";
    }

    if (state < 0)
    {
        return "LEFT_WHEEL_UP";
    }

    return "SIDE_LEVEL";
}

static void finish_side_event (side_event_t * event,
                               uint64_t timestamp,
                               const char * p_reason)
{
    if (0 == event->side)
    {
        return;
    }

    (void)timestamp;
    (void)p_reason;
    *event = (side_event_t){0};
}

static void start_peak_window (side_event_t * event,
                               uint64_t timestamp,
                               float32_t absolute_roll,
                               float32_t height_cm)
{
    event->window_active = true;
    event->window_start_us = timestamp;
    event->last_sample_us = timestamp;
    event->window_min_roll = absolute_roll;
    event->window_max_roll = absolute_roll;
    event->window_min_height = height_cm;
}

/* Fixed-size chronological window; insertion sort costs at most 55 comparisons.
 */
static void update_median (side_event_t * event, uint64_t timestamp,
                           float32_t absolute_roll, float32_t spacing_cm)
{
    uint32_t n = event->median_count;

    if (n && ((timestamp <= event->median_time_us[n - 1]) ||
              (timestamp - event->median_time_us[n - 1] > MAX_SAMPLE_GAP_US)))
    {
        n = 0;
    }

    if (11 == n)
    {
        memmove(event->median_roll, event->median_roll + 1,
                10 * sizeof(float32_t));
        memmove(event->median_time_us, event->median_time_us + 1,
                10 * sizeof(uint64_t));
        n = 10;
    }

    event->median_roll[n] = absolute_roll;
    event->median_time_us[n] = timestamp;
    event->median_count = ++n;

    if (11 != n)
    {
        return;
    }

    uint64_t span = timestamp - event->median_time_us[0];

    if ((span < 190000) || (span > 210000))
    {
        return;
    }

    float32_t sorted[11];
    memcpy(sorted, event->median_roll, sizeof(sorted));

    for (uint32_t idx = 1; idx < 11; ++idx)
    {
        float32_t value = sorted[idx];
        uint32_t jdx = idx;

        while (jdx && (sorted[jdx - 1] > value))
        {
            sorted[jdx] = sorted[jdx - 1];
            --jdx;
        }

        sorted[jdx] = value;
    }

    if ((!event->median_candidate_valid)
        || (sorted[5] > event->median_candidate_roll_deg))
    {
        event->median_candidate_roll_deg = sorted[5];
        event->median_candidate_height_cm =
                spacing_cm * sinf(sorted[5] * DEG_TO_RAD);
    }

    event->median_candidate_valid = true;
}

static void update_side_event (side_event_t * event,
                               int32_t side,
                               uint64_t timestamp,
                               float32_t roll,
                               float32_t height_cm)
{
    if ((0 != event->side) && (side != event->side))
    {
        finish_side_event(
                          event,
                          timestamp,
                          0 == side ? "RETURNED_TO_LEVEL" : "SIDE_CHANGED"
                          );
    }

    if (0 == side)
    {
        return;
    }

    if (0 == event->side)
    {
        *event = (side_event_t){0};
        event->side = side;
        event->start_us = timestamp;
    }

    float32_t absolute_roll = fabsf(roll);

    /*
     * Require fresh consecutive samples.
     * A long gap restarts the confirmation window.
     */
    if ((!event->window_active) ||
        (timestamp - event->last_sample_us > MAX_SAMPLE_GAP_US))
    {
        start_peak_window(
                          event, timestamp, absolute_roll, height_cm
                          );
        return;
    }

    event->last_sample_us = timestamp;

    if (absolute_roll < event->window_min_roll)
    {
        event->window_min_roll = absolute_roll;
    }

    if (absolute_roll > event->window_max_roll)
    {
        event->window_max_roll = absolute_roll;
    }

    if (height_cm < event->window_min_height)
    {
        event->window_min_height = height_cm;
    }

    /*
     * Restart if roll changes too much within the window.
     */
    if (event->window_max_roll - event->window_min_roll >
        PEAK_ALLOWED_CHANGE_DEG)
    {
        start_peak_window(
                          event, timestamp, absolute_roll, height_cm
                          );
        return;
    }

    if (timestamp - event->window_start_us >= PEAK_CONFIRM_US)
    {
        /*
         * Use the minimum sustained value from a stable window.
         * This is a conservative confirmed estimate.
         */
        if ((!event->peak_confirmed) ||
            (event->window_min_roll > event->peak_roll_deg))
        {
            event->peak_roll_deg = event->window_min_roll;
        }

        if ((!event->peak_confirmed) ||
            (event->window_min_height > event->peak_height_cm))
        {
            event->peak_height_cm = event->window_min_height;
        }

        event->peak_confirmed = true;
        event->window_active = false;
    }
}

buddy4_config_t buddy4_default_config (void)
{
    return (buddy4_config_t)
    {
        /* Preliminary acceleration calibration */
        .accel_bias_g = {0.03799f, 0.03790f, 0.08411f},
        .accel_scale = {1.01301f, 1.03419f, 1.01223f},
        .mounting_offsets_need_recalibration = false,
        .pitch_sign = 1.0f,
        .pitch_offset_deg = 0.97f,
        .roll_sign = 1.0f,
        /* Provisional mounting offset; pending flat/1 cm ramp hardware
         * verification. */
        .roll_offset_deg = 0.61f,
        .centre_spacing_cm = 11.3f,
        .centre_spacing_confirmed = true,
    };
}

void buddy4_init (buddy4_imu_t * p_imu, buddy4_bus_t bus,
                  buddy4_config_t config)
{
    memset(p_imu, 0, sizeof(*p_imu));
    p_imu->bus = bus;
    p_imu->config = config;
    p_imu->bump_armed = true;
}

int16_t buddy4_decode_signed16 (uint8_t low, uint8_t high)
{
    uint16_t word = (uint16_t)low | ((uint16_t)high << 8);
    return (int16_t)((word & 0x8000u) ? (int32_t)word - 65536 : (int32_t)word);
}

static bool read_registers (buddy4_imu_t * p_imu, uint8_t reg,
                            uint8_t * data, size_t count)
{
    if (count > 1)
    {
        reg |= LSM303D_AUTO_INCREMENT;
    }

    return (p_imu->bus.write) && (p_imu->bus.read) &&
            (1 == p_imu->bus.write(p_imu->bus.context, LSM303D_ADDRESS,
                                   &reg, 1, true)) &&
            (p_imu->bus.read(p_imu->bus.context, LSM303D_ADDRESS,
                           data, count, false) == (int32_t)count);
}

/* Clear per-call flags while retaining the last fresh measurements. */
static void invalidate_output (buddy4_imu_t * p_imu)
{
    p_imu->output.valid = false;
    p_imu->output.acceleration_valid = false;
    p_imu->output.tilt_valid = false;
    p_imu->output.quality = BUDDY4_QUALITY_UNAVAILABLE;
    p_imu->output.transient_event = false;
    p_imu->output.event_ended = false;
}

/* Abort interrupted detection without producing a completed-event pulse.
 * Keep the last measurements/timestamp and cumulative error count for
 * diagnostics. */
static void reset_detection (buddy4_imu_t * p_imu)
{
    invalidate_output(p_imu);
    p_imu->filter_valid = false;
    p_imu->tilt_filter_valid = false;
    p_imu->has_sample = false;
    p_imu->baseline = 0.0f;
    p_imu->bump_armed = true;
    p_imu->had_bump = false;
    p_imu->last_bump_us = 0;
    p_imu->output.transient_g = 0.0f;
    p_imu->output.event = (side_event_t){0};
    /* Retain the last completed event across errors/reconfiguration. */
    p_imu->output.terrain = 0;
    p_imu->output.side = 0;
}

bool buddy4_configure (buddy4_imu_t * p_imu)
{
    p_imu->ready = false;
    p_imu->sensor_config.valid = false;
    reset_detection(p_imu);

    if ((!isfinite(p_imu->config.centre_spacing_cm)) ||
        (p_imu->config.centre_spacing_cm <= 0) ||
        (!isfinite(p_imu->config.pitch_offset_deg)) ||
        (!isfinite(p_imu->config.roll_offset_deg)) ||
        (1 != fabsf(p_imu->config.pitch_sign)) ||
        (1 != fabsf(p_imu->config.roll_sign)))
    {
        goto fail;
    }

    for (int32_t axis = 0; axis < 3; ++axis)
    {
        if ((!isfinite(p_imu->config.accel_bias_g[axis])) ||
            (!isfinite(p_imu->config.accel_scale[axis])) ||
            (p_imu->config.accel_scale[axis] <= 0.0f))
        {
            goto fail;
        }
    }

    if ((!read_registers(p_imu, LSM303D_REG_WHO_AM_I,
                         &p_imu->sensor_config.who_am_i,
                         1)) ||
        (LSM303D_WHO_AM_I_VALUE != p_imu->sensor_config.who_am_i))
    {
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

    for (size_t index = 0; index < sizeof(settings)
         / sizeof(settings[0]); ++index)
    {
        if (p_imu->bus.write(p_imu->bus.context, LSM303D_ADDRESS,
                           settings[index], 2, false) != 2)
        {
            goto fail;
        }
    }

    return buddy4_verify_config(p_imu);

    fail:
            ++p_imu->output.error_count;
    return false;
}

bool buddy4_verify_config (buddy4_imu_t * p_imu)
{
    buddy4_sensor_config_t * d = &p_imu->sensor_config;
    d->valid = false;
    const uint8_t registers[] = {0x1f, 0x20, 0x21, 0x22, 0x23, 0x26};
    const uint8_t expected[] = {0, 0x6f, 0xc0, 0, 0, 2};
    bool b_matches =
            read_registers(p_imu, LSM303D_REG_WHO_AM_I, &d->who_am_i, 1);

    if (!b_matches)
    {
        goto fail;
    }

    b_matches = LSM303D_WHO_AM_I_VALUE == d->who_am_i;

    for (uint32_t idx = 0; idx < sizeof(registers); ++idx)
    {
        if (!read_registers(p_imu, registers[idx], &d->ctrl[idx], 1))
        {
            goto fail;
        }

        if (d->ctrl[idx] != expected[idx])
        {
            b_matches = false;
        }
    }

    if (!b_matches)
    {
        goto fail;
    }

    d->valid = p_imu->ready = true;
    return true;
    fail:
            ++p_imu->output.error_count;
    p_imu->ready = false;
    reset_detection(p_imu);
    return false;
}

buddy4_result_t buddy4_poll (buddy4_imu_t * p_imu, uint64_t timestamp_us)
{
    buddy4_output_t * output = &p_imu->output;
    invalidate_output(p_imu);

    if (!p_imu->ready)
    {
        return BUDDY4_NOT_READY;
    }

    uint8_t status;
    uint8_t bytes[6];

    if (!read_registers(p_imu, LSM303D_REG_STATUS_A, &status, 1))
    {
        goto fail;
    }

    output->status_a = status;

    if (!(status & LSM303D_XYZ_DATA_READY))
    {
        output->event.window_active = false;
        output->event.median_count = 0;
        return BUDDY4_STALE;
    }

    if (!read_registers(p_imu, LSM303D_REG_OUT_X_L_A, bytes, sizeof(bytes)))
    {
        goto fail;
    }

    /* Repeated/backwards timestamps cannot count toward peak stability. */
    if ((p_imu->has_sample) && (timestamp_us <= output->timestamp_us))
    {
        output->event.window_active = false;
        output->event.median_count = 0;
        return BUDDY4_STALE;
    }

    bool b_sample_gap = (p_imu->filter_valid) &&
            (timestamp_us - output->timestamp_us > MAX_SAMPLE_GAP_US);

    if (b_sample_gap)
    {
        p_imu->filter_valid = false;
        p_imu->tilt_filter_valid = false;
    }

    output->clipped_axes = 0;

    for (int32_t axis = 0; axis < 3; ++axis)
    {
        output->raw[axis] = buddy4_decode_signed16(bytes[2 * axis],
                bytes[2 * axis + 1]);

        if ((INT16_MIN == output->raw[axis])
            || (INT16_MAX == output->raw[axis]))
        {
            output->clipped_axes |= (uint8_t)(1u << axis);
        }

        /* Preserve sensor axes and counts; printed arrows do not remap axes. */
        output->uncorrected_accel_g[axis] = output->raw[axis] * G_PER_COUNT;
        output->accel_g[axis] =
                (output->uncorrected_accel_g[axis]
                 - p_imu->config.accel_bias_g[axis]) *
                p_imu->config.accel_scale[axis];
    }

    float32_t accel_x = output->accel_g[0];
    float32_t accel_y = output->accel_g[1];
    float32_t accel_z = output->accel_g[2];
    output->magnitude_g = sqrtf(accel_x * accel_x + accel_y * accel_y +
                                accel_z * accel_z);

    output->acceleration_valid = (!output->clipped_axes) &&
            (isfinite(accel_x)) && (isfinite(accel_y)) && (isfinite(accel_z)) &&
            (isfinite(output->magnitude_g));
    output->quality = output->clipped_axes ? BUDDY4_QUALITY_CLIPPED :
    !output->acceleration_valid ? BUDDY4_QUALITY_NONFINITE :
    (output->magnitude_g < TILT_MIN_G) || (output->magnitude_g > TILT_MAX_G) ?
            BUDDY4_QUALITY_NON_GRAVITY : BUDDY4_QUALITY_GRAVITY_LIKE;
    output->tilt_valid = BUDDY4_QUALITY_GRAVITY_LIKE == output->quality;

    if ((output->clipped_axes) && (UINT32_MAX != output->clipped_samples))
    {
        ++output->clipped_samples;
    }

    if ((!output->tilt_valid) && (UINT32_MAX != output->unsuitable_samples))
    {
        ++output->unsuitable_samples;
    }

    /* Gravity-based tilt; vehicle acceleration can disturb these estimates. */
    output->pitch_deg = p_imu->config.pitch_sign *
            atan2f(accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z)) *
            RAD_TO_DEG - p_imu->config.pitch_offset_deg;
    output->roll_deg = p_imu->config.roll_sign *
            atan2f(accel_y, sqrtf(accel_x * accel_x + accel_z * accel_z)) *
            RAD_TO_DEG - p_imu->config.roll_offset_deg;

    /* Clipped acceleration has unknown amplitude: break transient evidence too.
     * Non-clipped dynamic inputs keep the previous transient behaviour. */
    if (!output->acceleration_valid)
    {
        p_imu->filter_valid = false;
    }

    output->transient_g = 0;

    if (!output->acceleration_valid)
    {
        output->transient_event = false;
    }
    else if (!p_imu->filter_valid)
    {
        p_imu->baseline = output->magnitude_g;
        p_imu->filter_valid = true;
        p_imu->bump_armed = true;
    }
    else
    {
        output->transient_g = output->magnitude_g - p_imu->baseline;
        p_imu->baseline += BASELINE_ALPHA * output->transient_g;

        float32_t transient_amplitude = fabsf(output->transient_g);

        if (transient_amplitude < BUMP_REARM_G)
        {
            p_imu->bump_armed = true;
        }

        if ((p_imu->bump_armed) && (transient_amplitude >= BUMP_THRESHOLD_G) &&
            ((!p_imu->had_bump) ||
             (timestamp_us - p_imu->last_bump_us >= BUMP_COOLDOWN_US)))
        {
            output->transient_event = true;
            p_imu->bump_armed = false;
            p_imu->had_bump = true;
            p_imu->last_bump_us = timestamp_us;
        }
    }

    if (!output->tilt_valid)
    {
        /* Abort, do not complete an interrupted event or publish its peaks.
         * Completed history is separate and survives. Never blend bad tilt
         * into the recovery filter or bridge pending stability/median evidence.
         */
        p_imu->tilt_filter_valid = false;
        output->event = (side_event_t){0};
        /* State invalid, not measured flat. */
        output->terrain = output->side = 0;
        output->filtered_pitch_deg = output->filtered_roll_deg = NAN;
        output->wheel_height_cm = NAN;
    }
    else
    {
        if (!p_imu->tilt_filter_valid)
        {
            output->filtered_pitch_deg = output->pitch_deg;
            output->filtered_roll_deg = output->roll_deg;
            p_imu->tilt_filter_valid = true;
        }
        else
        {
            output->filtered_pitch_deg += ANGLE_ALPHA *
                    (output->pitch_deg - output->filtered_pitch_deg);
            output->filtered_roll_deg += ANGLE_ALPHA *
                    (output->roll_deg - output->filtered_roll_deg);
        }

        output->terrain =
                classify_angle(output->terrain, output->filtered_pitch_deg,
                               PITCH_ENTER_DEG, PITCH_EXIT_DEG);
        output->side = classify_angle(output->side, output->filtered_roll_deg,
                                      ROLL_ENTER_DEG, ROLL_EXIT_DEG);
        /* Preliminary wheel-height difference; centre spacing remains
         * configurable. */
        output->wheel_height_cm = p_imu->config.centre_spacing_cm *
                sinf(fabsf(output->filtered_roll_deg) * DEG_TO_RAD);
        output->height_geometry_confirmed =
                p_imu->config.centre_spacing_confirmed;

        if ((output->event.side) && (output->event.side != output->side))
        {
            output->ended_event = output->event;
            output->event_ended = true;
        }

        update_side_event(&output->event, output->side, timestamp_us,
                          output->filtered_roll_deg, output->wheel_height_cm);

        if (output->event.side)
        {
            update_median(&output->event, timestamp_us,
                          fabsf(output->filtered_roll_deg),
                          p_imu->config.centre_spacing_cm);
        }

    } /* tilt_valid */

    output->timestamp_us = timestamp_us;
    p_imu->has_sample = true;
    output->valid = true;
    return BUDDY4_FRESH;

    fail:
            ++output->error_count;
    p_imu->ready = false;
    reset_detection(p_imu);
    return BUDDY4_IO_ERROR;
}

/*** end of file ***/

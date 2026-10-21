#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/i2c.h"

/* Pico W connections and detected LSM303D address */
#define IMU_I2C             i2c0
#define SDA_PIN             16
#define SCL_PIN             17
#define SENSOR_ADDR         0x1D
#define I2C_TIMEOUT_US      5000

/* LSM303D registers */
#define WHO_AM_I            0x0F
#define CTRL0               0x1F
#define CTRL1               0x20
#define CTRL2               0x21
#define CTRL3               0x22
#define CTRL4               0x23
#define CTRL7               0x26
#define STATUS_A            0x27
#define OUT_X_L_A           0x28

/* Sensor at 100 Hz; CSV logging scheduled at 50 Hz */
#define SAMPLE_PERIOD_US    20000
#define DATA_TIMEOUT_US     15000

/* +/-2 g, full signed 16-bit output */
#define G_PER_COUNT         0.000061f
#define RAD_TO_DEG          57.2957795131f

/* Calibrated for your current sensor mounting */
#define PITCH_SIGN          (+1.0f)
#define PITCH_OFFSET_DEG    2.50f
#define ROLL_SIGN           (+1.0f)
#define ROLL_OFFSET_DEG     2.85f

#define ANGLE_ALPHA         0.15f

/* Initial pitch thresholds */
#define PITCH_ENTER_DEG     6.0f
#define PITCH_EXIT_DEG      3.0f

/* Initial sideways thresholds from your stationary tests */
#define ROLL_ENTER_DEG      4.0f
#define ROLL_EXIT_DEG       2.0f

/* Preliminary bump/impact detection; does not measure height */
#define BUMP_THRESHOLD_G    0.18f
#define BUMP_REARM_G        0.08f
#define BUMP_COOLDOWN_US    500000
#define BASELINE_ALPHA      0.05f

static bool write_reg(uint8_t reg, uint8_t value)
{
    uint8_t bytes[2] = {reg, value};

    return i2c_write_timeout_us(
        IMU_I2C, SENSOR_ADDR, bytes, 2, false,
        I2C_TIMEOUT_US
    ) == 2;
}

static bool read_regs(uint8_t reg, uint8_t *data, size_t count)
{
    if (count > 1) {
        reg |= 0x80; /* Enable auto-increment. */
    }

    if (i2c_write_timeout_us(
            IMU_I2C, SENSOR_ADDR, &reg, 1, true,
            I2C_TIMEOUT_US) != 1) {
        return false;
    }

    return i2c_read_timeout_us(
        IMU_I2C, SENSOR_ADDR, data, count, false,
        I2C_TIMEOUT_US
    ) == (int)count;
}

static bool configure_sensor(void)
{
    uint8_t id = 0;

    if (!read_regs(WHO_AM_I, &id, 1)) {
        printf("# ERROR: cannot read sensor at 0x1D\n");
        return false;
    }

    if (id != 0x49) {
        printf("# ERROR: expected ID 0x49, received 0x%02X\n",
               (unsigned)id);
        return false;
    }

    /*
     * Stop sampling during configuration.
     * Disable FIFO and interrupt outputs.
     * Select +/-2 g and 50 Hz anti-alias bandwidth.
     * Retain gravity in acceleration output.
     * Power down magnetometer.
     * Start at 100 Hz, block data update, XYZ enabled.
     */
    static const uint8_t settings[][2] = {
        {CTRL1, 0x00},
        {CTRL0, 0x00},
        {CTRL2, 0xC0},
        {CTRL3, 0x00},
        {CTRL4, 0x00},
        {CTRL7, 0x02},
        {CTRL1, 0x6F}
    };

    for (size_t i = 0;
         i < sizeof(settings) / sizeof(settings[0]);
         ++i) {
        if (!write_reg(settings[i][0], settings[i][1])) {
            printf("# ERROR: write failed at register 0x%02X\n",
                   (unsigned)settings[i][0]);
            return false;
        }
    }

    const uint8_t regs[] = {CTRL1, CTRL2, CTRL7};
    const uint8_t expected[] = {0x6F, 0xC0, 0x02};

    for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]); ++i) {
        uint8_t value = 0;

        if (!read_regs(regs[i], &value, 1)) {
            printf("# ERROR: configuration read failed\n");
            return false;
        }

        if (value != expected[i]) {
            printf("# ERROR: register 0x%02X expected 0x%02X got 0x%02X\n",
                   (unsigned)regs[i],
                   (unsigned)expected[i],
                   (unsigned)value);
            return false;
        }
    }

    sleep_ms(100);
    return true;
}

static int16_t decode_signed16(uint8_t low, uint8_t high)
{
    uint16_t word = (uint16_t)low | ((uint16_t)high << 8);

    int32_t value = (word & 0x8000u)
                  ? (int32_t)word - 65536
                  : (int32_t)word;

    return (int16_t)value;
}

/* Returns 0 on success, 1 on I2C failure, 2 on data timeout. */
static int read_acceleration(int16_t raw[3])
{
    uint64_t deadline = time_us_64() + DATA_TIMEOUT_US;

    while (true) {
        uint8_t status = 0;

        if (!read_regs(STATUS_A, &status, 1)) {
            return 1;
        }

        if ((status & 0x08u) != 0) {
            break;
        }

        if (time_us_64() >= deadline) {
            return 2;
        }

        sleep_us(500);
    }

    uint8_t bytes[6];

    if (!read_regs(OUT_X_L_A, bytes, sizeof(bytes))) {
        return 1;
    }

    /* LSM303D uses full 16-bit data: no right shift. */
    raw[0] = decode_signed16(bytes[0], bytes[1]);
    raw[1] = decode_signed16(bytes[2], bytes[3]);
    raw[2] = decode_signed16(bytes[4], bytes[5]);

    return 0;
}

/* Hysteresis classifier: +1, 0 or -1. */
static int classify_angle(int previous, float angle,
                          float enter_deg, float exit_deg)
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

static const char *terrain_name(int state)
{
    if (state > 0) {
        return "UPHILL";
    }

    if (state < 0) {
        return "DOWNHILL";
    }

    return "FLAT";
}

static const char *side_name(int state)
{
    /*
     * Mapping established from your labelled recordings.
     * Valid while the sensor mounting and signs remain unchanged.
     */
    if (state > 0) {
        return "RIGHT_WHEEL_UP";
    }

    if (state < 0) {
        return "LEFT_WHEEL_UP";
    }

    return "SIDE_LEVEL";
}

int main(void)
{
    stdio_init_all();

    i2c_init(IMU_I2C, 100000);

    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    uint32_t error_count = 0;

    while (true) {
        while (!stdio_usb_connected()) {
            sleep_ms(100);
        }

        sleep_ms(250);

        if (!configure_sensor()) {
            sleep_ms(1000);
            continue;
        }

        printf("# LSM303D ready: SDA=GP16 SCL=GP17 address=0x1D\n");

        printf("seq,time_us,raw_x,raw_y,raw_z,"
               "ax_g,ay_g,az_g,mag_g,"
               "pitch_deg,pitch_filtered_deg,"
               "roll_deg,roll_filtered_deg,"
               "terrain,side_tilt,transient_g,bump_event,error_count\n");

        uint32_t sequence = 0;
        unsigned consecutive_errors = 0;

        bool filter_valid = false;
        bool bump_armed = true;
        bool had_bump = false;

        float filtered_pitch = 0.0f;
        float filtered_roll = 0.0f;
        float baseline = 1.0f;

        int terrain_state = 0;
        int side_state = 0;

        uint64_t last_bump_us = 0;
        uint64_t next_sample_us = time_us_64();

        while (stdio_usb_connected()) {
            sleep_until(from_us_since_boot(next_sample_us));

            uint64_t timestamp = time_us_64();
            next_sample_us += SAMPLE_PERIOD_US;

            int16_t raw[3];
            int error = read_acceleration(raw);

            if (error != 0) {
                ++error_count;
                ++consecutive_errors;

                printf("# READ_ERROR,time_us=%llu,code=%d,total=%lu\n",
                       (unsigned long long)timestamp,
                       error,
                       (unsigned long)error_count);

                filter_valid = false;
                terrain_state = 0;
                side_state = 0;
                bump_armed = true;

                if (consecutive_errors >= 5) {
                    printf("# Reinitializing sensor\n");
                    break;
                }
            } else {
                consecutive_errors = 0;

                float ax = raw[0] * G_PER_COUNT;
                float ay = raw[1] * G_PER_COUNT;
                float az = raw[2] * G_PER_COUNT;

                float magnitude = sqrtf(
                    ax * ax + ay * ay + az * az
                );

                /*
                 * Gravity-based chassis tilt estimates.
                 * Suitable for the current mounting, including
                 * its downward-facing Z axis.
                 * Acceleration and braking can affect these angles.
                 */
                float pitch = PITCH_SIGN
                            * atan2f(ax, sqrtf(ay * ay + az * az))
                            * RAD_TO_DEG
                            - PITCH_OFFSET_DEG;

                float roll = ROLL_SIGN
                           * atan2f(ay, sqrtf(ax * ax + az * az))
                           * RAD_TO_DEG
                           - ROLL_OFFSET_DEG;

                float transient = 0.0f;
                int bump_event = 0;

                if (!filter_valid) {
                    filtered_pitch = pitch;
                    filtered_roll = roll;
                    baseline = magnitude;
                    filter_valid = true;
                } else {
                    filtered_pitch += ANGLE_ALPHA
                                    * (pitch - filtered_pitch);

                    filtered_roll += ANGLE_ALPHA
                                   * (roll - filtered_roll);

                    transient = magnitude - baseline;
                    baseline += BASELINE_ALPHA * transient;

                    float amplitude = fabsf(transient);

                    if (amplitude < BUMP_REARM_G) {
                        bump_armed = true;
                    }

                    if (bump_armed &&
                        amplitude >= BUMP_THRESHOLD_G &&
                        (!had_bump ||
                         timestamp - last_bump_us >= BUMP_COOLDOWN_US)) {
                        bump_event = 1;
                        bump_armed = false;
                        had_bump = true;
                        last_bump_us = timestamp;
                    }
                }

                terrain_state = classify_angle(
                    terrain_state,
                    filtered_pitch,
                    PITCH_ENTER_DEG,
                    PITCH_EXIT_DEG
                );

                side_state = classify_angle(
                    side_state,
                    filtered_roll,
                    ROLL_ENTER_DEG,
                    ROLL_EXIT_DEG
                );

                printf("%lu,%llu,%d,%d,%d,"
                       "%.4f,%.4f,%.4f,%.4f,"
                       "%.2f,%.2f,%.2f,%.2f,"
                       "%s,%s,%.4f,%d,%lu\n",
                       (unsigned long)sequence++,
                       (unsigned long long)timestamp,
                       (int)raw[0],
                       (int)raw[1],
                       (int)raw[2],
                       ax,
                       ay,
                       az,
                       magnitude,
                       pitch,
                       filtered_pitch,
                       roll,
                       filtered_roll,
                       terrain_name(terrain_state),
                       side_name(side_state),
                       transient,
                       bump_event,
                       (unsigned long)error_count);
            }

            /* Skip missed slots instead of printing catch-up bursts. */
            uint64_t now = time_us_64();

            if (now > next_sample_us) {
                next_sample_us +=
                    ((now - next_sample_us) / SAMPLE_PERIOD_US + 1)
                    * SAMPLE_PERIOD_US;
            }
        }
    }

    return 0;
}
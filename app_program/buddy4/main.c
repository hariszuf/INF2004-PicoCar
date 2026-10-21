#include "buddy4_imu.h"
#include "hardware/i2c.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

#include <stdio.h>

#define IMU_I2C_BUS i2c0
#define IMU_SDA_PIN 16
#define IMU_SCL_PIN 17
#define IMU_I2C_BAUD_HZ 100000
#define CONFIG_RETRY_US 1000000

static int bus_write(void *context, uint8_t address, const uint8_t *data,
                     size_t count, bool no_stop)
{
    (void)context;
    return i2c_write_timeout_us(IMU_I2C_BUS, address, data, count, no_stop,
                                BUDDY4_I2C_TIMEOUT_US);
}

static int bus_read(void *context, uint8_t address, uint8_t *data,
                    size_t count, bool no_stop)
{
    (void)context;
    return i2c_read_timeout_us(IMU_I2C_BUS, address, data, count, no_stop,
                               BUDDY4_I2C_TIMEOUT_US);
}

/* Reprint on each observed USB connection so a new recording has a schema. */
static void log_csv_header(const buddy4_config_t *config)
{
    printf("# preliminary_six_position,imperfect_orientation_alignment,"
           "mounting_offsets_need_recalibration=%d\n",
           config->mounting_offsets_need_recalibration);
    printf("seq,poll_time_us,result,valid,sample_time_us,error_count,"
           "raw_x,raw_y,raw_z,ax_uncorrected_g,ay_uncorrected_g,az_uncorrected_g,"
           "ax_corrected_g,ay_corrected_g,az_corrected_g,magnitude_g,"
           "pitch_deg,pitch_filtered_deg,roll_deg,roll_filtered_deg,"
           "terrain,side_tilt,transient_g,transient_event,"
           "wheel_height_cm,geometry_confirmed,side_event_active,"
           "peak_confirmed,peak_roll_deg,peak_wheel_height_cm\n");
}

/* Keep event metadata out of the fixed-column sample rows. */
static void log_side_event_summary(const buddy4_output_t *output)
{
    const side_event_t *event = &output->ended_event;
    printf("# SIDE_EVENT_END,side=%s,reason=%s,start_us=%llu,end_us=%llu,"
           "peak_confirmed=%d,peak_roll_deg=%.2f,peak_wheel_height_cm=%.2f\n",
           buddy4_side_name(event->side),
           output->side ? "SIDE_CHANGED" : "RETURNED_TO_LEVEL",
           (unsigned long long)event->start_us,
           (unsigned long long)output->timestamp_us,
           event->peak_confirmed,
           event->peak_roll_deg,
           event->peak_height_cm);
}

/* All results use the same columns. Invalid rows retain the last fresh values;
 * valid and sample_time_us distinguish these from a new measurement. */
static void log_csv_sample(uint32_t sequence, uint64_t poll_time_us,
                           buddy4_result_t result, const buddy4_output_t *output)
{
    printf("%lu,%llu,%d,%d,%llu,%lu,"
           "%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
           "%.2f,%.2f,%.2f,%.2f,%s,%s,%.4f,%d,"
           "%.2f,%d,%d,%d,%.2f,%.2f\n",
           (unsigned long)sequence,
           (unsigned long long)poll_time_us,
           (int)result,
           output->valid,
           (unsigned long long)output->timestamp_us,
           (unsigned long)output->error_count,
           (int)output->raw[0],
           (int)output->raw[1],
           (int)output->raw[2],
           output->uncorrected_accel_g[0],
           output->uncorrected_accel_g[1],
           output->uncorrected_accel_g[2],
           output->accel_g[0],
           output->accel_g[1],
           output->accel_g[2],
           output->magnitude_g,
           output->pitch_deg,
           output->filtered_pitch_deg,
           output->roll_deg,
           output->filtered_roll_deg,
           buddy4_terrain_name(output->terrain),
           buddy4_side_name(output->side),
           output->transient_g,
           output->transient_event,
           output->wheel_height_cm,
           output->height_geometry_confirmed,
           output->event.side != 0,
           output->event.peak_confirmed,
           output->event.peak_roll_deg,
           output->event.peak_height_cm);
}

/* Standalone sensor monitor: sampling never depends on a USB connection. */
int main(void)
{
    stdio_init_all();
    i2c_init(IMU_I2C_BUS, IMU_I2C_BAUD_HZ);
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);

    buddy4_imu_t imu;
    const buddy4_bus_t bus = {
        .context = NULL,
        .write = bus_write,
        .read = bus_read,
    };
    buddy4_init(&imu, bus, buddy4_default_config());

    uint64_t next_sample_us = time_us_64();
    uint64_t next_config_retry_us = 0;
    uint32_t sequence = 0;
    bool usb_was_connected = false;

    while (true) {
        sleep_until(from_us_since_boot(next_sample_us));
        uint64_t now_us = time_us_64();
        if (!imu.ready && now_us >= next_config_retry_us) {
            buddy4_configure(&imu);
            next_config_retry_us = now_us + CONFIG_RETRY_US;
        }
        buddy4_result_t result = buddy4_poll(&imu, now_us);

        /* USB writes have a bounded SDK timeout; logging is best effort. */
        bool usb_connected = stdio_usb_connected();
        if (usb_connected) {
            if (!usb_was_connected) {
                log_csv_header(&imu.config);
            }
            if (imu.output.event_ended) {
                log_side_event_summary(&imu.output);
            }
            log_csv_sample(sequence++, now_us, result, &imu.output);
        }
        usb_was_connected = usb_connected;

        /* Skip missed slots instead of producing catch-up sample bursts. */
        next_sample_us += BUDDY4_PERIOD_US;
        now_us = time_us_64();
        if (now_us >= next_sample_us) {
            next_sample_us +=
                ((now_us - next_sample_us) / BUDDY4_PERIOD_US + 1) *
                BUDDY4_PERIOD_US;
        }
    }
}

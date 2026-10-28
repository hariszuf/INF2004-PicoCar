/**
 * @file line_follow_demo_main.c
 * @brief Autonomous line following demo (no RTOS, no remote control).
 *
 * After power-on the car waits LINE_DEMO_START_DELAY_MS (time to put it on the
 * black line), then follows the line by itself. If the line is lost for longer
 * than LINE_RECOVER_MS it stops (it resumes if the line comes back). Switch the Robo Pico off to abort.
 *
 * Buddy 4's IMU (LSM303D on I2C0, GP16/GP17) is polled every 20 ms and its
 * slope / hump report is passed to Buddy 2 (motion_set_terrain()), which
 * slows down or adds power automatically. If the IMU is missing or fails,
 * Buddy 2 simply drives as on flat ground.
 *
 * Progress is printed over USB serial if a terminal happens to be connected,
 * which is optional.
 */
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "buddy4_imu.h"
#include "terrain_link.h"

#include "line_config.h"
#include "line_follow.h"
#include "line_sensors.h"
#include "motion_config.h"
#include "motion_control.h"

#define LINE_DEMO_START_DELAY_MS    (5000u)
#define LINE_DEMO_PRINT_MS          (200u)
#define IMU_I2C                     (i2c0)
#define IMU_I2C_HZ                  (100000u)
#define IMU_PIN_SDA                 (16u)
#define IMU_PIN_SCL                 (17u)
#define IMU_PERIOD_MS               (BUDDY4_PERIOD_US / 1000u)
#define IMU_RETRY_MS                (1000u)

static int imu_write(void *ctx, uint8_t addr, const uint8_t *p_data, size_t count, bool no_stop)
{
    (void)ctx;
    return i2c_write_timeout_us(IMU_I2C, addr, p_data, count, no_stop, BUDDY4_I2C_TIMEOUT_US);
}

static int imu_read(void *ctx, uint8_t addr, uint8_t *p_data, size_t count, bool no_stop)
{
    (void)ctx;
    return i2c_read_timeout_us(IMU_I2C, addr, p_data, count, no_stop, BUDDY4_I2C_TIMEOUT_US);
}

int main(void)
{
    uint32_t now_ms;
    uint32_t next_update_ms;
    uint32_t next_line_ms;
    uint32_t next_print_ms;
    uint32_t next_imu_ms;
    uint32_t next_imu_retry_ms;
    line_status_t status;
    motion_status_t motor;
    static buddy4_imu_t imu;
    buddy4_bus_t bus = { NULL, imu_write, imu_read };
    buddy4_result_t imu_result = BUDDY4_NOT_READY;

    stdio_init_all();

    /* Buddy 4 IMU: set up during the start delay. */
    i2c_init(IMU_I2C, IMU_I2C_HZ);
    gpio_set_function(IMU_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(IMU_PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_PIN_SDA);
    gpio_pull_up(IMU_PIN_SCL);
    buddy4_init(&imu, bus, buddy4_default_config());
    (void)buddy4_configure(&imu);

    sleep_ms(LINE_DEMO_START_DELAY_MS);

    (void)motion_init();
    (void)motion_reset_odometry();
    line_follow_init();
    (void)line_follow_start();

    now_ms = to_ms_since_boot(get_absolute_time());
    next_update_ms = now_ms;
    next_line_ms = now_ms;
    next_print_ms = now_ms;
    next_imu_ms = now_ms;
    next_imu_retry_ms = now_ms + IMU_RETRY_MS;

    for (;;)
    {
        now_ms = to_ms_since_boot(get_absolute_time());

        if (now_ms >= next_update_ms)
        {
            next_update_ms += MOTION_UPDATE_PERIOD_MS;
            motion_update();
        }

        if (now_ms >= next_imu_ms)
        {
            next_imu_ms += IMU_PERIOD_MS;
            if ((!imu.ready) && (now_ms >= next_imu_retry_ms))
            {
                next_imu_retry_ms = now_ms + IMU_RETRY_MS;
                (void)buddy4_configure(&imu);   /* bounded I2C transfers */
            }
            imu_result = buddy4_poll(&imu, time_us_64());
            terrain_link_feed(&imu, imu_result);   /* Buddy 4 -> Buddy 2 */
        }

        if (now_ms >= next_line_ms)
        {
            next_line_ms += LINE_UPDATE_PERIOD_MS;
            line_follow_update();
        }

        if (now_ms >= next_print_ms)
        {
            next_print_ms += LINE_DEMO_PRINT_MS;
            line_follow_get_status(&status);
            motion_get_status(&motor);
            printf("[IR] GP0=%d GP2=%d GP26=%d | state=%d | "
                   "target L/R=%.1f/%.1f | actual L/R=%.1f/%.1f | fault=%d | "
                   "imu=%d pitch=%.1f roll=%.1f terrain=%d scale=%.2f assist L/R=%.2f/%.2f\n",
                   gpio_get(LINE_PIN_GP0_IR) != 0,
                   gpio_get(LINE_PIN_GP2_IR) != 0,
                   gpio_get(LINE_PIN_BARCODE_IR) != 0,
                   (int)status.state,
                   (double)status.left_cmd_cm_s,
                   (double)status.right_cmd_cm_s,
                   (double)motor.left_speed_cm_s,
                   (double)motor.right_speed_cm_s,
                   (int)motor.fault,
                   (int)(imu.ready && imu.output.valid),
                   (double)imu.output.filtered_pitch_deg,
                   (double)imu.output.filtered_roll_deg,
                   (int)motor.terrain,
                   (double)motor.terrain_speed_scale,
                   (double)motor.left_terrain_duty,
                   (double)motor.right_terrain_duty);
        }
    }
}

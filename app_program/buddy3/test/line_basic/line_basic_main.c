/**
 * @file line_basic_main.c
 * @brief Basic line-following test: Buddy 2 motion + Buddy 3 line following. Nothing else.
 *
 * No barcode, no IMU. After power-on it waits START_DELAY_MS (place the car on the black
 * line, both IR sensors over the black), then follows the line by itself.
 * USB serial (115200) is optional and prints a status line every 200 ms.
 * Needs the MOTOR POWER on (batteries), not just USB.
 */
#include <stdio.h>

#include "pico/stdlib.h"

#include "line_config.h"
#include "line_follow.h"
#include "line_sensors.h"
#include "motion_config.h"
#include "motion_control.h"

#define START_DELAY_MS  (5000u)
#define PRINT_MS        (200u)

int main(void)
{
    uint32_t now_ms;
    uint32_t next_update_ms;
    uint32_t next_line_ms;
    uint32_t next_print_ms;
    line_status_t line;
    motion_status_t motor;
    motion_result_t init_result;
    bool started;

    stdio_init_all();
    sleep_ms(START_DELAY_MS);

    init_result = motion_init();
    (void)motion_reset_odometry();
    line_follow_init();
    started = line_follow_start();
    printf("Basic line test: motion_init=%d line_follow_start=%d\n", (int)init_result, (int)started);
    printf("state: 0 idle, 1 straight, 2 steer left, 3 steer right, 4 recovering, 5 STOPPED (no line), 6 FAULT\n");

    now_ms = to_ms_since_boot(get_absolute_time());
    next_update_ms = now_ms;
    next_line_ms = now_ms;
    next_print_ms = now_ms;

    for (;;)
    {
        now_ms = to_ms_since_boot(get_absolute_time());

        if (now_ms >= next_update_ms)
        {
            next_update_ms += MOTION_UPDATE_PERIOD_MS;
            motion_update();
        }
        if (now_ms >= next_line_ms)
        {
            next_line_ms += LINE_UPDATE_PERIOD_MS;
            line_follow_update();
        }
        if (now_ms >= next_print_ms)
        {
            next_print_ms += PRINT_MS;
            line_follow_get_status(&line);
            motion_get_status(&motor);
            printf("[LINE] state=%d mask=%u | target L/R=%.1f/%.1f cm/s | actual L/R=%.1f/%.1f cm/s | motor_state=%d fault=%d\n",
                   (int)line.state, (unsigned)line.sensor_mask,
                   (double)line.left_cmd_cm_s, (double)line.right_cmd_cm_s,
                   (double)motor.left_speed_cm_s, (double)motor.right_speed_cm_s,
                   (int)motor.state, (int)motor.fault);
        }
    }
}

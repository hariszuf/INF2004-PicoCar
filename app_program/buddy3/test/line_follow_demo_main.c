/**
 * @file line_follow_demo_main.c
 * @brief Autonomous line following demo (no RTOS, no remote control).
 *
 * After power-on the car waits LINE_DEMO_START_DELAY_MS (time to put it on the
 * black line), then follows the line by itself. If the line is lost for longer
 * than LINE_RECOVER_MS it stops (it resumes if the line comes back). Switch the Robo Pico off to abort.
 *
 * Progress is printed over USB serial if a terminal happens to be connected,
 * which is optional.
 */
#include <stdio.h>

#include "pico/stdlib.h"

#include "line_config.h"
#include "line_follow.h"
#include "line_sensors.h"
#include "motion_config.h"
#include "motion_control.h"

#define LINE_DEMO_START_DELAY_MS    (5000u)
#define LINE_DEMO_PRINT_MS          (200u)

int main(void)
{
    uint32_t now_ms;
    uint32_t next_update_ms;
    uint32_t next_line_ms;
    uint32_t next_print_ms;
    line_status_t status;
    motion_status_t motor;

    stdio_init_all();
    sleep_ms(LINE_DEMO_START_DELAY_MS);

    (void)motion_init();
    (void)motion_reset_odometry();
    line_follow_init();
    (void)line_follow_start();

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
            next_print_ms += LINE_DEMO_PRINT_MS;
            line_follow_get_status(&status);
            motion_get_status(&motor);
            printf("[IR] GP0=%d GP2=%d GP26=%d | state=%d | "
                   "target L/R=%.1f/%.1f | actual L/R=%.1f/%.1f | fault=%d\n",
                   gpio_get(LINE_PIN_GP0_IR) != 0,
                   gpio_get(LINE_PIN_GP2_IR) != 0,
                   gpio_get(LINE_PIN_BARCODE_IR) != 0,
                   (int)status.state,
                   (double)status.left_cmd_cm_s,
                   (double)status.right_cmd_cm_s,
                   (double)motor.left_speed_cm_s,
                   (double)motor.right_speed_cm_s,
                   (int)motor.fault);
        }
    }
}

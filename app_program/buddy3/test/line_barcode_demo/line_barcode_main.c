/**
 * @file line_barcode_main.c
 * @brief Buddy 2 + Buddy 3 line following with Buddy 3 barcode reading. No IMU.
 *
 * After power-on the car waits START_DELAY_MS (time to place it on the black line),
 * then follows the line by itself (Buddy 3 steering through Buddy 2 motion_drive()).
 * While it drives, the barcode sensor (GP26) is decoded; each letter is printed on
 * USB serial (115200 baud) and the car keeps following the line. The serial port is
 * optional: with no terminal connected the car still drives.
 *
 * Line following is unchanged. The barcode only watches GP26; it never touches
 * the motors.
 */
#include <stdio.h>

#include "pico/stdlib.h"

#include "barcode_scan.h"
#include "line_config.h"
#include "line_follow.h"
#include "line_sensors.h"
#include "motion_config.h"
#include "motion_control.h"

#define START_DELAY_MS      (5000u)
#define PRINT_MS            (200u)

void barcode_scan_on_decoded(const barcode39_result_t *p_result)
{
    printf("\n*** BARCODE: %s  (%s) ***\n\n", p_result->text,
           p_result->reversed ? "backwards" : "forwards");
}

void barcode_scan_on_run(bool black, uint32_t width_us)
{
    printf("  run %s %lu us\n", black ? "BLACK" : "white", (unsigned long)width_us);
}

int main(void)
{
    uint32_t now_ms;
    uint32_t next_update_ms;
    uint32_t next_line_ms;
    uint32_t next_print_ms;
    line_status_t line;
    motion_status_t motor;

    stdio_init_all();
    sleep_ms(START_DELAY_MS);

    (void)motion_init();
    (void)motion_reset_odometry();
    line_follow_init();              /* also resets the barcode scanner */
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
            line_follow_update();    /* steering + barcode sampling */
        }

        if (now_ms >= next_print_ms)
        {
            int l_mm;
            int r_mm;

            next_print_ms += PRINT_MS;
            line_follow_get_status(&line);
            motion_get_status(&motor);
            l_mm = (int)(motor.left_speed_cm_s * 10.0f);
            r_mm = (int)(motor.right_speed_cm_s * 10.0f);
            printf("[LINE] state=%d mask=%u | target L/R=%.1f/%.1f cm/s | "
                   "wheels L/R=%d/%d mm/s diff=%d | fault=%d | barcode ok=%lu rejected=%lu\n",
                   (int)line.state, (unsigned)line.sensor_mask,
                   (double)line.left_cmd_cm_s, (double)line.right_cmd_cm_s,
                   l_mm, r_mm, l_mm - r_mm, (int)motor.fault,
                   (unsigned long)barcode_scan_count(),
                   (unsigned long)barcode_scan_fail_count());
        }
    }
}

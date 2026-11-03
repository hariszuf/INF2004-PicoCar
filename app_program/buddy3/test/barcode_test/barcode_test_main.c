/**
 * @file barcode_test_main.c
 * @brief Stand-alone barcode reader test. NO MOTORS, NO LINE FOLLOWING.
 *
 * Push the car by hand over the barcode, with the barcode sensor (GP26) passing
 * over the bars at a steady, slow pace. The decoded letter is printed on USB serial
 * (115200 baud). Every black bar / white gap length is printed too, so a bad read
 * can be diagnosed.
 */
#include <stdio.h>

#include "pico/stdlib.h"

#include "barcode_scan.h"
#include "line_config.h"
#include "line_sensors.h"

void barcode_scan_on_decoded(const barcode39_result_t *p_result)
{
    printf("\n*** BARCODE: %s  (%s) ***\n\n", p_result->text,
           p_result->reversed ? "crossed backwards" : "crossed forwards");
}

void barcode_scan_on_run(bool black, uint32_t width_us)
{
    printf("  %s %6lu us\n", black ? "BLACK" : "white", (unsigned long)width_us);
}

int main(void)
{
    uint32_t last_print_ms = 0u;

    stdio_init_all();
    for (int i = 0; (i < 100) && !stdio_usb_connected(); i++)
    {
        sleep_ms(100);              /* wait up to 10 s for a terminal */
    }
    line_sensors_init();
    barcode_scan_init();

    printf("\nBarcode test. Push the car slowly over the barcode.\n");
    printf("Sensor: GP%u, black level %u. Debounce %u us.\n", (unsigned)LINE_PIN_BARCODE_IR,
           (unsigned)LINE_BLACK_LEVEL, (unsigned)BARCODE_DEBOUNCE_US);

    for (;;)
    {
        bool black = (line_sensors_read() & LINE_SENSOR_BARCODE) != 0u;
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());

        barcode_scan_update(black, time_us_32());

        if ((now_ms - last_print_ms) >= 2000u)
        {
            last_print_ms = now_ms;
            printf("[idle] sensor=%s decoded=%lu rejected=%lu last_reject=%u "
                   "(1 length,2 ratio,3 pattern,4 ends,5 gap,6 not-letter)\n",
                   black ? "BLACK" : "white", (unsigned long)barcode_scan_count(),
                   (unsigned long)barcode_scan_fail_count(), (unsigned)barcode_scan_last_fail());
        }
        sleep_us(1000);             /* 1 kHz sampling */
    }
}

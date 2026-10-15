
#include <stdio.h>
#include "pico/stdlib.h"
#include "ir_sensor.h"

int main(void)
{
    // Initialise USB serial
    stdio_init_all();

    // Wait for serial monitor
    sleep_ms(3000);

    printf("Starting Black Line Detection Test...\n");

    // Initialise IR sensors
    ir_sensor_init();

    while (true)
    {
        // Read sensor values
        ir_data_t data = ir_sensor_read();

        // Print when readings change
        ir_sensor_log_changes(&data);

        // Check every 100ms
        sleep_ms(100);
    }

    return 0;
}
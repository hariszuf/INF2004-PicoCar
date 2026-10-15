
#include <stdio.h>
#include "pico/stdlib.h"
#include "ir_sensor.h"

int main(void)
{
    // Initialise USB serial
    stdio_init_all();

    // Allow time for USB terminal connection
    sleep_ms(3000);

    printf("Starting IR Sensor Test...\n");

    // Initialise IR sensors
    ir_sensor_init();

    while (true)
    {
        // Read all three sensors
        ir_data_t data = ir_sensor_read();

        // Print only when a sensor reading changes
        ir_sensor_log_changes(&data);

        // Read every 100 milliseconds
        sleep_ms(100);
    }

    return 0;
}

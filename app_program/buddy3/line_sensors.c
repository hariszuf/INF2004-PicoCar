/**
 * @file line_sensors.c
 * @brief IR sensor input for the Pico (GP0, GP2 line sensors, GP26 barcode).
 */
#include "line_sensors.h"

#include "hardware/gpio.h"

#include "line_config.h"

static void init_pin(uint32_t pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
}

static bool is_black(uint32_t pin)
{
    bool level = gpio_get(pin);

    return (LINE_BLACK_LEVEL != 0u) ? level : (!level);
}

void line_sensors_init(void)
{
    init_pin(LINE_PIN_GP0_IR);
    init_pin(LINE_PIN_GP2_IR);
    init_pin(LINE_PIN_BARCODE_IR);
}

uint8_t line_sensors_read(void)
{
    uint8_t mask = 0u;
    uint8_t gp0_bit = (LINE_GP0_IS_CAR_LEFT != 0u) ? LINE_SENSOR_LEFT : LINE_SENSOR_RIGHT;
    uint8_t gp2_bit = (LINE_GP0_IS_CAR_LEFT != 0u) ? LINE_SENSOR_RIGHT : LINE_SENSOR_LEFT;

    if (is_black(LINE_PIN_GP0_IR))
    {
        mask |= gp0_bit;
    }
    if (is_black(LINE_PIN_GP2_IR))
    {
        mask |= gp2_bit;
    }
    if (is_black(LINE_PIN_BARCODE_IR))
    {
        mask |= LINE_SENSOR_BARCODE;
    }
    return mask;
}

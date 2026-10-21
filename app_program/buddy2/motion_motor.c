/**
 * @file motion_motor.c
 * @brief Robo Pico motor driver (Pico SDK). See motion_motor.h.
 */
#include "motion_motor.h"
#include "motion_config.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

static uint16_t s_pwm_wrap = 0u;
static volatile bool s_latched = false;

static uint32_t pin_a(motion_side_t side)
{
    uint32_t pin;

    if (side == MOTION_SIDE_LEFT)
    {
        pin = (MOTION_LEFT_MOTOR_INVERT != 0u) ? MOTION_PIN_LEFT_MOTOR_B
                                               : MOTION_PIN_LEFT_MOTOR_A;
    }
    else
    {
        pin = (MOTION_RIGHT_MOTOR_INVERT != 0u) ? MOTION_PIN_RIGHT_MOTOR_B
                                                : MOTION_PIN_RIGHT_MOTOR_A;
    }
    return pin;
}

static uint32_t pin_b(motion_side_t side)
{
    uint32_t pin;

    if (side == MOTION_SIDE_LEFT)
    {
        pin = (MOTION_LEFT_MOTOR_INVERT != 0u) ? MOTION_PIN_LEFT_MOTOR_A
                                               : MOTION_PIN_LEFT_MOTOR_B;
    }
    else
    {
        pin = (MOTION_RIGHT_MOTOR_INVERT != 0u) ? MOTION_PIN_RIGHT_MOTOR_A
                                                : MOTION_PIN_RIGHT_MOTOR_B;
    }
    return pin;
}

static void init_pin(uint32_t pin)
{
    uint32_t slice;
    pwm_config config;

    gpio_set_function(pin, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(pin);
    config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.0f);
    pwm_config_set_wrap(&config, s_pwm_wrap);
    pwm_init(slice, &config, true);
    pwm_set_gpio_level(pin, 0u);
}

void motion_motor_init(void)
{
    uint32_t clock_hz = clock_get_hz(clk_sys);

    s_pwm_wrap = (uint16_t)((clock_hz / MOTION_PWM_FREQ_HZ) - 1u);

    init_pin(MOTION_PIN_LEFT_MOTOR_A);
    init_pin(MOTION_PIN_LEFT_MOTOR_B);
    init_pin(MOTION_PIN_RIGHT_MOTOR_A);
    init_pin(MOTION_PIN_RIGHT_MOTOR_B);
    s_latched = false;
}

void motion_motor_brake(motion_side_t side)
{
    pwm_set_gpio_level(pin_a(side), 0u);
    pwm_set_gpio_level(pin_b(side), 0u);
}

void motion_motor_brake_all(void)
{
    motion_motor_brake(MOTION_SIDE_LEFT);
    motion_motor_brake(MOTION_SIDE_RIGHT);
}

void motion_motor_set(motion_side_t side, float duty)
{
    float magnitude = duty;
    uint16_t level;

    if (s_latched)
    {
        motion_motor_brake(side);
        return;
    }

    if (magnitude < 0.0f)
    {
        magnitude = -magnitude;
    }
    if (magnitude > 1.0f)
    {
        magnitude = 1.0f;
    }
    level = (uint16_t)(magnitude * (float)s_pwm_wrap);

    if (duty >= 0.0f)
    {
        pwm_set_gpio_level(pin_b(side), 0u);
        pwm_set_gpio_level(pin_a(side), level);
    }
    else
    {
        pwm_set_gpio_level(pin_a(side), 0u);
        pwm_set_gpio_level(pin_b(side), level);
    }
}

void motion_motor_set_latch(bool latched)
{
    s_latched = latched;
    if (latched)
    {
        motion_motor_brake_all();
    }
}

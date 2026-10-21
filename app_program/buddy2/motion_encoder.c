/**
 * @file motion_encoder.c
 * @brief Wheel encoder counting. See motion_encoder.h.
 */
#include "motion_encoder.h"
#include "motion_config.h"

#include "hardware/gpio.h"

static volatile int32_t s_count[MOTION_SIDE_COUNT];
static volatile int32_t s_direction[MOTION_SIDE_COUNT];

static uint32_t edge_events(void)
{
    uint32_t events;

    if (MOTION_ENCODER_EDGE_MODE == 2u)
    {
        events = GPIO_IRQ_EDGE_FALL;
    }
    else if (MOTION_ENCODER_EDGE_MODE == 3u)
    {
        events = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    }
    else
    {
        events = GPIO_IRQ_EDGE_RISE;
    }
    return events;
}

void motion_encoder_irq_handler(unsigned int gpio, uint32_t events)
{
    (void)events;

    if (gpio == MOTION_PIN_LEFT_ENCODER)
    {
        s_count[MOTION_SIDE_LEFT] += s_direction[MOTION_SIDE_LEFT];
    }
    else if (gpio == MOTION_PIN_RIGHT_ENCODER)
    {
        s_count[MOTION_SIDE_RIGHT] += s_direction[MOTION_SIDE_RIGHT];
    }
    else
    {
        /* Not an encoder pin: another module's GPIO, ignore. */
    }
}

static void init_pin(uint32_t pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

void motion_encoder_init(void)
{
    s_count[MOTION_SIDE_LEFT] = 0;
    s_count[MOTION_SIDE_RIGHT] = 0;
    s_direction[MOTION_SIDE_LEFT] = 1;
    s_direction[MOTION_SIDE_RIGHT] = 1;

    init_pin(MOTION_PIN_LEFT_ENCODER);
    init_pin(MOTION_PIN_RIGHT_ENCODER);

#ifdef MOTION_EXTERNAL_IRQ_DISPATCHER
    gpio_set_irq_enabled(MOTION_PIN_LEFT_ENCODER, edge_events(), true);
    gpio_set_irq_enabled(MOTION_PIN_RIGHT_ENCODER, edge_events(), true);
#else
    gpio_set_irq_enabled_with_callback(MOTION_PIN_LEFT_ENCODER, edge_events(),
                                       true, &motion_encoder_irq_handler);
    gpio_set_irq_enabled(MOTION_PIN_RIGHT_ENCODER, edge_events(), true);
#endif
}

int32_t motion_encoder_get_count(motion_side_t side)
{
    return s_count[side];
}

void motion_encoder_set_direction(motion_side_t side, int32_t direction)
{
    s_direction[side] = (direction < 0) ? -1 : 1;
}

void motion_encoder_clear(motion_side_t side)
{
    s_count[side] = 0;
}

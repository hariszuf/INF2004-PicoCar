/**
 * @file motion_encoder_mtk.c
 * @brief micro T-Kernel build of motion_encoder.h (Buddy 2 trial): single-phase
 *        encoder counting through the shared GPIO interrupt dispatcher.
 *        Same counting rule as motion_encoder.c.
 */
#include "motion_encoder.h"
#include "motion_config.h"
#include "motion_mtk_hw.h"

static volatile int32_t s_count[MOTION_SIDE_COUNT];
static volatile int32_t s_direction[MOTION_SIDE_COUNT];

static uint32_t edge_events(void)
{
    if (MOTION_ENCODER_EDGE_MODE == 2u)
    {
        return B2_GPIO_EDGE_FALL;
    }
    if (MOTION_ENCODER_EDGE_MODE == 3u)
    {
        return B2_GPIO_EDGE_RISE | B2_GPIO_EDGE_FALL;
    }
    return B2_GPIO_EDGE_RISE;
}

void motion_encoder_irq_handler(unsigned int gpio, uint32_t events)
{
    uint32_t edges = events & edge_events();
    int32_t n = (edges == (B2_GPIO_EDGE_RISE | B2_GPIO_EDGE_FALL)) ? 2 : ((edges != 0u) ? 1 : 0);

    if (gpio == MOTION_PIN_LEFT_ENCODER)
    {
        s_count[MOTION_SIDE_LEFT] += n * s_direction[MOTION_SIDE_LEFT];
    }
    else if (gpio == MOTION_PIN_RIGHT_ENCODER)
    {
        s_count[MOTION_SIDE_RIGHT] += n * s_direction[MOTION_SIDE_RIGHT];
    }
    else
    {
        /* not an encoder pin */
    }
}

static void on_edge(uint32_t pin, uint32_t events, uint32_t now_us)
{
    (void)now_us;
    motion_encoder_irq_handler(pin, events);
}

void motion_encoder_init(void)
{
    s_count[MOTION_SIDE_LEFT] = 0;
    s_count[MOTION_SIDE_RIGHT] = 0;
    s_direction[MOTION_SIDE_LEFT] = 1;
    s_direction[MOTION_SIDE_RIGHT] = 1;

    motion_mtk_gpio_input(MOTION_PIN_LEFT_ENCODER, 1u);    /* pull-up, as motion_encoder.c */
    motion_mtk_gpio_input(MOTION_PIN_RIGHT_ENCODER, 1u);
    (void)motion_mtk_gpio_irq_register(MOTION_PIN_LEFT_ENCODER, edge_events(), on_edge);
    (void)motion_mtk_gpio_irq_register(MOTION_PIN_RIGHT_ENCODER, edge_events(), on_edge);
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

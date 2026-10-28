#include "ultrasonic.h"
#include <stdint.h>

/* RP2040 SIO and TIMER memory-mapped registers. Pico W / RP2040 ONLY. */
#define REG32(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define SIO_GPIO_IN       REG32(0xD0000004u)
#define SIO_GPIO_OUT_SET  REG32(0xD0000014u)
#define SIO_GPIO_OUT_CLR  REG32(0xD0000018u)
#define SIO_GPIO_OE_SET   REG32(0xD0000024u)
#define SIO_GPIO_OE_CLR   REG32(0xD0000028u)
#define IO_BANK0_BASE     0x40014000u
#define GPIO_CTRL(pin)    REG32(IO_BANK0_BASE + 8u * (pin) + 4u)
#define PADS_BANK0_BASE   0x4001C000u
#define GPIO_PAD(pin)     REG32(PADS_BANK0_BASE + 4u + 4u * (pin))
#define TIMER_RAWL        REG32(0x40054028u)
#define TRIG_PIN          28u
#define ECHO_PIN          7u
#define FUNC_SIO          5u
#define ECHO_TIMEOUT_US   30000u
#define ECHO_MIN_US       116u    /* approximately 20 mm */
#define ECHO_MAX_US       23200u  /* approximately 3980 mm */

static uint32_t micros(void) { return TIMER_RAWL; }
static int pin_high(unsigned pin) { return (SIO_GPIO_IN & (1u << pin)) != 0u; }
static void wait_us(uint32_t us) {
    const uint32_t start = micros();
    while ((uint32_t)(micros() - start) < us) { }
}
/* Important: task pre-emption may increase effective 10 us trigger width. */
void ultrasonic_init(void) {
    /* Select SIO without changing neighboring GPIOs. */
    GPIO_CTRL(TRIG_PIN) = FUNC_SIO;
    GPIO_CTRL(ECHO_PIN) = FUNC_SIO;
    /* Configure TRIG driven output, ECHO input. */
    GPIO_PAD(TRIG_PIN) &= ~((1u << 7) | (1u << 2) | (1u << 3)); /* enable output, no pulls */
    SIO_GPIO_OUT_CLR = 1u << TRIG_PIN;
    SIO_GPIO_OE_SET = 1u << TRIG_PIN;
    SIO_GPIO_OE_CLR = 1u << ECHO_PIN;
    /* Pads: enable input on ECHO (IE bit 6), disable output drive (OD bit 7).
       No internal pull-up/down selected. Preserve other pad settings. */
    GPIO_PAD(ECHO_PIN) = (GPIO_PAD(ECHO_PIN) | (1u << 6) | (1u << 7)) & ~((1u << 2) | (1u << 3));
}
uint32_t ultrasonic_echo_to_mm(uint32_t echo_us) {
    /* mm = us * 343 / 2000, rounded. Safe below 12 million us. */
    return (echo_us * 343u + 1000u) / 2000u;
}
ultrasonic_result_t ultrasonic_read(void) {
    ultrasonic_result_t r = {US_OK, 0u, 0u};
    uint32_t start;
    /* Never start a new ping while ECHO is HIGH. */
    if (pin_high(ECHO_PIN)) { r.status = US_ECHO_STUCK_HIGH; return r; }
    SIO_GPIO_OUT_CLR = 1u << TRIG_PIN;
    wait_us(3u);
    SIO_GPIO_OUT_SET = 1u << TRIG_PIN;
    wait_us(10u);
    SIO_GPIO_OUT_CLR = 1u << TRIG_PIN;

    start = micros();
    while (!pin_high(ECHO_PIN)) {
        if ((uint32_t)(micros() - start) >= ECHO_TIMEOUT_US) {
            r.status = US_RISE_TIMEOUT; return r;
        }
    }
    start = micros();
    while (pin_high(ECHO_PIN)) {
        if ((uint32_t)(micros() - start) >= ECHO_TIMEOUT_US) {
            r.status = US_FALL_TIMEOUT; return r;
        }
    }
    r.echo_us = (uint32_t)(micros() - start);
    if (r.echo_us < ECHO_MIN_US || r.echo_us > ECHO_MAX_US) {
        r.status = US_OUT_OF_RANGE;
        return r;
    }
    r.distance_mm = ultrasonic_echo_to_mm(r.echo_us);
    return r;
}

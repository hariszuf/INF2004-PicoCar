/**
 * @file motion_mtk_hw.h
 * @brief Buddy 2 micro T-Kernel trial: RP2040 timer, interrupt masking and
 *        the shared GPIO interrupt dispatcher.
 *
 * The professor's port does not link the Pico SDK runtime, so the SDK calls used
 * by the CMake drivers (pwm, gpio irq, time_us_64, save_and_disable_interrupts)
 * do not exist. These functions replace them, written against the RP2040
 * registers. The port also never releases PWM or TIMER from reset and never
 * starts the 1 us timer tick; motion_mtk_hw_init() does both.
 *
 * GPIO interrupt: the RP2040 has ONE IO_IRQ_BANK0 line for all pins. This
 * module owns it (tk_def_int) and dispatches per pin, so later buddies (for
 * example a Buddy 5 echo pin) register here instead of calling tk_def_int(13).
 *
 * Qualified for SMP=0 only.
 */
#ifndef MOTION_MTK_HW_H
#define MOTION_MTK_HW_H

#include <stdbool.h>
#include <stdint.h>

#ifdef MOTION_MTK_HOST_TEST
/* Host tests: registers live in a fake memory map (tests/test_mtk_drivers.c). */
volatile uint32_t *motion_mtk_test_reg(uint32_t address);
#define B2_REG(address)     (*motion_mtk_test_reg((uint32_t)(address)))
#else
#define B2_REG(address)     (*(volatile uint32_t *)(uintptr_t)(address))
#endif

/* RP2040 register addresses (datasheet ch. 2.14, 2.19, 4.5, 4.6, 4.7) */
#define B2_SIO_GPIO_IN          (0xD0000004u)
#define B2_SIO_GPIO_OUT_SET     (0xD0000014u)
#define B2_SIO_GPIO_OUT_CLR     (0xD0000018u)
#define B2_SIO_GPIO_OE_SET      (0xD0000024u)
#define B2_SIO_GPIO_OE_CLR      (0xD0000028u)
#define B2_IO_GPIO_CTRL(pin)    (0x40014004u + (8u * (pin)))
#define B2_IO_INTR(n)           (0x400140F0u + (4u * (n)))
#define B2_IO_PROC0_INTE(n)     (0x40014100u + (4u * (n)))
#define B2_IO_PROC0_INTS(n)     (0x40014120u + (4u * (n)))
#define B2_PAD_GPIO(pin)        (0x4001C004u + (4u * (pin)))
#define B2_PAD_PDE              (1u << 2)
#define B2_PAD_PUE              (1u << 3)
#define B2_PAD_IE               (1u << 6)
#define B2_PAD_OD               (1u << 7)
#define B2_FUNCSEL_PWM          (4u)
#define B2_FUNCSEL_SIO          (5u)
#define B2_PWM_CSR(slice)       (0x40050000u + (0x14u * (slice)) + 0x00u)
#define B2_PWM_DIV(slice)       (0x40050000u + (0x14u * (slice)) + 0x04u)
#define B2_PWM_CTR(slice)       (0x40050000u + (0x14u * (slice)) + 0x08u)
#define B2_PWM_CC(slice)        (0x40050000u + (0x14u * (slice)) + 0x0Cu)
#define B2_PWM_TOP(slice)       (0x40050000u + (0x14u * (slice)) + 0x10u)
#define B2_RESETS_RESET_CLR     (0x4000C000u + 0x3000u)  /* atomic clear alias of RESET */
#define B2_RESETS_RESET_DONE    (0x4000C008u)
#define B2_RESET_PWM            (1u << 14)
#define B2_RESET_TIMER          (1u << 21)
#define B2_TIMER_RAWH           (0x40054024u)
#define B2_TIMER_RAWL           (0x40054028u)
#define B2_WATCHDOG_TICK        (0x4005802Cu)
#define B2_WATCHDOG_TICK_ENABLE (1u << 9)
#define B2_XOSC_MHZ             (12u)
#define B2_CLK_SYS_HZ           (125000000u)  /* port sets PLL_SYS to 125 MHz */
#define B2_GPIO_COUNT           (30u)

/* Same values as the Pico SDK's GPIO_IRQ_EDGE_FALL / GPIO_IRQ_EDGE_RISE. */
#define B2_GPIO_EDGE_FALL       (0x4u)
#define B2_GPIO_EDGE_RISE       (0x8u)

/** Edge callback, interrupt context. now_us = TIMER low word at entry. */
typedef void (*motion_mtk_gpio_handler_t)(uint32_t pin, uint32_t events, uint32_t now_us);

/** Release PWM/TIMER resets, start the 1 us tick, install the GPIO interrupt.
 *  Returns E_OK (0) or a negative kernel error. Safe to call twice. */
int32_t motion_mtk_hw_init(void);

uint64_t motion_mtk_time_us(void);
uint32_t motion_mtk_irq_save(void);
void motion_mtk_irq_restore(uint32_t saved);

/** pull: 0 none, 1 up, 2 down */
void motion_mtk_gpio_input(uint32_t pin, uint32_t pull);

/** Route pin edges (B2_GPIO_EDGE_*) to handler. One owner per pin. */
int32_t motion_mtk_gpio_irq_register(uint32_t pin, uint32_t edges, motion_mtk_gpio_handler_t handler);

/** The IO_IRQ_BANK0 handler (exposed for host tests). */
void motion_mtk_gpio_isr(uint32_t intno);

#endif /* MOTION_MTK_HW_H */

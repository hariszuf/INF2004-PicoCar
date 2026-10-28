/**
 * @file motion_mtk_hw.c
 * @brief RP2040 timer / interrupt masking / shared GPIO interrupt (see header).
 */
#include "motion_mtk_hw.h"

#ifdef MOTION_MTK_HOST_TEST
#include "tests/mtk_kernel_stub.h"
#else
#include <tk/tkernel.h>
#if TK_MAX_CORE > 1
#warning "Buddy 2 micro T-Kernel trial is qualified for SMP=0 only"
#endif
#endif

#define INTNO_IO_BANK0      (13u)   /* RP2040 IRQ 13: IO_IRQ_BANK0 */
#define INTPRI_IO_BANK0     (1)     /* highest external level allowed to call the kernel */

static motion_mtk_gpio_handler_t s_handler[B2_GPIO_COUNT];
static uint32_t s_owned = 0u;
static bool s_ready = false;

uint32_t motion_mtk_irq_save(void)
{
    UINT intsts;
    DI(intsts);
    return (uint32_t)intsts;
}

void motion_mtk_irq_restore(uint32_t saved)
{
    UINT intsts = (UINT)saved;
    EI(intsts);
}

uint64_t motion_mtk_time_us(void)
{
    uint32_t hi;
    uint32_t lo;

    do
    {
        hi = B2_REG(B2_TIMER_RAWH);
        lo = B2_REG(B2_TIMER_RAWL);
    } while (hi != B2_REG(B2_TIMER_RAWH));
    return ((uint64_t)hi << 32) | lo;
}

void motion_mtk_gpio_input(uint32_t pin, uint32_t pull)
{
    uint32_t pad = B2_REG(B2_PAD_GPIO(pin));

    B2_REG(B2_SIO_GPIO_OE_CLR) = 1u << pin;
    pad = (pad | B2_PAD_IE) & ~(B2_PAD_OD | B2_PAD_PUE | B2_PAD_PDE);
    if (pull == 1u)
    {
        pad |= B2_PAD_PUE;
    }
    else if (pull == 2u)
    {
        pad |= B2_PAD_PDE;
    }
    else
    {
        /* no pull */
    }
    B2_REG(B2_PAD_GPIO(pin)) = pad;
    B2_REG(B2_IO_GPIO_CTRL(pin)) = B2_FUNCSEL_SIO;
}

void motion_mtk_gpio_isr(uint32_t intno)
{
    uint32_t now = B2_REG(B2_TIMER_RAWL);
    uint32_t reg;
    uint32_t pin;
    uint32_t status;
    uint32_t shift;
    uint32_t events;

    (void)intno;
    for (reg = 0u; reg < 4u; reg++)
    {
        status = B2_REG(B2_IO_PROC0_INTS(reg));
        for (pin = reg * 8u; (status != 0u) && (pin < (reg * 8u) + 8u) && (pin < B2_GPIO_COUNT); pin++)
        {
            shift = 4u * (pin % 8u);
            events = (status >> shift) & (B2_GPIO_EDGE_FALL | B2_GPIO_EDGE_RISE);
            if (events != 0u)
            {
                B2_REG(B2_IO_INTR(reg)) = events << shift;   /* edge bits: write 1 to clear */
                if (s_handler[pin] != (motion_mtk_gpio_handler_t)0)
                {
                    s_handler[pin](pin, events, now);
                }
            }
        }
    }
}

static void gpio_isr_entry(UINT intno)
{
    motion_mtk_gpio_isr((uint32_t)intno);
}

int32_t motion_mtk_hw_init(void)
{
    T_DINT dint;
    ER err;

    if (s_ready)
    {
        return E_OK;
    }
    /* Motor PWM and the TIMER are held in reset by the port (PWM is released
       only with USE_PTMR=1). Release only; never put them back into reset. */
    B2_REG(B2_RESETS_RESET_CLR) = B2_RESET_PWM | B2_RESET_TIMER;
    while ((B2_REG(B2_RESETS_RESET_DONE) & (B2_RESET_PWM | B2_RESET_TIMER)) !=
           (B2_RESET_PWM | B2_RESET_TIMER))
    {
    }
    B2_REG(B2_WATCHDOG_TICK) = B2_WATCHDOG_TICK_ENABLE | B2_XOSC_MHZ;   /* 1 us tick */

    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)gpio_isr_entry;
    err = tk_def_int(INTNO_IO_BANK0, &dint);
    if (err < E_OK)
    {
        return (int32_t)err;
    }
    ClearInt(INTNO_IO_BANK0);
    EnableInt(INTNO_IO_BANK0, INTPRI_IO_BANK0);
    s_ready = true;
    return E_OK;
}

int32_t motion_mtk_gpio_irq_register(uint32_t pin, uint32_t edges, motion_mtk_gpio_handler_t handler)
{
    uint32_t saved;
    uint32_t reg = pin / 8u;
    uint32_t shift = 4u * (pin % 8u);
    uint32_t mask = (B2_GPIO_EDGE_FALL | B2_GPIO_EDGE_RISE) << shift;

    if ((pin >= B2_GPIO_COUNT) || (handler == (motion_mtk_gpio_handler_t)0) || !s_ready)
    {
        return E_PAR;
    }
    if ((s_owned & (1u << pin)) != 0u)
    {
        return E_OBJ;
    }
    saved = motion_mtk_irq_save();
    s_handler[pin] = handler;
    s_owned |= 1u << pin;
    B2_REG(B2_IO_INTR(reg)) = mask;   /* drop stale edges */
    B2_REG(B2_IO_PROC0_INTE(reg)) = (B2_REG(B2_IO_PROC0_INTE(reg)) & ~mask) |
                                   ((edges & (B2_GPIO_EDGE_FALL | B2_GPIO_EDGE_RISE)) << shift);
    motion_mtk_irq_restore(saved);
    return E_OK;
}

/**
 * @file motion_motor_mtk.c
 * @brief micro T-Kernel build of motion_motor.h (Buddy 2 trial).
 *
 * Same behaviour as motion_motor.c: 15 kHz PWM, brake = both inputs low,
 * PWM on input A (forward) or B (backward), e-stop latch. Each motor uses both
 * channels of ONE PWM slice (GP8/9 = slice 4, GP10/11 = slice 5), so a motor is
 * always updated with a single register store: safe if motion_emergency_stop()
 * interrupts the Motion task half-way.
 */
#include "motion_motor.h"
#include "motion_config.h"
#include "motion_mtk_hw.h"

#define PWM_CSR_EN      (1u << 0)
#define PWM_DIV_ONE     (1u << 4)    /* integer divider 1 (8.4 fixed point) */

static uint32_t s_pwm_wrap = 0u;
static volatile bool s_latched = false;

static uint32_t pin_a(motion_side_t side)
{
    if (side == MOTION_SIDE_LEFT)
    {
        return (MOTION_LEFT_MOTOR_INVERT != 0u) ? MOTION_PIN_LEFT_MOTOR_B : MOTION_PIN_LEFT_MOTOR_A;
    }
    return (MOTION_RIGHT_MOTOR_INVERT != 0u) ? MOTION_PIN_RIGHT_MOTOR_B : MOTION_PIN_RIGHT_MOTOR_A;
}

static uint32_t pin_b(motion_side_t side)
{
    if (side == MOTION_SIDE_LEFT)
    {
        return (MOTION_LEFT_MOTOR_INVERT != 0u) ? MOTION_PIN_LEFT_MOTOR_A : MOTION_PIN_LEFT_MOTOR_B;
    }
    return (MOTION_RIGHT_MOTOR_INVERT != 0u) ? MOTION_PIN_RIGHT_MOTOR_A : MOTION_PIN_RIGHT_MOTOR_B;
}

static uint32_t slice_of(uint32_t pin)
{
    return (pin >> 1) & 7u;
}

/* Compare value: 'level' on this pin's channel, 0 on the other (odd pin = B). */
static uint32_t cc_value(uint32_t pin, uint32_t level)
{
    return ((pin & 1u) != 0u) ? (level << 16) : level;
}

static void init_pin(uint32_t pin)
{
    uint32_t slice = slice_of(pin);

    B2_REG(B2_PWM_CSR(slice)) = 0u;
    B2_REG(B2_PWM_DIV(slice)) = PWM_DIV_ONE;
    B2_REG(B2_PWM_TOP(slice)) = s_pwm_wrap;
    B2_REG(B2_PWM_CTR(slice)) = 0u;
    B2_REG(B2_PWM_CC(slice)) = 0u;            /* both inputs low: brake */
    B2_REG(B2_PWM_CSR(slice)) = PWM_CSR_EN;
    B2_REG(B2_PAD_GPIO(pin)) = (B2_REG(B2_PAD_GPIO(pin)) | B2_PAD_IE) &
                               ~(B2_PAD_OD | B2_PAD_PUE | B2_PAD_PDE);
    B2_REG(B2_IO_GPIO_CTRL(pin)) = B2_FUNCSEL_PWM;
}

void motion_motor_init(void)
{
    s_pwm_wrap = (B2_CLK_SYS_HZ / MOTION_PWM_FREQ_HZ) - 1u;
    init_pin(MOTION_PIN_LEFT_MOTOR_A);
    init_pin(MOTION_PIN_LEFT_MOTOR_B);
    init_pin(MOTION_PIN_RIGHT_MOTOR_A);
    init_pin(MOTION_PIN_RIGHT_MOTOR_B);
    s_latched = false;
}

void motion_motor_brake(motion_side_t side)
{
    B2_REG(B2_PWM_CC(slice_of(pin_a(side)))) = 0u;
}

void motion_motor_brake_all(void)
{
    motion_motor_brake(MOTION_SIDE_LEFT);
    motion_motor_brake(MOTION_SIDE_RIGHT);
}

void motion_motor_set(motion_side_t side, float duty)
{
    float magnitude = (duty < 0.0f) ? -duty : duty;
    uint32_t pin;

    if (s_latched)
    {
        motion_motor_brake(side);
        return;
    }
    if (magnitude > 1.0f)
    {
        magnitude = 1.0f;
    }
    pin = (duty >= 0.0f) ? pin_a(side) : pin_b(side);
    B2_REG(B2_PWM_CC(slice_of(pin))) = cc_value(pin, (uint32_t)(magnitude * (float)s_pwm_wrap));
}

void motion_motor_set_latch(bool latched)
{
    s_latched = latched;
    if (latched)
    {
        motion_motor_brake_all();
    }
}

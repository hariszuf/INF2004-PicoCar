/**
 * @file test_mtk_drivers.c
 * @brief Host tests for the Buddy 2 micro T-Kernel drivers against a fake
 *        RP2040 register map: reset release / timer tick, GPIO interrupt
 *        dispatch, encoder counting, PWM motor outputs, and motion_control.c
 *        driving the real drivers end to end. Logic only, no hardware.
 */
#include <stdio.h>
#include <stdlib.h>
#include "tests/mtk_kernel_stub.h"
#include "motion_mtk_hw.h"
#include "motion_config.h"
#include "motion_control.h"
#include "motion_encoder.h"
#include "motion_motor.h"

static int s_fail = 0;
static int s_checks = 0;
#define CHECK(c, t) do { s_checks++; if (c) printf("  PASS  %s\n", t); else { printf("  FAIL  %s\n", t); s_fail++; } } while (0)

/* ---- fake register map ---- */
#define MAX_REGS 256
static uint32_t s_addr[MAX_REGS];
static volatile uint32_t s_val[MAX_REGS];
static int s_nregs = 0;
volatile uint32_t *motion_mtk_test_reg(uint32_t address)
{
    int i;
    for (i = 0; i < s_nregs; i++) { if (s_addr[i] == address) return &s_val[i]; }
    if (s_nregs >= MAX_REGS) { printf("register map full\n"); exit(2); }
    s_addr[s_nregs] = address; s_val[s_nregs] = 0u;
    return &s_val[s_nregs++];
}
#define REG(a) (*motion_mtk_test_reg(a))

/* ---- kernel stubs ---- */
UINT stub_irq_masked = 0u;
static UINT s_def_intno = 999u;
static FP s_def_hdr;
static INT s_enabled_level = -1;
ER tk_def_int(UINT intno, const T_DINT *p) { s_def_intno = intno; s_def_hdr = p->inthdr; return E_OK; }
void ClearInt(UINT intno) { (void)intno; }
void EnableInt(UINT intno, INT level) { (void)intno; s_enabled_level = level; }

static void advance_us(uint32_t us)
{
    uint64_t t = ((uint64_t)REG(B2_TIMER_RAWH) << 32) | REG(B2_TIMER_RAWL);
    t += us;
    REG(B2_TIMER_RAWH) = (uint32_t)(t >> 32);
    REG(B2_TIMER_RAWL) = (uint32_t)t;
}

static void edge(uint32_t pin, uint32_t events)
{
    REG(B2_IO_PROC0_INTS(pin / 8u)) = events << (4u * (pin % 8u));
    motion_mtk_gpio_isr(13u);
    REG(B2_IO_PROC0_INTS(pin / 8u)) = 0u;
}

static uint32_t cc(uint32_t slice) { return REG(B2_PWM_CC(slice)); }

static void test_hw_init(void)
{
    printf("\n[hardware init]\n");
    REG(B2_RESETS_RESET_DONE) = B2_RESET_PWM | B2_RESET_TIMER;   /* would hang otherwise */
    CHECK(motion_mtk_hw_init() == E_OK, "motion_mtk_hw_init() succeeds");
    CHECK(REG(B2_RESETS_RESET_CLR) == (B2_RESET_PWM | B2_RESET_TIMER),
          "PWM and TIMER released from reset (atomic clear alias)");
    CHECK(REG(B2_WATCHDOG_TICK) == ((1u << 9) | 12u), "1 us timer tick started from 12 MHz XOSC");
    CHECK(s_def_intno == 13u && s_def_hdr != 0, "IO_IRQ_BANK0 (IRQ 13) handler installed");
    CHECK(s_enabled_level == 1, "GPIO interrupt enabled at level 1");
    REG(B2_TIMER_RAWH) = 1u; REG(B2_TIMER_RAWL) = 5u;
    CHECK(motion_mtk_time_us() == ((1ull << 32) | 5u), "64-bit microsecond time from TIMERAWH/L");
    REG(B2_TIMER_RAWH) = 0u; REG(B2_TIMER_RAWL) = 0u;
}

static void test_motor(void)
{
    uint32_t wrap = (125000000u / MOTION_PWM_FREQ_HZ) - 1u;
    uint32_t half = (uint32_t)(0.5f * (float)wrap);
    uint32_t pin;

    printf("\n[PWM motor driver]\n");
    REG(B2_PAD_GPIO(8)) = B2_PAD_OD | B2_PAD_PUE;   /* as left by the I2C sample driver */
    motion_motor_init();
    CHECK(REG(B2_PWM_TOP(4)) == wrap && REG(B2_PWM_TOP(5)) == wrap, "slices 4/5 wrap = 15 kHz at 125 MHz");
    CHECK(REG(B2_PWM_DIV(4)) == 0x10u && (REG(B2_PWM_CSR(4)) & 1u) && (REG(B2_PWM_CSR(5)) & 1u),
          "divider 1, both slices enabled");
    for (pin = 8u; pin <= 11u; pin++)
    {
        if (REG(B2_IO_GPIO_CTRL(pin)) != 4u) break;
    }
    CHECK(pin == 12u, "GP8-GP11 switched to PWM function");
    CHECK((REG(B2_PAD_GPIO(8)) & (B2_PAD_OD | B2_PAD_PUE)) == 0u, "GP8 pad: output enabled, I2C pull-up removed");
    CHECK(cc(4) == 0u && cc(5) == 0u, "starts braked (both inputs low)");

    motion_motor_set(MOTION_SIDE_LEFT, 0.5f);
    CHECK(cc(4) == half, "left forward 50%: PWM on GP8 (M1A), GP9 low");
    motion_motor_set(MOTION_SIDE_LEFT, -0.5f);
    CHECK(cc(4) == (half << 16), "left backward 50%: PWM on GP9 (M1B), GP8 low");
    motion_motor_set(MOTION_SIDE_RIGHT, 1.5f);
    CHECK(cc(5) == wrap, "right forward clamped to 100% on GP10 (M2A)");
    motion_motor_brake(MOTION_SIDE_RIGHT);
    CHECK(cc(5) == 0u && cc(4) == (half << 16), "brake right only");
    motion_motor_set_latch(true);
    CHECK(cc(4) == 0u && cc(5) == 0u, "latch brakes both motors");
    motion_motor_set(MOTION_SIDE_LEFT, 0.8f);
    CHECK(cc(4) == 0u, "latched: drive request ignored");
    motion_motor_set_latch(false);
}

static void test_encoder(void)
{
    int32_t before;

    printf("\n[encoder + GPIO interrupt dispatcher]\n");
    motion_encoder_init();
    CHECK(REG(B2_IO_GPIO_CTRL(6)) == 5u && REG(B2_IO_GPIO_CTRL(4)) == 5u, "GP6/GP4 are SIO inputs");
    CHECK((REG(B2_PAD_GPIO(6)) & B2_PAD_PUE) && (REG(B2_PAD_GPIO(4)) & B2_PAD_PUE), "encoder pull-ups on");
    CHECK(REG(B2_IO_PROC0_INTE(0)) == ((0x8u << 24) | (0x8u << 16)), "rising-edge interrupts on GP6 and GP4 only");
    CHECK(motion_mtk_gpio_irq_register(6u, B2_GPIO_EDGE_RISE, (motion_mtk_gpio_handler_t)abort) == E_OBJ,
          "second owner for GP6 refused");

    before = motion_encoder_get_count(MOTION_SIDE_LEFT);
    edge(6u, B2_GPIO_EDGE_RISE);
    edge(6u, B2_GPIO_EDGE_RISE);
    CHECK(motion_encoder_get_count(MOTION_SIDE_LEFT) == before + 2, "GP6 edges count the LEFT wheel");
    CHECK(REG(B2_IO_INTR(0)) == (0x8u << 24), "edge acknowledged (write-1-to-clear)");
    edge(4u, B2_GPIO_EDGE_RISE);
    CHECK(motion_encoder_get_count(MOTION_SIDE_RIGHT) == 1, "GP4 edges count the RIGHT wheel");
    motion_encoder_set_direction(MOTION_SIDE_LEFT, -1);
    edge(6u, B2_GPIO_EDGE_RISE);
    CHECK(motion_encoder_get_count(MOTION_SIDE_LEFT) == before + 1, "backward direction counts down");
    motion_encoder_set_direction(MOTION_SIDE_LEFT, 1);
    edge(7u, B2_GPIO_EDGE_RISE);
    CHECK(motion_encoder_get_count(MOTION_SIDE_LEFT) == before + 1 &&
          motion_encoder_get_count(MOTION_SIDE_RIGHT) == 1 && REG(B2_IO_INTR(0)) == (0x8u << 28),
          "edge on an unowned pin is acknowledged and ignored");
    motion_encoder_clear(MOTION_SIDE_LEFT);
    motion_encoder_clear(MOTION_SIDE_RIGHT);
}

/* motion_control.c on top of the real drivers: commands reach the PWM registers. */
static void test_motion_end_to_end(void)
{
    int i;
    motion_status_t st;

    printf("\n[motion_control.c driving the micro T-Kernel drivers]\n");
    CHECK(motion_init() == MOTION_OK, "motion_init() with micro T-Kernel drivers");
    CHECK(motion_drive(10.0f, 10.0f) == MOTION_OK, "motion_drive accepted");
    for (i = 0; i < 10; i++)
    {
        int k;
        advance_us(MOTION_UPDATE_PERIOD_MS * 1000u);
        for (k = 0; k < 4; k++) { edge(6u, B2_GPIO_EDGE_RISE); edge(4u, B2_GPIO_EDGE_RISE); }
        motion_update();
    }
    CHECK(cc(4) != 0u && (cc(4) >> 16) == 0u, "left motor driven forward (GP8 PWM)");
    CHECK(cc(5) != 0u && (cc(5) >> 16) == 0u, "right motor driven forward (GP10 PWM)");
    motion_get_status(&st);
    CHECK(st.left_count == 40 && st.right_count == 40 && st.speed_cm_s > 0.0f,
          "encoder counts and speed reach motion status");
    CHECK(stub_irq_masked == 0u, "interrupts re-enabled after every critical section");
    motion_emergency_stop();
    CHECK(cc(4) == 0u && cc(5) == 0u, "emergency stop brakes immediately from the caller");
    advance_us(MOTION_UPDATE_PERIOD_MS * 1000u);
    motion_update();
    motion_get_status(&st);
    CHECK(st.state == MOTION_STATE_ERROR && st.fault == MOTION_FAULT_ESTOP, "ERROR / E-STOP latched");
}

int main(void)
{
    printf("Buddy 2 micro T-Kernel driver host tests (fake registers, logic only)\n");
    test_hw_init();
    test_motor();
    test_encoder();
    test_motion_end_to_end();
    printf("\n%d checks, %d failed\n", s_checks, s_fail);
    return s_fail ? 1 : 0;
}

#define BUDDY4_NATIVE_I2C_HOST_TEST
#include "../../../device/i2c/sysdepend/rp2040/i2c_rp2040.c"
#include <stdio.h>

/* Controller model: bus commands progress independently of driver state, raw
 * latches survive masking until read-to-clear, RX_FULL reflects the FIFO and
 * INTR_STAT is RAW & MASK. Cortex-M0+ IRQ entry does not set PRIMASK. */
typedef struct { UW address, value; } reg_t;
static reg_t regs[100];
static unsigned reg_count;
static UW flag, primask, raw_latches, clock_us;
static UW commands[32];
static unsigned command_count, consumed, received, rx_fifo, next_rx;
static unsigned wait_calls, flag_sets, dispatch_requests, irq_calls;
static int in_irq, waiting, dispatch_pending, before_wait, defer_irqs;
static int forced_timeout, abort_after, disabled_before_stop, pending_stop;
enum { STOP_WITH_RX, STOP_DURING_RX, STOP_LATER };
static int stop_delivery;

static UW *reg(UW address)
{
    for (unsigned i = 0; i < reg_count; i++)
        if (regs[i].address == address) return &regs[i].value;
    assert(reg_count < 100);
    regs[reg_count] = (reg_t){address, 0};
    return &regs[reg_count++].value;
}
static UW raw_status(void)
{
    UW raw = raw_latches;
    if (rx_fifo) raw |= I2C_INT_RX_FULL;
    if (consumed == command_count) raw |= I2C_INT_TX_EMPTY;
    return raw;
}
UW in_w(UW address)
{
    if (address == RESETS_RESET_DONE) return RESETS_RESET_I2C0 | RESETS_RESET_I2C1;
    if (address == TIMER_TIMERAWL) return clock_us;
    if (address == I2C_RAW_INTR_STAT(0)) return raw_status();
    if (address == I2C_INTR_STAT(0)) return raw_status() & *reg(I2C_INTR_MASK(0));
    if (address == I2C_RXFLR(0)) return rx_fifo;
    if (address == I2C_CLR_INTR(0)) { raw_latches = 0; return 0; }
    if (address == I2C_CLR_STOP_DET(0)) { raw_latches &= ~I2C_INT_STOP_DET; return 0; }
    if (address == I2C_CLR_TX_ABRT(0)) {
        raw_latches &= ~I2C_INT_TX_ABRT;
        *reg(I2C_TX_ABRT_SOURCE(0)) = 0;
        return 0;
    }
    if (address == I2C_CLR_RX_OVER(0)) { raw_latches &= ~I2C_INT_RX_OVER; return 0; }
    if (address == I2C_CLR_RX_UNDER(0)) { raw_latches &= ~I2C_INT_RX_UNDER; return 0; }
    if (address == I2C_CLR_TX_OVER(0)) { raw_latches &= ~I2C_INT_TX_OVER; return 0; }
    if (address == I2C_DATA_CMD(0)) {
        assert(rx_fifo > 0);
        rx_fifo--;
        received++;
        if (pending_stop && stop_delivery == STOP_DURING_RX) {
            raw_latches |= I2C_INT_STOP_DET;
            pending_stop = 0;
        }
        return next_rx++;
    }
    return *reg(address);
}
void out_w(UW address, UW value)
{
    if (address == I2C_DATA_CMD(0)) {
        assert(command_count < 32);
        commands[command_count++] = value;
    }
    if (address == I2C_ENABLE(0) && value == 0) {
        if (command_count && ll_devcb[0].state != I2C_STS_STOP) disabled_before_stop++;
        rx_fifo = 0;
        consumed = command_count;
    }
    *reg(address) = value;
}
void set_w(UW a, UW v) { out_w(a, *reg(a) | v); }
void clr_w(UW a, UW v) { out_w(a, *reg(a) & ~v); }
void ClearInt(UINT n) { *reg(NVIC_ISPR(n)) &= ~(1u << (n % 32)); }
void EnableInt(UINT n, PRI p) { (void)p; *reg(NVIC_ISER(n)) |= 1u << (n % 32); }
UW mock_disint(void) { UW old = primask; primask = 1; return old; }
static void pump(void);
void mock_restore_interrupts(UW mask)
{
    primask = mask;
    if (!primask && !in_irq && before_wait && ll_devcb[0].state != I2C_STS_STOP) {
        before_wait = 0;
        pump();
    }
}
ID tk_cre_flg(const T_CFLG *f) { (void)f; return 1; }
ER tk_del_flg(ID id) { (void)id; return 0; }
ER tk_def_int(UINT n, const T_DINT *d) { (void)n; (void)d; return 0; }
ER tk_set_flg(ID id, UINT flags)
{
    (void)id;
    flag |= flags;
    flag_sets++;
#ifdef BUDDY4_TIMED_KERNEL_TEST
    extern ER timed_set_flag(void);
    return timed_set_flag();
#else
    /* eventflag.c releases/removes the wait timer. cpu_status.h requests
     * PendSV only if entry PRIMASK was clear. EI and the IRQ wrapper do not
     * request dispatch later. The test waiter has higher task priority. */
    if (waiting && (flag & 1) && !primask) {
        dispatch_pending = 1;
        dispatch_requests++;
    }
    return 0;
#endif
}
ER tk_clr_flg(ID id, UINT flags) { (void)id; flag &= flags; return 0; }
static void bus_step(void)
{
    clock_us += 90; /* Simulated byte time, not a physical measurement. */
    if (pending_stop && stop_delivery == STOP_LATER) {
        raw_latches |= I2C_INT_STOP_DET;
        pending_stop = 0;
    } else if (consumed < command_count) {
        UW cmd = commands[consumed++];
        if (abort_after >= 0 && consumed >= (unsigned)abort_after) {
            raw_latches |= I2C_INT_TX_ABRT | I2C_INT_STOP_DET;
            *reg(I2C_TX_ABRT_SOURCE(0)) = 1;
            consumed = command_count;
            rx_fifo = 0;
        } else {
            if (cmd & I2C_DATA_CMD_CMD_READ) rx_fifo++;
            if (cmd & I2C_DATA_CMD_STOP) {
                if (!(cmd & I2C_DATA_CMD_CMD_READ) || stop_delivery == STOP_WITH_RX)
                    raw_latches |= I2C_INT_STOP_DET;
                else pending_stop = 1;
            }
        }
    }
}
static void service_step(void)
{
    bus_step();
    UW active = raw_status() & *reg(I2C_INTR_MASK(0));
    if (active) *reg(NVIC_ISPR(INTNO_I2C0)) |= 1u << INTNO_I2C0;
    if (defer_irqs > 0) { defer_irqs--; return; }
    if (active && !primask && (*reg(NVIC_ISER(INTNO_I2C0)) & (1u << INTNO_I2C0))) {
        in_irq = 1;
        irq_calls++;
        i2c_inthdr(INTNO_I2C0);
        in_irq = 0;
        assert(primask == 0);
    }
}
static void pump(void)
{
    for (unsigned i = 0; !flag; i++) {
        assert(i < 100);
        service_step();
    }
}
#ifndef BUDDY4_TIMED_KERNEL_TEST
ER tk_wai_flg(ID id, UINT pattern, UINT mode, UINT *result, TMO timeout)
{
    assert(id == 1 && pattern == 1 && mode == (TWF_ORW | TWF_BITCLR));
    assert(timeout == 10 && !primask);
    wait_calls++;
    if (forced_timeout) {
        if (forced_timeout == 2) {
            for (unsigned i = 0; received < 2; i++) { assert(i < 100); service_step(); }
            bus_step(); /* Third byte ready, IRQ cannot run before deadline. */
        } else if (forced_timeout == 3) {
            stop_delivery = STOP_WITH_RX;
            for (unsigned i = 0; received < 5; i++) { assert(i < 100); service_step(); }
            bus_step(); /* Final RX + STOP coexist before timeout cleanup. */
        } else if (forced_timeout == 4) {
            /* The kernel removed the waiter at its deadline. Completion IRQ
             * runs before the timed-out task reaches driver cleanup. */
            clock_us += 20000;
            pump();
            return E_TMOUT;
        }
        if (raw_status() & *reg(I2C_INTR_MASK(0)))
            *reg(NVIC_ISPR(INTNO_I2C0)) |= 1u << INTNO_I2C0;
        clock_us += 20000; /* timer.c adds TIMER_PERIOD to the 10 ms wait. */
        return E_TMOUT;
    }
    if (!flag) {
        waiting = 1;
        pump();
        waiting = 0;
        /* No unrelated timer/USB IRQ should be needed to run the waiter. */
        assert(dispatch_pending && "completion did not request PendSV");
    }
    *result = flag;
    flag &= ~pattern;
    return 0;
}
#endif
static void reset_test(void)
{
    assert(!primask && !in_irq);
    command_count = consumed = received = rx_fifo = next_rx = 0;
    wait_calls = flag_sets = dispatch_requests = irq_calls = 0;
    disabled_before_stop = pending_stop = flag = raw_latches = 0;
    forced_timeout = before_wait = defer_irqs = dispatch_pending = waiting = 0;
    abort_after = -1;
    stop_delivery = STOP_LATER;
    clock_us = 1000;
    *reg(I2C_INTR_MASK(0)) = 0;
    *reg(NVIC_ISPR(INTNO_I2C0)) = 0;
}
static void combined(unsigned n)
{
    UB address = 0xa8, rx[6] = {0};
    T_I2C_EXEC ex = {0x1d, 1, &address, (SZ)n, rx};
    assert(dev_i2c_llctl(0, LLD_I2C_EXEC, 0, 0, (UW*)&ex) == (W)(n+1));
    assert(command_count == n+1 && received == n && disabled_before_stop == 0);
    assert(wait_calls == 1 && flag_sets == 1 && flag == 0);
    assert(!(commands[0] & I2C_DATA_CMD_STOP));
    assert(commands[1] & I2C_DATA_CMD_RESTART);
    assert(commands[n] & I2C_DATA_CMD_STOP);
    for (unsigned i = 1; i < n; i++) assert(!(commands[i] & I2C_DATA_CMD_STOP));
    for (unsigned i = 0; i < n; i++) assert(rx[i] == i);
    assert(*reg(I2C_ENABLE(0)) == 0 && *reg(I2C_INTR_MASK(0)) == 0);
}
static void recover(void)
{
    /* Keep hardware latches/pending IRQ and the event bit from the failure.
     * Only the driver's cleanup/startup may clear them for the next call. */
    UW old_flag = flag, old_raw = raw_latches;
    UW old_pending = *reg(NVIC_ISPR(INTNO_I2C0));
    reset_test();
    flag = old_flag; raw_latches = old_raw;
    *reg(NVIC_ISPR(INTNO_I2C0)) = old_pending;
    combined(6);
}
#ifndef BUDDY4_TIMED_KERNEL_TEST
int main(void)
{
    T_I2C_DCB d = {0};
    assert(dev_i2c_llinit(&d) == 0);
    assert(*reg(GPIO_CTRL(16)) == 3 && *reg(GPIO_CTRL(17)) == 3);
    assert(*reg(GPIO_CTRL(8)) == 0 && *reg(GPIO_CTRL(9)) == 0);
    assert(dev_i2c_llctl(0, LLD_I2C_OPEN, 0, 0, NULL) == 0);
    assert((*reg(I2C_CON(0)) & I2C_CON_SPEED_FAST) == 0);
    assert(*reg(I2C_SS_SCL_HCNT(0)) == 500 && *reg(I2C_SS_SCL_LCNT(0)) == 750);
    UB tx[] = {0x20, 0x6f};
    reset_test();
    assert(dev_i2c_llctl(0, LLD_I2C_WRITE, 0x1d, 2, (UW*)tx) == 2);
    assert(command_count == 2 && (commands[1] & I2C_DATA_CMD_STOP));
    assert(disabled_before_stop == 0 && dispatch_requests == 1);
    for (int timing = STOP_WITH_RX; timing <= STOP_LATER; timing++) {
        for (unsigned n = 1; n <= 6; n++) {
            reset_test(); stop_delivery = timing;
            combined(n);
            assert(dispatch_requests == 1);
        }
    }
    reset_test(); before_wait = 1; combined(6); assert(dispatch_requests == 0);
    reset_test(); defer_irqs = 7; combined(6);
    reset_test(); clock_us = 0xffffff00u; combined(6);
    reset_test();
    UB rx[6];
    assert(dev_i2c_llctl(0, LLD_I2C_READ, 0x1d, 6, (UW*)rx) == 6);
    assert(received == 6 && !(commands[0] & I2C_DATA_CMD_RESTART));
    for (int phase = 1; phase <= 5; phase++) {
        reset_test(); abort_after = phase;
        UB addr = 0xa8;
        T_I2C_EXEC ex = {0x1d, 1, &addr, 6, rx};
        assert(dev_i2c_llctl(0, LLD_I2C_EXEC, 0, 0, (UW*)&ex) == E_IO);
        assert(wait_calls == 1 && flag_sets == 1 && dispatch_requests == 1);
        assert(*reg(I2C_ENABLE(0)) == 0 && *reg(I2C_INTR_MASK(0)) == 0);
        recover();
    }
    reset_test(); forced_timeout = 1;
    *reg(I2C_STATUS(0)) = 0x456;
    assert(dev_i2c_llctl(0, LLD_I2C_WRITE, 0x1d, 2, (UW*)tx) == E_TMOUT);
    assert(*reg(I2C_INTR_MASK(0)) == 0 && *reg(I2C_ENABLE(0)) == 0);
    unsigned int trace[32] = {0};
    buddy4_i2c_timeout_trace(0, trace);
    assert(trace[0] == I2C_STS_SEND && trace[1] == 0 && trace[2] == 2);
    assert(trace[3] == I2C_INT_TX_EMPTY && trace[4] != 0 && trace[6] == 0x456);
    assert(trace[8] == 20000 && trace[9] == ~0u && trace[10] == 0);
    assert(trace[12] == ~0u && trace[14] == 1 && trace[15] == 1);
    /* NVIC can retain a pending IRQ after cleanup masks its peripheral cause.
     * It must not publish a new error/completion with INTR_STAT == 0. */
    ER previous_error = ll_devcb[0].ioerr;
    unsigned previous_sets = flag_sets;
    in_irq = 1; i2c_inthdr(INTNO_I2C0); in_irq = 0;
    assert(ll_devcb[0].ioerr == previous_error && flag_sets == previous_sets);
    recover();
    buddy4_i2c_timeout_trace(0, trace);
    assert(trace[3] == I2C_INT_TX_EMPTY && trace[6] == 0x456);
    for (int deadline = 2; deadline <= 4; deadline++) {
        reset_test(); forced_timeout = deadline;
        UB addr = 0xa8;
        T_I2C_EXEC ex = {0x1d, 1, &addr, 6, rx};
        assert(dev_i2c_llctl(0, LLD_I2C_EXEC, 0, 0, (UW*)&ex) == E_TMOUT);
        buddy4_i2c_timeout_trace(0, trace);
        assert(trace[8] >= 20000 && trace[10] > 0);
        if (deadline == 2) {
            assert(trace[0] == I2C_STS_RECV && trace[1] == 4 && trace[7] == 1);
            assert(trace[3] & I2C_INT_RX_FULL);
        } else if (deadline == 3) {
            assert(trace[0] == I2C_STS_RECV && trace[1] == 1 && trace[7] == 1);
            assert((trace[3] & (I2C_INT_RX_FULL | I2C_INT_STOP_DET)) ==
                   (I2C_INT_RX_FULL | I2C_INT_STOP_DET));
        } else {
            assert(trace[0] == I2C_STS_STOP && trace[12] >= 20000 && flag == 1);
        }
        recover();
        unsigned int retained[32];
        buddy4_i2c_timeout_trace(0, retained);
        for (unsigned i = 0; i < 32; i++) assert(retained[i] == trace[i]);
    }
    puts("Native I2C regressions passed: dispatch at IRQ return, pre-wait completion, masked/late STOP, delayed IRQ, 1..6-byte reads, abort/timeout recovery");
    return 0;
}
#endif

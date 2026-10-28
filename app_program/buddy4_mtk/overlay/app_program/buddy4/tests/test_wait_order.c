/* Real I2C driver + real kernel timer insertion/expiry and wait release.
 * Flag/scheduler/CPU and bus delivery remain explicit host models. */
#define BUDDY4_TIMED_KERNEL_TEST
#include "test_native_i2c.c"
#include "../../../kernel/tkernel/timer.c"
#include "../../../kernel/tkernel/wait.c"
#include <string.h>

static TCB task;
TCB *knl_ctxtsk = &task;
static const WSPEC flg_spec = { TTW_FLG, NULL, NULL };
static GCB event_flag;
static ER waited_result;
static UW request_start, next_tick, due_us, cleanup_us;
enum { NO_BLACKOUT, COMPLETED_AFTER_EXPIRY, PARTIAL_AFTER_EXPIRY, DELAYED_RESUME };
static int scenario;

ER timed_set_flag(void)
{
    BEGIN_CRITICAL_SECTION;
    if (waiting && !isQueEmpty(&event_flag.wait_queue)) {
        knl_wait_release_ok(&task);
        waiting = 0;
        flag &= ~1u; /* eventflag.c TWF_BITCLR, after releasing the waiter. */
        dispatch_requests++;
    }
    END_CRITICAL_SECTION;
    return 0;
}
static void tick(void)
{
    in_irq = 1;
    knl_timer_handler();
    in_irq = 0;
    if (task.state == TS_READY) waiting = 0;
}
static void regular_step(void)
{
    /* I2C priority 2 precedes priority-3 SysTick if both are pending. */
    service_step();
    if (clock_us >= next_tick) { tick(); next_tick += 10000; }
}
ER tk_wai_flg(ID id, UINT pattern, UINT mode, UINT *result, TMO timeout)
{
    assert(id == 1 && pattern == 1 && mode == (TWF_ORW | TWF_BITCLR));
    assert(timeout == 10 && !primask);
    wait_calls++;
    if (flag) { *result = flag; flag &= ~pattern; return E_OK; }
    waited_result = E_OK;
    task.wercd = &waited_result;
    task.wspec = &flg_spec;
    task.state = TS_READY;
    task.tskid = 10;
    *reg(SYST_CVR) = (next_tick-clock_us)*TMCLK_MHz-1;
    waiting = 1;
    BEGIN_CRITICAL_SECTION;
    knl_gcb_make_wait(&event_flag, timeout);
    END_CRITICAL_SECTION;
    assert(task.wtmeb.time == (UW)knl_current_time + 20);
    /* Request starts 9.9 ms after a serviced tick. The queued deadline is
     * global kernel time 20 ms: only 10.1 ms from this request, not 20 ms. */
    due_us = next_tick + 10000 - request_start;
    assert(due_us == 20000-request_start);
    if (scenario == COMPLETED_AFTER_EXPIRY || scenario == PARTIAL_AFTER_EXPIRY) {
        for (unsigned i = 0; received < 2; i++) { assert(i < 100); regular_step(); }
        assert(knl_current_time == 10 && task.state == TS_WAIT);
        UW old = mock_disint();
        /* Deterministic interrupt blackout with polled-UART-scale duration.
         * Hardware ticks continue, but pending SysTicks coalesce under DI. */
        clock_us = request_start + (scenario == COMPLETED_AFTER_EXPIRY ? 18450 : 23860);
        mock_restore_interrupts(old);
        service_step(); /* Pending RX wins: byte three drained, next queued. */
        tick(); /* Expiry removes waiter before subsequent byte/STOP IRQs. */
        next_tick = (clock_us/10000 + 1)*10000;
        assert(waited_result == E_TMOUT && task.state == TS_READY && !waiting);
        if (scenario == COMPLETED_AFTER_EXPIRY) {
            for (unsigned i = 0; !flag; i++) { assert(i < 100); service_step(); }
            assert(ll_devcb[0].done_us == 18900 && irq_calls == 8);
            clock_us = request_start + 19500;
        } else {
            assert(ll_devcb[0].state == I2C_STS_RECV && ll_devcb[0].rdat_num == 3);
            clock_us = request_start + 24000;
        }
    } else {
        for (unsigned i = 0; waiting; i++) { assert(i < 100); regular_step(); }
        assert(waited_result == E_OK && isQueEmpty(&knl_timer_queue));
        if (scenario == DELAYED_RESUME) {
            clock_us = request_start + 19500;
            tick(); /* Removed wait timer cannot turn E_OK into a timeout. */
        }
    }
    cleanup_us = clock_us-request_start;
    *result = 1;
    return waited_result;
}
static void run(int which, UW phase)
{
    reset_test();
    memset(&task, 0, sizeof(task));
    memset(&event_flag, 0, sizeof(event_flag));
    event_flag.objid = 1;
    QueInit(&event_flag.wait_queue);
    QueInit(&task.tskque);
    QueInit(&task.wtmeb.queue);
    knl_timer_startup();
    clock_us = request_start = phase;
    next_tick = 10000;
    scenario = which;
    UB addr = 0xa8, rx[6] = {0};
    T_I2C_EXEC ex = {0x1d, 1, &addr, 6, rx};
    ER r = dev_i2c_llctl(0, LLD_I2C_EXEC, 0, 0, (UW*)&ex);
    if (which == NO_BLACKOUT || which == DELAYED_RESUME) {
        assert(r == 7 && received == 6 && irq_calls == 8);
        assert(ll_devcb[0].wait_probe[5] == ~0u);
        assert(ll_devcb[0].wait_probe[7] < due_us);
    } else {
        assert(r == E_TMOUT);
        unsigned int t[32]; buddy4_i2c_timeout_trace(0, t);
        assert(t[17] == 0 && t[18] == 20 && t[19] == 100);
        assert(t[21] >= due_us && t[22] == 20 && t[23] == ~0u);
        assert(t[25] == cleanup_us && t[29] == (UW)E_TMOUT);
        assert(t[30] == I2C_STS_RECV && t[31] == 3);
        if (which == COMPLETED_AFTER_EXPIRY) {
            assert(t[0] == I2C_STS_STOP && !t[1] && !t[2] && t[10] == 8);
            assert(t[21] < t[12] && t[12] == 18900 && t[12] <= t[24] && t[24] <= t[25]);
            assert(flag == 1); /* Set after expiry; it did not release a waiter. */
        } else {
            assert(t[0] == I2C_STS_RECV && t[1] == 3 && t[12] == ~0u);
            assert(t[25] == 24000);
        }
        printf("Reproduced %s: due=%u expiry=%u done=%u resume=%u read_left=%u\n",
            which == COMPLETED_AFTER_EXPIRY ? "completed" : "partial", due_us,t[21],t[12],t[25],t[1]);
    }
}
int main(void)
{
    T_I2C_DCB d = {0}; assert(dev_i2c_llinit(&d) == E_OK);
    assert(dev_i2c_llctl(0, LLD_I2C_OPEN, 0, 0, NULL) == E_OK);
    run(COMPLETED_AFTER_EXPIRY, 9900);
    run(PARTIAL_AFTER_EXPIRY, 9900);
    run(NO_BLACKOUT, 9900);
    run(DELAYED_RESUME, 9900);
    run(NO_BLACKOUT, 0);
    run(NO_BLACKOUT, 1);
    run(NO_BLACKOUT, 9999);
    puts("Real kernel timer/wait tests passed: phase deadline, expiry-before-STOP, partial expiry, on-time completion with late resumption");
}

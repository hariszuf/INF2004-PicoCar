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
enum
{
    NO_BLACKOUT, COMPLETED_AFTER_EXPIRY, PARTIAL_AFTER_EXPIRY, DELAYED_RESUME
};
static int scenario;

ER timed_set_flag (void)
{
    BEGIN_CRITICAL_SECTION;

    if (waiting && (!isQueEmpty(&event_flag.wait_queue)))
    {
        knl_wait_release_ok(&task);
        waiting = 0;
        flag &= ~1u; /* eventflag.c TWF_BITCLR, after releasing the waiter. */
        dispatch_requests++;
    }

    END_CRITICAL_SECTION;
    return 0;
}

static void tick (void)
{
    in_irq = 1;
    knl_timer_handler();
    in_irq = 0;

    if (TS_READY == task.state)
    {
        waiting = 0;
    }
}

static void regular_step (void)
{
    /* I2C priority 2 precedes priority-3 SysTick if both are pending. */
    service_step();

    if (clock_us >= next_tick)
    {
        tick();
        next_tick += 10000;
    }
}

ER tk_wai_flg (ID id, UINT pattern, UINT mode, UINT *result, TMO timeout)
{
    assert((1 == id) && (1 == pattern) && (mode == (TWF_ORW | TWF_BITCLR)));
    assert((10 == timeout) && (!primask));
    wait_calls++;

    if (flag)
    {
        *result = flag;
        flag &= ~pattern;
        return E_OK;
    }

    waited_result = E_OK;
    task.wercd = &waited_result;
    task.wspec = &flg_spec;
    task.state = TS_READY;
    task.tskid = 10;
    *reg(SYST_CVR) = (next_tick - clock_us) * TMCLK_MHz - 1;
    waiting = 1;
    BEGIN_CRITICAL_SECTION;
    knl_gcb_make_wait(&event_flag, timeout);
    END_CRITICAL_SECTION;
    assert(task.wtmeb.time == (UW)knl_current_time + 20);
    /* Request starts 9.9 ms after a serviced tick. The queued deadline is
     * global kernel time 20 ms: only 10.1 ms from this request, not 20 ms. */
    due_us = next_tick + 10000 - request_start;
    assert(due_us == 20000 - request_start);

    if ((COMPLETED_AFTER_EXPIRY == scenario)
        || (PARTIAL_AFTER_EXPIRY == scenario))
    {
        for (uint32_t idx = 0; received < 2; idx++)
        {
            assert(idx < 100);
            regular_step();
        }

        assert((10 == knl_current_time) && (TS_WAIT == task.state));
        UW old = mock_disint();
        /* Deterministic interrupt blackout with polled-UART-scale duration.
         * Hardware ticks continue, but pending SysTicks coalesce under DI. */
        clock_us = request_start
                + (COMPLETED_AFTER_EXPIRY == scenario ? 18450 : 23860);
        mock_restore_interrupts(old);
        service_step(); /* Pending RX wins: byte three drained, next queued. */
        tick(); /* Expiry removes waiter before subsequent byte/STOP IRQs. */
        next_tick = (clock_us / 10000 + 1) * 10000;
        assert((E_TMOUT == waited_result) && (TS_READY == task.state)
               && (!waiting));

        if (COMPLETED_AFTER_EXPIRY == scenario)
        {
            for (uint32_t idx = 0; !flag; idx++)
            {
                assert(idx < 100);
                service_step();
            }

            assert((18900 == ll_devcb[0].done_us) && (8 == irq_calls));
            clock_us = request_start + 19500;
        }
        else
        {
            assert((I2C_STS_RECV == ll_devcb[0].state)
                   && (3 == ll_devcb[0].rdat_num));
            clock_us = request_start + 24000;
        }
    }
    else
    {
        for (uint32_t idx = 0; waiting; idx++)
        {
            assert(idx < 100);
            regular_step();
        }

        assert((E_OK == waited_result) && (isQueEmpty(&knl_timer_queue)));

        if (DELAYED_RESUME == scenario)
        {
            clock_us = request_start + 19500;
            tick(); /* Removed wait timer cannot turn E_OK into a timeout. */
        }
    }

    cleanup_us = clock_us - request_start;
    *result = 1;
    return waited_result;
}

static void run (int which, UW phase)
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
    ER r = dev_i2c_llctl(0, LLD_I2C_EXEC, 0, 0, (UW*) & ex);

    if ((NO_BLACKOUT == which) || (DELAYED_RESUME == which))
    {
        assert((7 == r) && (6 == received) && (8 == irq_calls));
        assert(ll_devcb[0].wait_probe[5] == ~0u);
        assert(ll_devcb[0].wait_probe[7] < due_us);
    }
    else
    {
        assert(E_TMOUT == r);
        uint32_t t[32];
        buddy4_i2c_timeout_trace(0, t);
        assert((0 == t[17]) && (20 == t[18]) && (100 == t[19]));
        assert((t[21] >= due_us) && (20 == t[22]) && (t[23] == ~0u));
        assert((t[25] == cleanup_us) && (t[29] == (UW)E_TMOUT));
        assert((I2C_STS_RECV == t[30]) && (3 == t[31]));

        if (COMPLETED_AFTER_EXPIRY == which)
        {
            assert((I2C_STS_STOP == t[0]) && (!t[1]) && (!t[2])
                   && (8 == t[10]));
            assert((t[21] < t[12]) && (18900 == t[12]) && (t[12] <= t[24])
                   && (t[24] <= t[25]));
            /* Set after expiry; it did not release a waiter. */
            assert(1 == flag);
        }
        else
        {
            assert((I2C_STS_RECV == t[0]) && (3 == t[1]) && (t[12] == ~0u));
            assert(24000 == t[25]);
        }

        printf("Reproduced %s: due=%u expiry=%u done=%u resume=%u "
               "read_left=%u\n",
               COMPLETED_AFTER_EXPIRY == which ? "completed" : "partial",
                       due_us,
               t[21], t[12], t[25], t[1]);
    }
}

int main (void)
{
    T_I2C_DCB d = {0};
    assert(E_OK == dev_i2c_llinit(&d));
    assert(E_OK == dev_i2c_llctl(0, LLD_I2C_OPEN, 0, 0, NULL));
    run(COMPLETED_AFTER_EXPIRY, 9900);
    run(PARTIAL_AFTER_EXPIRY, 9900);
    run(NO_BLACKOUT, 9900);
    run(DELAYED_RESUME, 9900);
    run(NO_BLACKOUT, 0);
    run(NO_BLACKOUT, 1);
    run(NO_BLACKOUT, 9999);
    puts("Real kernel timer/wait tests passed: phase deadline, "
         "expiry-before-STOP, partial expiry, on-time completion with late "
         "resumption");
}

/*** end of file ***/

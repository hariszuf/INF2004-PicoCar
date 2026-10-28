/**
 * @file motion_task_mtk.c
 * @brief Buddy 2 Motion Control task for micro T-Kernel (trial).
 *
 * Owns the motors and encoders. Other tasks only call the motion_* API, which
 * queues commands and is safe across tasks. motion_update() measures its own
 * time step, so 10 ms kernel-tick jitter is tolerated.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "motion_config.h"
#include "motion_control.h"
#include "motion_mtk_hw.h"
#include "motion_task_mtk.h"

static ID s_task_id = 0;
static volatile UW s_updates = 0u;
static volatile UW s_max_gap_us = 0u;

static void buddy2_task(INT stacd, void *exinf)
{
    uint64_t last = motion_mtk_time_us();
    uint64_t now;
    UW gap;

    (void)stacd;
    (void)exinf;
    for (;;)
    {
        motion_update();
        now = motion_mtk_time_us();
        gap = (UW)(now - last);
        if ((s_updates > 0u) && (gap > s_max_gap_us))
        {
            s_max_gap_us = gap;
        }
        last = now;
        s_updates++;
        (void)tk_dly_tsk((TMO)MOTION_UPDATE_PERIOD_MS);
    }
}

static T_CTSK s_task_config = {
    .itskpri = BUDDY2_TASK_PRIORITY,
    .stksz = BUDDY2_TASK_STACK,
    .task = buddy2_task,
    .tskatr = TA_HLNG | TA_RNG3
};

ER buddy2_init(void)
{
    ER err = (ER)motion_mtk_hw_init();

    if (err < E_OK)
    {
        tm_printf((UB *)"[B2 Motion] hardware init failed: %d\n", (INT)err);
        return err;
    }
    if (motion_init() != MOTION_OK)
    {
        return E_SYS;
    }
    tm_printf((UB *)"[B2 Motion] Ready: motors braked (GP8-11), encoders GP6 left / GP4 right\n");
    return E_OK;
}

ER buddy2_start(void)
{
    if (s_task_id > 0)
    {
        return E_OBJ;
    }
    s_task_id = tk_cre_tsk(&s_task_config);
    if (s_task_id < E_OK)
    {
        return s_task_id;
    }
    return tk_sta_tsk(s_task_id, 0);
}

void buddy2_task_stats(UW *p_updates, UW *p_max_gap_us)
{
    *p_updates = s_updates;
    *p_max_gap_us = s_max_gap_us;
}

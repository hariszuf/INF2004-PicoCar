/**
 * @file motion_task.c
 * @brief Motion Control RTOS task entry for micro T-Kernel (skeleton).
 *
 * NOT built or tested here: it needs the team's micro T-Kernel template, which
 * was not available. Check the task creation call, the delay function and the
 * stack size against the template, and set the priority from the Week 6 task
 * table (Motion Control = High, below Safety Monitor).
 *
 * Define MOTION_USE_UTKERNEL in the build to compile this file.
 */
#ifdef MOTION_USE_UTKERNEL

#include <tk/tkernel.h>

#include "motion_config.h"
#include "motion_control.h"

/**
 * Task body. motion_update() measures its own time step, so a small delay
 * jitter is tolerated. Commands from the Vehicle Controller call the
 * motion_* functions directly; they only queue and are safe from other tasks.
 */
void motion_task_entry(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    (void)motion_init();

    for (;;)
    {
        motion_update();
        (void)tk_dly_tsk((TMO)MOTION_UPDATE_PERIOD_MS);
    }
}

#endif /* MOTION_USE_UTKERNEL */

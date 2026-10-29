/**
 * @file motion_task_mtk.h
 * @brief Buddy 2 Motion Control task for micro T-Kernel (trial).
 */
#ifndef MOTION_TASK_MTK_H
#define MOTION_TASK_MTK_H

#include <tk/tkernel.h>

/** Motion Control: highest application priority (Buddy 1 uses 6-9). */
#define BUDDY2_TASK_PRIORITY        (3)
#define BUDDY2_TASK_STACK           (4096)

/** Phase 1: release PWM/TIMER, motors braked, encoders counting. */
ER buddy2_init(void);

/** Phase 2: start the motion_update() task (every MOTION_UPDATE_PERIOD_MS). */
ER buddy2_start(void);

/** Number of motion_update() calls and the longest gap between two (us). */
void buddy2_task_stats(UW *p_updates, UW *p_max_gap_us);

#endif /* MOTION_TASK_MTK_H */

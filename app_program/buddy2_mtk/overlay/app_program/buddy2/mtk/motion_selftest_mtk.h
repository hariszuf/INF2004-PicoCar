/**
 * @file motion_selftest_mtk.h
 * @brief Buddy 2 micro T-Kernel trial: scripted motion self-test task.
 */
#ifndef MOTION_SELFTEST_MTK_H
#define MOTION_SELFTEST_MTK_H

#include <tk/tkernel.h>

#define BUDDY2_SELFTEST_PRIORITY    (12)     /* below Buddy 1 (6-9): print-only work */
#define BUDDY2_SELFTEST_DELAY_S     (10)     /* time to put the car on a stand / floor */

/** Start the self-test task (after buddy2_start()). */
ER buddy2_selftest_start(void);

#endif /* MOTION_SELFTEST_MTK_H */

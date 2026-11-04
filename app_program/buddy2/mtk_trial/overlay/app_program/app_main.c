/*
 * Buddy 2 micro T-Kernel trial - application entry point.
 *
 * This trial image runs Buddy 2 ONLY (Motion task + self-test), so its serial
 * output is just Buddy 2's and any problem found belongs to Buddy 2. The team
 * app_main.c (Buddy 1 + Buddy 5) is not changed; see buddy2_mtk/README.md.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "buddy2/mtk/motion_task_mtk.h"
#include "buddy2/mtk/motion_selftest_mtk.h"

EXPORT INT usermain(void)
{
    ER error_code;

    /* Phase 1: motors braked and encoders counting before any task runs. */
    error_code = buddy2_init();
    if (error_code < E_OK) {
        tm_printf((UB *)"[APP] Buddy 2 init failed: %d\n", error_code);
        return 1;
    }

    /* Phase 2: Motion task (priority 3), then the self-test (priority 12). */
    error_code = buddy2_start();
    if (error_code < E_OK) {
        tm_printf((UB *)"[APP] Buddy 2 start failed: %d\n", error_code);
        return 1;
    }
    error_code = buddy2_selftest_start();
    if (error_code < E_OK) {
        tm_printf((UB *)"[APP] Buddy 2 self-test start failed: %d\n", error_code);
        return 1;
    }

    /* Returning from usermain() would shut the application down. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}


#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "buddy1/buddy1.h"
/* Buddy 2 micro T-Kernel trial: Buddy 1 + Buddy 2 only, so any fault is ours.
 * Buddy 5 is left out of this image on purpose (see buddy2_mtk/README.md). */
#include "buddy2/mtk/motion_task_mtk.h"
#include "buddy2/mtk/motion_selftest_mtk.h"

/*
 * =====================================================
 * APPLICATION ENTRY POINT
 * =====================================================
 *
 * µT-Kernel calls usermain() after system startup.
 *
 * The application follows two phases:
 *
 * Phase 1: Initialise all subsystem resources.
 * Phase 2: Start all subsystem tasks.
 *
 * Once the other buddies are implemented,
 * they will be integrated into both phases.
 */

EXPORT INT usermain(void)
{
    ER error_code;

    /*
     * =================================================
     * PHASE 1: SUBSYSTEM INITIALISATION
     * =================================================
     *
     * All kernel IPC objects must be created
     * before application tasks begin running.
     */

    error_code = buddy1_init();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[APP] Buddy 1 init failed: %d\n",
            error_code
        );

        return 1;
    }


    /* Buddy 2 first after Buddy 1: motors braked before any task runs. */
    error_code = buddy2_init();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[APP] Buddy 2 init failed: %d\n",
            error_code
        );

        return 1;
    }

    /*
     * Future subsystem initialisation:
     *
     * buddy2_init();
     * buddy3_init();
     * buddy4_init();
     *
     * These calls will be added when
     * the respective buddies are ready.
     */


    /*
     * =================================================
     * PHASE 2: SUBSYSTEM TASK STARTUP
     * =================================================
     *
     * All required shared kernel objects
     * should now be available.
     */

    error_code = buddy1_start();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[APP] Buddy 1 start failed: %d\n",
            error_code
        );

        return 1;
    }

    error_code = buddy2_start();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[APP] Buddy 2 start failed: %d\n",
            error_code
        );

        return 1;
    }

    error_code = buddy2_selftest_start();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[APP] Buddy 2 self-test start failed: %d\n",
            error_code
        );

        return 1;
    }

    /*
     * Future subsystem startup:
     *
     * buddy2_start();
     * buddy3_start();
     * buddy4_start();
     */


    /*
     * =================================================
     * SYSTEM RUNNING
     * =================================================
     */

    tm_printf(
        (UB *)"[APP] All registered buddies started\n"
    );

    /*
     * Keep the initial task asleep.
     *
     * Returning from usermain() would shut down
     * the µT-Kernel application.
     */
    tk_slp_tsk(TMO_FEVR);

    return 0;
}

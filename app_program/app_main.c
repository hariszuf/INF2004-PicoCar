
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "buddy1/buddy1.h"
#include "buddy5/buddy5.h"

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


    error_code = buddy5_init();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[APP] Buddy 5 init failed: %d\n",
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

    error_code = buddy5_start();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[APP] Buddy 5 start failed: %d\n",
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

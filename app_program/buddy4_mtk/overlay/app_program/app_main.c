
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "buddy1/buddy1.h"
#if !APP_BUDDY1_ONLY
#include "buddy5/buddy5.h"
#endif
#if APP_BUDDY4_ENABLED && !APP_BUDDY1_ONLY
#include "buddy4/buddy4_task.h"
static buddy4_task_snapshot_t buddy4_diagnostic;
#endif

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


#if !APP_BUDDY1_ONLY
    error_code = buddy5_init();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[APP] Buddy 5 init failed: %d\n",
            error_code
        );

        return 1;
    }

#endif
#if APP_BUDDY4_ENABLED && !APP_BUDDY1_ONLY
    error_code = buddy4_task_init();
    if (error_code < E_OK) {
        tm_printf((UB *)"[APP] Buddy 4 init failed: %d\n", error_code);
        return 1;
    }
#endif
    /*
     * Future subsystem initialisation:
     *
     * buddy2_init();
     * buddy3_init();
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

#if !APP_BUDDY1_ONLY
    error_code = buddy5_start();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[APP] Buddy 5 start failed: %d\n",
            error_code
        );

        return 1;
    }

#endif
#if APP_BUDDY4_ENABLED && !APP_BUDDY1_ONLY
    error_code = buddy4_task_start();
    if (error_code < E_OK) {
        tm_printf((UB *)"[APP] Buddy 4 start failed: %d\n", error_code);
        return 1;
    }
#endif
    /*
     * Future subsystem startup:
     *
     * buddy2_start();
     * buddy3_start();
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
#if APP_BUDDY4_ENABLED && !APP_BUDDY1_ONLY
    /* Read-only low-priority diagnostics; native tm_printf has no float format.
     * milli_cm means 0.001 cm; mdeg means 0.001 degree. No motor actions. */
    (void)tk_chg_pri(TSK_SELF, 14);
    tm_printf((UB *)"# B4_MTK: seq valid errors pitch_mdeg roll_mdeg height_milli_cm events stable_milli_cm median_milli_cm skipped\n");
    for (;;) {
        if (buddy4_task_get(&buddy4_diagnostic) >= E_OK) {
            tm_printf((UB *)"B4,%u,%u,%u,%d,%d,%d,%u,%d,%d,%u\n",
                (UINT)buddy4_diagnostic.sequence, (UINT)buddy4_diagnostic.valid,
                (UINT)buddy4_diagnostic.io_errors,
                (INT)(buddy4_diagnostic.pitch_deg * 1000),
                (INT)(buddy4_diagnostic.roll_deg * 1000),
                (INT)(buddy4_diagnostic.height_cm * 1000),
                (UINT)buddy4_diagnostic.completed_events,
                buddy4_diagnostic.last_event.stable_valid ?
                    (INT)(buddy4_diagnostic.last_event.stable_cm * 1000) : -1,
                buddy4_diagnostic.last_event.median_valid ?
                    (INT)(buddy4_diagnostic.last_event.median_cm * 1000) : -1,
                (UINT)buddy4_diagnostic.skipped_slots);
        }
        (void)tk_dly_tsk(1000);
    }
#else
    tk_slp_tsk(TMO_FEVR);
#endif

    return 0;
}

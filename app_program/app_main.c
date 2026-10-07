#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "buddy1/heartbeat.h"
#include "buddy1/telemetry.h"


EXPORT INT usermain(void)
{
    ER error_code;


    /*
     * Start telemetry first because other subsystems depend on it.
     */
    error_code = buddy1_telemetry_start();

    if (error_code < E_OK) {

        tm_printf(
            (UB *)"[INIT] Telemetry start failed: %d\n",
            error_code
        );

        return 1;
    }


    /*
     * Heartbeat may now safely publish telemetry.
     */
    error_code = buddy1_heartbeat_start();

    if (error_code < E_OK) {

        tm_printf(
            (UB *)"[INIT] Heartbeat start failed: %d\n",
            error_code
        );

        return 1;
    }


    /*
     * The initial µT-Kernel task must stay alive.
     */
    tk_slp_tsk(TMO_FEVR);

    return 0;
}
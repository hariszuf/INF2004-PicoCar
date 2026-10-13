#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "buddy1/heartbeat.h"
#include "buddy1/telemetry.h"
#include "buddy1/command.h"
#include "buddy1/network.h"


EXPORT INT usermain(void)
{
    ER error_code;

    /*
     * Start telemetry first.
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
     * Start command subsystem.
     */
    error_code = buddy1_command_start();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[INIT] Command start failed: %d\n",
            error_code
        );

        return 1;
    }


    /*
     * Start network status monitoring.
     */
    error_code = buddy1_network_start();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[INIT] Network task start failed: %d\n",
            error_code
        );

        return 1;
    }


    /*
     * Start heartbeat last because it publishes telemetry.
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
     * Keep the initial µT-Kernel task alive.
     */
    tk_slp_tsk(TMO_FEVR);

    return 0;
}
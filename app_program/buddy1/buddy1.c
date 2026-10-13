
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "buddy1.h"

#include "telemetry.h"
#include "command.h"
#include "network.h"
#include "heartbeat.h"

/*
 * =====================================================
 * PHASE 1: INITIALISATION
 * =====================================================
 *
 * Initialise all shared Buddy 1 kernel resources
 * before any application task starts.
 */
ER buddy1_init(void)
{
    ER error_code;

    /*
     * Initialise telemetry message buffer.
     */
    error_code = buddy1_telemetry_init();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[Buddy1] Telemetry init failed: %d\n",
            error_code
        );

        return error_code;
    }

    /*
     * Initialise command message buffer.
     */
    error_code = buddy1_command_init();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[Buddy1] Command init failed: %d\n",
            error_code
        );

        return error_code;
    }

    /*
     * Network monitoring and heartbeat currently
     * require no additional shared kernel objects.
     *
     * Their tasks are created during Phase 2.
     */

    return E_OK;
}


/*
 * =====================================================
 * PHASE 2: TASK STARTUP
 * =====================================================
 *
 * All shared kernel objects must already exist.
 */
ER buddy1_start(void)
{
    ER error_code;

    /*
     * Start telemetry consumer first.
     */
    error_code = buddy1_telemetry_start();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[Buddy1] Telemetry start failed: %d\n",
            error_code
        );

        return error_code;
    }

    /*
     * Start command consumer.
     */
    error_code = buddy1_command_start();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[Buddy1] Command start failed: %d\n",
            error_code
        );

        return error_code;
    }

    /*
     * Start Wi-Fi/network status monitor.
     *
     * Actual Wi-Fi, DHCP and DNS operations remain
     * inside the kernel-owned networking service.
     */
    error_code = buddy1_network_start();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[Buddy1] Network start failed: %d\n",
            error_code
        );

        return error_code;
    }

    /*
     * Start heartbeat last because it depends
     * on the telemetry message buffer.
     */
    error_code = buddy1_heartbeat_start();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[Buddy1] Heartbeat start failed: %d\n",
            error_code
        );

        return error_code;
    }

    return E_OK;
}

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "buddy1_heartbeat.h"

EXPORT INT usermain(void)
{
    ER error_code;

    tm_printf((UB *)"\n=== PicoCar Buddy 1 ===\n");

    error_code = buddy1_heartbeat_start();

    if (error_code < E_OK) {
        tm_printf(
            (UB *)"[INIT] Heartbeat start failed: %d\n",
            error_code
        );

        return 1;
    }

    tk_slp_tsk(TMO_FEVR);

    return 0;
}
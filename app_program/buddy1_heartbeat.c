#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "buddy1_heartbeat.h"

LOCAL void heartbeat_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    while (1) {
        tm_printf((UB *)"[Heartbeat] Buddy 1 alive\n");

        tk_dly_tsk(1000);
    }
}

LOCAL T_CTSK heartbeat_task_config = {
    .itskpri = 8,
    .stksz   = 4096,
    .task    = heartbeat_task,
    .tskatr  = TA_HLNG | TA_RNG3
};

ER buddy1_heartbeat_start(void)
{
    ID task_id;

    task_id = tk_cre_tsk(&heartbeat_task_config);

    if (task_id < E_OK) {
        return task_id;
    }

    return tk_sta_tsk(task_id, 0);
}
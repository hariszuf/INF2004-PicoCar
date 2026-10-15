#include <stdint.h>

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "heartbeat.h"
#include "telemetry.h"


LOCAL void heartbeat_task(INT stacd, void *exinf)
{
    telemetry_message_t message;
    uint32_t sequence = 0U;
    ER error_code;

    (void)stacd;
    (void)exinf;


    while (1) {

        sequence++;

        message.source = TELEMETRY_SOURCE_BUDDY1;
        message.type = TELEMETRY_TYPE_HEARTBEAT;
        message.flags = 0U;

        message.value_1 = 1;
        message.value_2 = 0;

        message.sequence = sequence;


        /*
         * This call is non-blocking.
         */
        error_code = buddy1_telemetry_submit(
            &message
        );


        if (error_code < E_OK) {

            tm_printf(
                (UB *)"[Heartbeat] Telemetry submit failed: %d\n",
                error_code
            );
        }


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

    task_id = tk_cre_tsk(
        &heartbeat_task_config
    );

    if (task_id < E_OK) {
        return task_id;
    }

    return tk_sta_tsk(
        task_id,
        0
    );
}
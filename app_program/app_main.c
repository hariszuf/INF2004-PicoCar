#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "buddy1/heartbeat.h"
#include "buddy1/telemetry.h"
#include "buddy1/command.h"

LOCAL void command_test_task(INT stacd, void *exinf)
{
    command_message_t message;
    uint32_t sequence = 0U;

    (void)stacd;
    (void)exinf;


    while (1) {

        sequence++;

        message.type = COMMAND_TYPE_START;
        message.source = 1U;
        message.flags = 0U;
        message.value = 0;
        message.sequence = sequence;

        buddy1_command_submit(&message);

        tk_dly_tsk(3000);


        sequence++;

        message.type = COMMAND_TYPE_STOP;
        message.sequence = sequence;

        buddy1_command_submit(&message);

        tk_dly_tsk(3000);
    }
}


LOCAL T_CTSK command_test_task_config = {
    .itskpri = 9,
    .stksz   = 4096,
    .task    = command_test_task,
    .tskatr  = TA_HLNG | TA_RNG3
};

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

    error_code = buddy1_command_start();

if (error_code < E_OK) {

    tm_printf(
        (UB *)"[INIT] Command start failed: %d\n",
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
     * Command test task is a temporary task that submits commands to the command subsystem.
     */

    ID command_test_task_id;

command_test_task_id = tk_cre_tsk(
    &command_test_task_config
);

if (command_test_task_id < E_OK) {

    tm_printf(
        (UB *)"[INIT] Command test task creation failed: %d\n",
        command_test_task_id
    );

    return 1;
}

error_code = tk_sta_tsk(
    command_test_task_id,
    0
);

if (error_code < E_OK) {

    tm_printf(
        (UB *)"[INIT] Command test task start failed: %d\n",
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
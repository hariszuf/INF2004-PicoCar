#include <stdint.h>

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "telemetry.h"


#define TELEMETRY_QUEUE_DEPTH    8U
#define TELEMETRY_TASK_PRIORITY  7
#define TELEMETRY_STACK_SIZE     4096


/*
 * Kernel message buffer ID.
 *
 * It becomes valid after buddy1_telemetry_start().
 */
LOCAL ID telemetry_buffer_id = 0;


/*
 * Number of messages that could not be queued.
 */
LOCAL uint32_t telemetry_dropped_count = 0U;


/*
 * Backing memory used by the µT-Kernel message buffer.
 *
 * Extra space is provided for the message-buffer bookkeeping
 * used internally by µT-Kernel.
 */
#define TELEMETRY_MESSAGE_WORDS \
    ((sizeof(telemetry_message_t) + sizeof(UW) - 1U) / sizeof(UW))

LOCAL UW telemetry_buffer_memory[
    (TELEMETRY_QUEUE_DEPTH * TELEMETRY_MESSAGE_WORDS)
    + (TELEMETRY_QUEUE_DEPTH * 4U)
];


/*
 * µT-Kernel message-buffer configuration.
 */
LOCAL T_CMBF telemetry_buffer_config = {
    .mbfatr = TA_TFIFO,
    .bufsz  = sizeof(telemetry_buffer_memory),
    .maxmsz = sizeof(telemetry_message_t),
    .bufptr = telemetry_buffer_memory
};


/*
 * Buddy 1 telemetry consumer task.
 *
 * For now this prints the received data.
 *
 * Later this is where messages will be encoded and passed
 * towards the MQTT/network layer.
 */
LOCAL void telemetry_task(INT stacd, void *exinf)
{
    telemetry_message_t message;
    INT received_size;

    (void)stacd;
    (void)exinf;

    while (1) {

        /*
         * This task is allowed to block because its entire job is
         * waiting for telemetry.
         */
        received_size = tk_rcv_mbf(
            telemetry_buffer_id,
            &message,
            TMO_FEVR
        );

        if (received_size != (INT)sizeof(telemetry_message_t)) {

            tm_printf(
                (UB *)"[Telemetry] Invalid message size: %d\n",
                received_size
            );

            continue;
        }

        tm_printf(
            (UB *)"[Telemetry] src=%u type=%u seq=%u value1=%d value2=%d\n",
            (unsigned int)message.source,
            (unsigned int)message.type,
            (unsigned int)message.sequence,
            (int)message.value_1,
            (int)message.value_2
        );
    }
}


/*
 * Telemetry task configuration.
 */
LOCAL T_CTSK telemetry_task_config = {
    .itskpri = TELEMETRY_TASK_PRIORITY,
    .stksz   = TELEMETRY_STACK_SIZE,
    .task    = telemetry_task,
    .tskatr  = TA_HLNG | TA_RNG3
};


ER buddy1_telemetry_start(void)
{
    ID task_id;
    ER error_code;

    /*
     * Create the message buffer first.
     */
    telemetry_buffer_id = tk_cre_mbf(
        &telemetry_buffer_config
    );

    if (telemetry_buffer_id < E_OK) {
        return telemetry_buffer_id;
    }


    /*
     * Then create the consumer task.
     */
    task_id = tk_cre_tsk(
        &telemetry_task_config
    );

    if (task_id < E_OK) {
        return task_id;
    }


    /*
     * Start the task.
     */
    error_code = tk_sta_tsk(
        task_id,
        0
    );

    return error_code;
}


ER buddy1_telemetry_submit(
    const telemetry_message_t *message
)
{
    ER error_code;

    if (message == NULL) {
        return E_PAR;
    }

    if (telemetry_buffer_id <= 0) {
        return E_OBJ;
    }


    /*
     * TMO_POL means POLL.
     *
     * If the message buffer is full, do NOT put the calling
     * task to sleep waiting for space.
     */
    error_code = tk_snd_mbf(
        telemetry_buffer_id,
        message,
        sizeof(telemetry_message_t),
        TMO_POL
    );


    /*
     * A full queue produces E_TMOUT when using TMO_POL.
     *
     * Record the loss but do not block the producer.
     */
    if (error_code == E_TMOUT) {
        telemetry_dropped_count++;
    }

    return error_code;
}


uint32_t buddy1_telemetry_get_dropped_count(void)
{
    return telemetry_dropped_count;
}
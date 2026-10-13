
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
 * Created during Phase 1.
 */
LOCAL ID telemetry_buffer_id = 0;


/*
 * Count messages dropped due to a full queue.
 */
LOCAL uint32_t telemetry_dropped_count = 0U;


/*
 * Backing memory for the telemetry message buffer.
 */
#define TELEMETRY_MESSAGE_WORDS \
    ((sizeof(telemetry_message_t) + sizeof(UW) - 1U) / sizeof(UW))

LOCAL UW telemetry_buffer_memory[
    (TELEMETRY_QUEUE_DEPTH * TELEMETRY_MESSAGE_WORDS)
    + (TELEMETRY_QUEUE_DEPTH * 4U)
];


/*
 * Message buffer configuration.
 */
LOCAL T_CMBF telemetry_buffer_config = {
    .mbfatr = TA_TFIFO,
    .bufsz  = sizeof(telemetry_buffer_memory),
    .maxmsz = sizeof(telemetry_message_t),
    .bufptr = telemetry_buffer_memory
};


/*
 * Telemetry consumer task.
 *
 * Receives telemetry messages from the kernel
 * message buffer and prints them through USB.
 *
 * Later, this task will forward messages to
 * the MQTT networking layer.
 */
LOCAL void telemetry_task(INT stacd, void *exinf)
{
    telemetry_message_t message;
    INT received_size;

    (void)stacd;
    (void)exinf;

    while (1) {

        received_size = tk_rcv_mbf(
            telemetry_buffer_id,
            &message,
            TMO_FEVR
        );

        if (received_size !=
            (INT)sizeof(telemetry_message_t)) {

            tm_printf(
                (UB *)"[Telemetry] Invalid message size: %d\n",
                received_size
            );

            continue;
        }

        tm_printf(
            (UB *)"[Telemetry-2Phase] src=%u type=%u seq=%u value1=%d value2=%d\n",
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


/*
 * =========================================================
 * PHASE 1: INITIALISATION
 * =========================================================
 *
 * Create the telemetry message buffer.
 *
 * No application task is started here.
 */
ER buddy1_telemetry_init(void)
{
    /*
     * Avoid creating the message buffer twice.
     */
    if (telemetry_buffer_id > 0) {
        return E_OK;
    }

    telemetry_buffer_id = tk_cre_mbf(
        &telemetry_buffer_config
    );

    if (telemetry_buffer_id < E_OK) {
        return telemetry_buffer_id;
    }

    return E_OK;
}


/*
 * =========================================================
 * PHASE 2: TASK STARTUP
 * =========================================================
 *
 * Create and start the telemetry consumer task.
 *
 * The message buffer must already exist.
 */
ER buddy1_telemetry_start(void)
{
    ID task_id;

    /*
     * Verify Phase 1 completed successfully.
     */
    if (telemetry_buffer_id <= 0) {
        return E_OBJ;
    }

    /*
     * Create the telemetry consumer task.
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
    return tk_sta_tsk(
        task_id,
        0
    );
}


/*
 * =========================================================
 * TELEMETRY SUBMISSION API
 * =========================================================
 *
 * Non-blocking producer API used by other buddies.
 */
ER buddy1_telemetry_submit(
    const telemetry_message_t *message
)
{
    ER error_code;

    if (message == NULL) {
        return E_PAR;
    }

    /*
     * The message buffer must be initialised.
     */
    if (telemetry_buffer_id <= 0) {
        return E_OBJ;
    }

    /*
     * TMO_POL prevents the producer from blocking.
     */
    error_code = tk_snd_mbf(
        telemetry_buffer_id,
        message,
        sizeof(telemetry_message_t),
        TMO_POL
    );

    /*
     * Count messages dropped when the buffer is full.
     */
    if (error_code == E_TMOUT) {
        telemetry_dropped_count++;
    }

    return error_code;
}


/*
 * Retrieve the dropped-message counter.
 */
uint32_t buddy1_telemetry_get_dropped_count(void)
{
    return telemetry_dropped_count;
}

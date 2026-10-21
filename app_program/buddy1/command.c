
#include <stdint.h>

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "command.h"

#include "mqtt_command_bridge.h"

#define COMMAND_QUEUE_DEPTH    8U
#define COMMAND_TASK_PRIORITY  6
#define COMMAND_STACK_SIZE     4096

LOCAL ID command_buffer_id = 0;
LOCAL uint32_t command_dropped_count = 0U;

#define COMMAND_MESSAGE_WORDS \
    ((sizeof(command_message_t) + sizeof(UW) - 1U) / sizeof(UW))

LOCAL UW command_buffer_memory[
    (COMMAND_QUEUE_DEPTH * COMMAND_MESSAGE_WORDS)
    + (COMMAND_QUEUE_DEPTH * 4U)
];

LOCAL T_CMBF command_buffer_config = {
    .mbfatr = TA_TFIFO,
    .bufsz  = sizeof(command_buffer_memory),
    .maxmsz = sizeof(command_message_t),
    .bufptr = command_buffer_memory
};

LOCAL const char *command_type_to_string(uint8_t type)
{
    switch (type) {
        case COMMAND_TYPE_START:
            return "START";

        case COMMAND_TYPE_STOP:
            return "STOP";

        case COMMAND_TYPE_PAUSE:
            return "PAUSE";

        case COMMAND_TYPE_RESUME:
            return "RESUME";

        default:
            return "UNKNOWN";
    }
}

/*
 * Command consumer task.
 */
LOCAL void command_task(INT stacd, void *exinf)
{
    command_message_t message;
    INT received_size;

    (void)stacd;
    (void)exinf;

    while (1) {
        received_size = tk_rcv_mbf(
            command_buffer_id,
            &message,
            TMO_FEVR
        );

        if (received_size != (INT)sizeof(command_message_t)) {
            tm_printf(
                (UB *)"[Command] Invalid message size: %d\n",
                received_size
            );
            continue;
        }

        tm_printf(
            (UB *)"[Command] type=%s seq=%u value=%d\n",
            (UB *)command_type_to_string(message.type),
            (unsigned int)message.sequence,
            (int)message.value
        );
    }
}

LOCAL T_CTSK command_task_config = {
    .itskpri = COMMAND_TASK_PRIORITY,
    .stksz   = COMMAND_STACK_SIZE,
    .task    = command_task,
    .tskatr  = TA_HLNG | TA_RNG3
};

/*
 * PHASE 1: Create the message buffer.
 */
ER buddy1_command_init(void)
{
    if (command_buffer_id > 0) {
        return E_OK;
    }

    command_buffer_id = tk_cre_mbf(
        &command_buffer_config
    );

    if (command_buffer_id < E_OK) {
        return command_buffer_id;
    }

    return E_OK;
}

/*
 * PHASE 2: Create and start the consumer task.
 */
ER buddy1_command_start(void)
{
    ID task_id;

    if (command_buffer_id <= 0) {
        return E_OBJ;
    }

    task_id = tk_cre_tsk(
        &command_task_config
    );

    if (task_id < E_OK) {
        return task_id;
    }

    return tk_sta_tsk(task_id, 0);
}

/*
 * Non-blocking command producer API.
 */
ER buddy1_command_submit(
    const command_message_t *message
)
{
    ER error_code;

    if (message == NULL) {
        return E_PAR;
    }

    if (command_buffer_id <= 0) {
        return E_OBJ;
    }

    error_code = tk_snd_mbf(
        command_buffer_id,
        message,
        sizeof(command_message_t),
        TMO_POL
    );

    if (error_code == E_TMOUT) {
        command_dropped_count++;
    }

    return error_code;
}

uint32_t buddy1_command_get_dropped_count(void)
{
    return command_dropped_count;
}

/*
 * MQTT command sequence counter.
 * Only the MQTT networking owner task calls this bridge.
 */
static uint32_t mqtt_command_sequence = 0U;

/*
 * Compare an MQTT payload against an expected command.
 * Use explicit lengths because MQTT payloads are binary data
 * and are not guaranteed to be null-terminated.
 */
static uint8_t command_text_matches(
    const char *payload,
    uint16_t payload_length,
    const char *expected,
    uint16_t expected_length)
{
    uint16_t i;

    if (payload_length != expected_length) {
        return 0U;
    }

    for (i = 0U; i < expected_length; i++) {
        if (payload[i] != expected[i]) {
            return 0U;
        }
    }

    return 1U;
}

/*
 * MQTT -> µT-Kernel command bridge.
 *
 * Convert a command string into command_message_t
 * and submit it using the existing non-blocking API.
 */
int32_t buddy1_mqtt_command_ingest(
    const char *payload,
    uint16_t payload_length)
{
    command_message_t message = {0};

    if (payload == NULL) {
        return (int32_t)E_PAR;
    }

    if (command_text_matches(payload, payload_length,
                             "START", 5U) != 0U) {
        message.type = COMMAND_TYPE_START;
    }
    else if (command_text_matches(payload, payload_length,
                                  "STOP", 4U) != 0U) {
        message.type = COMMAND_TYPE_STOP;
    }
    else if (command_text_matches(payload, payload_length,
                                  "PAUSE", 5U) != 0U) {
        message.type = COMMAND_TYPE_PAUSE;
    }
    else if (command_text_matches(payload, payload_length,
                                  "RESUME", 6U) != 0U) {
        message.type = COMMAND_TYPE_RESUME;
    }
    else {
        return (int32_t)E_PAR;
    }

    /*
     * Source 1 represents Wi-Fi/Buddy 1.
     * No motor action is performed at this stage.
     */
    message.source = 1U;
    message.flags = 0U;
    message.value = 0;

    mqtt_command_sequence++;
    message.sequence = mqtt_command_sequence;

    return (int32_t)buddy1_command_submit(&message);
}

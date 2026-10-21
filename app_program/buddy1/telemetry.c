
#include <stdint.h>

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "telemetry.h"
#include "mqtt_telemetry_bridge.h"

#ifndef TM_WIFI_MQTT
#define TM_WIFI_MQTT 0
#endif

#define TELEMETRY_QUEUE_DEPTH    8U
#define TELEMETRY_TASK_PRIORITY  7
#define TELEMETRY_STACK_SIZE     4096

#define MQTT_OUTBOX_DEPTH        8U

#define TELEMETRY_MESSAGE_WORDS \
    ((sizeof(telemetry_message_t) + sizeof(UW) - 1U) / sizeof(UW))

/*
 * Existing telemetry input message buffer.
 */
LOCAL ID telemetry_buffer_id = 0;

LOCAL uint32_t telemetry_dropped_count = 0U;

LOCAL UW telemetry_buffer_memory[
    (TELEMETRY_QUEUE_DEPTH * TELEMETRY_MESSAGE_WORDS)
    + (TELEMETRY_QUEUE_DEPTH * 4U)
];

LOCAL T_CMBF telemetry_buffer_config = {
    .mbfatr = TA_TFIFO,
    .bufsz = sizeof(telemetry_buffer_memory),
    .maxmsz = sizeof(telemetry_message_t),
    .bufptr = telemetry_buffer_memory
};

/*
 * Additional MQTT outbox.
 *
 * The telemetry task sends copies here.
 * The networking owner task receives them.
 */
#if TM_WIFI_MQTT

LOCAL ID mqtt_outbox_id = 0;

LOCAL uint32_t mqtt_outbox_dropped_count = 0U;

LOCAL UW mqtt_outbox_memory[
    (MQTT_OUTBOX_DEPTH * TELEMETRY_MESSAGE_WORDS)
    + (MQTT_OUTBOX_DEPTH * 4U)
];

LOCAL T_CMBF mqtt_outbox_config = {
    .mbfatr = TA_TFIFO,
    .bufsz = sizeof(mqtt_outbox_memory),
    .maxmsz = sizeof(telemetry_message_t),
    .bufptr = mqtt_outbox_memory
};

#endif

/*
 * Telemetry consumer task.
 *
 * Receives messages from other buddies,
 * prints them, and forwards a copy to
 * the MQTT outbox when MQTT is enabled.
 */
LOCAL void telemetry_task(INT stacd, void *exinf)
{
    telemetry_message_t message;
    INT received_size;

#if TM_WIFI_MQTT
    ER send_result;
#endif

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

        /*
         * Existing USB serial diagnostics.
         */
        tm_printf(
            (UB *)"[Telemetry-2Phase] "
                  "src=%u type=%u seq=%u "
                  "value1=%d value2=%d\n",
            (unsigned int)message.source,
            (unsigned int)message.type,
            (unsigned int)message.sequence,
            (int)message.value_1,
            (int)message.value_2
        );

#if TM_WIFI_MQTT
        /*
         * Forward telemetry without blocking.
         *
         * A full outbox causes message loss,
         * not a delay in the telemetry task.
         */
        if (mqtt_outbox_id > 0) {

            send_result = tk_snd_mbf(
                mqtt_outbox_id,
                &message,
                sizeof(telemetry_message_t),
                TMO_POL
            );

            if (send_result != E_OK) {
                mqtt_outbox_dropped_count++;
            }
        }
#endif
    }
}

/*
 * Telemetry task configuration.
 */
LOCAL T_CTSK telemetry_task_config = {
    .itskpri = TELEMETRY_TASK_PRIORITY,
    .stksz = TELEMETRY_STACK_SIZE,
    .task = telemetry_task,
    .tskatr = TA_HLNG | TA_RNG3
};

/*
 * Phase 1: Create kernel IPC objects.
 */
ER buddy1_telemetry_init(void)
{
    if (telemetry_buffer_id <= 0) {

        telemetry_buffer_id = tk_cre_mbf(
            &telemetry_buffer_config
        );

        if (telemetry_buffer_id < E_OK) {
            return telemetry_buffer_id;
        }
    }

#if TM_WIFI_MQTT
    if (mqtt_outbox_id <= 0) {

        mqtt_outbox_id = tk_cre_mbf(
            &mqtt_outbox_config
        );

        if (mqtt_outbox_id < E_OK) {
            return mqtt_outbox_id;
        }
    }
#endif

    return E_OK;
}

/*
 * Phase 2: Start telemetry consumer task.
 */
ER buddy1_telemetry_start(void)
{
    ID task_id;

    if (telemetry_buffer_id <= 0) {
        return E_OBJ;
    }

    task_id = tk_cre_tsk(
        &telemetry_task_config
    );

    if (task_id < E_OK) {
        return task_id;
    }

    return tk_sta_tsk(
        task_id,
        0
    );
}

/*
 * Non-blocking telemetry producer API.
 *
 * Other buddies submit telemetry here.
 */
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

    error_code = tk_snd_mbf(
        telemetry_buffer_id,
        message,
        sizeof(telemetry_message_t),
        TMO_POL
    );

    if (error_code == E_TMOUT) {
        telemetry_dropped_count++;
    }

    return error_code;
}

/*
 * Non-blocking MQTT outbox receive API.
 *
 * Called by the Wi-Fi/lwIP network owner.
 * Returns E_TMOUT when the outbox is empty.
 */
ER buddy1_mqtt_telemetry_receive(
    telemetry_message_t *message
)
{
#if TM_WIFI_MQTT
    INT received_size;

    if (message == NULL) {
        return E_PAR;
    }

    if (mqtt_outbox_id <= 0) {
        return E_OBJ;
    }

    received_size = tk_rcv_mbf(
        mqtt_outbox_id,
        message,
        TMO_POL
    );

    if (received_size ==
        (INT)sizeof(telemetry_message_t)) {
        return E_OK;
    }

    if (received_size < 0) {
        return (ER)received_size;
    }

    return E_OBJ;
#else
    (void)message;
    return E_OBJ;
#endif
}

/*
 * Diagnostic counters.
 */
uint32_t buddy1_telemetry_get_dropped_count(void)
{
    return telemetry_dropped_count;
}

uint32_t buddy1_mqtt_telemetry_get_dropped_count(void)
{
#if TM_WIFI_MQTT
    return mqtt_outbox_dropped_count;
#else
    return 0U;
#endif
}

/*
 * Transfer one telemetry message to the
 * MQTT network-owner task without exposing
 * µT-Kernel headers to lwIP.
 */
int32_t buddy1_mqtt_telemetry_fetch(
    mqtt_telemetry_data_t *data
)
{
    telemetry_message_t message;
    ER result;

    if (data == NULL) {
        return (int32_t)E_PAR;
    }

    result = buddy1_mqtt_telemetry_receive(&message);

    if (result != E_OK) {
        return (int32_t)result;
    }

    data->source = message.source;
    data->type = message.type;
    data->flags = message.flags;
    data->value_1 = message.value_1;
    data->value_2 = message.value_2;
    data->sequence = message.sequence;

    return 0;
}

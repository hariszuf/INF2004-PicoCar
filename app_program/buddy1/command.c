
#include "../barrc_types.h"
#include <stdint.h>

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "command.h"

#include "mqtt_command_bridge.h"
#include "../telemetry_frame.h"

#define COMMAND_QUEUE_DEPTH    8U
#define COMMAND_TASK_PRIORITY  6
#define COMMAND_STACK_SIZE     4096

LOCAL ID s_command_buffer_id = 0;
LOCAL uint32_t s_command_dropped_count = 0U;

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

LOCAL const char * command_type_to_string (uint8_t type)
{
    switch (type)
    {
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
LOCAL void command_task (INT stacd, void * p_exinf)
{
    command_message_t message;
    INT received_size;

    /* cast: result intentionally ignored */
    (void)stacd;
    /* cast: result intentionally ignored */
    (void)p_exinf;

    for (;;)
    {
        received_size = tk_rcv_mbf(
                                   s_command_buffer_id,
                                   &message,
                                   TMO_FEVR
                                   );

        if (received_size != (INT)sizeof(command_message_t))
        {
            tm_printf(
                      (UB *)"[Command] Invalid message size: %d\n",
                      received_size
                      );
            continue;
        }

        tm_printf(
                  (UB *)"[Command] type=%s seq=%u value=%d\n",
                  (UB *)command_type_to_string(message.type),
                  (uint32_t)message.sequence,
                  (int32_t)message.value
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
ER buddy1_command_init (void)
{
    if (s_command_buffer_id > 0)
    {
        return E_OK;
    }

    s_command_buffer_id = tk_cre_mbf(
                                     &command_buffer_config
                                     );

    if (s_command_buffer_id < E_OK)
    {
        return s_command_buffer_id;
    }

    return E_OK;
}

/*
 * PHASE 2: Create and start the consumer task.
 */
ER buddy1_command_start (void)
{
    ID task_id;

    if (s_command_buffer_id <= 0)
    {
        return E_OBJ;
    }

    task_id = tk_cre_tsk(
                         &command_task_config
                         );

    if (task_id < E_OK)
    {
        return task_id;
    }

    return tk_sta_tsk(task_id, 0);
}

/*
 * Non-blocking command producer API.
 */
ER buddy1_command_submit (
                          const command_message_t * message
                          )
{
    ER error_code;

    if (NULL == message)
    {
        return E_PAR;
    }

    if (s_command_buffer_id <= 0)
    {
        return E_OBJ;
    }

    error_code = tk_snd_mbf(
                            s_command_buffer_id,
                            message,
                            sizeof(command_message_t),
                            TMO_POL
                            );

    if (E_TMOUT == error_code)
    {
        s_command_dropped_count++;
    }

    return error_code;
}

uint32_t buddy1_command_get_dropped_count (void)
{
    return s_command_dropped_count;
}

/*
 * MQTT command sequence counter.
 * Only the MQTT networking owner task calls this bridge.
 */
static uint32_t s_mqtt_command_sequence = 0U;

/*
 * Compare an MQTT payload against an expected command.
 * Use explicit lengths because MQTT payloads are binary data
 * and are not guaranteed to be null-terminated.
 */
static uint8_t command_text_matches (
                                     const char * p_payload,
                                     uint16_t payload_length,
                                     const char * p_expected,
                                     uint16_t expected_length)
{
    uint16_t idx;

    if (payload_length != expected_length)
    {
        return 0U;
    }

    for (idx = 0U; idx < expected_length; idx++)
    {
        if (p_payload[idx] != p_expected[idx])
        {
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
/*
 * Integrated vehicle only: non-blocking START/STOP intent for the single
 * controller task. Weak, so profiles
 * without the vehicle controller still link (the pointer is then NULL). It only
 * sets flags: MQTT code never
 * calls a motor API, and it bypasses the diagnostic message buffer below so a
 * full buffer cannot drop a STOP.
 */
extern void b5_remote_command_intent (uint32_t command) __attribute__((weak));

/*
 * Barcode map (Buddy 3): "MAP " followed by 26 letters (one per barcode letter
 * A..Z, each L S R U or N) replaces
 * the A-Z table, "MAPRESET" restores the built-in defaults. Rejected while the
 * car is running. All weak, so
 * profiles without Buddy 3 / the vehicle controller still link. The result is
 * reported back through telemetry.
 */
extern int32_t junction_map_apply (const char * codes,
                                   uint32_t length) __attribute__((weak));
extern void junction_map_reset (void) __attribute__((weak));
extern int32_t b5_run_active (void) __attribute__((weak));
/* 0 none yet, 1 applied, 2 bad format, 3 rejected (car running) */
static volatile int32_t s_barcode_map_result = 0;

int32_t buddy1_barcode_map_result (void)
{
    return s_barcode_map_result;
}

static int32_t barcode_map_command (const char * p_payload,
                                    uint16_t payload_length)
{
    if ((NULL == junction_map_apply) || (NULL == junction_map_reset))
    {
        s_barcode_map_result = 2;
        return (int32_t)E_NOSPT;
    }

    if ((NULL != b5_run_active) && (0 != b5_run_active()))
    {
        s_barcode_map_result = 3;
        return (int32_t)E_BUSY;
    }

    if (0U != command_text_matches(p_payload, payload_length, "MAPRESET", 8U))
    {
        junction_map_reset();
        s_barcode_map_result = 1;
        return 0;
    }

    if ((30U == payload_length) && (' ' == p_payload[3]) &&
        (0 == junction_map_apply(&p_payload[4], 26U)))
    {
        s_barcode_map_result = 1;
        return 0;
    }

    s_barcode_map_result = 2;
    return (int32_t)E_PAR;
}

int32_t buddy1_mqtt_command_ingest (
                                    const char * p_payload,
                                    uint16_t payload_length)
{
    command_message_t message = {0};

    if (NULL == p_payload)
    {
        return (int32_t)E_PAR;
    }

    if ((payload_length >= 1U)
        && (0U != ((uint8_t)p_payload[0] & TF_BINARY_MARK)))
    {
        /* Compact binary command from the dashboard (see telemetry_frame.h). */
        uint8_t opcode = (uint8_t)p_payload[0] & 0x7FU;

        if (TF_CMD_MAP == opcode)
        {
            char text[4U + TF_MAP_LETTERS + 1U] = { 'M', 'A', 'P', ' ' };

            if ((TF_CMD_MAP_LENGTH != payload_length) ||
                (!tf_decode_map((const uint8_t *) & p_payload[1], &text[4])))
            {
                s_barcode_map_result = 2;
                return (int32_t)E_PAR;
            }

            return barcode_map_command(text, (uint16_t)(4U + TF_MAP_LETTERS));
        }

        if (1U != payload_length)
        {
            return (int32_t)E_PAR;
        }

        if (TF_CMD_MAPRESET == opcode)
        {
            return barcode_map_command("MAPRESET", 8U);
        }
        else if (TF_CMD_START == opcode)
        {
            message.type = COMMAND_TYPE_START;
        }
        else if (TF_CMD_STOP == opcode)
        {
            message.type = COMMAND_TYPE_STOP;
        }
        else if (TF_CMD_PAUSE == opcode)
        {
            message.type = COMMAND_TYPE_PAUSE;
        }
        else if (TF_CMD_RESUME == opcode)
        {
            message.type = COMMAND_TYPE_RESUME;
        }
        else
        {
            return (int32_t)E_PAR;
        }
    }
    else if ((payload_length >= 3U) && ('M' == p_payload[0])
             && ('A' == p_payload[1]) && ('P' == p_payload[2]))
    {
        return barcode_map_command(p_payload, payload_length);
    }
    else if (command_text_matches(p_payload, payload_length,
                                  "START", 5U) != 0U)
    {
        message.type = COMMAND_TYPE_START;
    }
    else if (command_text_matches(p_payload, payload_length,
                                  "STOP", 4U) != 0U)
    {
        message.type = COMMAND_TYPE_STOP;
    }
    else if (command_text_matches(p_payload, payload_length,
                                  "PAUSE", 5U) != 0U)
    {
        message.type = COMMAND_TYPE_PAUSE;
    }
    else if (command_text_matches(p_payload, payload_length,
                                  "RESUME", 6U) != 0U)
    {
        message.type = COMMAND_TYPE_RESUME;
    }
    else
    {
        return (int32_t)E_PAR;
    }

    /*
     * Source 1 represents Wi-Fi/Buddy 1.
     * No motor action is performed at this stage.
     */
    if (NULL != b5_remote_command_intent)
    {
        if (COMMAND_TYPE_STOP == message.type)
        {
            b5_remote_command_intent((uint32_t)'S');
        }
        else if (COMMAND_TYPE_START == message.type)
        {
            b5_remote_command_intent((uint32_t)'R');
        }
        else
        {
            /* no action required */
        }

        /* PAUSE / RESUME stay parsed and logged only. */
    }

    message.source = 1U;
    message.flags = 0U;
    message.value = 0;

    s_mqtt_command_sequence++;
    message.sequence = s_mqtt_command_sequence;

    return (int32_t)buddy1_command_submit(&message);
}

/*** end of file ***/

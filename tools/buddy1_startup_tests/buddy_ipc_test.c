/* Production Buddy 1 command/telemetry code; only kernel IPC is simulated. */
#include "../../app_program/barrc_types.h"
#include <assert.h>
#include <setjmp.h>
#include <string.h>
#include "firmware_buddy.c"

static struct
{
    uint32_t count;
    uint32_t size;
    uint8_t data[8][32];
} queues[4];
static uint32_t s_next_queue;
static jmp_buf s_task_idle;
ID tk_cre_mbf (const T_CMBF *definition)
{
    assert((++s_next_queue < 4) && (definition->maxmsz <= 32));
    queues[s_next_queue].size = definition->maxmsz;
    return s_next_queue;
}

ER tk_snd_mbf (ID id, const void * message, INT size, INT timeout)
{
    assert((id > 0) && (id < 4) && (TMO_POL == timeout));
    assert((uint32_t)size == queues[id].size);

    if (8 == queues[id].count)
    {
        return E_TMOUT;
    }

    memcpy(queues[id].data[queues[id].count++], message, size);
    return E_OK;
}

INT tk_rcv_mbf (ID id, void * message, INT timeout)
{
    assert((id > 0) && (id < 4));

    if (!queues[id].count)
    {
        if (TMO_FEVR == timeout)
        {
            longjmp(s_task_idle, 1);
        }

        assert(TMO_POL == timeout);
        return E_TMOUT;
    }

    memcpy(message, queues[id].data[0], queues[id].size);
    queues[id].count--;
    memmove(queues[id].data[0], queues[id].data[1], queues[id].count * 32);
    return queues[id].size;
}

ID tk_cre_tsk (const T_CTSK *definition)
{
    /* cast: result intentionally ignored */
    (void)definition;
    return 1;
}

ER tk_sta_tsk (ID id, INT code)
{
    /* cast: result intentionally ignored */
    (void)id;
    /* cast: result intentionally ignored */
    (void)code;
    return E_OK;
}

void test_buddy_init (void)
{
    assert(E_OK == buddy1_command_init());
    assert(E_OK == buddy1_telemetry_init());
}

void test_queue_telemetry (void)
{
    telemetry_message_t message = {.source = 1, .type = 1, .sequence = 42,
        .value_1 = 7, .value_2 = -9};
    assert(E_OK == buddy1_telemetry_submit(&message));

    if (0 == setjmp(s_task_idle))
    {
        telemetry_task(0, NULL);
    }

    assert(1 == queues[s_mqtt_outbox_id].count);
}

void test_consume_commands (void)
{
    assert(2 == queues[s_command_buffer_id].count);

    for (uint32_t idx = 0; idx < 2; idx++)
    {
        command_message_t message;
        memcpy(&message, queues[s_command_buffer_id].data[idx],
               sizeof(message));
        assert(message.type == (idx ? COMMAND_TYPE_STOP : COMMAND_TYPE_START));
        assert((message.sequence == idx + 1) && (1 == message.source)
               && (0 == message.value));
    }

    if (0 == setjmp(s_task_idle))
    {
        command_task(0, NULL);
    }

    assert(0 == queues[s_command_buffer_id].count);
}

/*** end of file ***/

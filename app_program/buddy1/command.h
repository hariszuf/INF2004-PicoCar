#ifndef BUDDY1_COMMAND_H
#define BUDDY1_COMMAND_H

#include <stdint.h>
#include <tk/tkernel.h>


#define COMMAND_TYPE_START     ((uint8_t)1U)
#define COMMAND_TYPE_STOP      ((uint8_t)2U)
#define COMMAND_TYPE_PAUSE     ((uint8_t)3U)
#define COMMAND_TYPE_RESUME    ((uint8_t)4U)


typedef struct
{
    uint8_t type;
    uint8_t source;
    uint16_t flags;

    int32_t value;

    uint32_t sequence;

} command_message_t;


/*
 * Creates the command message buffer and consumer task.
 */
ER buddy1_command_start(void);


/*
 * Non-blocking command submission.
 *
 * Later, MQTT will call this after receiving a remote command.
 */
ER buddy1_command_submit(const command_message_t *message);


uint32_t buddy1_command_get_dropped_count(void);


#endif
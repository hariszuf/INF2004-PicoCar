
#ifndef BUDDY1_TELEMETRY_H
#define BUDDY1_TELEMETRY_H

#include <stdint.h>
#include <tk/tkernel.h>

/*
 * Telemetry source identifiers.
 */
#define TELEMETRY_SOURCE_BUDDY1 ((uint8_t)1U)
#define TELEMETRY_SOURCE_BUDDY2 ((uint8_t)2U)
#define TELEMETRY_SOURCE_BUDDY3 ((uint8_t)3U)
#define TELEMETRY_SOURCE_BUDDY4 ((uint8_t)4U)
#define TELEMETRY_SOURCE_BUDDY5 ((uint8_t)5U)

/*
 * Telemetry event types.
 */
#define TELEMETRY_TYPE_HEARTBEAT   ((uint8_t)1U)
#define TELEMETRY_TYPE_STATE       ((uint8_t)2U)
#define TELEMETRY_TYPE_MOTOR_SPEED ((uint8_t)3U)
#define TELEMETRY_TYPE_ENCODER     ((uint8_t)4U)
#define TELEMETRY_TYPE_LINE_SENSOR ((uint8_t)5U)
#define TELEMETRY_TYPE_BARCODE     ((uint8_t)6U)
#define TELEMETRY_TYPE_IMU         ((uint8_t)7U)
#define TELEMETRY_TYPE_HUMP        ((uint8_t)8U)
#define TELEMETRY_TYPE_ULTRASONIC  ((uint8_t)9U)

/*
 * Common telemetry message.
 *
 * All buddies can submit telemetry using this structure.
 */
typedef struct
{
    uint8_t source;
    uint8_t type;
    uint16_t flags;

    int32_t value_1;
    int32_t value_2;

    uint32_t sequence;

} telemetry_message_t;

/*
 * Phase 1:
 * Create telemetry kernel objects.
 * No tasks are started here.
 */
ER buddy1_telemetry_init(void);

/*
 * Phase 2:
 * Create and start the telemetry consumer task.
 */
ER buddy1_telemetry_start(void);

/*
 * Non-blocking telemetry submission.
 *
 * Other buddies can call this function without
 * waiting for Buddy 1 to process the message.
 */
ER buddy1_telemetry_submit(
    const telemetry_message_t *message
);

/*
 * Return the number of dropped telemetry messages.
 */
uint32_t buddy1_telemetry_get_dropped_count(void);

#endif /* BUDDY1_TELEMETRY_H */

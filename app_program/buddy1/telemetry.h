#ifndef BUDDY1_TELEMETRY_H
#define BUDDY1_TELEMETRY_H

#include <stdint.h>
#include <tk/tkernel.h>

/*
 * Which subsystem generated the telemetry.
 *
 * Fixed uint8_t values are used instead of storing C enums
 * directly inside the message so the message size is predictable.
 */
#define TELEMETRY_SOURCE_BUDDY1    ((uint8_t)1U)
#define TELEMETRY_SOURCE_BUDDY2    ((uint8_t)2U)
#define TELEMETRY_SOURCE_BUDDY3    ((uint8_t)3U)
#define TELEMETRY_SOURCE_BUDDY4    ((uint8_t)4U)
#define TELEMETRY_SOURCE_BUDDY5    ((uint8_t)5U)


/*
 * Telemetry event types.
 *
 * We will expand this list as each Buddy begins integration.
 */
#define TELEMETRY_TYPE_HEARTBEAT       ((uint8_t)1U)
#define TELEMETRY_TYPE_STATE           ((uint8_t)2U)
#define TELEMETRY_TYPE_MOTOR_SPEED     ((uint8_t)3U)
#define TELEMETRY_TYPE_ENCODER         ((uint8_t)4U)
#define TELEMETRY_TYPE_LINE_SENSOR     ((uint8_t)5U)
#define TELEMETRY_TYPE_BARCODE         ((uint8_t)6U)
#define TELEMETRY_TYPE_IMU             ((uint8_t)7U)
#define TELEMETRY_TYPE_HUMP            ((uint8_t)8U)
#define TELEMETRY_TYPE_ULTRASONIC      ((uint8_t)9U)


/*
 * Common telemetry message.
 *
 * value_1 and value_2 are generic for now.
 *
 * Examples later:
 *
 * Motor speed:
 *   value_1 = left RPM
 *   value_2 = right RPM
 *
 * IMU:
 *   value_1 = pitch
 *   value_2 = roll
 *
 * Ultrasonic:
 *   value_1 = distance
 *   value_2 = obstacle width
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
 * Creates the telemetry message buffer and telemetry task.
 */
ER buddy1_telemetry_start(void);


/*
 * Non-blocking telemetry submission.
 *
 * This function must return quickly so a sensor/motor task
 * is never delayed by Buddy 1 networking.
 */
ER buddy1_telemetry_submit(const telemetry_message_t *message);


/*
 * Number of telemetry messages dropped because the queue
 * was full.
 */
uint32_t buddy1_telemetry_get_dropped_count(void);


#endif
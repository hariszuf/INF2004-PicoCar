
#ifndef ROBOT_MESSAGES_H
#define ROBOT_MESSAGES_H

#include <stdint.h>

/*
 * Shared message format for PicoCar.
 *
 * Discrete events between buddies.
 * Does not create kernel IPC objects.
 */

typedef enum {
    ROBOT_SOURCE_WIFI     = 1,
    ROBOT_SOURCE_MOTION   = 2,
    ROBOT_SOURCE_BARCODE  = 3,
    ROBOT_SOURCE_IMU      = 4,
    ROBOT_SOURCE_OBSTACLE = 5
} robot_source_t;

typedef enum {
    ROBOT_EVENT_LINE_SAMPLE = 1,
    ROBOT_EVENT_BARCODE_DETECTED,
    ROBOT_EVENT_IMU_SAMPLE,
    ROBOT_EVENT_HUMP_DETECTED,
    ROBOT_EVENT_OBSTACLE_SAMPLE,
    ROBOT_EVENT_REMOTE_COMMAND
} robot_event_type_t;

#define ROBOT_MSG_VALID 0x0001U

typedef struct {
    uint32_t timestamp_ms;
    uint32_t sequence;

    int32_t value_1;
    int32_t value_2;

    uint8_t source;
    uint8_t type;
    uint16_t flags;
} robot_event_t;

_Static_assert(
    sizeof(robot_event_t) == 20U,
    "Unexpected robot_event_t layout"
);

#endif

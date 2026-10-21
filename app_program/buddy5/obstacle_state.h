#ifndef BUDDY5_OBSTACLE_STATE_H
#define BUDDY5_OBSTACLE_STATE_H

#include "ultrasonic.h"

/* Stationary-test thresholds; driving values require braking-distance testing. */
#define BUDDY5_OBSTACLE_MM 200u
#define BUDDY5_CLEAR_MM    250u

typedef enum {
    BUDDY5_SENSOR_FAULT = 0,
    BUDDY5_OBSTACLE,
    BUDDY5_CLEAR
} buddy5_state_t;

/* A fault/startup stays unresolved in the dead band until a threshold is crossed. */
static inline buddy5_state_t buddy5_next_state(
    buddy5_state_t previous, ultrasonic_result_t reading)
{
    if (reading.status != US_OK) {
        return BUDDY5_SENSOR_FAULT;
    }
    if (reading.distance_mm <= BUDDY5_OBSTACLE_MM) {
        return BUDDY5_OBSTACLE;
    }
    if (reading.distance_mm >= BUDDY5_CLEAR_MM) {
        return BUDDY5_CLEAR;
    }
    return previous;
}

#endif

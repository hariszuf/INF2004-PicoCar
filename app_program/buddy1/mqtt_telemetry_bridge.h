#ifndef PICOCAR_MQTT_TELEMETRY_BRIDGE_H
#define PICOCAR_MQTT_TELEMETRY_BRIDGE_H

#include <stdint.h>

typedef struct
{
    uint8_t source;
    uint8_t type;
    uint16_t flags;

    int32_t value_1;
    int32_t value_2;

    uint32_t sequence;

} mqtt_telemetry_data_t;

int32_t buddy1_mqtt_telemetry_fetch(
    mqtt_telemetry_data_t *data
);

#endif
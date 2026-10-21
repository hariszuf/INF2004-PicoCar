#ifndef PICOCAR_MQTT_COMMAND_BRIDGE_H
#define PICOCAR_MQTT_COMMAND_BRIDGE_H

#include <stdint.h>

/*
 * Converts a validated MQTT payload into a Buddy 1 command.
 *
 * Called from the networking owner task.
 * Uses non-blocking µT-Kernel IPC.
 *
 * Returns 0 on success, or an error code.
 */
int32_t buddy1_mqtt_command_ingest(
    const char *payload,
    uint16_t payload_length
);

#endif
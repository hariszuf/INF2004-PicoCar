#ifndef BUDDY4_MQTT_TELEMETRY_H
#define BUDDY4_MQTT_TELEMETRY_H
#include <stdbool.h>

bool mqtt_telemetry_start(void);
void mqtt_telemetry_poll(void);
bool mqtt_telemetry_publish(const char *terrain, float pitch, float x, float y, float z);
#endif

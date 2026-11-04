#ifndef PICOCAR_MQTT_DASHBOARD_READY_H
#define PICOCAR_MQTT_DASHBOARD_READY_H

#include <stdbool.h>

/* Read-only snapshot for the single-core vehicle tasks. */
bool mqtt_dashboard_ready(void);

#endif

#ifndef PICOCAR_MQTT_DASHBOARD_READY_H
#define PICOCAR_MQTT_DASHBOARD_READY_H

#include <stdbool.h>
#include <stdint.h>

/* Read-only snapshot for the single-core vehicle tasks. */
bool mqtt_dashboard_ready(void);
/* Requires the dashboard acknowledgement and enough time since the broker connected. */
bool mqtt_dashboard_ready_after_connect(uint32_t delay_ms);

#endif

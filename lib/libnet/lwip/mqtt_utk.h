#ifndef PICOCAR_MQTT_UTK_H
#define PICOCAR_MQTT_UTK_H

#include "lwip/netif.h"

/*
 * Called ONLY from the CYW43439/lwIP owner task.
 * Must not be called directly from Buddy 1 application tasks.
 */
void mqtt_utk_poll(struct netif *netif);

/* Diagnostic image only; same owner-task restriction. */
int mqtt_utk_is_connected(void);

#endif
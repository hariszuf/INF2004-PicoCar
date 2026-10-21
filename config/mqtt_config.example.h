
#ifndef PICOCAR_MQTT_CONFIG_H
#define PICOCAR_MQTT_CONFIG_H

/*
 * PicoCar MQTT configuration example.
 *
 * Copy this file to:
 *   config/mqtt_config.h
 *
 * Update MQTT_BROKER_IP to the IPv4 address
 * of the computer running Mosquitto.
 *
 * Both the Pico W and the broker computer
 * must be reachable on the same network.
 */

#define MQTT_BROKER_IP "192.168.1.100"
#define MQTT_BROKER_PORT 1883U

#define MQTT_CLIENT_ID "picocar-vehicle1"

#define MQTT_STATUS_TOPIC \
    "picocar/vehicle1/status"

#define MQTT_TELEMETRY_TOPIC \
    "picocar/vehicle1/telemetry"

#define MQTT_COMMAND_TOPIC \
    "picocar/vehicle1/command"

#endif /* PICOCAR_MQTT_CONFIG_H */

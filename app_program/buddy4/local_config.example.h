// COPY THIS FILE TO local_config.h when you're ready for MQTT.
// Never commit local_config.h to a public GitHub repository!
#pragma once
#define WIFI_SSID "YOUR_WIFI_NAME"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
// Use an IP address reachable from your Pico W on the same network.
// Port 1883 is unencrypted MQTT; use only on a trusted private network.
#define MQTT_BROKER_IP "192.168.1.100"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID "buddy4-picow"
#define MQTT_TOPIC "inf2004/buddy4/terrain"
// Optional, leave empty strings for an unauthenticated local broker.
#define MQTT_USER ""
#define MQTT_PASSWORD ""

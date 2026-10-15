// MQTT stage for Raspberry Pi Pico W only. Plain MQTT to a private LAN broker.
#include "mqtt_telemetry.h"
#include "local_config.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/ip_addr.h"
#include "lwip/apps/mqtt.h"
#include <stdio.h>
#include <string.h>

static mqtt_client_t *s_client = NULL;
static ip_addr_t s_broker_ip;
static volatile bool s_connected = false;
static bool s_started = false;
static uint32_t s_last_try_ms = 0;
static struct mqtt_connect_client_info_t s_info;

static void connection_cb(mqtt_client_t *client, void *arg,
                          mqtt_connection_status_t status) {
    (void)client; (void)arg;
    // This callback runs in the networking context; avoid blocking here.
    s_connected = (status == MQTT_CONNECT_ACCEPTED);
}

static void connect_broker(void) {
    if (!s_client) return;
    s_last_try_ms = to_ms_since_boot(get_absolute_time());
    cyw43_arch_lwip_begin();
    err_t err = mqtt_client_connect(s_client, &s_broker_ip,
                                    MQTT_BROKER_PORT, connection_cb,
                                    NULL, &s_info);
    cyw43_arch_lwip_end();
    if (err != ERR_OK) printf("MQTT connect request error %d\n", err);
    else printf("MQTT broker connect requested...\n");
}

bool mqtt_telemetry_start(void) {
    if (cyw43_arch_init()) {
        printf("Wi-Fi driver initialization failed\n");
        return false;
    }
    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");
    int err = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 15000);
    if (err != 0) {
        printf("Wi-Fi connection failed (%d). Sensor will still print over USB.\n", err);
        return false;
    }
    printf("Wi-Fi connected\n");
    if (!ipaddr_aton(MQTT_BROKER_IP, &s_broker_ip)) {
        printf("Invalid MQTT broker IP. Set MQTT_BROKER_IP in local_config.h\n");
        return false;
    }
    cyw43_arch_lwip_begin();
    s_client = mqtt_client_new();
    cyw43_arch_lwip_end();
    if (!s_client) {
        printf("Unable to allocate MQTT client\n");
        return false;
    }
    memset(&s_info, 0, sizeof(s_info));
    s_info.client_id = MQTT_CLIENT_ID;
    s_info.keep_alive = 30;
    s_info.client_user = MQTT_USER[0] ? MQTT_USER : NULL;
    s_info.client_pass = MQTT_PASSWORD[0] ? MQTT_PASSWORD : NULL;
    s_started = true;
    connect_broker();
    return true;
}

void mqtt_telemetry_poll(void) {
    if (!s_started || s_connected) return;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if ((uint32_t)(now - s_last_try_ms) >= 10000) {
        // Retry broker connection periodically; do not block sensor reads.
        connect_broker();
    }
}

bool mqtt_telemetry_publish(const char *terrain, float pitch,
                            float x, float y, float z) {
    if (!s_started || !s_connected) return false;
    char msg[180];
    int len = snprintf(msg, sizeof(msg),
        "{\"terrain\":\"%s\",\"pitch_deg\":%.2f,"
        "\"ax_g\":%.3f,\"ay_g\":%.3f,\"az_g\":%.3f}",
        terrain, pitch, x, y, z);
    if (len < 0 || len >= (int)sizeof(msg)) return false;
    cyw43_arch_lwip_begin();
    err_t err = mqtt_publish(s_client, MQTT_TOPIC, msg, (u16_t)len,
                             0, 0, NULL, NULL);
    cyw43_arch_lwip_end();
    if (err != ERR_OK) {
        printf("MQTT publish failed (%d)\n", err);
        return false;
    }
    return true;
}

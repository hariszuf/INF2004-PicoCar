
#include <stdint.h>

#include "lwip/apps/mqtt.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/sys.h"

#include <tm/tmonitor.h>

#include "mqtt_config.h"
#include "mqtt_utk.h"

#include "../../../app_program/buddy1/mqtt_command_bridge.h"
#include "../../../app_program/buddy1/mqtt_telemetry_bridge.h"

#define MQTT_HEARTBEAT_MS 5000U
#define MQTT_RETRY_MS     3000U
#define MQTT_RX_MAX      96U

static mqtt_client_t *mqtt_client = NULL;
static ip_addr_t broker_address;

static uint8_t connecting = 0U;
static uint8_t connected = 0U;
static uint8_t initialized = 0U;

static uint32_t last_attempt_ms = 0U;
static uint32_t last_heartbeat_ms = 0U;
static uint32_t heartbeat_sequence = 0U;

static char rx_topic[64];
static char rx_payload[MQTT_RX_MAX];

static uint16_t rx_length = 0U;
static uint8_t rx_overflow = 0U;

static const struct mqtt_connect_client_info_t client_info = {
    .client_id = MQTT_CLIENT_ID,
    .keep_alive = 60
};

/*
 * Called when an incoming MQTT publication begins.
 */
static void incoming_publish(
    void *arg,
    const char *topic,
    u32_t total_length)
{
    unsigned int i;

    (void)arg;

    for (i = 0U;
         i < sizeof(rx_topic) - 1U &&
         topic[i] != '\0';
         i++) {

        rx_topic[i] = topic[i];
    }

    rx_topic[i] = '\0';

    rx_length = 0U;

    rx_overflow = (
        total_length >= MQTT_RX_MAX ||
        topic[i] != '\0'
    ) ? 1U : 0U;
}

/*
 * Compare two MQTT topic strings.
 */
static uint8_t mqtt_topic_matches(
    const char *received,
    const char *expected)
{
    while ((*received != '\0') &&
           (*expected != '\0')) {

        if (*received != *expected) {
            return 0U;
        }

        received++;
        expected++;
    }

    return (
        (*received == '\0') &&
        (*expected == '\0')
    ) ? 1U : 0U;
}

/*
 * Receive MQTT payload fragments.
 *
 * A complete command is forwarded through
 * Buddy 1's µT-Kernel command bridge.
 */
static void incoming_data(
    void *arg,
    const u8_t *data,
    u16_t length,
    u8_t flags)
{
    uint16_t i;

    (void)arg;

    for (i = 0U; i < length; i++) {

        if (rx_length < MQTT_RX_MAX - 1U) {

            rx_payload[rx_length++] =
                (char)data[i];

        } else {

            rx_overflow = 1U;
        }
    }

    if ((flags & MQTT_DATA_FLAG_LAST) != 0U) {

        rx_payload[rx_length] = '\0';

        if (rx_overflow == 0U) {

            tm_printf(
                (UB *)"[MQTT] RX topic=%s payload=%s\n",
                (UB *)rx_topic,
                (UB *)rx_payload
            );

            if (mqtt_topic_matches(
                    rx_topic,
                    MQTT_COMMAND_TOPIC) != 0U) {

                int32_t result;

                result = buddy1_mqtt_command_ingest(
                    rx_payload,
                    rx_length
                );

                if (result != 0) {

                    tm_printf(
                        (UB *)"[MQTT] Command rejected, result=%d\n",
                        (int)result
                    );
                }
            }

        } else {

            tm_printf(
                (UB *)"[MQTT] Incoming message too large\n"
            );
        }
    }
}

/*
 * MQTT subscription completion callback.
 */
static void subscribe_callback(
    void *arg,
    err_t result)
{
    (void)arg;

    tm_printf(
        (UB *)"[MQTT] Subscribe result=%d\n",
        (int)result
    );
}

/*
 * MQTT publication completion callback.
 */
static void publish_callback(
    void *arg,
    err_t result)
{
    (void)arg;

    if (result != ERR_OK) {

        tm_printf(
            (UB *)"[MQTT] Publish failed=%d\n",
            (int)result
        );
    }
}

/*
 * MQTT broker connection callback.
 */
static void connection_callback(
    mqtt_client_t *client,
    void *arg,
    mqtt_connection_status_t status)
{
    err_t result;

    (void)arg;

    connecting = 0U;

    if (status != MQTT_CONNECT_ACCEPTED) {

        connected = 0U;
        last_attempt_ms = sys_now();

        tm_printf(
            (UB *)"[MQTT] Connection status=%d\n",
            (int)status
        );

        return;
    }

    connected = 1U;
    last_heartbeat_ms = sys_now();

    tm_printf(
        (UB *)"[MQTT] Connected to %s:%u\n",
        (UB *)MQTT_BROKER_IP,
        (unsigned int)MQTT_BROKER_PORT
    );

    mqtt_set_inpub_callback(
        client,
        incoming_publish,
        incoming_data,
        NULL
    );

    result = mqtt_subscribe(
        client,
        MQTT_COMMAND_TOPIC,
        0,
        subscribe_callback,
        NULL
    );

    tm_printf(
        (UB *)"[MQTT] Subscribe request=%d\n",
        (int)result
    );
}

/*
 * Existing MQTT heartbeat.
 *
 * This is a network heartbeat, separate
 * from the µT-Kernel heartbeat telemetry.
 */
static void publish_heartbeat(void)
{
    UB payload[48];
    INT length;
    err_t result;

    length = tm_sprintf(
        payload,
        (const UB *)"heartbeat seq=%u",
        (unsigned int)heartbeat_sequence
    );

    if (length <= 0 ||
        length >= (INT)sizeof(payload)) {
        return;
    }

    result = mqtt_publish(
        mqtt_client,
        MQTT_STATUS_TOPIC,
        payload,
        (u16_t)length,
        0,
        0,
        publish_callback,
        NULL
    );

    if (result == ERR_OK) {

        tm_printf(
            (UB *)"[MQTT] TX %s\n",
            payload
        );

        heartbeat_sequence++;

    } else {

        tm_printf(
            (UB *)"[MQTT] Publish request failed=%d\n",
            (int)result
        );
    }
}

/*
 * NEW: Publish µT-Kernel telemetry over MQTT.
 *
 * Called only by the network-owner task.
 *
 * Receives one message from the MQTT outbox
 * and publishes it to the telemetry topic.
 */
static void publish_telemetry(void)
{
    mqtt_telemetry_data_t message;

    UB payload[128];

    INT length;
    int32_t receive_result;
    err_t publish_result;

    /*
     * Non-blocking message-buffer receive.
     */
    receive_result = buddy1_mqtt_telemetry_fetch(
        &message
    );

    if (receive_result != 0) {
        return;
    }

    /*
     * Temporary readable payload.
     *
     * Later we will replace this with
     * compact binary encoding.
     */
    length = tm_sprintf(
        payload,
        (const UB *)
            "src=%u type=%u seq=%u "
            "value1=%d value2=%d",
        (unsigned int)message.source,
        (unsigned int)message.type,
        (unsigned int)message.sequence,
        (int)message.value_1,
        (int)message.value_2
    );

    if (length <= 0 ||
        length >= (INT)sizeof(payload)) {

        tm_printf(
            (UB *)"[MQTT] Invalid telemetry payload length\n"
        );

        return;
    }

    /*
     * Publish on the configured telemetry topic.
     *
     * QoS 0 is used for the initial integration.
     */
    publish_result = mqtt_publish(
        mqtt_client,
        MQTT_TELEMETRY_TOPIC,
        payload,
        (u16_t)length,
        0,
        0,
        publish_callback,
        NULL
    );

    if (publish_result != ERR_OK) {

        tm_printf(
            (UB *)"[MQTT] Telemetry publish failed=%d\n",
            (int)publish_result
        );
    }
}

/*
 * Network-owner polling function.
 *
 * All lwIP and MQTT operations remain
 * inside the existing networking task.
 */
void mqtt_utk_poll(struct netif *netif)
{
    uint32_t now;
    err_t result;

    /*
     * Temporary connection diagnostics.
     */
    static uint32_t last_debug_ms = 0U;
    uint32_t debug_now = sys_now();

    if ((uint32_t)(debug_now - last_debug_ms) >=
        5000U) {

        last_debug_ms = debug_now;

        tm_printf(
            (UB *)"[MQTT-DBG] connected=%u "
                  "connecting=%u "
                  "lwip_connected=%u "
                  "heartbeat_age=%u ms "
                  "up=%u link=%u ip=%u.%u.%u.%u\n",
            (unsigned int)connected,
            (unsigned int)connecting,
            (unsigned int)(
                mqtt_client != NULL &&
                mqtt_client_is_connected(mqtt_client)
            ),
            (unsigned int)(
                debug_now - last_heartbeat_ms
            ),
            (unsigned int)(netif != NULL && netif_is_up(netif)),
            (unsigned int)(netif != NULL && netif_is_link_up(netif)),
            netif != NULL ? ip4_addr1(netif_ip4_addr(netif)) : 0U,
            netif != NULL ? ip4_addr2(netif_ip4_addr(netif)) : 0U,
            netif != NULL ? ip4_addr3(netif_ip4_addr(netif)) : 0U,
            netif != NULL ? ip4_addr4(netif_ip4_addr(netif)) : 0U
        );
    }

    /*
     * Wait for a valid Wi-Fi interface and IP.
     */
    if (netif == NULL ||
        !netif_is_up(netif) ||
        !netif_is_link_up(netif) ||
        ip4_addr_isany_val(
            *netif_ip4_addr(netif)
        )) {

        return;
    }

    now = sys_now();

    if (initialized == 0U) {

        initialized = 1U;

        last_attempt_ms =
            now - MQTT_RETRY_MS;
    }

    /*
     * Allocate the MQTT client.
     */
    if (mqtt_client == NULL) {

        if ((uint32_t)(
                now - last_attempt_ms
            ) < MQTT_RETRY_MS) {

            return;
        }

        last_attempt_ms = now;

        mqtt_client = mqtt_client_new();

        if (mqtt_client == NULL) {

            tm_printf(
                (UB *)"[MQTT] Client allocation failed\n"
            );

            return;
        }
    }

    /*
     * Connected: publish status and telemetry.
     */
    if (connected != 0U &&
        mqtt_client_is_connected(mqtt_client)) {

        /*
         * Publish the existing heartbeat
         * every five seconds.
         */
        if ((uint32_t)(
                now - last_heartbeat_ms
            ) >= MQTT_HEARTBEAT_MS) {

            last_heartbeat_ms = now;

            publish_heartbeat();
        }

        /*
         * NEW: Forward one µT-Kernel
         * telemetry message per poll.
         */
        publish_telemetry();

        return;
    }

    /*
     * Handle a lost MQTT connection.
     */
    if (connected != 0U) {

        connected = 0U;
        last_attempt_ms = now;
    }

    /*
     * Wait for the current connection attempt
     * or retry interval.
     */
    if (connecting != 0U ||
        (uint32_t)(
            now - last_attempt_ms
        ) < MQTT_RETRY_MS) {

        return;
    }

    /*
     * Convert configured broker IP address.
     */
    if (!ip4addr_aton(
            MQTT_BROKER_IP,
            ip_2_ip4(&broker_address))) {

        tm_printf(
            (UB *)"[MQTT] Invalid broker IP\n"
        );

        last_attempt_ms = now;

        return;
    }

    IP_SET_TYPE(
        &broker_address,
        IPADDR_TYPE_V4
    );

    connecting = 1U;
    last_attempt_ms = now;

    /*
     * Begin MQTT connection.
     */
    tm_printf(
        (UB *)"[MQTT] Connecting client=%s to %s:%u uptime=%u ms\n",
        (UB *)MQTT_CLIENT_ID,
        (UB *)MQTT_BROKER_IP,
        (unsigned int)MQTT_BROKER_PORT,
        (unsigned int)now
    );

    result = mqtt_client_connect(
        mqtt_client,
        &broker_address,
        (u16_t)MQTT_BROKER_PORT,
        connection_callback,
        NULL,
        &client_info
    );

    if (result != ERR_OK) {

        connecting = 0U;

        tm_printf(
            (UB *)"[MQTT] Connect request failed=%d\n",
            (int)result
        );
    }
}

/* Diagnostic image only; read from the existing radio-owner task. */
int mqtt_utk_is_connected(void)
{
    return connected != 0U && mqtt_client != NULL
        && mqtt_client_is_connected(mqtt_client);
}

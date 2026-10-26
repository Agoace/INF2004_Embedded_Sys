#include "buddy1_network.h"
#include <stdio.h>
#include <string.h>
#include "pico/cyw43_arch.h"
#include "pico/rand.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"

#if !PICO_CYW43_ARCH_POLL || !NO_SYS
#error "Buddy 1 standalone requires the polling NO_SYS=1 architecture"
#endif

static uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

static bool valid_config(const b1_network_config_t *c)
{
    if (c == NULL || c->ssid == NULL || c->ssid[0] == '\0' || c->wifi_password == NULL ||
        c->broker_ip == NULL || c->robot_id == NULL || c->robot_id[0] == '\0' || c->broker_port == 0) return false;
    for (size_t i = 0; c->robot_id[i] != '\0'; ++i) {
        const unsigned char ch = (unsigned char)c->robot_id[i];
        if (i >= 32 || !((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') ||
                         (ch >= '0' && ch <= '9') || ch == '-' || ch == '_')) return false;
    }
    return true;
}

static err_t publish(b1_network_t *n, const char *topic, const void *payload,
                     uint16_t length, uint8_t qos, uint8_t retain, mqtt_request_cb_t cb)
{
    const err_t error = mqtt_publish(n->client, topic, payload, length, qos, retain, cb, n);
    if (error == ERR_OK) {
        n->tx_blocked = false;
    } else {
        ++n->publish_errors;
        if (!n->tx_blocked) { n->tx_blocked = true; n->tx_blocked_since_ms = now_ms(); }
    }
    return error;
}

static void connection(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    (void)client;
    b1_network_t *n = arg;
    if (n->service.phase == B1_BACKOFF) { return; }
    if (status == MQTT_CONNECT_ACCEPTED && n->service.phase == B1_MQTT) {
        n->online = true;
        b1_service_phase(&n->service, B1_SUBSCRIBING, now_ms());
        n->status_dirty = true;
    } else { n->failed = true; }
}

static void subscribed(void *arg, err_t error)
{
    b1_network_t *n = arg;
    if (n->service.phase != B1_SUBSCRIBING) { return; }
    if (error != ERR_OK) { n->failed = true; return; }
    b1_service_phase(&n->service, B1_READY, now_ms());
    n->status_dirty = true;
}

static void status_complete(void *arg, err_t error)
{
    b1_network_t *n = arg;
    if (n->service.phase == B1_BACKOFF) { return; }
    n->status_inflight = false;
    if (error != ERR_OK) { n->failed = true; return; }
    uint8_t current[5];
    b1_service_status(&n->service, current);
    n->status_dirty = memcmp(current, n->sent_status, 5) != 0;
}

static void incoming(void *arg, const char *topic, u32_t total)
{
    b1_network_t *n = arg;
    b1_service_update_time(&n->service, now_ms());
    n->rx_command = strcmp(topic, n->command_topic) == 0;
    if (n->rx_command) { b1_service_rx_begin(&n->service, total); }
    else { b1_assembler_abort(&n->service.assembler); n->service.receiving = false; }
}

static void incoming_data(void *arg, const u8_t *data, u16_t length, u8_t flags)
{
    b1_network_t *n = arg;
    b1_service_update_time(&n->service, now_ms());
    if (n->rx_command) {
        b1_service_rx_data(&n->service, data, length, (flags & MQTT_DATA_FLAG_LAST) != 0);
    }
}

static void fail(b1_network_t *n, uint32_t now)
{
    const bool leaving_wifi = n->service.phase == B1_WIFI;
    /* Mark BACKOFF before disconnect cancels requests and may call callbacks. */
    b1_service_disconnect(&n->service, now);
    n->online = n->subscribed = n->rx_command = false;
    n->status_inflight = n->status_dirty = n->failed = false;
    n->tx_blocked = false;
    mqtt_disconnect(n->client);
    if (leaving_wifi) { (void)cyw43_wifi_leave(&cyw43_state, CYW43_ITF_STA); }
}

static void start_mqtt(b1_network_t *n, uint32_t now)
{
    uint32_t session = get_rand_32();
    /* Bounded even if the entropy source repeats. This is not authentication. */
    if (session == n->service.sequence.session) ++session;
    b1_service_session(&n->service, session, now);
    n->telemetry_sequence = 0;
    b1_service_status(&n->service, n->will); /* Offline, same session as commands. */
    n->client_info.will_msg = (const char *)n->will;
    n->client_info.will_msg_len = sizeof(n->will);
    ++n->reconnects;
    if (mqtt_client_connect(n->client, &n->broker, n->config.broker_port,
                            connection, n, &n->client_info) != ERR_OK) { n->failed = true; }
}

bool b1_network_init(b1_network_t *n, const b1_network_config_t *config, uint32_t now)
{
    if (n == NULL || !valid_config(config)) return false;
    memset(n, 0, sizeof(*n));
    n->config = *config;
    b1_service_init(&n->service, now);
    n->last_poll_ms = now;
    if (!ipaddr_aton(config->broker_ip, &n->broker) || strlen(config->robot_id) > 32) { return false; }
    if (cyw43_arch_init() != 0) { return false; }
    cyw43_arch_enable_sta_mode();
    n->client = mqtt_client_new();
    if (n->client == NULL) { cyw43_arch_deinit(); return false; }
    char board_id[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    pico_get_unique_board_id_string(board_id, sizeof(board_id));
    (void)snprintf(n->client_id, sizeof(n->client_id), "buddy1-%s-%s", config->robot_id, board_id);
    (void)snprintf(n->telemetry_topic, sizeof(n->telemetry_topic), "robot/%s/telemetry", config->robot_id);
    (void)snprintf(n->command_topic, sizeof(n->command_topic), "robot/%s/cmd", config->robot_id);
    (void)snprintf(n->event_topic, sizeof(n->event_topic), "robot/%s/event", config->robot_id);
    (void)snprintf(n->status_topic, sizeof(n->status_topic), "robot/%s/status", config->robot_id);
    (void)snprintf(n->heartbeat_topic, sizeof(n->heartbeat_topic), "robot/%s/heartbeat", config->robot_id);
    n->client_info.client_id = n->client_id;
    n->client_info.client_user = config->mqtt_user;
    n->client_info.client_pass = config->mqtt_password;
    n->client_info.keep_alive = 10;
    n->client_info.will_topic = n->status_topic;
    n->client_info.will_qos = 1;
    n->client_info.will_retain = 1;
    mqtt_set_inpub_callback(n->client, incoming, incoming_data, n);
    return true;
}

void b1_network_step(b1_network_t *n, const b1_tlm_frame_t *telemetry)
{
    if (n == NULL) return;
    uint32_t now = now_ms();
    b1_service_update_time(&n->service, now);
    const uint32_t gap = now - n->last_poll_ms;
    if (gap > n->max_poll_gap_ms) { n->max_poll_gap_ms = gap; }
    n->last_poll_ms = now;
    cyw43_arch_poll();
    now = now_ms();
    b1_service_update_time(&n->service, now);
    int link = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    if (n->failed || b1_service_timeout(&n->service, now) ||
        (n->service.phase >= B1_MQTT && link != CYW43_LINK_UP) ||
        (n->tx_blocked && (uint32_t)(now - n->tx_blocked_since_ms) >= B1_CONNECT_TIMEOUT_MS)) {
        fail(n, now);
        return;
    }
    if (b1_service_retry_due(&n->service, now)) {
        if (link == CYW43_LINK_UP) { start_mqtt(n, now); }
        else {
            b1_service_phase(&n->service, B1_WIFI, now);
            if (cyw43_arch_wifi_connect_async(n->config.ssid, n->config.wifi_password,
                                              CYW43_AUTH_WPA2_AES_PSK) != 0) { n->failed = true; }
        }
    }
    if (n->service.phase == B1_WIFI && link == CYW43_LINK_UP) { start_mqtt(n, now); }
    if (n->service.phase == B1_SUBSCRIBING && !n->subscribed) {
        if (mqtt_subscribe(n->client, n->command_topic, 1, subscribed, n) == ERR_OK) { n->subscribed = true; }
    }
    if (!n->online) { return; }
    if (n->status_dirty && !n->status_inflight && (uint32_t)(now - n->status_attempt_ms) >= 200) {
        n->status_attempt_ms = now;
        b1_service_status(&n->service, n->sent_status);
        if (publish(n, n->status_topic, n->sent_status, 5, 1, 1, status_complete) == ERR_OK) {
            n->status_inflight = true;
        }
    }
    b1_service_process(&n->service);
    const b1_reply_t *reply = b1_service_reply(&n->service);
    if (reply != NULL) {
        if (publish(n, n->event_topic, reply->data, reply->length, 1, 0, NULL) == ERR_OK) {
            b1_service_reply_sent(&n->service);
        }
    }
    if (b1_service_heartbeat_due(&n->service, now)) {
        uint8_t status[5];
        b1_service_status(&n->service, status);
        (void)publish(n, n->heartbeat_topic, status, 5, 0, 0, NULL);
    }
    if (b1_service_telemetry_due(&n->service, now) && telemetry != NULL) {
        b1_tlm_frame_t frame = *telemetry;
        frame.session = n->service.sequence.session;
        frame.sequence = n->telemetry_sequence++;
        uint8_t bytes[B1_TLM_BYTES];
        ++n->telemetry_attempts;
        if (b1_tlm_encode(&frame, bytes, sizeof(bytes)) != B1_TLM_OK) {
            ++n->publish_errors; /* Local data error; not evidence of transport backpressure. */
        } else if (publish(n, n->telemetry_topic, bytes, sizeof(bytes), 0, 0, NULL) == ERR_OK) {
            ++n->telemetry_enqueued;
        }
    }
}

#ifndef BUDDY1_NETWORK_H
#define BUDDY1_NETWORK_H

#include "buddy1_service.h"
#include "buddy1_telemetry.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"

typedef struct {
    const char *ssid, *wifi_password, *broker_ip, *robot_id;
    const char *mqtt_user, *mqtt_password;
    uint16_t broker_port;
} b1_network_config_t;

typedef struct {
    b1_service_t service;
    b1_network_config_t config;
    mqtt_client_t *client;
    struct mqtt_connect_client_info_t client_info;
    ip_addr_t broker;
    char client_id[80];
    char telemetry_topic[80], command_topic[80], event_topic[80];
    char status_topic[80], heartbeat_topic[80];
    uint8_t will[5], sent_status[5];
    bool failed, online, subscribed, rx_command, status_dirty, status_inflight;
    bool tx_blocked;
    uint32_t tx_blocked_since_ms;
    uint32_t status_attempt_ms, last_poll_ms, max_poll_gap_ms;
    uint32_t telemetry_attempts, telemetry_enqueued, publish_errors, reconnects;
    uint16_t telemetry_sequence;
} b1_network_t;

/* Initialize once on the single polling owner/core. Config strings must outlive n.
 * No callbacks execute robot code. NULL telemetry skips publishing, while
 * commands/recovery/heartbeat continue. Caller supplies measurement timestamp. */
bool b1_network_init(b1_network_t *n, const b1_network_config_t *config, uint32_t now);
void b1_network_step(b1_network_t *n, const b1_tlm_frame_t *telemetry);

#endif

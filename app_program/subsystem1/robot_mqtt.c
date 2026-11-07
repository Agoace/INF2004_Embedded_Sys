/* All lwIP entry points run on cyw43_utk_task, never on a driving task. */
#if TM_MQTT
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include <string.h>
#include <tk/tkernel.h>
#include <tk/syslib.h>
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "robot_network.h"
#include "network_queue.h"
#include "robot_health.h"
#include "telemetry.h"
#include "log_stream.h"
#include "robot_mqtt_config.h"
#include "../platform/drv_time.h"
#include "../../lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.h"
#define RETRY_MS 3000u
static mqtt_client_t *client;
static ip_addr_t broker;
static bool configured;
static bool connecting;
static bool subscribed;
static bool subscribe_pending;
static bool commands_enabled;
static uint8_t token[RM_TOKEN_SIZE];
static uint8_t rx[RM_COMMAND_SIZE];
static uint32_t rx_length;
static uint32_t rx_expected;
static bool rx_valid;
static uint32_t retry_at;
static uint32_t subscribe_at;
static uint32_t state_at;
static uint32_t heartbeat_at;
static uint32_t state_sequence;
static uint32_t sonar_cursor;
static rm_heartbeat_t metrics;
static rm_heartbeat_t health_cache = {.stack_min_free = 65535u};
static bool health_present;
static rm_event_t pending_event;
static bool have_event;
static rm_sonar_t pending_sonar;
static bool have_sonar;

static void inc(uint32_t *value)
{
    UINT irq;
    DI(irq);
    ++*value;
    EI(irq);
}

static bool due(uint32_t now, uint32_t at)
{
    return (int32_t)(now - at) >= 0;
}

static int hex(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

static void configure(void)
{
    const char *s = ROBOT_MQTT_COMMAND_TOKEN_HEX;
    bool good = strlen(s) == 32u;
    uint8_t any = 0u;
    for (unsigned int i = 0u; good && i < RM_TOKEN_SIZE; ++i) {
        const int a = hex(s[i * 2u]);
        const int b = hex(s[i * 2u + 1u]);
        if (a < 0 || b < 0) {
            good = false;
            break;
        }
        token[i] = (uint8_t)((unsigned int)a * 16u + (unsigned int)b);
        any |= token[i];
    }
    /* Absent/empty/all-zero example token: ESTOP-only fallback in decoder.
     * A valid configured token restricts EVERY command, including ESTOP. */
    commands_enabled = good && any != 0u;
    configured = true;
}

void __attribute__((weak)) robot_network_state(rm_state_t *out)
{
    *out = (rm_state_t){.line_age_ms = 65535u,
                        .motion_age_ms = 65535u,
                        .vehicle_age_ms = 65535u};
}

int __attribute__((weak)) robot_network_sonar(uint32_t *cursor, rm_sonar_t *out,
                                              uint32_t *missed)
{
    (void)cursor;
    (void)out;
    *missed = 0u;
    return E_TMOUT;
}

void robot_network_metrics(rm_heartbeat_t *out)
{
    UINT irq;
    DI(irq);
    out->connect_attempts = metrics.connect_attempts;
    out->sonar_history_missed = metrics.sonar_history_missed;
    out->mqtt_hook_max_us = metrics.mqtt_hook_max_us;
    out->publish_ok = metrics.publish_ok;
    out->publish_failed = metrics.publish_failed;
    out->latest_sonar_history_sequence = metrics.latest_sonar_history_sequence;
    out->mqtt_state = metrics.mqtt_state;
    out->wifi_observed_priority = metrics.wifi_observed_priority;
    const bool connected = metrics.mqtt_state >= 2u;
    if (connected) {
        out->flags |= 2u;
    }
    if (subscribed) {
        out->flags |= 4u;
    }
    if (log_stream_dump_active()) {
        out->flags |= 8u;
    }
    if (commands_enabled) {
        out->flags |= 16u;
    }
    T_CYW43_UTK_STATUS status;
    cyw43_utk_get_status(&status);
    out->wifi_link_status = (int8_t)status.link_status;
    if (status.link_status == 3) {
        out->flags |= 1u;
    }
    EI(irq);
    T_CYW43_TIMING timing;
    (void)cyw43_utk_timing_read(&timing);
    out->wifi_poll_gap_max_us = timing.gap_max_us;
    out->wifi_poll_gap_samples = timing.gaps;
    out->wifi_start_status = cyw43_utk_start_status();
    out->wifi_requested_priority = 20u;
    robot_queue_metrics(out);
}

int robot_network_dump(uint32_t request_id)
{
    return log_stream_dump_request(request_id);
}

static void incoming_publish(void *arg, const char *topic, u32_t length)
{
    (void)arg;
    rx_length = 0u;
    rx_expected = length;
    rx_valid = strcmp(topic, "robot/cmd") == 0 && length == RM_COMMAND_SIZE;
}

static void incoming_data(void *arg, const u8_t *data, u16_t length, u8_t flags)
{
    (void)arg;
    if (length > RM_COMMAND_SIZE - rx_length) {
        rx_valid = false;
    }
    if (rx_valid) {
        for (unsigned int i = 0u; i < length; ++i) {
            rx[rx_length++] = data[i];
        }
    }
    if ((flags & MQTT_DATA_FLAG_LAST) != 0u) {
        rm_command_t command;
        if (rx_valid && rx_length == rx_expected &&
            rm_command_decode(rx, rx_length, token, commands_enabled,
                              &command)) {
            const robot_command_t q = {command.opcode, command.request_id};
            (void)robot_command_push(&q);
        } else {
            robot_command_lost();
        }
        rx_valid = false;
    }
}

static void subscription_done(void *arg, err_t error)
{
    (void)arg;
    subscribe_pending = false;
    subscribed = error == ERR_OK;
    metrics.mqtt_state = subscribed ? 3u : 2u;
    subscribe_at = drv_time_ms() + RETRY_MS;
    if (error != ERR_OK) {
        inc(&metrics.publish_failed);
    }
}

static void connection_done(mqtt_client_t *c, void *arg,
                            mqtt_connection_status_t status)
{
    (void)c;
    (void)arg;
    connecting = false;
    subscribed = false;
    subscribe_pending = false;
    if (status == MQTT_CONNECT_ACCEPTED) {
        metrics.mqtt_state = 2u;
        mqtt_set_inpub_callback(client, incoming_publish, incoming_data, NULL);
        subscribe_at = drv_time_ms();
        state_at = subscribe_at;
        heartbeat_at = subscribe_at;
    } else {
        metrics.mqtt_state = 0u;
        retry_at = drv_time_ms() + RETRY_MS;
        inc(&metrics.publish_failed); /* asynchronous refusal/timeout/drop */
        /* lwIP mqtt_close deletes pending requests WITHOUT callbacks: the log
         * streams rewind to their acknowledged cursors and live jumps. */
        log_stream_link_down();
    }
}

static void published(void *arg, err_t error)
{
    (void)arg;
    if (error != ERR_OK) {
        inc(&metrics.publish_failed);
    }
}

static bool publish(const char *topic, const uint8_t *data, unsigned int length)
{
    const err_t error = mqtt_publish(client, topic, data, (u16_t)length, 0u, 0u,
                                     published, NULL);
    inc(error == ERR_OK ? &metrics.publish_ok : &metrics.publish_failed);
    return error == ERR_OK;
}

/* QoS 1 acknowledgement (DUMP only; the live stream is QoS 0). */
static void log_done(void *arg, err_t error)
{
    log_stream_complete((uint32_t)(uintptr_t)arg, error == ERR_OK);
    if (error != ERR_OK) {
        inc(&metrics.publish_failed);
    }
}

/* Publish sink for log_stream: one frame on robot/log. */
static int log_publish(const uint8_t *data, size_t length, uint8_t qos,
                       uint32_t serial)
{
    const err_t error =
        mqtt_publish(client, "robot/log", data, (u16_t)length, qos, 0u,
                     qos != 0u ? log_done : NULL, (void *)(uintptr_t)serial);
    inc(error == ERR_OK ? &metrics.publish_ok : &metrics.publish_failed);
    return error == ERR_OK ? 0 : 1;
}

static void service(uint32_t now)
{
    unsigned int budget = 4u;
    if (!subscribed && !subscribe_pending && due(now, subscribe_at)) {
        subscribe_pending = true;
        if (mqtt_subscribe(client, "robot/cmd", 1u, subscription_done, NULL) !=
            ERR_OK) {
            subscribe_pending = false;
            subscribe_at = now + RETRY_MS;
            inc(&metrics.publish_failed);
        }
    }
    if (due(now, state_at)) {
        rm_state_t v = {0};
        robot_network_state(&v);
        v.uptime_ms = drv_time_ms();
        v.sequence = ++state_sequence;
        uint8_t bytes[RM_STATE_SIZE];
        rm_pack_state(bytes, &v);
        (void)publish("robot/state", bytes, sizeof(bytes));
        state_at = now + 200u;
        --budget;
    }
    if (due(now, heartbeat_at)) {
        rm_heartbeat_t v;
        uint32_t us;
        const int er = robot_health_try_read(&v, &us);
        if (er >= E_OK) {
            health_cache = v;
            health_present = true;
        } else {
            robot_network_busy(ROBOT_BOARD_HEALTH);
        }
        /* Producer uptime is in milliseconds: a stalled health board must
         * not become fresh again at the 71-minute microsecond wrap. */
        v = health_cache;
        const uint32_t age =
            health_present ? drv_time_ms() - health_cache.uptime_ms : 65535u;
        v.flags = 0u;
        v.uptime_ms = drv_time_ms();
        v.health_age_ms = age > 65535u ? 65535u : (uint16_t)age;
        if (er >= E_OK && age <= 2500u) {
            v.flags |= 32u;
        }
        robot_network_metrics(&v);
        uint8_t bytes[RM_HEARTBEAT_SIZE];
        rm_pack_heartbeat(bytes, &v);
        (void)publish("robot/heartbeat", bytes, sizeof(bytes));
        heartbeat_at = now + 1000u;
        --budget;
    }
    if (!have_event) {
        have_event =
            robot_event_pop(&pending_event) == (int)sizeof(pending_event);
    }
    if (have_event && budget > 0u) {
        uint8_t bytes[RM_EVENT_SIZE];
        rm_pack_event(bytes, &pending_event);
        if (publish("robot/event", bytes, sizeof(bytes))) {
            have_event = false;
        }
        --budget;
    }
    for (unsigned int n = 0u; n < 2u && budget > 0u; ++n) {
        if (!have_sonar) {
            uint32_t missed = 0u;
            const int er =
                robot_network_sonar(&sonar_cursor, &pending_sonar, &missed);
            metrics.sonar_history_missed += missed;
            if (er == E_OBJ) {
                robot_network_busy(ROBOT_BOARD_SONAR);
            }
            have_sonar = er >= E_OK;
        }
        if (!have_sonar) {
            break;
        }
        uint8_t bytes[RM_SONAR_SIZE];
        rm_pack_sonar(bytes, &pending_sonar);
        if (!publish("robot/sonar", bytes, sizeof(bytes))) {
            --budget;
            break;
        }
        metrics.latest_sonar_history_sequence = pending_sonar.history_sequence;
        have_sonar = false;
        --budget;
    }
    /* Own bounded budget: at most 1 live batch + 2 DUMP frames per poll. */
    log_stream_service(log_publish);
}

/* Strong symbol replaces the lib weak demo hook; prove binding in the ELF. */
void cyw43_utk_app_poll(void)
{
    const uint32_t started = drv_time_us();
    const uint32_t now = drv_time_ms();
    if (!configured) {
        configure();
    }
    if (metrics.wifi_observed_priority == 0u) {
        T_RTSK status;
        if (tk_ref_tsk(TSK_SELF, &status) >= E_OK) {
            metrics.wifi_observed_priority = (uint8_t)status.tskpri;
        }
    }
    if (client != NULL && mqtt_client_is_connected(client)) {
        service(now);
    } else if (!connecting && due(now, retry_at)) {
        retry_at = now + RETRY_MS;
        if (ip4addr_aton(ROBOT_MQTT_BROKER_IP, ip_2_ip4(&broker)) &&
            !ip4_addr_isany_val(*ip_2_ip4(&broker))) {
            IP_SET_TYPE(&broker, IPADDR_TYPE_V4);
            if (client == NULL) {
                client = mqtt_client_new();
            }
            if (client != NULL) {
                const struct mqtt_connect_client_info_t info = {
                    .client_id = ROBOT_MQTT_CLIENT_ID, .keep_alive = 30u};
                connecting = true;
                metrics.mqtt_state = 1u;
                inc(&metrics.connect_attempts);
                if (mqtt_client_connect(client, &broker, ROBOT_MQTT_BROKER_PORT,
                                        connection_done, NULL,
                                        &info) != ERR_OK) {
                    connecting = false;
                    metrics.mqtt_state = 0u;
                    inc(&metrics.publish_failed);
                }
            } else {
                inc(&metrics.publish_failed);
            }
        }
    }
    const uint32_t elapsed = drv_time_us() - started;
    if (elapsed > metrics.mqtt_hook_max_us) {
        metrics.mqtt_hook_max_us = elapsed;
    }
}
#endif

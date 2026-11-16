/* All lwIP entry points run on cyw43_utk_task, never on a driving task. */
#if TM_MQTT
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>

/* Task timing and resource limits. */
#define HEALTH_FRESH_MS      2500u
#define WIFI_TASK_PRIORITY   20u
#define ROUTINE_FRAME_BUDGET 4u
#define SONAR_FRAME_BUDGET   2u

#include <string.h>
#include <tk/tkernel.h>
#include <tk/syslib.h>
#include <tk/smp.h>   /* Interrupt-preserving metrics lock. */
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "robot_network.h"
#include "network_queue.h"
#include "robot_health.h"
#include "telemetry.h"
#include "log_stream.h"
#include "telemetry_mode.h"
#include "robot_mqtt_config.h"
#include "../platform/drv_time.h"
#include "../platform/boot_id.h"
#include "../platform/app_startup.h"
#include "../../lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.h"
#define RETRY_MS 3000u

/*
 * Connection liveness. The broker drops a silent client after 1.5 times the
 * keep-alive time, then publishes the will message (robot/status offline).
 * lwIP checks the timer every 5 s, so the value is a multiple of 5.
 * 5 s gives about 7.5 s detection. While the robot publishes, TCP
 * acknowledgements reset the timer and no PINGREQ is sent.
 */
#define KEEPALIVE_S       5u
#define STATUS_RETRY_MS   500u
#define TOPIC_STATUS      "robot/status"
#define TOPIC_OBSTACLE    "robot/obstacle"
#define TOPIC_DETOUR      "robot/detour"
#define TOPIC_SUMMARY     "robot/summary"
#define QOS_AT_LEAST_ONCE 1u
#define RETAIN_ON         1u
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

/* B1: Health and Wi-Fi share a core in the control and UP profiles.
 * Mask local interrupts before the cross-core claim, so Wi-Fi cannot preempt
 * a Health holder and spin against it. Restore the caller's exact mask.
 * A zero-filled static already holds the unlocked state. */
static T_SPLOCK metrics_lock;
static rm_heartbeat_t health_cache = {.stack_min_free = RM_NO_DATA_U16,
                                      .cpu_load_permille = RM_CPU_LOAD_UNKNOWN};
static bool health_present;
static rm_event_t pending_event;
static bool have_event;
static rm_sonar_t pending_sonar;
static bool have_sonar;
static uint32_t obstacle_cursor;
static rm_obstacle_t pending_obstacle;
static bool have_obstacle;
static uint32_t detour_cursor;
static rm_detour_t pending_detour;
static bool have_detour;
static uint32_t summary_cursor;
static uint32_t summary_retry_at;
static pm_tracker_t mode_tracker;
static pm_mode_t mode = PM_MODE_RUN;
static bool status_pending;
static uint32_t status_retry_at;
static uint8_t will_payload[RM_STATUS_SIZE];

/*
 * Increment a diagnostic counter without racing another context.
 */
static void inc(uint32_t *value)
{
    UINT intsts;
    (void)ISpinLock(&metrics_lock, &intsts);
    ++*value;
    (void)ISpinUnlock(&metrics_lock, intsts);
}

/*
 * RT-0058 finding 1: Health copies this group on core 0, so EVERY write to a
 * copied field goes through one of these setters and takes metrics_lock.  Before
 * this, the snapshot was locked but the writers were not, which protected the
 * reader from nothing.  Each setter is short and contains no lwIP call, so no
 * network operation ever runs with the lock held.
 */
static void metrics_state_set(uint8_t state)
{
    UINT intsts;
    (void)ISpinLock(&metrics_lock, &intsts);
    metrics.mqtt_state = state;
    (void)ISpinUnlock(&metrics_lock, intsts);
}

static void metrics_subscribed_set(bool value)
{
    UINT intsts;
    (void)ISpinLock(&metrics_lock, &intsts);
    subscribed = value;
    (void)ISpinUnlock(&metrics_lock, intsts);
}

/* Subscription result and state are one fact; set them in one section so Health
 * cannot see the flag and the state disagree. */
static void metrics_subscribed_and_state(bool value, uint8_t state)
{
    UINT intsts;
    (void)ISpinLock(&metrics_lock, &intsts);
    subscribed = value;
    metrics.mqtt_state = state;
    (void)ISpinUnlock(&metrics_lock, intsts);
}

static void metrics_commands_enabled_set(bool value)
{
    UINT intsts;
    (void)ISpinLock(&metrics_lock, &intsts);
    commands_enabled = value;
    (void)ISpinUnlock(&metrics_lock, intsts);
}

static void metrics_sonar_missed_add(uint32_t missed)
{
    UINT intsts;
    (void)ISpinLock(&metrics_lock, &intsts);
    metrics.sonar_history_missed += missed;
    (void)ISpinUnlock(&metrics_lock, intsts);
}

static void metrics_sonar_sequence_set(uint32_t sequence)
{
    UINT intsts;
    (void)ISpinLock(&metrics_lock, &intsts);
    metrics.latest_sonar_history_sequence = sequence;
    (void)ISpinUnlock(&metrics_lock, intsts);
}

static bool metrics_observed_priority_unset(void)
{
    UINT intsts;
    (void)ISpinLock(&metrics_lock, &intsts);
    const bool unset = metrics.wifi_observed_priority == 0u;
    (void)ISpinUnlock(&metrics_lock, intsts);
    return unset;
}

static void metrics_observed_priority_set_if_unset(uint8_t priority)
{
    UINT intsts;
    (void)ISpinLock(&metrics_lock, &intsts);
    if (metrics.wifi_observed_priority == 0u) {
        metrics.wifi_observed_priority = priority;
    }
    (void)ISpinUnlock(&metrics_lock, intsts);
}

static void metrics_hook_max_update(uint32_t elapsed)
{
    UINT intsts;
    (void)ISpinLock(&metrics_lock, &intsts);
    if (elapsed > metrics.mqtt_hook_max_us) {
        metrics.mqtt_hook_max_us = elapsed;
    }
    (void)ISpinUnlock(&metrics_lock, intsts);
}

/*
 * Test a wrapping millisecond deadline.
 */
static bool due(uint32_t now, uint32_t at)
{
    return (int32_t)(now - at) >= 0;
}

/*
 * Decode one hexadecimal digit for private-token configuration.
 */
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

/*
 * Load the broker address and validate the optional command token.
 */
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
    metrics_commands_enabled_set(good && any != 0u);
    log_stream_init(boot_id_get());
    pm_init(&mode_tracker);
    configured = true;
}

/*
 * Provide the vehicle run phase for the publication mode. The default has
 * no vehicle consumer, so TEST and BENCH images always publish at RUN rates.
 */
int __attribute__((weak)) robot_network_vehicle_phase(uint8_t *phase,
                                                      bool *active)
{
    *phase = 0u;
    *active = false;
    return E_OBJ;
}

/*
 * Update the publication mode from one bounded vehicle board read.
 */
static void update_mode(uint32_t now)
{
    uint8_t phase = 0u;
    bool active = false;
    const int er = robot_network_vehicle_phase(&phase, &active);
    pm_board_t board = PM_BOARD_OK;

    if ((er == E_OBJ) || (er == E_NOEXS)) {
        board = PM_BOARD_NONE;
    } else if (er < E_OK) {
        board = PM_BOARD_BUSY;
    } else {
        /* Readable board: use the phase and the run flag. */
    }
    const pm_mode_t next = pm_update(&mode_tracker, board, phase, active, now);

    if ((next == PM_MODE_RUN) && (mode == PM_MODE_IDLE)) {
        /* Leave IDLE at once: the next state and heartbeat are not delayed
         * by the long idle periods. */
        state_at = now;
        heartbeat_at = now;
    }
    mode = next;
}

/*
 * Provide current state, using unavailable values when no provider exists.
 */
void __attribute__((weak)) robot_network_state(rm_state_t *out)
{
    *out = (rm_state_t){.line_age_ms = RM_NO_DATA_U16,
                        .motion_age_ms = RM_NO_DATA_U16,
                        .vehicle_age_ms = RM_NO_DATA_U16};
}

/*
 * Provide one sonar history entry, or report no available provider.
 */
int __attribute__((weak)) robot_network_sonar(uint32_t *cursor, rm_sonar_t *out,
                                              uint32_t *missed)
{
    (void)cursor;
    (void)out;
    *missed = 0u;
    return E_TMOUT;
}

/*
 * Skip the sonar cursor to the newest entry, or report no provider.
 */
int __attribute__((weak)) robot_network_sonar_skip(uint32_t *cursor)
{
    (void)cursor;
    return E_OBJ;
}

/*
 * Provide the newest scan report, or report no available provider.
 */
int __attribute__((weak)) robot_network_obstacle(uint32_t *cursor,
                                                 rm_obstacle_t *out)
{
    (void)cursor;
    (void)out;
    return E_NOEXS;
}

/*
 * Provide the newest detour report, or report no available provider.
 */
int __attribute__((weak)) robot_network_detour(uint32_t *cursor,
                                               rm_detour_t *out)
{
    (void)cursor;
    (void)out;
    return E_NOEXS;
}

/*
 * Provide the newest run summary, or report no available provider.
 */
int __attribute__((weak)) robot_network_summary(uint32_t *cursor,
                                                rm_summary_t *out)
{
    (void)cursor;
    (void)out;
    return E_NOEXS;
}

/*
 * Collect link and queue counters without waiting on sensor tasks.
 */
void robot_network_metrics(rm_heartbeat_t *out)
{
    /* Snapshot under the lock, then RELEASE it before any call that takes its own
     * lock - log_stream_dump_active(), cyw43_utk_get_status() and
     * cyw43_utk_timing_read() below.  Holding one lock while acquiring another
     * invites an ordering problem, and the previous code masked interrupts across
     * all of those calls. */
    rm_heartbeat_t snapshot;
    bool connected;
    bool subscribed_now;
    bool commands_now;
    UINT intsts;
    (void)ISpinLock(&metrics_lock, &intsts);
    snapshot = metrics;
    connected = metrics.mqtt_state >= 2u;
    subscribed_now = subscribed;
    commands_now = commands_enabled;
    (void)ISpinUnlock(&metrics_lock, intsts);

    out->connect_attempts = snapshot.connect_attempts;
    out->sonar_history_missed = snapshot.sonar_history_missed;
    out->mqtt_hook_max_us = snapshot.mqtt_hook_max_us;
    out->publish_ok = snapshot.publish_ok;
    out->publish_failed = snapshot.publish_failed;
    out->latest_sonar_history_sequence = snapshot.latest_sonar_history_sequence;
    out->mqtt_state = snapshot.mqtt_state;
    out->wifi_observed_priority = snapshot.wifi_observed_priority;
    if (connected) {
        out->flags |= 2u;
    }
    if (subscribed_now) {
        out->flags |= 4u;
    }
    if (log_stream_dump_active()) {
        out->flags |= 8u;
    }
    if (commands_now) {
        out->flags |= 16u;
    }
    T_CYW43_UTK_STATUS status;
    cyw43_utk_get_status(&status);
    out->wifi_link_status = (int8_t)status.link_status;
    if (status.link_status == 3) {
        out->flags |= 1u;
    }
    T_CYW43_TIMING timing;
    (void)cyw43_utk_timing_read(&timing);
    out->wifi_poll_gap_max_us = timing.gap_max_us;
    out->wifi_poll_gap_samples = timing.gaps;
    out->wifi_start_status = cyw43_utk_start_status();
    out->wifi_requested_priority = WIFI_TASK_PRIORITY;
    robot_queue_metrics(out);
    out->boot_id = boot_id_get();
}

/*
 * Forward a DUMP request to the bounded stream state machine.
 */
int robot_network_dump(uint32_t request_id)
{
    return log_stream_dump_request(request_id);
}

/*
 * Bound the command payload before accepting incoming fragments.
 */
static void incoming_publish(void *arg, const char *topic, u32_t length)
{
    (void)arg;
    rx_length = 0u;
    rx_expected = length;
    rx_valid = strcmp(topic, "robot/cmd") == 0 && length == RM_COMMAND_SIZE;
}

/*
 * Assemble and validate a command, then queue it without waiting.
 */
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

/*
 * Record subscription completion and schedule retries on failure.
 */
static void subscription_done(void *arg, err_t error)
{
    (void)arg;
    subscribe_pending = false;
    metrics_subscribed_and_state(error == ERR_OK,
                                 error == ERR_OK ? 3u : 2u);
    subscribe_at = drv_time_ms() + RETRY_MS;
    if (error != ERR_OK) {
        inc(&metrics.publish_failed);
    }
}

/*
 * Reset connection-local state and rewind streams after link loss.
 */
static void connection_done(mqtt_client_t *c, void *arg,
                            mqtt_connection_status_t status)
{
    (void)c;
    (void)arg;
    connecting = false;
    metrics_subscribed_set(false);
    subscribe_pending = false;
    if (status == MQTT_CONNECT_ACCEPTED) {
        metrics_state_set(2u);
        mqtt_set_inpub_callback(client, incoming_publish, incoming_data, NULL);
        subscribe_at = drv_time_ms();
        status_pending = true;
        status_retry_at = subscribe_at;
        summary_cursor = 0u; /* publish the last run summary again */
        summary_retry_at = subscribe_at;
        state_at = subscribe_at;
        heartbeat_at = subscribe_at;
    } else {
        metrics_state_set(0u);
        retry_at = drv_time_ms() + RETRY_MS;
        inc(&metrics.publish_failed); /* asynchronous refusal/timeout/drop */
        /* lwIP mqtt_close deletes pending requests WITHOUT callbacks: the log
         * streams rewind to their acknowledged cursors and live jumps. */
        log_stream_link_down();
    }
}

/*
 * Acknowledgement of the retained "online" status. A failure or timeout makes
 * the service publish it again. A dropped link deletes the request without a
 * callback, but every new connection publishes the status again.
 */
static void status_done(void *arg, err_t error)
{
    (void)arg;
    if (error != ERR_OK) {
        status_pending = true;
        status_retry_at = drv_time_ms() + STATUS_RETRY_MS;
        inc(&metrics.publish_failed);
    }
}

/* Publish the retained "online" status once per connection (QoS 1). */
static void service_status(uint32_t now)
{
    uint8_t bytes[RM_STATUS_SIZE];
    err_t error;

    if (!status_pending || !due(now, status_retry_at)) {
        return;
    }
    rm_pack_status(bytes, RM_STATUS_ONLINE, boot_id_get());
    error = mqtt_publish(client, TOPIC_STATUS, bytes, RM_STATUS_SIZE,
                         QOS_AT_LEAST_ONCE, RETAIN_ON, status_done, NULL);
    inc(error == ERR_OK ? &metrics.publish_ok : &metrics.publish_failed);
    if (error == ERR_OK) {
        status_pending = false;
    } else {
        status_retry_at = now + STATUS_RETRY_MS;
    }
}

/* Acknowledgement of a run summary. A failure makes the service read the
 * newest summary and publish it again. */
static void summary_done(void *arg, err_t error)
{
    (void)arg;
    if (error != ERR_OK) {
        summary_cursor = 0u;
        summary_retry_at = drv_time_ms() + STATUS_RETRY_MS;
        inc(&metrics.publish_failed);
    }
}

/*
 * Publish the newest run summary (QoS 1, retained) when it is new. The
 * cursor is the run id. Every new connection and every failure clears it, so
 * the last summary is published again.
 */
static void service_summary(uint32_t now)
{
    rm_summary_t value;
    uint8_t bytes[RM_SUMMARY_SIZE];
    uint32_t cursor = summary_cursor;
    err_t error;

    if (!due(now, summary_retry_at) ||
        robot_network_summary(&cursor, &value) < E_OK) {
        return;
    }
    value.boot_id = boot_id_get();
    rm_pack_summary(bytes, &value);
    error = mqtt_publish(client, TOPIC_SUMMARY, bytes, RM_SUMMARY_SIZE,
                         QOS_AT_LEAST_ONCE, RETAIN_ON, summary_done, NULL);
    inc(error == ERR_OK ? &metrics.publish_ok : &metrics.publish_failed);
    if (error == ERR_OK) {
        summary_cursor = cursor;
    } else {
        summary_retry_at = now + STATUS_RETRY_MS;
    }
}

/*
 * Count an asynchronous publish failure without affecting driving.
 */
static void published(void *arg, err_t error)
{
    (void)arg;
    if (error != ERR_OK) {
        inc(&metrics.publish_failed);
    }
}

/*
 * Send one non-retained observation; failure does not affect driving.
 */
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

/*
 * Service due publications with bounded per-poll work.
 */
static void service(uint32_t now)
{
    unsigned int budget = ROUTINE_FRAME_BUDGET;
    update_mode(now);
    service_status(now);
    service_summary(now);
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
        v.mode = (mode == PM_MODE_IDLE) ? RM_STATE_MODE_IDLE : 0u;
        uint8_t bytes[RM_STATE_SIZE];
        v.boot_id = boot_id_get();
        rm_pack_state(bytes, &v);
        (void)publish("robot/state", bytes, sizeof(bytes));
        state_at = now + pm_state_period_ms(mode);
        --budget;
    }
    if (due(now, heartbeat_at)) {
        rm_heartbeat_t v;
        uint32_t us;
        const int er = robot_health_try_read(&v, &us);
        if (er >= E_OK) {
            health_cache = v;
            health_present = true;
        }
        robot_network_board_result(ROBOT_BOARD_HEALTH, er);
        /* Producer uptime is in milliseconds: a stalled health board must
         * not become fresh again at the 71-minute microsecond wrap. */
        v = health_cache;
        const uint32_t age = health_present
                                 ? drv_time_ms() - health_cache.uptime_ms
                                 : RM_NO_DATA_U16;
        v.flags = 0u;
        v.uptime_ms = drv_time_ms();
        v.health_age_ms = age > RM_NO_DATA_U16 ? RM_NO_DATA_U16 : (uint16_t)age;
        if (er >= E_OK && age <= HEALTH_FRESH_MS) {
            v.flags |= 32u;
        }
        robot_network_metrics(&v);
        if (mode == PM_MODE_IDLE) {
            v.flags |= RM_HEARTBEAT_FLAG_IDLE;
        }
        uint8_t bytes[RM_HEARTBEAT_SIZE];
        rm_pack_heartbeat(bytes, &v);
        (void)publish("robot/heartbeat", bytes, sizeof(bytes));
        heartbeat_at = now + pm_heartbeat_period_ms(mode);
        --budget;
    }
    if (!have_event) {
        have_event =
            robot_event_pop(&pending_event) == (int)sizeof(pending_event);
    }
    if (have_event && budget > 0u) {
        uint8_t bytes[RM_EVENT_SIZE];
        pending_event.boot_id = boot_id_get();
        rm_pack_event(bytes, &pending_event);
        if (publish("robot/event", bytes, sizeof(bytes))) {
            have_event = false;
        }
        --budget;
    }
    if (!have_obstacle && (budget > 0u)) {
        have_obstacle =
            robot_network_obstacle(&obstacle_cursor, &pending_obstacle) >= E_OK;
    }
    if (have_obstacle && (budget > 0u)) {
        uint8_t bytes[RM_OBSTACLE_SIZE];
        pending_obstacle.boot_id = boot_id_get();
        rm_pack_obstacle(bytes, &pending_obstacle);
        if (publish(TOPIC_OBSTACLE, bytes, sizeof(bytes))) {
            have_obstacle = false;
        }
        --budget;
    }
    if (!have_detour && (budget > 0u)) {
        have_detour =
            robot_network_detour(&detour_cursor, &pending_detour) >= E_OK;
    }
    if (have_detour && (budget > 0u)) {
        uint8_t bytes[RM_DETOUR_SIZE];
        pending_detour.boot_id = boot_id_get();
        rm_pack_detour(bytes, &pending_detour);
        if (publish(TOPIC_DETOUR, bytes, sizeof(bytes))) {
            have_detour = false;
        }
        --budget;
    }
    if (!pm_streams_live(mode)) {
        /* IDLE: a deliberate pause, like the live-log jump. Keep the cursor
         * at the newest entry, so RUN publishes only later results and the
         * pause is not counted in sonar_history_missed. Drop a held entry. */
        (void)robot_network_sonar_skip(&sonar_cursor);
        have_sonar = false;
    }
    for (unsigned int n = 0u;
         pm_streams_live(mode) && (n < SONAR_FRAME_BUDGET) && (budget > 0u);
         ++n) {
        if (!have_sonar) {
            uint32_t missed = 0u;
            const int er =
                robot_network_sonar(&sonar_cursor, &pending_sonar, &missed);
            metrics_sonar_missed_add(missed);
            if (er == E_TMOUT) {
                robot_network_busy(ROBOT_BOARD_SONAR);
            }
            have_sonar = er >= E_OK;
        }
        if (!have_sonar) {
            break;
        }
        uint8_t bytes[RM_SONAR_SIZE];
        pending_sonar.boot_id = boot_id_get();
        rm_pack_sonar(bytes, &pending_sonar);
        if (!publish("robot/sonar", bytes, sizeof(bytes))) {
            --budget;
            break;
        }
        metrics_sonar_sequence_set(pending_sonar.history_sequence);
        have_sonar = false;
        --budget;
    }
    /* Own bounded budget: at most 1 live batch + 2 DUMP frames per poll.
     * IDLE stops only the live batches; a DUMP refill still runs. */
    log_stream_service(log_publish, pm_streams_live(mode));
}

/* Strong symbol replaces the lib weak demo hook; prove binding in the ELF. */
void cyw43_utk_app_poll(void)
{
    /* The radio can run before usermain initializes application providers. */
    if (!app_startup_complete()) {
        return;
    }
    if (boot_id_get() == 0u) {
        /* Failed entropy cannot publish a repeatable boot identity. */
        return;
    }
    const uint32_t started = drv_time_us();
    const uint32_t now = drv_time_ms();
    if (!configured) {
        configure();
    }
    if (metrics_observed_priority_unset()) {
        T_RTSK status;
        if (tk_ref_tsk(TSK_SELF, &status) >= E_OK) {
            metrics_observed_priority_set_if_unset((uint8_t)status.tskpri);
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
                rm_pack_status(will_payload, RM_STATUS_OFFLINE, boot_id_get());
                const struct mqtt_connect_client_info_t info = {
                    .client_id = ROBOT_MQTT_CLIENT_ID,
                    .keep_alive = KEEPALIVE_S,
                    .will_topic = TOPIC_STATUS,
                    .will_msg = (const char *)will_payload,
                    .will_msg_len = RM_STATUS_SIZE,
                    .will_qos = QOS_AT_LEAST_ONCE,
                    .will_retain = RETAIN_ON};
                connecting = true;
                metrics_state_set(1u);
                inc(&metrics.connect_attempts);
                if (mqtt_client_connect(client, &broker, ROBOT_MQTT_BROKER_PORT,
                                        connection_done, NULL,
                                        &info) != ERR_OK) {
                    connecting = false;
                    metrics_state_set(0u);
                    inc(&metrics.publish_failed);
                }
            } else {
                inc(&metrics.publish_failed);
            }
        }
    }
    const uint32_t elapsed = drv_time_us() - started;
    metrics_hook_max_update(elapsed);
}
#endif

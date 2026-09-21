#include "telemetry.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"

#include "lwip/err.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#if __has_include("wifi_config.h")
#include "wifi_config.h"
#else
#error "Missing src/wifi_config.h; copy src/wifi_config.h.example to src/wifi_config.h and set the credentials."
#endif

#define TELEMETRY_DESTINATION_IP "192.168.18.98"
#define TELEMETRY_DESTINATION_PORT 4242
#define TELEMETRY_PROTOCOL_VERSION 1
#define WIFI_CONNECT_TIMEOUT_MS 30000
#define TELEMETRY_DEVICE_ID_SIZE 64

#ifndef TELEMETRY_VERBOSE
#define TELEMETRY_VERBOSE 1
#endif

typedef struct {
    struct udp_pcb *udp;
    ip_addr_t destination;
    char device_id[TELEMETRY_DEVICE_ID_SIZE];
    uint32_t session_id;
    uint32_t sequence;
    uint32_t errors;
    bool cyw43_started;
} telemetry_state_t;

static telemetry_state_t telemetry_state;

static const char *link_status_name(int status) {
    switch (status) {
        case CYW43_LINK_DOWN:
            return "down";
        case CYW43_LINK_JOIN:
            return "join";
        case CYW43_LINK_NOIP:
            return "noip";
        case CYW43_LINK_UP:
            return "up";
        case CYW43_LINK_FAIL:
            return "fail";
        case CYW43_LINK_NONET:
            return "nonet";
        case CYW43_LINK_BADAUTH:
            return "badauth";
        default:
            return "unknown";
    }
}

static void record_error(const char *message) {
    ++telemetry_state.errors;
    printf("Telemetry error %lu: %s\n",
           (unsigned long)telemetry_state.errors, message);
}

static uint32_t make_session_id(void) {
    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);
    uint32_t hash = 2166136261u;

    for (size_t index = 0; index < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; ++index) {
        hash ^= board_id.id[index];
        hash *= 16777619u;
    }

    const uint32_t boot_time =
        to_us_since_boot(get_absolute_time());
    hash ^= boot_time;
    hash *= 16777619u;
    return hash;
}

static int current_link_status(void) {
    if (!telemetry_state.cyw43_started) {
        return CYW43_LINK_DOWN;
    }

    int status;
    /* Threadsafe background runs lwIP asynchronously; lock every main-loop access. */
    cyw43_arch_lwip_begin();
    status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
    cyw43_arch_lwip_end();
    return status;
}

static void print_network_details(int link_status) {
    char ip_address[16];
    char gateway[16];
    bool have_network_details = false;

    /* Copy lwIP-owned values while locked; keep all potentially blocking I/O outside. */
    cyw43_arch_lwip_begin();
    if (netif_default != NULL) {
        ip4addr_ntoa_r(netif_ip4_addr(netif_default),
                       ip_address, sizeof(ip_address));
        ip4addr_ntoa_r(netif_ip4_gw(netif_default),
                       gateway, sizeof(gateway));
        have_network_details = true;
    }
    cyw43_arch_lwip_end();

    printf("Link status: %d (%s)\n",
           link_status, link_status_name(link_status));
    if (!have_network_details) {
        printf("IP address: unavailable\n");
        printf("Gateway: unavailable\n");
        return;
    }
    printf("IP address: %s\n", ip_address);
    printf("Gateway: %s\n", gateway);
}

static bool safe_field_text(const char *text) {
    if (text == NULL || text[0] == '\0') {
        return false;
    }
    for (const char *character = text; *character != '\0'; ++character) {
        if ((unsigned char)*character < 32u ||
            (unsigned char)*character > 126u ||
            *character == ',' || *character == '=' ||
            *character == '\r' || *character == '\n') {
            return false;
        }
    }
    return true;
}

static bool append_pair(char *buffer, size_t capacity, size_t *used,
                        const char *key, const char *value) {
    const int written = snprintf(buffer + *used, capacity - *used,
                                 "%s%s=%s", *used == 0 ? "" : ",",
                                 key, value);
    if (written < 0 || (size_t)written >= capacity - *used) {
        return false;
    }
    *used += (size_t)written;
    return true;
}

static bool append_text_field(char *buffer, size_t capacity, size_t *used,
                              const char *key, const char *value) {
    if (value == NULL) {
        return true;
    }
    return safe_field_text(value) &&
           append_pair(buffer, capacity, used, key, value);
}

static bool append_number_field(char *buffer, size_t capacity, size_t *used,
                               const char *key, const char *format, ...) {
    char value[32];
    va_list arguments;
    va_start(arguments, format);
    const int written = vsnprintf(value, sizeof(value), format, arguments);
    va_end(arguments);
    if (written < 0 || (size_t)written >= sizeof(value)) {
        return false;
    }
    return append_pair(buffer, capacity, used, key, value);
}

static bool optional_fields_are_safe(const char *fields) {
    if (fields == NULL) {
        return true;
    }
    for (const char *character = fields; *character != '\0'; ++character) {
        if ((unsigned char)*character < 32u ||
            (unsigned char)*character > 126u ||
            *character == '\r' || *character == '\n') {
            return false;
        }
    }
    return true;
}

static bool send_packet(const char *line, size_t length) {
    struct pbuf *packet;
    err_t result = ERR_MEM;

    /* The background CYW43 handler can enter lwIP concurrently with this loop. */
    cyw43_arch_lwip_begin();
    packet = pbuf_alloc(PBUF_TRANSPORT, (u16_t)length, PBUF_RAM);
    if (packet != NULL) {
        memcpy(packet->payload, line, length);
        result = udp_sendto(telemetry_state.udp, packet,
                            &telemetry_state.destination,
                            TELEMETRY_DESTINATION_PORT);
        pbuf_free(packet);
    }
    cyw43_arch_lwip_end();

    if (packet == NULL) {
        record_error("packet allocation failed");
        return false;
    }
    if (result != ERR_OK) {
        char error_message[48];
        snprintf(error_message, sizeof(error_message),
                 "UDP send failed with %d", result);
        record_error(error_message);
        return false;
    }
    return true;
}

bool telemetry_init(const char *device_id) {
    if (!safe_field_text(device_id) ||
        strlen(device_id) >= sizeof(telemetry_state.device_id)) {
        printf("Telemetry initialisation failed: invalid device identifier.\n");
        return false;
    }

    strncpy(telemetry_state.device_id, device_id,
            sizeof(telemetry_state.device_id));
    telemetry_state.session_id = make_session_id();
    telemetry_state.sequence = 0;
    telemetry_state.errors = 0;

    printf("Initialising Wi-Fi station mode...\n");
    if (cyw43_arch_init()) {
        printf("Wi-Fi initialisation failed.\n");
        return false;
    }
    telemetry_state.cyw43_started = true;

    cyw43_arch_enable_sta_mode();
    printf("Starting connection to the configured 2.4 GHz network...\n");
    const int start_result = cyw43_arch_wifi_connect_async(
        WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK);
    if (start_result != 0) {
        printf("Wi-Fi connection start failed: %d\n", start_result);
        cyw43_arch_deinit();
        telemetry_state.cyw43_started = false;
        return false;
    }

    const uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    int previous_status = -100;
    while (true) {
        const int status = current_link_status();
        if (status != previous_status) {
            printf("Wi-Fi progress: %d (%s)\n",
                   status, link_status_name(status));
            previous_status = status;
        }

        if (status == CYW43_LINK_UP) {
            print_network_details(status);
            break;
        }
        if (status < 0) {
            printf("Wi-Fi connection failed with status %d (%s).\n",
                   status, link_status_name(status));
            cyw43_arch_deinit();
            telemetry_state.cyw43_started = false;
            return false;
        }

        const uint32_t elapsed_ms =
            to_ms_since_boot(get_absolute_time()) - start_ms;
        if (elapsed_ms >= WIFI_CONNECT_TIMEOUT_MS) {
            printf("Wi-Fi connection timed out after %u ms.\n",
                   WIFI_CONNECT_TIMEOUT_MS);
            print_network_details(status);
            cyw43_arch_deinit();
            telemetry_state.cyw43_started = false;
            return false;
        }
        sleep_ms(250);
    }

    bool valid_destination;
    cyw43_arch_lwip_begin();
    valid_destination = ipaddr_aton(TELEMETRY_DESTINATION_IP,
                                    &telemetry_state.destination);
    cyw43_arch_lwip_end();
    if (!valid_destination) {
        printf("Invalid telemetry destination address.\n");
        cyw43_arch_deinit();
        telemetry_state.cyw43_started = false;
        return false;
    }

    cyw43_arch_lwip_begin();
    telemetry_state.udp = udp_new();
    cyw43_arch_lwip_end();
    if (telemetry_state.udp == NULL) {
        printf("UDP control block allocation failed; stopping telemetry.\n");
        cyw43_arch_deinit();
        telemetry_state.cyw43_started = false;
        return false;
    }

    printf("Telemetry session: %lu\n",
           (unsigned long)telemetry_state.session_id);
    printf("Telemetry destination: %s:%u\n",
           TELEMETRY_DESTINATION_IP, TELEMETRY_DESTINATION_PORT);
    return true;
}

bool telemetry_send_line(const char *optional_fields) {
    if (!telemetry_state.cyw43_started || telemetry_state.udp == NULL) {
        record_error("telemetry is not initialised");
        return false;
    }
    if (!optional_fields_are_safe(optional_fields)) {
        record_error("optional fields contain unsafe text");
        return false;
    }

    const int link_status = current_link_status();
    const uint32_t sequence = telemetry_state.sequence++;
    const uint32_t timestamp_ms =
        to_ms_since_boot(get_absolute_time());
    const char *separator = optional_fields != NULL &&
                                    optional_fields[0] != '\0' ? "," : "";
    char line[TELEMETRY_LINE_SIZE];
    const int line_length = snprintf(
        line, sizeof(line),
        "ver=%u,device=%s,session=%lu,seq=%lu,ms=%lu,link=%d%s%s\n",
        TELEMETRY_PROTOCOL_VERSION, telemetry_state.device_id,
        (unsigned long)telemetry_state.session_id, (unsigned long)sequence,
        (unsigned long)timestamp_ms, link_status, separator,
        optional_fields == NULL ? "" : optional_fields);

    if (line_length < 0 || line_length >= (int)sizeof(line)) {
        record_error("telemetry line formatting failed");
        return false;
    }

    const bool sent = send_packet(line, (size_t)line_length);
#if TELEMETRY_VERBOSE
    if (sent) {
        printf("Sent telemetry seq=%lu link=%d.\n",
               (unsigned long)sequence, link_status);
    }
#endif
    return sent;
}

bool telemetry_is_connected(void) {
    return telemetry_state.cyw43_started &&
           current_link_status() == CYW43_LINK_UP;
}

uint32_t telemetry_error_count(void) {
    return telemetry_state.errors;
}

uint32_t telemetry_session_id(void) {
    return telemetry_state.session_id;
}

bool telemetry_send_motor_line(const telemetry_motor_sample_t *sample) {
    if (sample == NULL) {
        record_error("motor telemetry sample is NULL");
        return false;
    }

    char fields[TELEMETRY_LINE_SIZE];
    size_t used = 0;
    bool valid = true;
#define ADD_TEXT(key, member) \
    valid = valid && append_text_field(fields, sizeof(fields), &used, \
                                       key, sample->member)
#define ADD_U32(key, member) \
    valid = valid && append_number_field(fields, sizeof(fields), &used, \
                                         key, "%lu", \
                                         (unsigned long)sample->member)
#define ADD_I32(key, member) \
    valid = valid && append_number_field(fields, sizeof(fields), &used, \
                                         key, "%ld", (long)sample->member)

    ADD_TEXT("test", test_type);
    ADD_TEXT("state", state);
    ADD_TEXT("direction", direction);
    ADD_TEXT("startup", startup_state);
    ADD_TEXT("stall", stall_state);
    ADD_TEXT("emergency", emergency_state);
    ADD_TEXT("error", error_state);
    if (sample->has_motor_number) {
        ADD_U32("motor", motor_number);
    }
    if (sample->has_duty_percent) {
        ADD_U32("duty", duty_percent);
    }
    if (sample->has_revolutions_requested) {
        ADD_U32("revolutions", revolutions_requested);
    }
    if (sample->has_revolution_index) {
        ADD_U32("revolution", revolution_index);
    }
    if (sample->has_absolute_count) {
        ADD_U32("absolute_count", absolute_count);
    }
    if (sample->has_elapsed_ms) {
        ADD_U32("elapsed_ms", elapsed_ms);
    }
    if (sample->has_ideal_counts) {
        ADD_U32("ideal_counts", ideal_counts);
    }
    if (sample->has_target_counts) {
        ADD_U32("target_counts", target_counts);
    }
    if (sample->has_count_at_stop) {
        ADD_I32("count_at_stop", count_at_stop);
    }
    if (sample->has_absolute_count_at_stop) {
        ADD_U32("absolute_count_at_stop", absolute_count_at_stop);
    }
    if (sample->has_count_at_trigger) {
        ADD_I32("count_at_trigger", count_at_trigger);
    }
    if (sample->has_absolute_count_at_trigger) {
        ADD_U32("absolute_count_at_trigger", absolute_count_at_trigger);
    }
    if (sample->has_count_after_settle) {
        ADD_I32("count_after_settle", count_after_settle);
    }
    if (sample->has_absolute_count_after_settle) {
        ADD_U32("absolute_count_after_settle", absolute_count_after_settle);
    }
    if (sample->has_coast_overshoot_counts) {
        ADD_I32("coast_overshoot", coast_overshoot_counts);
    }
    if (sample->has_boundary_count_reached) {
        ADD_U32("boundary_count", boundary_count_reached);
    }
    if (sample->has_undershoot_counts) {
        ADD_U32("undershoot", undershoot_counts);
    }
    if (sample->has_drift_counts) {
        ADD_I32("drift_counts", drift_counts);
    }
    if (sample->has_drift_millidegrees) {
        ADD_I32("drift_mdeg", drift_millidegrees);
    }
    if (sample->has_drift_per_revolution_millidegrees) {
        ADD_I32("drift_per_rev_mdeg", drift_per_revolution_millidegrees);
    }
    if (sample->has_elapsed_us) {
        ADD_U32("elapsed_us", elapsed_us);
    }
    if (sample->has_mean_us_per_revolution) {
        ADD_U32("mean_us_per_rev", mean_us_per_revolution);
    }
    if (sample->has_mean_ms_per_revolution) {
        ADD_U32("mean_ms_per_rev", mean_ms_per_revolution);
    }
    if (sample->has_counts_per_rev_milli) {
        ADD_U32("counts_per_rev_milli", counts_per_rev_milli);
    }
    if (sample->has_sensitivity_millicounts) {
        ADD_U32("sensitivity_milli_counts", sensitivity_millicounts);
    }
    if (sample->has_jog_counts) {
        ADD_I32("jog_counts", jog_counts);
    }
    if (sample->has_jog_final_absolute_count) {
        ADD_U32("jog_final_absolute_count", jog_final_absolute_count);
    }
    if (sample->has_jog_combined_count) {
        ADD_U32("jog_combined_count", jog_combined_count);
    }
    if (sample->has_smallest_jog_movement) {
        ADD_U32("smallest_jog_movement", smallest_jog_movement);
    }
    if (sample->has_alignment_resolution_cpr_milli) {
        ADD_U32("alignment_resolution_cpr_milli", alignment_resolution_cpr_milli);
    }
    if (sample->has_jog_primary_cpr_milli) {
        ADD_U32("jog_primary_cpr_milli", jog_primary_cpr_milli);
    }
    if (sample->has_jog_secondary_cpr_milli) {
        ADD_U32("jog_secondary_cpr_milli", jog_secondary_cpr_milli);
    }
    if (sample->has_split_index) {
        ADD_U32("split_index", split_index);
    }
    if (sample->has_split_total) {
        ADD_U32("split_total", split_total);
    }
    if (sample->has_split_revolution) {
        ADD_U32("split_revolution", split_revolution);
    }
    if (sample->has_split_absolute_count) {
        ADD_U32("split_absolute_count", split_absolute_count);
    }
    if (sample->has_split_elapsed_us) {
        ADD_U32("split_elapsed_us", split_elapsed_us);
    }
    if (sample->has_split_delta_counts) {
        ADD_U32("split_delta_counts", split_delta_counts);
    }
    if (sample->has_split_delta_us) {
        ADD_U32("split_delta_us", split_delta_us);
    }
    if (sample->has_completed) {
        valid = valid && append_number_field(fields, sizeof(fields), &used,
                                             "completed", "%u",
                                             sample->completed ? 1u : 0u);
    }
#undef ADD_TEXT
#undef ADD_U32
#undef ADD_I32

    if (!valid) {
        record_error("motor telemetry field formatting failed");
        return false;
    }

    fields[used] = '\0';
    return telemetry_send_line(fields);
}

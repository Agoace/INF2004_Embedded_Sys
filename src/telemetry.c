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
#define TELEMETRY_SCHEMA_VERSION 2
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

typedef struct {
    char *buffer;
    size_t capacity;
    size_t used;
    bool valid;
} json_writer_t;

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

static bool safe_identifier(const char *text) {
    if (text == NULL || text[0] == '\0') {
        return false;
    }
    for (const char *character = text; *character != '\0'; ++character) {
        const unsigned char value = (unsigned char)*character;
        const bool alpha = (value >= 'A' && value <= 'Z') ||
                           (value >= 'a' && value <= 'z');
        const bool digit = value >= '0' && value <= '9';
        if (!alpha && !digit && value != '-' && value != '_') {
            return false;
        }
    }
    return true;
}

static void json_writer_init(json_writer_t *writer, char *buffer,
                             size_t capacity) {
    writer->buffer = buffer;
    writer->capacity = capacity;
    writer->used = 0;
    writer->valid = true;
}

static bool json_append(json_writer_t *writer, const char *format, ...) {
    if (!writer->valid || writer->used >= writer->capacity) {
        writer->valid = false;
        return false;
    }

    va_list arguments;
    va_start(arguments, format);
    const int written = vsnprintf(writer->buffer + writer->used,
                                  writer->capacity - writer->used,
                                  format, arguments);
    va_end(arguments);
    if (written < 0 || (size_t)written >= writer->capacity - writer->used) {
        writer->valid = false;
        return false;
    }
    writer->used += (size_t)written;
    return true;
}

static bool json_prefix(json_writer_t *writer, bool *first) {
    if (!*first) {
        return json_append(writer, ",");
    }
    *first = false;
    return true;
}

static bool json_add_string(json_writer_t *writer, bool *first,
                            const char *key, const char *value) {
    if (!safe_identifier(value) || !json_prefix(writer, first)) {
        return false;
    }
    return json_append(writer, "\"%s\":\"%s\"", key, value);
}

static bool json_add_optional_string(json_writer_t *writer, bool *first,
                                     const char *key, const char *value) {
    if (value == NULL) {
        return true;
    }
    return json_add_string(writer, first, key, value);
}

static bool json_add_u32(json_writer_t *writer, bool *first,
                         const char *key, uint32_t value) {
    if (!json_prefix(writer, first)) {
        return false;
    }
    return json_append(writer, "\"%s\":%lu", key, (unsigned long)value);
}

static bool json_add_i32(json_writer_t *writer, bool *first,
                         const char *key, int32_t value) {
    if (!json_prefix(writer, first)) {
        return false;
    }
    return json_append(writer, "\"%s\":%ld", key, (long)value);
}

static bool telemetry_ready(void) {
    return telemetry_state.cyw43_started && telemetry_state.udp != NULL;
}

static bool begin_packet(json_writer_t *writer, bool *object_first,
                         const char *test_type, const char *object_name) {
    if (!telemetry_ready()) {
        record_error("telemetry is not initialised");
        return false;
    }
    if (!safe_identifier(test_type) || !safe_identifier(object_name)) {
        record_error("invalid telemetry identifier");
        return false;
    }

    const int link_status = current_link_status();
    const uint32_t sequence = telemetry_state.sequence++;
    const uint32_t timestamp_ms =
        to_ms_since_boot(get_absolute_time());
    bool envelope_first = true;
    char *buffer = writer->buffer;
    json_writer_init(writer, buffer, TELEMETRY_MAX_PACKET_BYTES);
    if (!json_append(writer, "{")) {
        return false;
    }
    if (!json_add_u32(writer, &envelope_first, "ver",
                      TELEMETRY_SCHEMA_VERSION) ||
        !json_add_string(writer, &envelope_first, "device",
                         telemetry_state.device_id) ||
        !json_add_u32(writer, &envelope_first, "session",
                      telemetry_state.session_id) ||
        !json_add_u32(writer, &envelope_first, "seq", sequence) ||
        !json_add_u32(writer, &envelope_first, "ms", timestamp_ms) ||
        !json_add_i32(writer, &envelope_first, "link", link_status) ||
        !json_add_string(writer, &envelope_first, "test", test_type) ||
        !json_prefix(writer, &envelope_first) ||
        !json_append(writer, "\"%s\":{", object_name)) {
        return false;
    }
    *object_first = true;
    return true;
}

static bool finish_packet(json_writer_t *writer) {
    if (!json_append(writer, "}}\n")) {
        record_error("telemetry JSON exceeds the packet limit");
        return false;
    }
    if (!writer->valid || writer->used > TELEMETRY_MAX_PACKET_BYTES) {
        record_error("telemetry JSON exceeds the packet limit");
        return false;
    }

    struct pbuf *packet;
    err_t result = ERR_MEM;
    /* The background CYW43 handler can enter lwIP concurrently with this loop. */
    cyw43_arch_lwip_begin();
    packet = pbuf_alloc(PBUF_TRANSPORT, (u16_t)writer->used, PBUF_RAM);
    if (packet != NULL) {
        memcpy(packet->payload, writer->buffer, writer->used);
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
        const int written = snprintf(error_message, sizeof(error_message),
                                     "UDP send failed with %d", result);
        if (written < 0 || (size_t)written >= sizeof(error_message)) {
            record_error("UDP send failed");
        } else {
            record_error(error_message);
        }
        return false;
    }
    return true;
}

bool telemetry_init(const char *device_id) {
    if (!safe_identifier(device_id) ||
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

static bool send_finished_packet(json_writer_t *writer, bool valid,
                                 const char *error_message) {
    if (!valid) {
        record_error(error_message);
        return false;
    }
    const bool sent = finish_packet(writer);
#if TELEMETRY_VERBOSE
    if (sent) {
        printf("Sent telemetry seq=%lu link=%d.\n",
               (unsigned long)(telemetry_state.sequence - 1u),
               current_link_status());
    }
#endif
    return sent;
}

bool telemetry_send_readings(const char *test,
                             const telemetry_readings_t *readings) {
    if (readings == NULL) {
        record_error("telemetry readings are NULL");
        return false;
    }

    char line[TELEMETRY_MAX_PACKET_BYTES];
    json_writer_t writer;
    bool object_first;
    json_writer_init(&writer, line, sizeof(line));
    bool valid = begin_packet(&writer, &object_first, test, "readings");
#define ADD_TEXT(key, member) \
    valid = valid && json_add_optional_string(&writer, &object_first, \
                                              key, readings->member)
#define ADD_U32(flag, key, member) \
    if (readings->flag) { \
        valid = valid && json_add_u32(&writer, &object_first, \
                                      key, readings->member); \
    }

    valid = valid && json_add_string(&writer, &object_first,
                                     "state", readings->state);
    ADD_U32(has_motor_1_speed_milli_rpm, "motor_1_speed_mrpm",
            motor_1_speed_milli_rpm);
    ADD_U32(has_motor_2_speed_milli_rpm, "motor_2_speed_mrpm",
            motor_2_speed_milli_rpm);
    ADD_U32(has_motor_1_duty_percent, "motor_1_duty_pct",
            motor_1_duty_percent);
    ADD_U32(has_motor_2_duty_percent, "motor_2_duty_pct",
            motor_2_duty_percent);
    ADD_TEXT("motor_1_direction", motor_1_direction);
    ADD_TEXT("motor_2_direction", motor_2_direction);
    ADD_U32(has_range_mm, "range_mm", range_mm);
    ADD_U32(has_ir_mv, "ir_mv", ir_mv);
    ADD_U32(has_supply_mv, "supply_mv", supply_mv);
    ADD_U32(has_progress_percent, "progress_pct", progress_percent);
#undef ADD_TEXT
#undef ADD_U32

    return send_finished_packet(&writer, valid,
                                "readings JSON formatting failed");
}

bool telemetry_send_terminal_notice(const char *test, const char *outcome,
                                    bool fault) {
    char line[TELEMETRY_MAX_PACKET_BYTES];
    json_writer_t writer;
    bool object_first;
    json_writer_init(&writer, line, sizeof(line));
    bool valid = begin_packet(&writer, &object_first, test, "notice");
    valid = valid && json_add_string(&writer, &object_first,
                                     "outcome", outcome);
    valid = valid && json_add_u32(&writer, &object_first, "fault",
                                  fault ? 1u : 0u);
    return send_finished_packet(&writer, valid,
                                "terminal notice JSON formatting failed");
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

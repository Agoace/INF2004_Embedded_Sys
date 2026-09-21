#include <stdbool.h>
#include <stdint.h>
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

/* These are the Raspberry Pi 400 LAN endpoint, not localhost or its tailnet address. */
#define TELEMETRY_DESTINATION_IP "192.168.18.98"
#define TELEMETRY_DESTINATION_PORT 4242

#define TELEMETRY_PROTOCOL_VERSION 1
#define TELEMETRY_DEVICE_ID "pico-w-telemetry-probe"
#define WIFI_CONNECT_TIMEOUT_MS 30000
#define TELEMETRY_INTERVAL_MS 1000
#define TELEMETRY_BUFFER_SIZE 256
#define TELEMETRY_VERBOSE 1 /* Set to 0 inside timed motor tests: stdout can block. */

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

static int current_link_status(void) {
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

static bool connect_to_wifi(void) {
    printf("Initialising Wi-Fi station mode...\n");
    if (cyw43_arch_init()) {
        printf("Wi-Fi initialisation failed.\n");
        return false;
    }

    cyw43_arch_enable_sta_mode();
    printf("Starting connection to the configured 2.4 GHz network...\n");
    const int start_result = cyw43_arch_wifi_connect_async(
        WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK);
    if (start_result != 0) {
        printf("Wi-Fi connection start failed: %d\n", start_result);
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
            return true;
        }
        if (status < 0) {
            printf("Wi-Fi connection failed with status %d (%s).\n",
                   status, link_status_name(status));
            return false;
        }

        const uint32_t elapsed_ms =
            to_ms_since_boot(get_absolute_time()) - start_ms;
        if (elapsed_ms >= WIFI_CONNECT_TIMEOUT_MS) {
            printf("Wi-Fi connection timed out after %u ms.\n",
                   WIFI_CONNECT_TIMEOUT_MS);
            print_network_details(status);
            return false;
        }
        sleep_ms(250);
    }
}

static bool send_telemetry(struct udp_pcb *udp,
                           const ip_addr_t *destination,
                           uint32_t session_id,
                           uint32_t sequence,
                           uint32_t *send_errors) {
    const uint32_t timestamp_ms =
        to_ms_since_boot(get_absolute_time());
    const int link_status = current_link_status();
    char payload[TELEMETRY_BUFFER_SIZE];
    const int payload_length = snprintf(
        payload, sizeof(payload),
        "ver=%u,device=%s,session=%lu,seq=%lu,ms=%lu,link=%d\n",
        TELEMETRY_PROTOCOL_VERSION, TELEMETRY_DEVICE_ID,
        (unsigned long)session_id, (unsigned long)sequence,
        (unsigned long)timestamp_ms, link_status);

    if (payload_length < 0 || payload_length >= (int)sizeof(payload)) {
        ++*send_errors;
        printf("Telemetry error %lu: payload formatting failed.\n",
               (unsigned long)*send_errors);
        return false;
    }
    if (udp == NULL) {
        ++*send_errors;
        printf("Telemetry error %lu: UDP control block unavailable.\n",
               (unsigned long)*send_errors);
        return false;
    }

    struct pbuf *packet;
    err_t result = ERR_MEM;

    /* The background CYW43 handler can enter lwIP concurrently with this loop. */
    cyw43_arch_lwip_begin();
    packet = pbuf_alloc(PBUF_TRANSPORT, (u16_t)payload_length, PBUF_RAM);
    if (packet != NULL) {
        memcpy(packet->payload, payload, (size_t)payload_length);
        result = udp_sendto(udp, packet, destination,
                            TELEMETRY_DESTINATION_PORT);
        pbuf_free(packet);
    }
    cyw43_arch_lwip_end();

    if (packet == NULL) {
        ++*send_errors;
        printf("Telemetry error %lu: packet allocation failed.\n",
               (unsigned long)*send_errors);
        return false;
    }
    if (result != ERR_OK) {
        ++*send_errors;
        printf("Telemetry error %lu: UDP send failed with %d.\n",
               (unsigned long)*send_errors, result);
        return false;
    }

#if TELEMETRY_VERBOSE
    printf("Sent telemetry seq=%lu link=%d.\n",
           (unsigned long)sequence, link_status);
#endif
    return true;
}

int main(void) {
    stdio_init_all();
    const uint32_t session_id = make_session_id();
    sleep_ms(1000);
    printf("\nWi-Fi telemetry probe\n");
    printf("Destination: %s:%u\n",
           TELEMETRY_DESTINATION_IP, TELEMETRY_DESTINATION_PORT);

    if (!connect_to_wifi()) {
        printf("Telemetry probe stopped before UDP transmission.\n");
        cyw43_arch_deinit();
        return 1;
    }

    ip_addr_t destination;
    bool valid_destination;
    cyw43_arch_lwip_begin();
    valid_destination = ipaddr_aton(TELEMETRY_DESTINATION_IP, &destination);
    cyw43_arch_lwip_end();
    if (!valid_destination) {
        printf("Invalid telemetry destination address.\n");
        cyw43_arch_deinit();
        return 1;
    }

    struct udp_pcb *udp;
    cyw43_arch_lwip_begin();
    udp = udp_new();
    cyw43_arch_lwip_end();
    if (udp == NULL) {
        printf("UDP control block allocation failed; stopping probe.\n");
        cyw43_arch_deinit();
        return 1;
    }
    uint32_t send_errors = 0;
    uint32_t sequence = 0;
    printf("Session: %lu\n", (unsigned long)session_id);
    printf("Network failure must not stall the main loop; send errors are "
           "counted and transmission continues.\n");

    while (true) {
        send_telemetry(udp, &destination, session_id,
                       sequence, &send_errors);
        ++sequence;
        sleep_ms(TELEMETRY_INTERVAL_MS);
    }
}

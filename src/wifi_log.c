#include "wifi_log.h"

#include <stdio.h>
#include <string.h>

#include "steer_log.h"

#if __has_include("wifi_config.h")
#include "wifi_config.h"
#define WIFI_LOG_ENABLED 1
#else
#warning "src/wifi_config.h is missing: building without Wi-Fi logging"
#define WIFI_LOG_ENABLED 0
#endif

#if WIFI_LOG_ENABLED

#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "pico/cyw43_arch.h"
#include "pico/rand.h"
#include "pico/stdio/driver.h"
#include "pico/stdlib.h"

#define LINK_CHECK_MS 100u
#define REJOIN_MS 15000u   /* not up after this long: start joining again */
#define TEXT_BUFFER_BYTES 2048u
#define TEXT_PACKET_BYTES 1000u

static struct {
    bool started;
    bool up;
    bool mirror;
    struct udp_pcb *pcb;
    uint16_t boot_id;
    uint16_t seq;
    uint32_t dropped;
    uint32_t next_check_ms;
    uint32_t join_ms;
    int status; /* last CYW43_LINK_* seen */
    /* Receiver address from its HELLO packets; written in the lwIP
     * receive callback, read in the main loop, both under the lwIP lock. */
    bool have_receiver;
    bool receiver_reported;
    ip_addr_t receiver;
    char text[TEXT_BUFFER_BYTES];
    size_t text_used;
} w;

static void out_chars(const char *buf, int len) {
    if (!w.mirror || len <= 0) {
        return;
    }
    const size_t n = (size_t)len;
    if (w.text_used + n > sizeof w.text) {
        ++w.dropped;
        return;
    }
    memcpy(&w.text[w.text_used], buf, n);
    w.text_used += n;
}

static stdio_driver_t text_driver = {
    .out_chars = out_chars,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
    .crlf_enabled = false,
#endif
};

static const char *status_name(int status) {
    switch (status) {
        case CYW43_LINK_DOWN: return "down";
        case CYW43_LINK_JOIN: return "joining";
        case CYW43_LINK_NOIP: return "joined, waiting for an IP address";
        case CYW43_LINK_UP: return "up";
        case CYW43_LINK_FAIL: return "failed";
        case CYW43_LINK_NONET: return "network not found";
        case CYW43_LINK_BADAUTH: return "wrong password";
        default: return "unknown";
    }
}

/* lwIP receive callback (lwIP lock held). */
static void on_receive(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr,
                       u16_t port) {
    (void)arg;
    (void)pcb;
    (void)port;
    uint8_t first = 0u;
    if (p->tot_len >= SL_HEADER_BYTES && pbuf_copy_partial(p, &first, 1u, 0u) == 1u &&
        first == (uint8_t)(SL_TYPE_HELLO | (SL_VERSION << 4))) {
        ip_addr_copy(w.receiver, *addr);
        w.have_receiver = true;
    }
    pbuf_free(p);
}

static void join(uint32_t now_ms) {
    w.join_ms = now_ms;
    const int r =
        cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK);
    printf("[wifi] joining \"%s\" (start %d)\n", WIFI_SSID, r);
}

void wifi_log_init(void) {
    w.mirror = true;
    w.boot_id = (uint16_t)get_rand_32();
    if (cyw43_arch_init() != 0) {
        printf("[wifi] chip start failed: no Wi-Fi log\n");
        return;
    }
    cyw43_arch_enable_sta_mode();
    cyw43_arch_lwip_begin();
    w.pcb = udp_new();
    if (w.pcb != NULL) {
        (void)udp_bind(w.pcb, IP_ANY_TYPE, SL_PORT);
        udp_recv(w.pcb, on_receive, NULL);
    }
    cyw43_arch_lwip_end();
    if (w.pcb == NULL) {
        printf("[wifi] no UDP block: no Wi-Fi log\n");
        return;
    }
    w.started = true;
    stdio_set_driver_enabled(&text_driver, true);
    w.status = CYW43_LINK_DOWN;
    join(to_ms_since_boot(get_absolute_time()));
}

/* Send header + payload to the receiver, or to the subnet broadcast
 * address until a receiver has said HELLO. */
static bool send_packet(unsigned int type, const uint8_t *payload, size_t length) {
    if (!w.up) {
        ++w.dropped;
        return false;
    }
    const size_t total = SL_HEADER_BYTES + length;
    bool sent = false;
    cyw43_arch_lwip_begin();
    struct netif *nif = netif_default;
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, (u16_t)total, PBUF_RAM);
    if (p != NULL && nif != NULL) {
        uint8_t *out = (uint8_t *)p->payload;
        (void)sl_write_header(out, type, w.boot_id, w.seq);
        memcpy(&out[SL_HEADER_BYTES], payload, length);
        ip_addr_t to;
        if (w.have_receiver) {
            ip_addr_copy(to, w.receiver);
        } else {
            ip_addr_set_ip4_u32(&to, ip4_addr_get_u32(netif_ip4_addr(nif)) |
                                         ~ip4_addr_get_u32(netif_ip4_netmask(nif)));
        }
        sent = udp_sendto(w.pcb, p, &to, SL_PORT) == ERR_OK;
    }
    if (p != NULL) {
        pbuf_free(p);
    }
    cyw43_arch_lwip_end();
    ++w.seq; /* also on failure, so the receiver sees the loss */
    if (!sent) {
        ++w.dropped;
    }
    return sent;
}

void wifi_log_poll(uint32_t now_ms) {
    if (!w.started) {
        return;
    }
    if ((int32_t)(now_ms - w.next_check_ms) >= 0) {
        w.next_check_ms = now_ms + LINK_CHECK_MS;
        cyw43_arch_lwip_begin();
        const int status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
        cyw43_arch_lwip_end();
        if (status != w.status) {
            printf("[wifi] status %d: %s\n", status, status_name(status));
            w.status = status;
        }
        const bool up = status == CYW43_LINK_UP;
        if (up != w.up) {
            w.up = up;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, up);
            if (up) {
                char ip[16];
                cyw43_arch_lwip_begin();
                ip4addr_ntoa_r(netif_ip4_addr(netif_default), ip, sizeof ip);
                cyw43_arch_lwip_end();
                printf("[wifi] up, IP %s, broadcast to UDP port %u\n", ip, SL_PORT);
            } else {
                printf("[wifi] link lost (status %d)\n", status);
                w.join_ms = now_ms;
            }
        }
        if (w.have_receiver && !w.receiver_reported) {
            char ip[IPADDR_STRLEN_MAX];
            cyw43_arch_lwip_begin();
            ipaddr_ntoa_r(&w.receiver, ip, sizeof ip);
            cyw43_arch_lwip_end();
            printf("[wifi] receiver at %s: sending to it directly\n", ip);
            w.receiver_reported = true;
        }
        if (!up && (status < 0 || now_ms - w.join_ms >= REJOIN_MS)) {
            join(now_ms);
        }
    }
    if (w.up && w.text_used > 0u) {
        const size_t n = w.text_used < TEXT_PACKET_BYTES ? w.text_used : TEXT_PACKET_BYTES;
        (void)send_packet(SL_TYPE_TEXT, (const uint8_t *)w.text, n);
        memmove(w.text, &w.text[n], w.text_used - n);
        w.text_used -= n;
    }
}

bool wifi_log_up(void) {
    return w.up;
}

bool wifi_log_send(unsigned int type, const uint8_t *payload, size_t length) {
    if (!w.started) {
        return false;
    }
    return send_packet(type, payload, length);
}

void wifi_log_mirror_text(bool on) {
    w.mirror = on;
}

uint32_t wifi_log_dropped(void) {
    return w.dropped;
}

#else /* no Wi-Fi */

void wifi_log_init(void) {
    printf("[wifi] built without src/wifi_config.h: no Wi-Fi log\n");
}
void wifi_log_poll(uint32_t now_ms) {
    (void)now_ms;
}
bool wifi_log_up(void) {
    return false;
}
bool wifi_log_send(unsigned int type, const uint8_t *payload, size_t length) {
    (void)type;
    (void)payload;
    (void)length;
    return false;
}
void wifi_log_mirror_text(bool on) {
    (void)on;
}
uint32_t wifi_log_dropped(void) {
    return 0u;
}

#endif

#define PROHIBIT_DEF_SIZE_T
#include "pil_gate1_rxlog.h"
#include <tk/tkernel.h>
extern void tm_snd_dat(const UB *bytes, INT length);

static uint32_t *packet(pil_gate1_rxlog_t *d, uint32_t tag)
{
    return !d->sealed && tag < PIL_RXLOG_PRE + PIL_RXLOG_POST ?
           d->packet[tag] : NULL;
}
uint32_t pil_gate1_rxlog_tag(const pil_gate1_rxlog_t *d)
{
    if (d->sealed) { return PIL_RXLOG_NONE; }
    if (!d->faulted) { return d->pre_total & (PIL_RXLOG_PRE - 1u); }
    return d->post_count < PIL_RXLOG_POST ?
           PIL_RXLOG_PRE + d->post_count : PIL_RXLOG_NONE;
}
void pil_gate1_rxlog_rx(pil_gate1_rxlog_t *d, uint32_t stamp,
                        uint32_t length, uint32_t depth, uint32_t result)
{
    if (d->sealed) { return; }
    const uint32_t tag = pil_gate1_rxlog_tag(d);
    if (tag == PIL_RXLOG_NONE) { ++d->post_lost; return; }
    uint32_t *p = d->packet[tag];
    for (unsigned int i = 0u; i < 10u; ++i) { p[i] = PIL_RXLOG_NONE; }
    p[0] = d->serial++; p[1] = stamp;
    p[9] = (length & 255u) | ((depth & 15u) << 8u) |
           (15u << 12u) | (15u << 16u) | ((result & 15u) << 20u) |
           (d->faulted ? 1u << 24u : 0u);
    if (d->faulted) { ++d->post_count; } else { ++d->pre_total; }
}
void pil_gate1_rxlog_worker(pil_gate1_rxlog_t *d, uint32_t tag,
                            uint32_t stamp, uint32_t depth)
{
    uint32_t *p = packet(d, tag);
    if (p == NULL) { return; }
    p[2] = stamp; p[9] = (p[9] & ~(15u << 12u)) | ((depth & 15u) << 12u);
}
void pil_gate1_rxlog_decode(pil_gate1_rxlog_t *d, uint32_t tag,
                            uint32_t stamp, int result,
                            const pil_frame_t *frame)
{
    uint32_t *p = packet(d, tag);
    if (p == NULL) { return; }
    p[3] = stamp;
    p[8] = (p[8] & 0xffff00ffu) | (((uint32_t)result & 255u) << 8u);
    if (result == PIL_OK) {
        p[6] = frame->sequence; p[7] = frame->source_us;
        p[8] = (p[8] & 0xffffff00u) | frame->type;
    }
}
void pil_gate1_rxlog_stage(pil_gate1_rxlog_t *d, uint32_t tag,
                           uint32_t stamp, uint32_t reason, bool done,
                           uint32_t depth)
{
    uint32_t *p = packet(d, tag);
    if (p == NULL) { return; }
    p[done ? 5u : 4u] = stamp;
    p[8] = (p[8] & 65535u) | ((reason & 65535u) << 16u);
    if (d->faulted) { p[9] |= 2u << 24u; }
    if (done) {
        p[9] = (p[9] & ~(15u << 16u)) | ((depth & 15u) << 16u);
    }
}
void pil_gate1_rxlog_fault(void *context, uint32_t stamp, uint32_t reason)
{
    pil_gate1_rxlog_t *d = context;
    if (!d->faulted) {
        d->fault_us = stamp; d->reason = reason; d->faulted = true;
    }
}
uint32_t pil_gate1_rxlog_alarm(pil_gate1_rxlog_t *d, uint32_t demand,
                               uint32_t entry, uint32_t sequence,
                               uint32_t ready)
{
    if (d->faulted || d->sealed) { return PIL_RXLOG_NONE; }
    const uint32_t tag = d->alarm_total++ & (PIL_RXLOG_ALARMS - 1u);
    uint32_t *a = d->alarm[tag];
    a[0] = demand; a[1] = entry; a[2] = sequence; a[3] = ready;
    a[4] = a[5] = PIL_RXLOG_NONE;
    return tag;
}
void pil_gate1_rxlog_alarm_done(pil_gate1_rxlog_t *d, uint32_t tag,
                                uint32_t reason, uint32_t stamp)
{
    if (!d->sealed && tag < PIL_RXLOG_ALARMS) {
        d->alarm[tag][4] = reason; d->alarm[tag][5] = stamp;
    }
}
void pil_gate1_rxlog_seal(pil_gate1_rxlog_t *d) { d->sealed = true; }

/* Canonical LE words, standard CRC32C. Only used after the runtime freezes. */
static void chain(pil_gate1_rxlog_t *d, const uint32_t *w, unsigned int n)
{
    for (unsigned int i = 0u; i < n; ++i) {
        uint32_t word = w[i];
        for (unsigned int byte = 0u; byte < 4u; ++byte) {
            d->crc ^= word & 255u; word >>= 8u;
            for (unsigned int bit = 0u; bit < 8u; ++bit) {
                d->crc = (d->crc >> 1u) ^
                    ((d->crc & 1u) != 0u ? 0x82f63b78u : 0u);
            }
        }
    }
}
/* One atomic console append per row; never mix another task's text into it. */
static void row(const char *kind, const uint32_t *w, unsigned int n)
{
    static const char prefix[] = "[PILG1RX1 ";
    static const char hex[] = "0123456789abcdef";
    UB line[112]; unsigned int at = 0u;
    for (unsigned int i = 0u; prefix[i] != '\0'; ++i) {
        line[at++] = (UB)prefix[i];
    }
    for (unsigned int i = 0u; kind[i] != '\0'; ++i) {
        line[at++] = (UB)kind[i];
    }
    for (unsigned int i = 0u; i < n; ++i) {
        line[at++] = ' ';
        for (unsigned int digit = 0u; digit < 8u; ++digit) {
            line[at++] = (UB)hex[(w[i] >> (28u - digit * 4u)) & 15u];
        }
    }
    line[at++] = ']'; line[at++] = '\n';
    tm_snd_dat(line, (INT)at);
}
bool pil_gate1_rxlog_print(pil_gate1_rxlog_t *d, uint32_t nonce,
                           uint32_t session)
{
    if (!d->sealed) { return false; }
    const uint32_t pre = d->pre_total < PIL_RXLOG_PRE ?
                         d->pre_total : PIL_RXLOG_PRE;
    const uint32_t alarms = d->alarm_total < PIL_RXLOG_ALARMS ?
                            d->alarm_total : PIL_RXLOG_ALARMS;
    const uint32_t packets = pre + d->post_count;
    uint32_t index = d->printed;
    if (index == 0u) {
        const uint32_t h[8] = {nonce, session, d->fault_us, d->reason,
                               pre, d->post_count, alarms, d->post_lost};
        d->crc = UINT32_MAX; chain(d, h, 8u);
        row("BEGIN", h, 8u);
    } else if (--index < packets) {
        const uint32_t slot = index < pre ?
            (d->pre_total - pre + index) & (PIL_RXLOG_PRE - 1u) :
            PIL_RXLOG_PRE + index - pre;
        const uint32_t *p = d->packet[slot]; chain(d, p, 10u);
        row("P", p, 10u);
    } else if ((index -= packets) < alarms) {
        const uint32_t slot = (d->alarm_total - alarms + index) &
                               (PIL_RXLOG_ALARMS - 1u);
        const uint32_t *a = d->alarm[slot]; chain(d, a, 6u);
        row("A", a, 6u);
    } else {
        if (index == alarms) {
            const uint32_t end[3] = {packets, alarms, ~d->crc};
            row("END", end, 3u);
            ++d->printed;
        }
        return true;
    }
    ++d->printed; return false;
}

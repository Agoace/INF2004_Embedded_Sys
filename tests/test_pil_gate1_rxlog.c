#include "pil_gate1_rxlog.h"
#include <tk/tkernel.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
static char output[32768];
static size_t output_size;
void tm_snd_dat(const UB *bytes, INT length)
{
    assert(length > 0 && (size_t)length <= 112u);
    assert(bytes[0] == '[' && bytes[length - 1] == '\n');
    assert(output_size + (size_t)length < sizeof(output));
    memcpy(output + output_size, bytes, (size_t)length);
    output_size += (size_t)length;
}
static uint32_t receive(pil_gate1_rxlog_t *d, uint32_t n)
{
    const uint32_t tag = pil_gate1_rxlog_tag(d);
    const uint32_t stamp = d->faulted ? 1830000u + (n - 71u) * 100u :
                                      10000u + n * 100u;
    pil_gate1_rxlog_rx(d, stamp, 66u, 1u, 1u);
    return tag;
}
int main(int argc, char **argv)
{
    pil_gate1_rxlog_t d = {0};
    assert(sizeof(d) < 4096u);
    assert(!pil_gate1_rxlog_print(&d, 123u, 456u));
    for (uint32_t n = 0u; n < 70u; ++n) {
        const uint32_t tag = receive(&d, n);
        pil_frame_t f = {.type = PIL_SENSOR, .sequence = n,
                         .source_us = n * 1000u};
        pil_gate1_rxlog_worker(&d, tag, 10001u + n * 100u, 1u);
        pil_gate1_rxlog_decode(&d, tag, 10002u + n * 100u, PIL_OK, &f);
        pil_gate1_rxlog_stage(&d, tag, 10003u + n * 100u, 0u, false, 0u);
        pil_gate1_rxlog_stage(&d, tag, 10004u + n * 100u, 0u, true, 0u);
        const uint32_t a = pil_gate1_rxlog_alarm(&d, n * 1000u,
                                                n * 1000u + 1u, n, 3u);
        pil_gate1_rxlog_alarm_done(&d, a, 0u, n * 1000u + 2u);
    }
    /* Accepted RX exists before fault, but decode/stage complete afterward. */
    const uint32_t crossing = pil_gate1_rxlog_tag(&d);
    pil_gate1_rxlog_rx(&d, 1827000u, 66u, 3u, 1u);
    pil_gate1_rxlog_worker(&d, crossing, 1827135u, 3u);
    const uint32_t alarm = pil_gate1_rxlog_alarm(&d, 1827263u, 1827300u,
                                                657u, 1u);
    pil_gate1_rxlog_fault(&d, 1827305u, 0x100u);
    pil_gate1_rxlog_fault(&d, 1829999u, 7u);
    pil_gate1_rxlog_alarm_done(&d, alarm, 0x100u, 1827320u);
    assert(d.reason == 0x100u && d.fault_us == 1827305u);
    pil_frame_t f = {.type = PIL_SENSOR, .sequence = 658u,
                     .source_us = 658000u};
    pil_gate1_rxlog_decode(&d, crossing, 1827401u, PIL_OK, &f);
    pil_gate1_rxlog_stage(&d, crossing, 1827403u, 0x100u, false, 0u);
    pil_gate1_rxlog_stage(&d, crossing, 1827407u, 0x100u, true, 3u);
    assert(d.packet[crossing][6] == 658u);
    assert(d.packet[crossing][9] >> 24u == 2u);
    uint32_t history[64][10]; memcpy(history, d.packet, sizeof(history));
    for (uint32_t n = 0u; n < 22u; ++n) {
        const uint32_t tag = receive(&d, n + 71u);
        f.sequence = 659u + n; f.source_us = f.sequence * 1000u;
        pil_gate1_rxlog_worker(&d, tag, 1830001u + n * 100u, 1u);
        pil_gate1_rxlog_decode(&d, tag, 1830002u + n * 100u, PIL_OK, &f);
        pil_gate1_rxlog_stage(&d, tag, 1830003u + n * 100u,
                              0x100u, false, 0u);
        pil_gate1_rxlog_stage(&d, tag, 1830004u + n * 100u,
                              0x100u, true, 0u);
        assert(pil_gate1_rxlog_alarm(&d, 1u, 2u, 3u, 0u) == PIL_RXLOG_NONE);
    }
    assert(memcmp(history, d.packet, sizeof(history)) == 0);
    assert(d.pre_total == 71u && d.post_count == 16u && d.post_lost == 6u);
    assert(d.packet[64][9] >> 24u == 3u);
    pil_gate1_rxlog_seal(&d);
    uint32_t snapshot[80][10]; memcpy(snapshot, d.packet, sizeof(snapshot));
    pil_gate1_rxlog_worker(&d, crossing, 0u, 0u);
    pil_gate1_rxlog_decode(&d, crossing, 0u, PIL_OK, &f);
    pil_gate1_rxlog_stage(&d, crossing, 0u, 0u, true, 0u);
    pil_gate1_rxlog_rx(&d, 0u, 0u, 0u, 1u);
    assert(memcmp(snapshot, d.packet, sizeof(snapshot)) == 0);
    for (unsigned int i = 0u; i < 128u; ++i) {
        if (pil_gate1_rxlog_print(&d, 123u, 456u)) { break; }
    }
    assert(strstr(output, "[PILG1RX1 BEGIN ") == output);
    assert(strstr(output, "[PILG1RX1 END 00000050 00000010 ") != NULL);
    const size_t final = output_size;
    assert(pil_gate1_rxlog_print(&d, 123u, 456u) && output_size == final);
    /* Verify canonical order, ring wrap and CRC independent of formatter. */
    uint8_t canonical[32u + 80u * 40u + 16u * 24u];
    size_t used = 0u;
    const char *line = output;
    unsigned int pcount = 0u, acount = 0u;
    uint32_t footer_crc = 0u;
    while (line != NULL && *line != '\0') {
        const char *values = strchr(line, ' '); assert(values != NULL);
        values = strchr(values + 1, ' '); assert(values != NULL); ++values;
        unsigned int n = 0u;
        if (strncmp(line, "[PILG1RX1 BEGIN ", 15u) == 0) { n = 8u; }
        else if (strncmp(line, "[PILG1RX1 P ", 11u) == 0) {
            n = 10u; ++pcount;
        } else if (strncmp(line, "[PILG1RX1 A ", 11u) == 0) {
            n = 6u; ++acount;
        } else {
            unsigned int ps, as, crc;
            assert(sscanf(values, "%x %x %x", &ps, &as, &crc) == 3);
            assert(ps == pcount && as == acount); footer_crc = crc;
        }
        for (unsigned int i = 0u; i < n; ++i) {
            unsigned int word;
            assert(sscanf(values, "%8x", &word) == 1);
            if (n == 10u && i == 0u) { assert(word == pcount + 6u); }
            assert(used + 4u <= sizeof(canonical));
            pil_put_u32(canonical + used, word); used += 4u; values += 9;
        }
        line = strchr(line, '\n'); if (line != NULL) { ++line; }
    }
    assert(pcount == 80u && acount == 16u && used == sizeof(canonical));
    assert(pil_crc32c(canonical, used) == footer_crc);
    if (argc > 1) {
        FILE *out = fopen(argv[1], "wb"); assert(out != NULL);
        assert(fwrite(output, 1u, output_size, out) == output_size);
        assert(fclose(out) == 0);
    }
    return 0;
}

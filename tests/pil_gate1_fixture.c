/* SYNTHETIC host fixture, NOT Pico/Probe timing or a runtime session test. */
#include "pil_gate1_dump.h"
#include "pil_gate1_metrics.h"
#include "pil_wire.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define N 600001u
#define STAGED (N + 3u)
#define E UINT32_C(0xfffff000)

static pil_gate1_record_t record;
static uint32_t chain;

static void emit(unsigned int kind, const uint32_t *words, size_t n)
{
    record.kind = (uint16_t)kind;
    record.length = (uint32_t)(n * 4u);
    for (size_t i = 0u; i < n; ++i) {
        pil_put_u32(record.payload + i * 4u, words[i]);
    }
    uint8_t wire[PIL_GATE1_D1_WIRE];
    size_t length;
    assert(pil_gate1_dump_encode(&record, wire, sizeof(wire), &length));
    assert(fwrite(wire, 1u, length, stdout) == length);
    if (kind != PIL_GATE1_D1_MANIFEST) {
        assert(pil_gate1_dump_chain(&chain, &record));
    }
    ++record.sequence;
}

int main(int argc, char **argv)
{
    if (argc == 2) { assert(freopen(argv[1], "wb", stdout) != NULL); }
    record.nonce = UINT32_C(0x12345678);
    record.session = UINT32_C(0x9abcdef0);
    const uint32_t rx_ops = 2u * (STAGED + 11u);
    const uint32_t tx_ops = 2u * (N + 111u);
    const uint32_t context[26] = {PIL_SCHEMA_ID, 4u, E, E - 200000u,
        125000000u, 921600u, 8u, 31u, 1000u, 4000u, 5000u, 8u,
        0u, STAGED - 1u, 0u, N - 1u, 0u, N - 1u,
        E + 20u, E + N * 1000u + 5000u, 1u, 4u, 0x43fu, 0x79u, 12u, 0u};
    const uint32_t counts[26] = {STAGED, N, N, N, N, N, rx_ops, tx_ops,
        0u,0u,0u,0u,0u,0u,0u,0u,0u,0u,0u,0u,0u,0u,0u,3u,0u,0u};
    emit(1u, context, 26u);
    emit(2u, counts, 26u);
    for (unsigned int id = 1u; id <= 12u; ++id) {
        pil_gate1_metric_t m;
        assert(pil_gate1_metric_init(&m, id));
        const uint32_t samples = id == 1u || id == 3u || id == 7u ?
            STAGED : id == 9u ? rx_ops : id == 10u ? tx_ops : N;
        for (uint32_t i = 0u; i < samples; ++i) {
            uint32_t target = E + 1000u * i;
            if (id == 9u) {
                const uint32_t p = i / 2u;
                target = p < 10u ? E - 100000u + p * 1000u :
                    p < STAGED + 10u ? E + (p - 10u) * 1000u - 3000u :
                    E + (N - 1u) * 1000u + 700u;
            }
            if (id == 10u) {
                const uint32_t p = i / 2u;
                target = p < 110u ? E - 110000u + p * 1000u :
                    p < N + 110u ? E + (p - 110u) * 1000u + 50u :
                    E + (N - 1u) * 1000u + 800u;
            }
            uint32_t lo = target + 20u, hi = target + 24u;
            uint32_t ref = target - 4000u, seq = i, depth = 0u;
            if (id == 1u || id == 3u) {
                lo = ref + 1000u; hi = ref + 1002u;
                if (id == 3u) { ref = target - 1000u; }
            }
            if (id == 4u) { ref = target; }
            if (id == 5u) { lo = hi = target + 10u; ref = target; }
            if (id == 6u) { ref = target + 10u; }
            if (id >= 7u && id <= 10u) {
                lo = hi = target; ref = 0u;
                depth = id == 7u ? (i < 3u ? i + 1u : 4u) :
                        id == 8u ? 3u : 1u - i % 2u;
            }
            if (id >= 9u) { seq = PIL_NO_ACK; ref = 0u; }
            if (id >= 11u) { lo = target + 30u; hi = target + 35u; }
            assert(pil_gate1_metric_observe(&m, seq, lo, hi, ref, depth));
        }
        uint8_t bytes[48];
        uint32_t words[12];
        assert(pil_gate1_metric_payload(&m, bytes));
        for (unsigned int k = 0u; k < 12u; ++k) {
            words[k] = pil_get_u32(bytes + 4u * k);
        }
        emit(3u, words, 12u);
    }
    const uint32_t manifest[4] = {14u, 12u, 0u, chain};
    emit(5u, manifest, 4u);
    return 0;
}

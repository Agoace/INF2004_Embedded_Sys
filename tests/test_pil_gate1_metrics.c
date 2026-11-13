#include "pil_gate1_metrics.h"
#include "pil_wire.h"
#include <assert.h>
#include <limits.h>
#include <string.h>

static void oracle(unsigned int id)
{
    pil_gate1_metric_t m;
    assert(pil_gate1_metric_init(&m, id));
    assert(m.count == 0u && m.min_seq == PIL_NO_ACK);
    assert(m.max_seq == PIL_NO_ACK);
    int64_t minimum = INT64_MAX, maximum = INT64_MIN;
    uint32_t first_min = 0u, first_max = 0u;
    for (uint32_t i = 0u; i < 30000u; ++i) {
        const uint64_t ref = UINT64_C(0xfffff000) + (uint64_t)i * 1000u;
        const uint32_t jitter = (i * 7919u) % 501u;
        uint64_t lo = ref + jitter, hi = lo + i % 7u;
        uint32_t reference = (uint32_t)ref;
        uint32_t seq = i, depth = 0u;
        int64_t low = (int64_t)(lo - ref), high = (int64_t)(hi - ref);
        if (id == 3u) {
            low = -(int64_t)(hi - ref); high = -(int64_t)jitter;
        }
        if (id == 5u) { hi = lo; high = low; }
        if (id >= 7u && id <= 10u) {
            hi = lo;
            depth = i % 9u;
            low = high = depth;
            reference = 0u;
        }
        if (id >= 9u) { seq = PIL_NO_ACK; reference = 0u; }
        if (id >= 11u) { low = high = (int64_t)(hi - lo); }
        assert(pil_gate1_metric_observe(&m, seq, (uint32_t)lo,
                                       (uint32_t)hi, reference, depth));
        if (low < minimum) { minimum = low; first_min = i; }
        if (high > maximum) { maximum = high; first_max = i; }
        assert(m.minimum == minimum && m.maximum == maximum);
        assert(m.count == i + 1u);
        assert(m.min_seq == (id >= 9u ? PIL_NO_ACK : first_min));
        assert(m.max_seq == (id >= 9u ? PIL_NO_ACK : first_max));
    }
    uint8_t payload[48];
    assert(pil_gate1_metric_payload(&m, payload));
    assert(pil_get_u32(payload) == id);
    assert(pil_get_u32(payload + 4u) == 30000u);
    assert(pil_get_u32(payload + 8u) == (uint32_t)m.minimum);
    assert(pil_get_u32(payload + 28u) == (uint32_t)m.maximum);
}

int main(void)
{
    for (unsigned int id = 1u; id <= 12u; ++id) { oracle(id); }
    pil_gate1_metric_t m;
    memset(&m, 0xa5, sizeof(m));
    const pil_gate1_metric_t saved = m;
    assert(!pil_gate1_metric_init(&m, 0u));
    assert(!pil_gate1_metric_init(&m, 13u));
    assert(memcmp(&m, &saved, sizeof(m)) == 0);
    assert(!pil_gate1_metric_init(NULL, 1u));
    assert(pil_gate1_metric_init(&m, 1u));
    const pil_gate1_metric_t empty = m;
    uint8_t zero[48];
    assert(pil_gate1_metric_payload(&m, zero));
    for (unsigned int i = 0u; i < 12u; ++i) {
        const uint32_t expected = i == 0u ? 1u :
            i == 3u || i == 8u ? PIL_NO_ACK : 0u;
        assert(pil_get_u32(zero + i * 4u) == expected);
    }
    assert(!pil_gate1_metric_observe(&m, 0u, 2u, 1u, 0u, 0u));
    assert(!pil_gate1_metric_observe(&m, 0u, 0u, 0u,
                                   UINT32_C(0x80000000), 0u));
    assert(!pil_gate1_metric_observe(&m, PIL_NO_ACK, 1u, 1u, 0u, 0u));
    assert(memcmp(&m, &empty, sizeof(m)) == 0);
    m.count = UINT32_MAX;
    const pil_gate1_metric_t full = m;
    assert(!pil_gate1_metric_observe(&m, 0u, 1u, 1u, 0u, 0u));
    assert(memcmp(&m, &full, sizeof(m)) == 0);
    assert(pil_gate1_metric_init(&m, 9u));
    assert(!pil_gate1_metric_observe(&m, PIL_NO_ACK, 1u, 1u, 0u, 9u));
    assert(!pil_gate1_metric_observe(&m, 0u, 1u, 1u, 0u, 0u));
    assert(!pil_gate1_metric_observe(NULL, 0u, 0u, 0u, 0u, 0u));
    assert(!pil_gate1_metric_payload(NULL, NULL));
    assert(!pil_gate1_metric_payload(&m, NULL));
    return 0;
}

#include "pil_gate1_metrics.h"
#include "pil_wire.h"
#include <limits.h>

_Static_assert(sizeof(pil_gate1_metric_t) == 48u, "D1 metric RAM size");

static bool difference(uint32_t a, uint32_t b, int32_t *out)
{
    const uint32_t d = a - b;
    if (d == UINT32_C(0x80000000)) { return false; }
    *out = d <= INT32_MAX ? (int32_t)d :
           -(int32_t)(UINT32_MAX - d) - 1;
    return true;
}

bool pil_gate1_metric_init(pil_gate1_metric_t *m, unsigned int id)
{
    if (m == NULL || id == 0u || id > PIL_GATE1_METRIC_COUNT) {
        return false;
    }
    *m = (pil_gate1_metric_t){.id = id,
        .min_seq = PIL_NO_ACK, .max_seq = PIL_NO_ACK};
    return true;
}

bool pil_gate1_metric_observe(pil_gate1_metric_t *m, uint32_t seq,
                             uint32_t lo, uint32_t hi,
                             uint32_t ref, uint32_t depth)
{
    if (m == NULL || m->id == 0u || m->id > PIL_GATE1_METRIC_COUNT ||
        m->count == UINT32_MAX || hi - lo > INT32_MAX) {
        return false;
    }
    const bool queue = m->id >= 7u && m->id <= 10u;
    if ((m->id <= 8u && seq > PIL_MAX_SOURCE_US / 1000u) ||
        (m->id >= 9u && (seq != PIL_NO_ACK || ref != 0u)) ||
        (queue && (depth > 8u || lo != hi || ref != 0u)) ||
        (!queue && depth != 0u) || (m->id == 5u && lo != hi)) {
        return false;
    }
    int32_t low, high;
    if (queue) {
        low = high = (int32_t)depth;
    } else if (m->id >= 11u) {
        low = high = (int32_t)(hi - lo);
    } else if (m->id == 3u) {
        if (!difference(ref, hi, &low) || !difference(ref, lo, &high)) {
            return false;
        }
    } else if (!difference(lo, ref, &low) || !difference(hi, ref, &high)) {
        return false;
    }
    if (low > high) { return false; }
    if (m->count == 0u || low < m->minimum) {
        m->minimum = low;
        m->min_seq = seq; m->min_lo = lo; m->min_hi = hi; m->min_ref = ref;
    }
    if (m->count == 0u || high > m->maximum) {
        m->maximum = high;
        m->max_seq = seq; m->max_lo = lo; m->max_hi = hi; m->max_ref = ref;
    }
    ++m->count;
    return true;
}

bool pil_gate1_metric_payload(const pil_gate1_metric_t *m, uint8_t *out)
{
    if (m == NULL || out == NULL || m->id == 0u ||
        m->id > PIL_GATE1_METRIC_COUNT) { return false; }
    const uint32_t words[12] = {m->id, m->count, (uint32_t)m->minimum,
        m->min_seq, m->min_lo, m->min_hi, m->min_ref, (uint32_t)m->maximum,
        m->max_seq, m->max_lo, m->max_hi, m->max_ref};
    for (unsigned int i = 0u; i < 12u; ++i) {
        pil_put_u32(out + 4u * i, words[i]);
    }
    return true;
}

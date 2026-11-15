/* D1 all-event reductions. Raw TIMER endpoints; no quantization adjustment. */
#ifndef PIL_GATE1_METRICS_H
#define PIL_GATE1_METRICS_H
#include <stdbool.h>
#include <stdint.h>

#define PIL_GATE1_METRIC_COUNT 12u
#define PIL_GATE1_METRIC_BYTES 48u

enum pil_gate1_metric_id {
    PIL_G1_AVAILABLE = 1, PIL_G1_APPLIED = 2, PIL_G1_SLACK = 3,
    PIL_G1_APPLY_JITTER = 4, PIL_G1_NATIVE_JITTER = 5,
    PIL_G1_APPLY_DELAY = 6, PIL_G1_STAGE_DEPTH = 7,
    PIL_G1_APPLY_DEPTH = 8, PIL_G1_RX_DEPTH = 9,
    PIL_G1_TX_DEPTH = 10, PIL_G1_IRQ_COST = 11, PIL_G1_WORK_COST = 12
};

typedef struct {
    uint32_t id, count;
    int32_t minimum;
    uint32_t min_seq, min_lo, min_hi, min_ref;
    int32_t maximum;
    uint32_t max_seq, max_lo, max_hi, max_ref;
} pil_gate1_metric_t;

/* Caller owns/serializes storage; this helper has no locks or shared globals.
 * Failures preserve the destination, including overflow. First tie wins.
 * IDs1..8 require a source sequence. IDs9..12 require NO_ACK/reference0.
 * Queue IDs7..10 require equal event stamps, reference0 and depth0..8.
 * Call only for admitted events; revoked staging must not reach this API. */
bool pil_gate1_metric_init(pil_gate1_metric_t *metric, unsigned int id);
bool pil_gate1_metric_observe(pil_gate1_metric_t *metric, uint32_t seq,
                             uint32_t raw_lo, uint32_t raw_hi,
                             uint32_t reference, uint32_t depth);
bool pil_gate1_metric_payload(const pil_gate1_metric_t *metric, uint8_t *out);
#endif

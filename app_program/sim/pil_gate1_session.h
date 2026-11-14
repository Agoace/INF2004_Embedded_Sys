/* Neutral transport-only cap4 state. Caller excludes core0 IRQ access. */
#ifndef PIL_GATE1_SESSION_H
#define PIL_GATE1_SESSION_H
#include "pil_wire.h"
#include "pil_gate1_metrics.h"

enum pil_gate1_counter {
    G1_STAGED, G1_APPLIED, G1_DEMANDED, G1_OUTPUT, G1_ENQUEUED, G1_FIFO,
    G1_RX_MUT, G1_TX_MUT, G1_RX_FULL, G1_TX_FULL, G1_DROPPED, G1_BAD,
    G1_MISSED, G1_OVERRUN, G1_FRAMING, G1_PARITY, G1_BREAK, G1_GAP,
    G1_SLOT_FULL, G1_LATE, G1_EXPIRED, G1_TIMING, G1_REFUSED, G1_CANCELLED,
    G1_COUNT_OVERFLOW, G1_LEASE, G1_COUNTERS
};
typedef uint32_t (*pil_gate1_clock_t)(void *context);
typedef struct {
    uint32_t ready, sequence, alo, ahi;
    uint8_t payload[40];
} pil_gate1_slot_t;
typedef struct {
    pil_gate1_clock_t clock;
    void *clock_context;
    void (*fault_observer)(void *context, uint32_t stamp, uint32_t reason);
    void *fault_context;
    uint32_t nonce, session, epoch, grid_anchor, grid_next, lease;
    uint32_t flags, reason, coverage, next_stage, next_apply;
    uint32_t seen, received[9], transmitted[9], hello_ack;
    uint32_t run_start, stop_time, last_demand, write_time, servo, generation;
    uint32_t counts[G1_COUNTERS];
    pil_gate1_metric_t metrics[12];
    pil_gate1_slot_t slot[8];
    uint32_t trace[64][16], trace_total;
    bool grid_ready, armed_once, frozen;
} pil_gate1_session_t;

void pil_gate1_session_init(pil_gate1_session_t *s, uint32_t nonce,
                           pil_gate1_clock_t clock, void *context);
void pil_gate1_grid(pil_gate1_session_t *s, uint32_t next);
void pil_gate1_boot(pil_gate1_session_t *s, pil_frame_t *out);
bool pil_gate1_receive(pil_gate1_session_t *s, const pil_frame_t *frame,
                       uint32_t rx_time, pil_frame_t *reply);
bool pil_gate1_alarm(pil_gate1_session_t *s, uint32_t demand, uint32_t entry,
                     pil_frame_t *out);
void pil_gate1_check_lease(pil_gate1_session_t *s, uint32_t now);
void pil_gate1_fail(pil_gate1_session_t *s, uint32_t reason);
void pil_gate1_motor(pil_gate1_session_t *s, int16_t left, int16_t right,
                     uint16_t servo, uint32_t stamp);
void pil_gate1_count(pil_gate1_session_t *s, unsigned int counter);
void pil_gate1_note(pil_gate1_session_t *s, unsigned int metric,
                    uint32_t seq, uint32_t lo, uint32_t hi,
                    uint32_t reference, uint32_t depth);
void pil_gate1_output(pil_gate1_session_t *s, pil_frame_t *out);
void pil_gate1_trace_get(const pil_gate1_session_t *s, unsigned int index,
                         uint32_t out[16]);
#endif

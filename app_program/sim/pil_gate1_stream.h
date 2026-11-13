/* Core-0 IRQ/task SPSC packet handoff. Caller owns reset/IRQ exclusion. */
#ifndef PIL_GATE1_STREAM_H
#define PIL_GATE1_STREAM_H
#include "pil_wire.h"

typedef struct {
    uint32_t stamp, errors, tag, length;
    uint8_t bytes[PIL_MAX_PACKET];
} pil_gate1_packet_t;
typedef struct {
    pil_gate1_packet_t slot[8];
    volatile uint32_t head, tail;
} pil_gate1_queue_t;
typedef struct {
    pil_gate1_queue_t queue;
    pil_gate1_packet_t partial;
    bool discarding;
} pil_gate1_rx_t;
enum pil_gate1_rx_result {
    PIL_G1_RX_NONE, PIL_G1_RX_PACKET, PIL_G1_RX_FULL, PIL_G1_RX_OVERSIZE
};
/* Front remains owned by consumer until drop. Exactly one producer/consumer.
 * All live access is core0; do not reset while IRQ can access this storage. */
uint32_t pil_gate1_queue_depth(const pil_gate1_queue_t *queue);
bool pil_gate1_queue_put(pil_gate1_queue_t *q, const pil_gate1_packet_t *p);
const pil_gate1_packet_t *pil_gate1_queue_front(const pil_gate1_queue_t *q);
bool pil_gate1_queue_drop(pil_gate1_queue_t *q);
void pil_gate1_rx_init(pil_gate1_rx_t *rx);
int pil_gate1_rx_byte(pil_gate1_rx_t *rx, uint8_t byte, uint32_t stamp,
                      uint32_t errors);
#endif

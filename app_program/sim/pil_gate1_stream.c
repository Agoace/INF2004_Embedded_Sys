#include "pil_gate1_stream.h"
#include <string.h>

static void fence(void)
{
#if defined(__arm__) || defined(__thumb__)
    __asm__ volatile("dmb" ::: "memory");
#else
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
#endif
}

uint32_t pil_gate1_queue_depth(const pil_gate1_queue_t *q)
{
    return q->head - q->tail;
}

bool pil_gate1_queue_put(pil_gate1_queue_t *q, const pil_gate1_packet_t *p)
{
    if (p == NULL || p->length == 0u || p->length > PIL_MAX_PACKET ||
        pil_gate1_queue_depth(q) >= 8u) { return false; }
    q->slot[q->head & 7u] = *p;
    fence();
    ++q->head;
    return true;
}

const pil_gate1_packet_t *pil_gate1_queue_front(const pil_gate1_queue_t *q)
{
    if (q->tail == q->head) { return NULL; }
    fence();
    return &q->slot[q->tail & 7u];
}

bool pil_gate1_queue_drop(pil_gate1_queue_t *q)
{
    if (q->tail == q->head) { return false; }
    fence();
    ++q->tail;
    return true;
}

void pil_gate1_rx_init(pil_gate1_rx_t *rx)
{
    memset(rx, 0, sizeof(*rx));
}

int pil_gate1_rx_byte(pil_gate1_rx_t *rx, uint8_t byte, uint32_t stamp,
                      uint32_t errors)
{
    rx->partial.errors |= errors;
    if (byte == 0u) {
        int result = PIL_G1_RX_NONE;
        if (!rx->discarding && rx->partial.length != 0u) {
            rx->partial.bytes[rx->partial.length++] = 0u;
            rx->partial.stamp = stamp;
            result = pil_gate1_queue_put(&rx->queue, &rx->partial) ?
                     PIL_G1_RX_PACKET : PIL_G1_RX_FULL;
        }
        rx->partial.length = 0u;
        rx->partial.errors = 0u;
        rx->discarding = false;
        return result;
    }
    if (rx->discarding) { return PIL_G1_RX_NONE; }
    if (rx->partial.length == PIL_MAX_PACKET - 1u) {
        rx->partial.length = 0u;
        rx->discarding = true;
        return PIL_G1_RX_OVERSIZE;
    }
    rx->partial.bytes[rx->partial.length++] = byte;
    return PIL_G1_RX_NONE;
}

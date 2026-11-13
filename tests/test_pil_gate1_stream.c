#include "pil_gate1_stream.h"
#include <assert.h>
#include <string.h>

int main(void)
{
    pil_gate1_rx_t rx;
    pil_gate1_rx_init(&rx);
    pil_gate1_packet_t p = {.length = 2u, .stamp = 77u, .tag = 9u};
    p.bytes[0] = 1u; p.bytes[1] = 0u;
    pil_gate1_queue_t q = {0};
    q.head = q.tail = UINT32_MAX - 3u;
    for (unsigned int i = 0u; i < 8u; ++i) {
        p.tag = i;
        assert(pil_gate1_queue_put(&q, &p));
    }
    assert(pil_gate1_queue_depth(&q) == 8u);
    assert(!pil_gate1_queue_put(&q, &p));
    for (unsigned int i = 0u; i < 8u; ++i) {
        const pil_gate1_packet_t *v = pil_gate1_queue_front(&q);
        assert(v != NULL && v->tag == i && v->stamp == 77u);
        assert(memcmp(v->bytes, p.bytes, 2u) == 0);
        assert(pil_gate1_queue_drop(&q));
    }
    assert(!pil_gate1_queue_drop(&q));
    assert(pil_gate1_queue_front(&q) == NULL);
    assert(pil_gate1_rx_byte(&rx, 0u, 1u, 0u) == PIL_G1_RX_NONE);
    for (unsigned int i = 0u; i < 8u; ++i) {
        assert(pil_gate1_rx_byte(&rx, 1u, 2u, 4u) == PIL_G1_RX_NONE);
        assert(pil_gate1_rx_byte(&rx, 0u, 3u, 0u) == PIL_G1_RX_PACKET);
    }
    (void)pil_gate1_rx_byte(&rx, 1u, 4u, 0u);
    assert(pil_gate1_rx_byte(&rx, 0u, 5u, 0u) == PIL_G1_RX_FULL);
    const pil_gate1_packet_t *v = pil_gate1_queue_front(&rx.queue);
    assert(v->stamp == 3u && v->errors == 4u && v->length == 2u);
    pil_gate1_rx_init(&rx);
    for (unsigned int i = 0u; i < PIL_MAX_PACKET - 1u; ++i) {
        assert(pil_gate1_rx_byte(&rx, 1u, i, 0u) == PIL_G1_RX_NONE);
    }
    assert(pil_gate1_rx_byte(&rx, 1u, 1u, 0u) == PIL_G1_RX_OVERSIZE);
    assert(pil_gate1_rx_byte(&rx, 2u, 2u, 0u) == PIL_G1_RX_NONE);
    assert(pil_gate1_rx_byte(&rx, 0u, 3u, 0u) == PIL_G1_RX_NONE);
    (void)pil_gate1_rx_byte(&rx, 1u, 4u, 0u);
    assert(pil_gate1_rx_byte(&rx, 0u, 5u, 0u) == PIL_G1_RX_PACKET);
    v = pil_gate1_queue_front(&rx.queue);
    assert(v->stamp == 5u && v->errors == 0u && v->length == 2u);
    for (unsigned int run = 0u; run < 100000u; ++run) {
        assert(pil_gate1_queue_drop(&rx.queue));
        (void)pil_gate1_rx_byte(&rx, 3u, run, 0u);
        (void)pil_gate1_rx_byte(&rx, 1u, run, 0u);
        (void)pil_gate1_rx_byte(&rx, 2u, run, 0u);
        assert(pil_gate1_rx_byte(&rx, 0u, run, 0u) == PIL_G1_RX_PACKET);
        assert(pil_gate1_queue_front(&rx.queue)->length == 4u);
    }
    return 0;
}

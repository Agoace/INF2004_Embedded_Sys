#include "pil_gate1_session.h"
#include <assert.h>
#include <string.h>

static uint32_t now;
static uint32_t clock_read(void *unused)
{
    (void)unused;
    return now++;
}
static pil_frame_t sensor(uint32_t seq, uint32_t session)
{
    pil_frame_t f = {.type = PIL_SENSOR, .payload_length = 40u,
        .session = session, .sequence = seq, .source_us = seq * 1000u,
        .ack = PIL_NO_ACK};
    f.payload[7] = 1u;
    pil_put_u16(f.payload + 36u, 0xc18u);
    return f;
}
static void negotiate(pil_gate1_session_t *s, uint32_t start)
{
    now = start;
    pil_gate1_session_init(s, 123u, clock_read, NULL);
    pil_gate1_grid(s, start + 1000u);
    pil_gate1_motor(s, 0, 0, 1500u, start);
    pil_frame_t f = {.type = PIL_HELLO, .payload_length = 24u,
        .session = 456u, .ack = 0u};
    pil_put_u32(f.payload, 123u);
    pil_put_u32(f.payload + 4u, 456u);
    pil_put_u32(f.payload + 8u, 4u);
    pil_put_u32(f.payload + 12u, 1000u);
    pil_put_u32(f.payload + 16u, 4000u);
    pil_put_u32(f.payload + 20u, 5000u);
    pil_frame_t out;
    pil_gate1_boot(s, &out);
    assert(out.type == PIL_BOOT && pil_get_u32(out.payload + 4u) == 4u);
    assert(pil_gate1_receive(s, &f, now, &out));
    assert(out.type == PIL_HELLO_ACK && out.ack == 0u);
    assert(s->epoch - out.source_us >= 100000u);
    assert((s->epoch - s->grid_anchor) % 1000u == 0u);
    f = (pil_frame_t){.type = PIL_ARM, .payload_length = 8u,
        .session = 456u, .ack = out.sequence};
    pil_put_u32(f.payload, 123u);
    pil_put_u32(f.payload + 4u, 456u);
    assert(!pil_gate1_receive(s, &f, now, &out));
    assert(s->flags == 20u);
}
static void preload(pil_gate1_session_t *s, unsigned int count)
{
    pil_frame_t out;
    for (unsigned int i = 0u; i < count; ++i) {
        pil_frame_t f = sensor(i, 456u);
        now = s->epoch + i * 1000u - 3000u;
        assert(!pil_gate1_receive(s, &f, now, &out));
        assert(s->counts[G1_STAGED] == i + 1u);
    }
}
int main(void)
{
    pil_gate1_session_t s;
    pil_frame_t out;
    for (unsigned int wrap = 0u; wrap < 2u; ++wrap) {
        negotiate(&s, wrap ? UINT32_C(0xffff0000) : 1000000u);
        preload(&s, 3u);
        const uint32_t lease = s.lease;
        pil_frame_t duplicate = sensor(2u, 456u);
        assert(!pil_gate1_receive(&s, &duplicate, now, &out));
        assert(s.lease == lease && s.counts[G1_DROPPED] == 1u);
        now = s.epoch + 10u;
        assert(pil_gate1_alarm(&s, s.epoch, now, &out));
        assert(s.flags == 1u && out.ack == 0u);
        assert(pil_frame_validate(&out) == PIL_OK);
        assert(s.counts[G1_APPLIED] == 1u);
        pil_frame_t stop = {.type = PIL_DISARM, .payload_length = 4u,
            .session = 0u, .ack = PIL_NO_ACK};
        pil_put_u32(stop.payload, 1u);
        assert(!pil_gate1_receive(&s, &stop, now, &out));
        assert(s.reason == 1u && s.counts[G1_CANCELLED] == 2u);
        assert(s.counts[G1_STAGED] == s.counts[G1_APPLIED] +
               s.counts[G1_CANCELLED]);
        pil_gate1_fail(&s, 0x100u);
        assert(s.reason == 1u && s.flags == 4u);
    }
    negotiate(&s, 1000000u);
    preload(&s, 1u);
    now = s.epoch + 10u;
    assert(pil_gate1_alarm(&s, s.epoch, now, &out));
    assert(s.reason == 0x100u && s.flags == 14u);
    assert(s.counts[G1_APPLIED] == 0u);
    negotiate(&s, 1000000u);
    pil_frame_t f = sensor(0u, 456u);
    now = s.epoch - 1000u;
    assert(!pil_gate1_receive(&s, &f, now, &out));
    assert(s.reason == 0x101u && s.counts[G1_STAGED] == 0u);
    negotiate(&s, 1000000u);
    preload(&s, 2u);
    pil_gate1_check_lease(&s, s.lease - 1u);
    assert(s.reason == 0u);
    pil_gate1_check_lease(&s, s.lease);
    assert(s.reason == 0x104u);
    negotiate(&s, 1000000u);
    preload(&s, 4u);
    for (uint32_t i = 0u; i < 10000u; ++i) {
        now = s.epoch + i * 1000u + 10u;
        assert(pil_gate1_alarm(&s, s.epoch + i * 1000u, now, &out));
        assert(s.reason == 0u && out.ack == i);
        assert(pil_frame_validate(&out) == PIL_OK);
        f = sensor(i + 4u, 456u);
        now = s.epoch + i * 1000u + 1000u;
        assert(!pil_gate1_receive(&s, &f, now, &out));
    }
    assert(s.counts[G1_APPLIED] == 10000u);
    assert(s.metrics[0].count == 10004u && s.metrics[1].count == 10000u);
    uint32_t t[16]; pil_gate1_trace_get(&s, 63u, t);
    assert(t[0] == 1u && t[1] == 10003u);
    negotiate(&s, 1000000u);
    f = sensor(0u, 456u);
    pil_put_u16(f.payload + 36u, 0x418u);
    assert(!pil_gate1_receive(&s, &f, now, &out));
    assert(s.reason == 0x105u && s.counts[G1_BAD] == 0u);
    negotiate(&s, 1000000u);
    f = sensor(1u, 456u);
    assert(!pil_gate1_receive(&s, &f, now, &out));
    assert(s.reason == 0x100u && s.counts[G1_GAP] == 1u);
    negotiate(&s, 1000000u);
    f = sensor(0u, 789u);
    assert(!pil_gate1_receive(&s, &f, now, &out));
    assert(s.reason == 0u && s.counts[G1_DROPPED] == 1u);
    s.counts[G1_BAD] = UINT32_MAX;
    pil_gate1_count(&s, G1_BAD);
    assert(s.reason == 0x10eu && s.counts[G1_BAD] == UINT32_MAX);
    negotiate(&s, 1000000u);
    preload(&s, 2u);
    const uint32_t saved_lease = s.lease;
    f = (pil_frame_t){.type = PIL_SYNC_REQ, .payload_length = 8u,
        .session = 456u, .ack = PIL_NO_ACK};
    pil_put_u32(f.payload, now); f.source_us = now;
    const uint32_t receive_time = now - 20u;
    assert(pil_gate1_receive(&s, &f, receive_time, &out));
    assert(out.type == PIL_SYNC_REPLY && out.ack == 0u);
    assert(pil_get_u32(out.payload + 8u) == receive_time);
    assert(s.lease == saved_lease);
    assert(!pil_gate1_receive(&s, &f, receive_time, &out));
    assert(s.lease == saved_lease);
    now = s.epoch + 250u;
    assert(pil_gate1_alarm(&s, s.epoch, now, &out));
    assert(s.reason == 0x10au && s.counts[G1_APPLIED] == 0u);
    negotiate(&s, 1000000u);
    preload(&s, 8u);
    f = sensor(8u, 456u); now = s.epoch - 2000u;
    assert(!pil_gate1_receive(&s, &f, now, &out));
    assert(s.reason == 0x102u);
    return 0;
}

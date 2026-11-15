#include "pil_gate1_session.h"
#include <string.h>

static uint32_t clock_now(pil_gate1_session_t *s)
{
    return s->clock(s->clock_context);
}
static bool reached(uint32_t now, uint32_t deadline)
{
    return now == deadline || pil_newer(now, deadline);
}
static void trace(pil_gate1_session_t *s, uint32_t event, uint32_t seq,
                  uint32_t demand, uint32_t stamp, uint32_t alo,
                  uint32_t ahi, uint32_t plo, uint32_t phi)
{
    if (s->frozen) { return; }
    const uint32_t i = s->trace_total < 32u ? s->trace_total :
                       32u + (s->trace_total - 32u) % 32u;
    /* Caller excludes core0 IRQ mutation. Dump readers run only frozen.
     * Write owned storage directly: no64B automatic array on the IRQ chain. */
    uint32_t *words = s->trace[i];
    words[0] = event; words[1] = seq;
    words[2] = seq == PIL_NO_ACK ? 0u : seq * 1000u;
    words[3] = demand; words[4] = stamp;
    words[5] = alo; words[6] = ahi; words[7] = plo; words[8] = phi;
    words[9] = s->next_stage - s->next_apply; words[10] = s->flags;
    words[11] = s->counts[G1_DROPPED]; words[12] = s->counts[G1_BAD];
    words[13] = s->counts[G1_MISSED]; words[14] = s->counts[G1_OVERRUN];
    words[15] = s->reason;
    if (s->trace_total != UINT32_MAX) { ++s->trace_total; }
}
void pil_gate1_trace_get(const pil_gate1_session_t *s, unsigned int index,
                         uint32_t out[16])
{
    const uint32_t start = s->trace_total > 64u ?
                           (s->trace_total - 64u) % 32u : 0u;
    const uint32_t pos = index < 32u ? index :
                         32u + (start + index - 32u) % 32u;
    memcpy(out, s->trace[pos], 64u);
}
void pil_gate1_fail(pil_gate1_session_t *s, uint32_t reason)
{
    if (s->frozen || s->reason != 0u) { return; }
    s->reason = reason;
    s->stop_time = clock_now(s);
    if (s->fault_observer != NULL) {
        s->fault_observer(s->fault_context, s->stop_time, reason);
    }
    s->flags = 4u;
    if (reason >= 0x100u) {
        s->flags |= 2u;
        s->coverage |= 1u << 7u;
        if (reason == 0x100u) { s->flags |= 8u; }
    } else {
        s->coverage |= 1u << 5u;
        s->counts[G1_CANCELLED] = s->next_stage - s->next_apply;
    }
    trace(s, reason < 0x100u ? 4u : 5u, PIL_NO_ACK,
          0u, s->stop_time, 0u, 0u, 0u, 0u);
}
void pil_gate1_count(pil_gate1_session_t *s, unsigned int counter)
{
    if (s->frozen || counter >= G1_COUNTERS) { return; }
    if (s->counts[counter] == UINT32_MAX) {
        s->coverage |= 1u << 6u;
        s->counts[G1_COUNT_OVERFLOW] = 1u;
        pil_gate1_fail(s, 0x10eu);
    } else { ++s->counts[counter]; }
}
void pil_gate1_note(pil_gate1_session_t *s, unsigned int metric,
                    uint32_t seq, uint32_t lo, uint32_t hi,
                    uint32_t ref, uint32_t depth)
{
    if (s->frozen) { return; }
    if (metric == 0u || metric > 12u ||
        !pil_gate1_metric_observe(&s->metrics[metric - 1u], seq,
                                  lo, hi, ref, depth)) {
        s->coverage &= ~8u;
        pil_gate1_fail(s, 0x10eu);
    }
}
void pil_gate1_session_init(pil_gate1_session_t *s, uint32_t nonce,
                           pil_gate1_clock_t clock, void *context)
{
    memset(s, 0, sizeof(*s));
    s->nonce = nonce; s->clock = clock; s->clock_context = context;
    s->flags = 4u; s->coverage = 15u; s->last_demand = PIL_NO_ACK;
    s->hello_ack = PIL_NO_ACK;
    for (unsigned int i = 0u; i < 12u; ++i) {
        (void)pil_gate1_metric_init(&s->metrics[i], i + 1u);
    }
}
void pil_gate1_grid(pil_gate1_session_t *s, uint32_t next)
{
    if (!s->grid_ready) { s->grid_anchor = next; s->grid_ready = true; }
    s->grid_next = next;
}
static void frame(pil_gate1_session_t *s, pil_frame_t *f, uint8_t type,
                   uint16_t length, uint32_t ack)
{
    *f = (pil_frame_t){.type = type, .payload_length = length,
        .session = type == PIL_BOOT ? 0u : s->session,
        .sequence = s->transmitted[type]++, .source_us = clock_now(s),
        .ack = ack};
}
void pil_gate1_boot(pil_gate1_session_t *s, pil_frame_t *out)
{
    frame(s, out, PIL_BOOT, 16u, PIL_NO_ACK);
    pil_put_u32(out->payload, s->nonce);
    pil_put_u32(out->payload + 4u, 4u);
    pil_put_u32(out->payload + 8u, 921600u);
    pil_put_u32(out->payload + 12u, PIL_SCHEMA_ID);
}
void pil_gate1_motor(pil_gate1_session_t *s, int16_t left, int16_t right,
                     uint16_t servo, uint32_t stamp)
{
    if (s->frozen) { return; }
    if (left != 0 || right != 0 || servo < 1100u || servo > 1900u) {
        pil_gate1_fail(s, 0x10bu); return;
    }
    if (s->servo != 0u && s->servo != servo) { ++s->generation; }
    s->servo = servo; s->write_time = stamp;
}
static bool fresh(const pil_gate1_session_t *s, const pil_frame_t *f)
{
    return (s->seen & (1u << f->type)) == 0u ||
           pil_newer(f->sequence, s->received[f->type]);
}
static void accept(pil_gate1_session_t *s, const pil_frame_t *f)
{
    s->seen |= 1u << f->type; s->received[f->type] = f->sequence;
}
static void stage(pil_gate1_session_t *s, const pil_frame_t *f)
{
    if (s->reason != 0u || (s->flags != 20u && s->flags != 1u) ||
        f->session != s->session) {
        pil_gate1_count(s, G1_DROPPED); return;
    }
    if (f->sequence != s->next_stage) {
        pil_gate1_count(s, G1_DROPPED);
        if (pil_newer(f->sequence, s->next_stage)) {
            pil_gate1_count(s, G1_GAP); pil_gate1_fail(s, 0x100u);
        }
        return;
    }
    if (s->next_stage - s->next_apply == 8u) {
        pil_gate1_count(s, G1_DROPPED);
        pil_gate1_count(s, G1_SLOT_FULL); pil_gate1_fail(s, 0x102u); return;
    }
    const uint16_t valid = pil_get_u16(f->payload + 36u);
    if ((valid & 0x800u) == 0u || pil_get_u16(f->payload + 38u) != 0u) {
        pil_gate1_count(s, G1_DROPPED);
        pil_gate1_count(s, G1_TIMING); pil_gate1_fail(s, 0x105u); return;
    }
    if (f->payload[6] != 0u || pil_get_u32(f->payload + 8u) != 0u ||
        pil_get_u32(f->payload + 12u) != 0u || valid != 0xc18u ||
        f->ack != PIL_NO_ACK) {
        pil_gate1_count(s, G1_DROPPED); pil_gate1_fail(s, 0x106u); return;
    }
    pil_gate1_slot_t *slot = &s->slot[f->sequence & 7u];
    memcpy(slot->payload, f->payload, 40u);
    slot->sequence = f->sequence;
    slot->alo = clock_now(s);
    slot->ready = 1u;
    slot->ahi = clock_now(s);
    const uint32_t target = s->epoch + f->source_us;
    if (reached(slot->ahi + 1u, s->lease)) {
        slot->ready = 0u;
        pil_gate1_count(s, G1_DROPPED); pil_gate1_count(s, G1_LEASE);
        pil_gate1_fail(s, 0x104u); return;
    }
    if (pil_newer(slot->ahi + 1u, target - 1000u)) {
        slot->ready = 0u;
        pil_gate1_count(s, G1_DROPPED); pil_gate1_count(s, G1_LATE);
        pil_gate1_fail(s, 0x101u); return;
    }
    ++s->next_stage;
    s->lease = slot->ahi + 5000u;
    pil_gate1_count(s, G1_STAGED);
    pil_gate1_note(s, 1u, f->sequence, slot->alo, slot->ahi,
                   target - 4000u, 0u);
    pil_gate1_note(s, 3u, f->sequence, slot->alo, slot->ahi,
                   target - 1000u, 0u);
    pil_gate1_note(s, 7u, f->sequence, slot->ahi, slot->ahi,
                   0u, s->next_stage - s->next_apply);
    trace(s, 1u, f->sequence, target, slot->ahi,
          slot->alo, slot->ahi, 0u, 0u);
}
bool pil_gate1_receive(pil_gate1_session_t *s, const pil_frame_t *f,
                       uint32_t rx_time, pil_frame_t *reply)
{
    if (pil_frame_validate(f) != PIL_OK) {
        pil_gate1_count(s, G1_BAD); return false;
    }
    if (f->type == PIL_DISARM) {
        pil_gate1_fail(s, pil_get_u32(f->payload)); return false;
    }
    if (s->frozen) { return false; }
    if (s->armed_once) { pil_gate1_check_lease(s, clock_now(s)); }
    if (f->type == PIL_SENSOR) { stage(s, f); return false; }
    if (f->type == PIL_HELLO && s->reason == 0u && !s->armed_once &&
        s->grid_ready && s->nonce != 0u &&
        s->transmitted[PIL_BOOT] != 0u && f->ack != PIL_NO_ACK &&
        !pil_newer(f->ack, s->transmitted[PIL_BOOT] - 1u) &&
        pil_get_u32(f->payload) == s->nonce &&
        (pil_get_u32(f->payload + 8u) & ~4u) == 0u) {
        if (f->session == s->session && !fresh(s, f)) {
            pil_gate1_count(s, G1_REFUSED); return false;
        }
        const uint32_t stamp = clock_now(s);
        const uint32_t delta = stamp + 100000u - s->grid_next;
        if (delta > 101000u) {
            pil_gate1_count(s, G1_REFUSED); return false;
        }
        if (f->session != s->session) {
            s->seen = 0u; s->session = f->session;
        }
        accept(s, f);
        *reply = (pil_frame_t){.type = PIL_HELLO_ACK, .payload_length = 28u,
            .session = s->session, .sequence = s->transmitted[PIL_HELLO_ACK]++,
            .source_us = stamp, .ack = f->sequence};
        s->epoch = s->grid_next + ((delta + 999u) / 1000u) * 1000u;
        memcpy(reply->payload, f->payload, 24u);
        pil_put_u32(reply->payload + 24u, s->epoch);
        s->hello_ack = reply->sequence;
        return true;
    }
    if (f->session != s->session || s->session == 0u ||
        s->hello_ack == PIL_NO_ACK || s->reason != 0u || !fresh(s, f)) {
        pil_gate1_count(s, G1_REFUSED); return false;
    }
    if (f->type == PIL_SYNC_REQ) {
        accept(s, f);
        frame(s, reply, PIL_SYNC_REPLY, 16u, f->sequence);
        memcpy(reply->payload, f->payload, 8u);
        pil_put_u32(reply->payload + 8u, rx_time);
        return true;
    }
    if (f->type == PIL_ARM && !s->armed_once &&
        f->ack == s->hello_ack && pil_get_u32(f->payload) == s->nonce &&
        s->servo != 0u && !reached(clock_now(s), s->epoch)) {
        accept(s, f);
        s->armed_once = true; s->flags = 20u;
        s->lease = s->epoch + 1000u;
        return false;
    }
    pil_gate1_count(s, G1_REFUSED);
    return false;
}
void pil_gate1_check_lease(pil_gate1_session_t *s, uint32_t now)
{
    if (s->armed_once && s->reason == 0u && reached(now, s->lease)) {
        pil_gate1_count(s, G1_LEASE); pil_gate1_fail(s, 0x104u);
    }
}
void pil_gate1_output(pil_gate1_session_t *s, pil_frame_t *out)
{
    frame(s, out, PIL_OUTPUT, 32u,
          s->next_apply == 0u ? PIL_NO_ACK : s->next_apply - 1u);
    pil_put_u16(out->payload + 4u, (uint16_t)s->servo);
    out->payload[7] = (uint8_t)s->flags;
    pil_put_u16(out->payload + 8u, (uint16_t)s->generation);
    out->payload[10] = s->reason == 0u ?
                       (uint8_t)(s->next_stage - s->next_apply) : 0u;
    pil_put_u32(out->payload + 12u, s->write_time);
    const unsigned int indices[4] = {G1_DROPPED, G1_BAD, G1_MISSED, G1_OVERRUN};
    for (unsigned int i = 0u; i < 4u; ++i) {
        pil_put_u16(out->payload + 24u + 2u * i,
                    (uint16_t)s->counts[indices[i]]);
    }
}
bool pil_gate1_alarm(pil_gate1_session_t *s, uint32_t demand, uint32_t entry,
                     pil_frame_t *out)
{
    if (s->session == 0u || s->reason != 0u) { return false; }
    if (!s->armed_once || !reached(demand, s->epoch)) {
        pil_gate1_output(s, out); return true;
    }
    const uint32_t n = s->next_apply;
    const uint32_t target = s->epoch + n * 1000u;
    s->last_demand = (demand - s->epoch) / 1000u;
    pil_gate1_count(s, G1_DEMANDED);
    pil_gate1_note(s, 5u, n, entry, entry, demand, 0u);
    if (demand != target || entry - demand > 249u) {
        s->coverage &= ~4u; s->coverage |= 1u << 9u;
        pil_gate1_count(s, G1_MISSED); pil_gate1_fail(s, 0x10au);
    } else if (n >= PIL_MAX_SOURCE_US / 1000u) {
        pil_gate1_fail(s, 0x10du);
    } else if (s->slot[n & 7u].ready == 0u ||
               s->slot[(n + 1u) & 7u].ready == 0u) {
        pil_gate1_count(s, G1_MISSED); pil_gate1_fail(s, 0x100u);
    } else {
        const uint32_t lo = clock_now(s);
        s->slot[n & 7u].ready = 0u;
        ++s->next_apply;
        const uint32_t hi = clock_now(s);
        if (hi - demand > 249u) {
            --s->next_apply;
            s->slot[n & 7u].ready = 1u;
            pil_gate1_count(s, G1_MISSED); pil_gate1_fail(s, 0x10au);
        } else {
            if (n == 0u) { s->run_start = lo; }
            s->flags = 1u;
            pil_gate1_count(s, G1_APPLIED);
            pil_gate1_note(s, 2u, n, lo, hi, target - 4000u, 0u);
            pil_gate1_note(s, 4u, n, lo, hi, target, 0u);
            pil_gate1_note(s, 6u, n, lo, hi, entry, 0u);
            pil_gate1_note(s, 8u, n, hi, hi,
                           0u, s->next_stage - s->next_apply);
            trace(s, 2u, n, demand, entry, 0u, 0u, lo, hi);
            pil_gate1_count(s, G1_OUTPUT);
        }
    }
    pil_gate1_output(s, out);
    return true;
}

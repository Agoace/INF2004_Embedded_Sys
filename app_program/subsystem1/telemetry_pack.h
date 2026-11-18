/* Private RAM representation; public values and MQTT/USB formats stay native. */
#ifndef TELEMETRY_PACK_H
#define TELEMETRY_PACK_H
#include "telemetry.h"

typedef struct { uint8_t bytes[9]; } telemetry_event_packed_t;
_Static_assert(sizeof(telemetry_event_packed_t) == 9u, "Stored event size");

/* ms32, word16, detail8, kind4, from3, to3, six zero spare bits.
 * All byte accesses are local or under Telemetry's existing stats_lock.
 * There are no C bit-fields, packed structs or unaligned word accesses.
 */
static inline bool telemetry_event_pack(telemetry_event_packed_t *out,
                                        const telemetry_event_t *v)
{
    if (out == 0 || v == 0 || v->kind < 1u || v->kind > 10u ||
        v->from > 5u || v->to > 5u) {
        return false;
    }
    telemetry_event_packed_t p = {{0}};
    for (unsigned int i = 0u; i < 4u; ++i) {
        p.bytes[i] = (uint8_t)(v->ms >> (8u * i));
    }
    p.bytes[4] = (uint8_t)v->word;
    p.bytes[5] = (uint8_t)(v->word >> 8);
    p.bytes[6] = v->detail;
    p.bytes[7] = (uint8_t)((uint32_t)v->kind | ((uint32_t)v->from << 4) |
                           (((uint32_t)v->to & 1u) << 7));
    p.bytes[8] = (uint8_t)(v->to >> 1);
    *out = p;
    return true;
}

static inline bool telemetry_event_unpack(telemetry_event_t *out,
                                          const telemetry_event_packed_t *p)
{
    if (out == 0 || p == 0 || (p->bytes[8] & 0xfcu) != 0u) {
        return false;
    }
    telemetry_event_t v = {0};
    for (unsigned int i = 0u; i < 4u; ++i) {
        v.ms |= (uint32_t)p->bytes[i] << (8u * i);
    }
    v.word = (uint16_t)((uint32_t)p->bytes[4] | ((uint32_t)p->bytes[5] << 8));
    v.detail = p->bytes[6];
    v.kind = p->bytes[7] & 15u;
    v.from = (uint8_t)((p->bytes[7] >> 4) & 7u);
    v.to = (uint8_t)((uint32_t)(p->bytes[7] >> 7) | ((uint32_t)p->bytes[8] << 1));
    if (v.kind < 1u || v.kind > 10u || v.from > 5u || v.to > 5u) {
        return false;
    }
    *out = v;
    return true;
}
#if TM_MQTT
/* Full generation/lost/newest time plus bounded committed counts. */
typedef struct { uint32_t words[4]; } telemetry_index_packed_t;
static inline bool telemetry_index_pack(telemetry_index_packed_t *out,
                                        const telemetry_log_index_t *v)
{
    if (out == 0 || v == 0 || v->rows > TELEMETRY_ROWS ||
        v->events > TELEMETRY_EVENTS) {
        return false;
    }
    const telemetry_index_packed_t p = {{v->generation, v->lost, v->newest_ms,
        v->rows | (v->events << 13)}};
    *out = p;
    return true;
}
static inline bool telemetry_index_unpack(telemetry_log_index_t *out,
                                          const telemetry_index_packed_t *p)
{
    if (out == 0 || p == 0 || (p->words[3] & UINT32_C(0xff800000)) != 0u) {
        return false;
    }
    const telemetry_log_index_t v = {.generation=p->words[0], .lost=p->words[1],
        .newest_ms=p->words[2], .rows=p->words[3] & UINT32_C(0x1fff),
        .events=(p->words[3] >> 13) & UINT32_C(0x3ff)};
    if (v.rows > TELEMETRY_ROWS || v.events > TELEMETRY_EVENTS) {
        return false;
    }
    *out = v;
    return true;
}
#endif
#endif

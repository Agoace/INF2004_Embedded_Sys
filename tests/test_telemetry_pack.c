#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../app_program/subsystem1/telemetry_pack.h"

static void roundtrip(const telemetry_event_t *v)
{
    telemetry_event_packed_t p;
    memset(&p, 0xff, sizeof(p));
    assert(telemetry_event_pack(&p, v));
    assert((p.bytes[8] & 0xfcu) == 0u);
    telemetry_event_t out = {0};
    assert(telemetry_event_unpack(&out, &p));
    assert(out.ms == v->ms && out.word == v->word && out.detail == v->detail);
    assert(out.kind == v->kind && out.from == v->from && out.to == v->to);
}

int main(void)
{
    assert(sizeof(telemetry_event_packed_t) == 9u);
    assert(sizeof(telemetry_event_t) == 12u);
    for (unsigned int kind = 1u; kind <= 10u; ++kind) {
        for (unsigned int from = 0u; from <= 5u; ++from) {
            for (unsigned int to = 0u; to <= 5u; ++to) {
                for (unsigned int detail = 0u; detail <= UINT8_MAX; ++detail) {
                    telemetry_event_t v = {.ms = UINT32_MAX, .word = UINT16_MAX,
                        .kind = (uint8_t)kind, .from = (uint8_t)from,
                        .to = (uint8_t)to, .detail = (uint8_t)detail};
                    roundtrip(&v);
                }
            }
        }
    }
    telemetry_event_t v = {.kind = 10u, .from = 2u, .to = 5u};
    for (uint32_t w = 0u; w <= UINT16_MAX; ++w) {
        v.word = (uint16_t)w;
        v.ms = w * UINT32_C(65537); /* includes both u32 endpoints */
        roundtrip(&v);
    }
    telemetry_event_packed_t p;
    assert(telemetry_event_pack(&p, &v));
    const telemetry_event_packed_t saved = p;
    telemetry_event_t out = {.ms = 99u, .word = 33u, .kind = 1u};
    unsigned char before[sizeof(out)];
    memcpy(before, &out, sizeof(out));
    for (unsigned int b = 2u; b < 8u; ++b) {
        p = saved;
        p.bytes[8] |= (uint8_t)(1u << b);
        assert(!telemetry_event_unpack(&out, &p));
        assert(memcmp(before, &out, sizeof(out)) == 0);
    }
    p = saved;
    v.kind = 0u;
    assert(!telemetry_event_pack(&p, &v) && memcmp(&p, &saved, sizeof(p)) == 0);
    v.kind = 11u;
    assert(!telemetry_event_pack(&p, &v));
    v.kind = 1u; v.from = 6u;
    assert(!telemetry_event_pack(&p, &v));
    v.from = 0u; v.to = 6u;
    assert(!telemetry_event_pack(&p, &v));
    assert(!telemetry_event_pack(NULL, &v));
    assert(!telemetry_event_pack(&p, NULL));
    assert(!telemetry_event_unpack(NULL, &p));
    assert(!telemetry_event_unpack(&out, NULL));
    puts("test_telemetry_pack: PASS");
    return 0;
}

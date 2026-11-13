#include "pil_gate1_dump.h"
#include "pil_wire.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static const uint8_t golden[] = {
    0x00,0x06,0x50,0x47,0x44,0x31,0x01,0x02,0x05,0x02,0x0e,0x01,
    0x01,0x0a,0x78,0x56,0x34,0x12,0xf0,0xde,0xbc,0x9a,0x10,0x01,
    0x01,0x02,0x0e,0x01,0x01,0x02,0x0c,0x01,0x01,0x01,0x01,0x01,
    0x01,0x01,0x01,0x01,0x01,0x05,0x06,0xc3,0x49,0x0a,0x00
};

int main(int argc, char **argv)
{
    if (argc == 3) {
        uint8_t packet[256];
        const size_t n = strlen(argv[2]) / 2u;
        assert(n <= sizeof(packet));
        for (size_t i = 0u; i < n; ++i) {
            unsigned int value;
            assert(sscanf(argv[2] + i * 2u, "%2x", &value) == 1);
            packet[i] = (uint8_t)value;
        }
        pil_gate1_record_t frame;
        memset(&frame, 0xa5, sizeof(frame));
        const pil_gate1_record_t before = frame;
        const bool expected = strcmp(argv[1], "accept") == 0;
        assert(pil_gate1_dump_decode(packet, n, &frame) == expected);
        if (expected) {
            uint8_t encoded[PIL_GATE1_D1_WIRE];
            size_t length;
            assert(pil_gate1_dump_encode(&frame, encoded,
                                        sizeof(encoded), &length));
            assert(length == n && memcmp(packet, encoded, n) == 0);
        } else { assert(memcmp(&frame, &before, sizeof(frame)) == 0); }
        return 0;
    }
    assert(PIL_MAX_BODY == 128u && PIL_GATE1_D1_BODY == 132u);
    pil_gate1_record_t r = {.kind = PIL_GATE1_D1_MANIFEST,
        .sequence = 14u, .nonce = UINT32_C(0x12345678),
        .session = UINT32_C(0x9abcdef0), .length = 16u};
    pil_put_u32(r.payload, 14u);
    pil_put_u32(r.payload + 4u, 12u);
    uint8_t wire[PIL_GATE1_D1_WIRE] = {0};
    size_t length = 0u;
    assert(pil_gate1_dump_encode(&r, wire, sizeof(wire), &length));
    assert(length == sizeof(golden));
    assert(memcmp(wire, golden, length) == 0);
    pil_gate1_record_t decoded;
    memset(&decoded, 0xa5, sizeof(decoded));
    assert(pil_gate1_dump_decode(golden, sizeof(golden), &decoded));
    assert(decoded.kind == r.kind && decoded.nonce == r.nonce);
    assert(decoded.session == r.session && decoded.sequence == r.sequence);
    assert(decoded.length == r.length);
    assert(memcmp(decoded.payload, r.payload, r.length) == 0);
    const pil_gate1_record_t saved = decoded;
    for (size_t n = 0u; n < sizeof(golden); ++n) {
        assert(!pil_gate1_dump_decode(golden, n, &decoded));
        assert(memcmp(&saved, &decoded, sizeof(saved)) == 0);
    }
    for (size_t bit = 0u; bit < 8u * sizeof(golden); ++bit) {
        uint8_t bad[sizeof(golden)];
        memcpy(bad, golden, sizeof(bad));
        bad[bit / 8u] ^= (uint8_t)(1u << (bit % 8u));
        assert(!pil_gate1_dump_decode(bad, sizeof(bad), &decoded));
        assert(memcmp(&saved, &decoded, sizeof(saved)) == 0);
    }
    for (unsigned int kind = 1u; kind <= 5u; ++kind) {
        r.kind = (uint16_t)kind;
        r.length = kind <= 2u ? 104u : kind == 3u ? 48u :
                   kind == 4u ? 64u : 16u;
        for (size_t i = 0u; i < r.length; ++i) {
            r.payload[i] = (uint8_t)i;
        }
        assert(pil_gate1_dump_encode(&r, wire, sizeof(wire), &length));
        assert(length <= 135u);
        assert(pil_gate1_dump_decode(wire, length, &decoded));
        assert(decoded.length == r.length);
        assert(memcmp(decoded.payload, r.payload, r.length) == 0);
        const pil_gate1_record_t unchanged = decoded;
        for (size_t prefix = 0u; prefix < length; ++prefix) {
            assert(!pil_gate1_dump_decode(wire, prefix, &decoded));
            assert(memcmp(&decoded, &unchanged, sizeof(decoded)) == 0);
        }
        uint8_t before[sizeof(wire)];
        memcpy(before, wire, sizeof(before));
        size_t sentinel = 12345u;
        assert(!pil_gate1_dump_encode(&r, wire, length - 1u, &sentinel));
        assert(sentinel == 12345u);
        assert(memcmp(wire, before, sizeof(wire)) == 0);
    }
    uint32_t chain = 0u;
    assert(pil_gate1_dump_chain(&chain, &r));
    assert(chain == UINT32_C(0x74403877));
    assert(pil_gate1_dump_chain(&chain, &r));
    assert(chain == UINT32_C(0xcaaa8d96));
    const uint32_t previous = chain;
    r.kind = 6u;
    assert(!pil_gate1_dump_chain(&chain, &r));
    assert(chain == previous);
    assert(!pil_gate1_dump_encode(&r, wire, sizeof(wire), &length));
    r.kind = 5u; r.nonce = 0u; r.session = 0u;
    assert(pil_gate1_dump_encode(&r, wire, sizeof(wire), &length));
    r.length = 0u;
    assert(!pil_gate1_dump_encode(&r, wire, sizeof(wire), &length));
    assert(!pil_gate1_dump_decode(NULL, 1u, &decoded));
    assert(!pil_gate1_dump_decode(golden, sizeof(golden), NULL));
    assert(!pil_gate1_dump_encode(NULL, wire, sizeof(wire), &length));
    assert(!pil_gate1_dump_encode(&r, NULL, sizeof(wire), &length));
    assert(!pil_gate1_dump_encode(&r, wire, sizeof(wire), NULL));
    assert(!pil_gate1_dump_chain(NULL, &r));
    uint32_t random = UINT32_C(0x31444750);
    for (unsigned int trial = 0u; trial < 100000u; ++trial) {
        uint8_t noise[PIL_GATE1_D1_WIRE + 1u];
        for (size_t i = 0u; i < sizeof(noise); ++i) {
            random ^= random << 13u;
            random ^= random >> 17u;
            random ^= random << 5u;
            noise[i] = (uint8_t)random;
        }
        const pil_gate1_record_t before = decoded;
        if (!pil_gate1_dump_decode(noise, trial % (sizeof(noise) + 1u),
                                   &decoded)) {
            assert(memcmp(&before, &decoded, sizeof(decoded)) == 0);
        }
    }
    return 0;
}

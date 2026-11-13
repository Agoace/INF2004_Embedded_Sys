#include "pil_gate1_dump.h"
#include "pil_wire.h"
#include <string.h>

static bool shape(const pil_gate1_record_t *r)
{
    static const uint32_t sizes[6] = {0u, 104u, 104u, 48u, 64u, 16u};
    return r != NULL && r->kind > 0u && r->kind <= 5u &&
           r->length == sizes[r->kind];
}

static size_t header(const pil_gate1_record_t *r, uint8_t *body)
{
    pil_put_u32(body, PIL_GATE1_D1_MAGIC);
    pil_put_u16(body + 4u, 1u);
    pil_put_u16(body + 6u, r->kind);
    pil_put_u32(body + 8u, r->sequence);
    pil_put_u32(body + 12u, r->nonce);
    pil_put_u32(body + 16u, r->session);
    pil_put_u32(body + 20u, r->length);
    memcpy(body + 24u, r->payload, r->length);
    return 24u + r->length;
}

bool pil_gate1_dump_encode(const pil_gate1_record_t *r, uint8_t *out,
                          size_t capacity, size_t *length)
{
    if (!shape(r) || out == NULL || length == NULL) { return false; }
    uint8_t body[PIL_GATE1_D1_BODY];
    size_t n = header(r, body);
    pil_put_u32(body + n, pil_crc32c(body, n));
    n += 4u;
    uint8_t wire[PIL_GATE1_D1_WIRE] = {0};
    size_t code_at = 1u, pos = 2u;
    uint8_t code = 1u;
    for (size_t i = 0u; i < n; ++i) {
        if (body[i] == 0u) {
            wire[code_at] = code;
            code_at = pos++;
            code = 1u;
        } else {
            wire[pos++] = body[i];
            ++code; /* n<=132, so no 254-byte block exists. */
        }
    }
    wire[code_at] = code;
    wire[pos++] = 0u;
    if (pos > capacity) { return false; }
    memcpy(out, wire, pos);
    *length = pos;
    return true;
}

bool pil_gate1_dump_decode(const uint8_t *packet, size_t length,
                          pil_gate1_record_t *r)
{
    if (packet == NULL || r == NULL || length < 3u ||
        length > PIL_GATE1_D1_WIRE || packet[0] != 0u ||
        packet[length - 1u] != 0u) { return false; }
    uint8_t body[PIL_GATE1_D1_BODY];
    size_t n = 0u, pos = 1u;
    while (pos < length - 1u) {
        const uint8_t code = packet[pos++];
        if (code == 0u || (size_t)code - 1u > length - 1u - pos) {
            return false;
        }
        for (unsigned int i = 1u; i < code; ++i) {
            if (n == sizeof(body) || packet[pos] == 0u) { return false; }
            body[n++] = packet[pos++];
        }
        if (code != 255u && pos < length - 1u) {
            if (n == sizeof(body)) { return false; }
            body[n++] = 0u;
        }
    }
    if (n < 28u || pil_get_u32(body) != PIL_GATE1_D1_MAGIC ||
        pil_get_u16(body + 4u) != 1u ||
        pil_get_u32(body + n - 4u) != pil_crc32c(body, n - 4u)) {
        return false;
    }
    pil_gate1_record_t value = {.kind = pil_get_u16(body + 6u),
        .sequence = pil_get_u32(body + 8u),
        .nonce = pil_get_u32(body + 12u),
        .session = pil_get_u32(body + 16u),
        .length = pil_get_u32(body + 20u)};
    if (!shape(&value) || n != 28u + value.length) { return false; }
    memcpy(value.payload, body + 24u, value.length);
    *r = value;
    return true;
}

bool pil_gate1_dump_chain(uint32_t *crc, const pil_gate1_record_t *r)
{
    if (crc == NULL || !shape(r)) { return false; }
    uint8_t body[24u + PIL_GATE1_D1_PAYLOAD];
    const size_t n = header(r, body);
    uint32_t value = *crc ^ UINT32_MAX;
    for (size_t i = 0u; i < n; ++i) {
        value ^= body[i];
        for (unsigned int bit = 0u; bit < 8u; ++bit) {
            value = (value >> 1u) ^ ((value & 1u) != 0u ?
                    UINT32_C(0x82f63b78) : 0u);
        }
    }
    *crc = value ^ UINT32_MAX;
    return true;
}

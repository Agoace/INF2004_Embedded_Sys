#include "pil_wire.h"
#include <string.h>

uint16_t pil_get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8u));
}

uint32_t pil_get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8u) |
           ((uint32_t)p[2] << 16u) | ((uint32_t)p[3] << 24u);
}

void pil_put_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8u);
}

void pil_put_u32(uint8_t *p, uint32_t value)
{
    for (unsigned int i = 0u; i < 4u; ++i) {
        p[i] = (uint8_t)(value >> (8u * i));
    }
}

uint32_t pil_crc32c(const uint8_t *data, size_t length)
{
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0u; i < length; ++i) {
        crc ^= data[i];
        for (unsigned int bit = 0u; bit < 8u; ++bit) {
            const uint32_t low = crc & 1u;
            crc = (crc >> 1u) ^ (low != 0u ? UINT32_C(0x82f63b78) : 0u);
        }
    }
    return crc ^ UINT32_MAX;
}

bool pil_newer(uint32_t value, uint32_t previous)
{
    const uint32_t delta = value - previous;
    return delta != 0u && delta < UINT32_C(0x80000000);
}

static int32_t signed16(const uint8_t *p)
{
    const uint16_t value = pil_get_u16(p);
    return value < 32768u ? (int32_t)value : (int32_t)value - 65536;
}

static bool pulse_valid(uint16_t pulse)
{
    return pulse >= 1100u && pulse <= 1900u;
}

static bool due(uint32_t source_us, unsigned int rate)
{
    return source_us == 0u ||
        ((uint64_t)source_us * rate / 1000000u !=
         (uint64_t)(source_us - 1000u) * rate / 1000000u);
}

int pil_sensor_validate(const uint8_t *p, uint32_t source_us)
{
    if (p == NULL || source_us > PIL_MAX_SOURCE_US || source_us % 1000u != 0u) {
        return PIL_INVALID;
    }
    const uint16_t valid = pil_get_u16(p + 36u);
    if ((p[6] & 0xfcu) != 0u || (p[7] & 0xc0u) != 0u ||
        (valid & 0xe000u) != 0u) {
        return PIL_INVALID;
    }
    for (unsigned int axis = 0u; axis < 3u; ++axis) {
        if (pil_get_u16(p + 2u * axis) > 4095u ||
            (pil_get_u16(p + 16u + 2u * axis) & 15u) != 0u) {
            return PIL_INVALID;
        }
        const int32_t mag = signed16(p + 22u + 2u * axis);
        if (mag < -2048 || mag > 2047) {
            if (mag != -4096 || (valid & 0x140u) != 0u) {
                return PIL_INVALID;
            }
        }
    }
    const bool bus_fault = (p[7] & 12u) != 0u;
    for (unsigned int device = 0u; device < 2u; ++device) {
        const bool fresh = (valid & (1u << (5u + device))) != 0u;
        const bool good = (valid & (1u << (7u + device))) != 0u;
        const bool ready = due(source_us, device == 0u ? 400u : 75u);
        if ((fresh && (!good || !ready)) ||
            (good && ready && !bus_fault && !fresh)) {
            return PIL_INVALID;
        }
    }
    const bool sonar_valid = (valid & 0x200u) != 0u;
    if ((valid & 0x1000u) != 0u && !sonar_valid) {
        return PIL_INVALID;
    }
    if (sonar_valid) {
        const unsigned int mode = p[7] & 3u;
        const uint16_t rise = pil_get_u16(p + 28u);
        const uint16_t high = pil_get_u16(p + 30u);
        if (!pulse_valid(pil_get_u16(p + 34u))) {
            return PIL_INVALID;
        }
        if (mode == 1u || mode == 2u) {
            if (rise != 0u || high != 0u) {
                return PIL_INVALID;
            }
        } else if (rise == 0u || rise >= 2000u ||
                   (mode == 0u && (high == 0u || high > 60000u)) ||
                   (mode == 3u && high != 0u)) {
            return PIL_INVALID;
        }
    }
    return PIL_OK;
}

int pil_output_validate(const uint8_t *p)
{
    if (p == NULL) {
        return PIL_INVALID;
    }
    const uint8_t flags = p[7];
    const bool armed = (flags & 1u) != 0u;
    const bool fault = (flags & 2u) != 0u;
    const bool inhibited = (flags & 4u) != 0u;
    const bool pending = (flags & 16u) != 0u;
    if ((p[6] & 0xfcu) != 0u || (flags & 0xe0u) != 0u ||
        p[10] > 8u || p[11] != 0u ||
        (armed && (fault || inhibited || pending)) ||
        ((fault || pending) && !inhibited) ||
        ((flags & 8u) != 0u && (!fault || !inhibited))) {
        return PIL_INVALID;
    }
    for (unsigned int wheel = 0u; wheel < 2u; ++wheel) {
        const int32_t compare = signed16(p + 2u * wheel);
        if (compare < -6250 || compare > 6250 || (!armed && compare != 0)) {
            return PIL_INVALID;
        }
    }
    const uint16_t pulse = pil_get_u16(p + 4u);
    return pulse_valid(pulse) || (pulse == 0u && inhibited) ?
           PIL_OK : PIL_INVALID;
}

int pil_frame_validate(const pil_frame_t *f)
{
    static const uint16_t sizes[] = {16u, 40u, 32u, 24u, 28u,
                                     8u, 4u, 8u, 16u};
    if (f == NULL || f->type > PIL_SYNC_REPLY ||
        f->payload_length != sizes[f->type] ||
        (f->type != PIL_BOOT && f->type != PIL_DISARM && f->session == 0u)) {
        return PIL_INVALID;
    }
    const uint8_t *p = f->payload;
    switch (f->type) {
    case PIL_BOOT:
        return f->session == 0u && f->ack == PIL_NO_ACK &&
               pil_get_u32(p) != 0u &&
               (pil_get_u32(p + 4u) & ~PIL_CAPABILITIES) == 0u &&
               pil_get_u32(p + 8u) == 921600u &&
               pil_get_u32(p + 12u) == PIL_SCHEMA_ID ? PIL_OK : PIL_INVALID;
    case PIL_SENSOR:
        return f->sequence == f->source_us / 1000u ?
               pil_sensor_validate(p, f->source_us) : PIL_INVALID;
    case PIL_OUTPUT:
        return pil_output_validate(p);
    case PIL_HELLO:
    case PIL_HELLO_ACK:
        return pil_get_u32(p) != 0u && pil_get_u32(p + 4u) == f->session &&
               (pil_get_u32(p + 8u) & ~PIL_CAPABILITIES) == 0u &&
               pil_get_u32(p + 12u) == 1000u &&
               pil_get_u32(p + 16u) == 4000u &&
               pil_get_u32(p + 20u) == 5000u && f->ack != PIL_NO_ACK ?
               PIL_OK : PIL_INVALID;
    case PIL_ARM:
        return pil_get_u32(p) != 0u && pil_get_u32(p + 4u) == f->session &&
               f->ack != PIL_NO_ACK ? PIL_OK : PIL_INVALID;
    case PIL_DISARM:
        return pil_get_u32(p) >= 1u && pil_get_u32(p) <= 8u ?
               PIL_OK : PIL_INVALID;
    case PIL_SYNC_REQ:
        return pil_get_u32(p) == f->source_us ? PIL_OK : PIL_INVALID;
    case PIL_SYNC_REPLY:
        return pil_get_u32(p + 12u) == 0u && f->ack != PIL_NO_ACK ?
               PIL_OK : PIL_INVALID;
    default:
        return PIL_INVALID;
    }
}

int pil_encode(const pil_frame_t *f, uint8_t *out, size_t capacity,
               size_t *length)
{
    if (out == NULL || length == NULL || pil_frame_validate(f) != PIL_OK) {
        return PIL_INVALID;
    }
    const size_t body_size = 24u + f->payload_length;
    if (capacity < body_size + 2u) {
        return PIL_SIZE;
    }
    uint8_t body[PIL_MAX_BODY];
    body[0] = PIL_VERSION;
    body[1] = f->type;
    pil_put_u16(body + 2u, f->payload_length);
    pil_put_u32(body + 4u, f->session);
    pil_put_u32(body + 8u, f->sequence);
    pil_put_u32(body + 12u, f->source_us);
    pil_put_u32(body + 16u, f->ack);
    memcpy(body + 20u, f->payload, f->payload_length);
    pil_put_u32(body + body_size - 4u, pil_crc32c(body, body_size - 4u));
    size_t code_at = 0u, write_at = 1u;
    uint8_t code = 1u;
    for (size_t i = 0u; i < body_size; ++i) {
        if (body[i] == 0u) {
            out[code_at] = code;
            code_at = write_at++;
            code = 1u;
        } else {
            out[write_at++] = body[i];
            ++code;
        }
    }
    out[code_at] = code;
    out[write_at++] = 0u;
    *length = write_at;
    return PIL_OK;
}

int pil_decode(const uint8_t *packet, size_t length, pil_frame_t *frame)
{
    if (packet == NULL || frame == NULL) {
        return PIL_INVALID;
    }
    if (length < 2u || length > PIL_MAX_PACKET) {
        return PIL_SIZE;
    }
    if (packet[length - 1u] != 0u) {
        return PIL_INVALID;
    }
    uint8_t body[PIL_MAX_BODY];
    size_t read_at = 0u, write_at = 0u;
    while (read_at < length - 1u) {
        const uint8_t code = packet[read_at++];
        if (code == 0u || (size_t)(code - 1u) > length - 1u - read_at) {
            return PIL_INVALID;
        }
        for (unsigned int i = 1u; i < code; ++i) {
            if (write_at == sizeof(body) || packet[read_at] == 0u) {
                return PIL_INVALID;
            }
            body[write_at++] = packet[read_at++];
        }
        if (code != 255u && read_at < length - 1u) {
            if (write_at == sizeof(body)) {
                return PIL_SIZE;
            }
            body[write_at++] = 0u;
        }
    }
    if (write_at < 24u || body[0] != PIL_VERSION ||
        pil_get_u16(body + 2u) > PIL_MAX_PAYLOAD ||
        (size_t)pil_get_u16(body + 2u) + 24u != write_at) {
        return PIL_INVALID;
    }
    if (pil_crc32c(body, write_at - 4u) !=
        pil_get_u32(body + write_at - 4u)) {
        return PIL_CRC;
    }
    pil_frame_t result = {0};
    result.type = body[1];
    result.payload_length = pil_get_u16(body + 2u);
    result.session = pil_get_u32(body + 4u);
    result.sequence = pil_get_u32(body + 8u);
    result.source_us = pil_get_u32(body + 12u);
    result.ack = pil_get_u32(body + 16u);
    memcpy(result.payload, body + 20u, result.payload_length);
    if (pil_frame_validate(&result) != PIL_OK) {
        return PIL_INVALID;
    }
    *frame = result;
    return PIL_OK;
}

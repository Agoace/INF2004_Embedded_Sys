#include "pil_wire.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static size_t unhex(const char *text, uint8_t *out, size_t capacity)
{
    const size_t length = strlen(text);
    assert(length % 2u == 0u && length / 2u <= capacity);
    for (size_t i = 0u; i < length / 2u; ++i) {
        char pair[3] = {text[2u * i], text[2u * i + 1u], '\0'};
        char *end;
        const unsigned long value = strtoul(pair, &end, 16);
        assert(*end == '\0' && value <= 255u);
        out[i] = (uint8_t)value;
    }
    return length / 2u;
}

static void fixture(const char *raw_text, const char *packet_text, bool valid)
{
    uint8_t raw[PIL_MAX_BODY], wire[PIL_MAX_PACKET], encoded[PIL_MAX_PACKET];
    const size_t raw_size = unhex(raw_text, raw, sizeof(raw));
    const size_t wire_size = unhex(packet_text, wire, sizeof(wire));
    assert(raw_size >= 24u);
    assert(pil_crc32c(raw, raw_size - 4u) ==
           pil_get_u32(raw + raw_size - 4u));
    pil_frame_t frame;
    memset(&frame, 0xa5, sizeof(frame));
    const pil_frame_t before = frame;
    const int result = pil_decode(wire, wire_size, &frame);
    if (!valid) {
        assert(result != PIL_OK);
        assert(memcmp(&before, &frame, sizeof(frame)) == 0);
        return;
    }
    assert(result == PIL_OK);
    assert(frame.type == raw[1]);
    assert(frame.payload_length == pil_get_u16(raw + 2u));
    assert(frame.session == pil_get_u32(raw + 4u));
    assert(frame.sequence == pil_get_u32(raw + 8u));
    assert(frame.source_us == pil_get_u32(raw + 12u));
    assert(frame.ack == pil_get_u32(raw + 16u));
    assert(memcmp(frame.payload, raw + 20u, frame.payload_length) == 0);
    size_t encoded_size = 999u;
    assert(pil_encode(&frame, encoded, sizeof(encoded), &encoded_size) ==
           PIL_OK);
    assert(encoded_size == wire_size && memcmp(encoded, wire, wire_size) == 0);
    for (size_t prefix = 0u; prefix < wire_size; ++prefix) {
        assert(pil_decode(wire, prefix, &frame) != PIL_OK);
    }
    for (size_t byte = 0u; byte < wire_size; ++byte) {
        for (unsigned int bit = 0u; bit < 8u; ++bit) {
            memcpy(encoded, wire, wire_size);
            encoded[byte] ^= (uint8_t)(1u << bit);
            assert(pil_decode(encoded, wire_size, &frame) != PIL_OK);
        }
    }
}

static void core(void)
{
    static const uint8_t check[] = "123456789";
    assert(pil_crc32c(check, 9u) == UINT32_C(0xe3069283));
    assert(pil_crc32c(NULL, 0u) == 0u);
    assert(pil_newer(0u, UINT32_MAX));
    assert(!pil_newer(UINT32_MAX, 0u));
    assert(!pil_newer(0u, 0u));
    assert(!pil_newer(UINT32_C(0x80000000), 0u));
    pil_frame_t frame = {.type = PIL_DISARM, .payload_length = 4u,
                         .session = 0u, .ack = PIL_NO_ACK};
    pil_put_u32(frame.payload, 2u);
    uint8_t packet[PIL_MAX_PACKET];
    memset(packet, 0xa5, sizeof(packet));
    size_t length = 999u;
    assert(pil_encode(&frame, packet, 29u, &length) == PIL_SIZE);
    assert(length == 999u && packet[0] == 0xa5u);
    assert(pil_encode(&frame, packet, 30u, &length) == PIL_OK);
    assert(length == 30u);
    assert(pil_encode(NULL, packet, sizeof(packet), &length) == PIL_INVALID);
    assert(pil_encode(&frame, NULL, sizeof(packet), &length) == PIL_INVALID);
    assert(pil_encode(&frame, packet, sizeof(packet), NULL) == PIL_INVALID);
    assert(pil_decode(NULL, length, &frame) == PIL_INVALID);
    assert(pil_decode(packet, length, NULL) == PIL_INVALID);
    assert(pil_sensor_validate(NULL, 0u) == PIL_INVALID);
    assert(pil_output_validate(NULL) == PIL_INVALID);
    frame.type = 255u;
    assert(pil_frame_validate(&frame) == PIL_INVALID);
    frame.type = PIL_DISARM;
    frame.payload_length = 105u;
    assert(pil_frame_validate(&frame) == PIL_INVALID);
    uint32_t random = UINT32_C(0x12345678);
    for (unsigned int trial = 0u; trial < 100000u; ++trial) {
        for (size_t i = 0u; i < sizeof(packet); ++i) {
            random ^= random << 13u;
            random ^= random >> 17u;
            random ^= random << 5u;
            packet[i] = (uint8_t)random;
        }
        packet[sizeof(packet) - 1u] = 0u;
        (void)pil_decode(packet, trial % (PIL_MAX_PACKET + 2u), &frame);
    }
}

int main(int argc, char **argv)
{
    if (argc == 1) {
        core();
    } else {
        assert(argc == 4);
        fixture(argv[2], argv[3], strcmp(argv[1], "accept") == 0);
    }
    return 0;
}

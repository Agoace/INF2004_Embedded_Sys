/* Frozen PIL v1 byte codec. No kernel, SDK or transport ownership. */
#ifndef PIL_WIRE_H
#define PIL_WIRE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PIL_VERSION 1u
#define PIL_SCHEMA_ID UINT32_C(0x50494c01)
#define PIL_CAPABILITIES UINT32_C(0x1f)
#define PIL_NO_ACK UINT32_MAX
#define PIL_MAX_PAYLOAD 104u
#define PIL_MAX_BODY 128u
#define PIL_MAX_PACKET 130u
#define PIL_SENSOR_SIZE 40u
#define PIL_OUTPUT_SIZE 32u
#define PIL_MAX_SOURCE_US UINT32_C(1800000000)

enum pil_type {
    PIL_BOOT = 0, PIL_SENSOR = 1, PIL_OUTPUT = 2, PIL_HELLO = 3,
    PIL_HELLO_ACK = 4, PIL_ARM = 5, PIL_DISARM = 6,
    PIL_SYNC_REQ = 7, PIL_SYNC_REPLY = 8
};
enum pil_status {
    PIL_OK = 0, PIL_INVALID = -1, PIL_SIZE = -2, PIL_CRC = -3
};

typedef struct {
    uint8_t type;
    uint16_t payload_length;
    uint32_t session;
    uint32_t sequence;
    uint32_t source_us;
    uint32_t ack;
    uint8_t payload[PIL_MAX_PAYLOAD];
} pil_frame_t;

uint16_t pil_get_u16(const uint8_t *p);
uint32_t pil_get_u32(const uint8_t *p);
void pil_put_u16(uint8_t *p, uint16_t value);
void pil_put_u32(uint8_t *p, uint32_t value);
uint32_t pil_crc32c(const uint8_t *data, size_t length);
bool pil_newer(uint32_t value, uint32_t previous);
int pil_sensor_validate(const uint8_t *payload, uint32_t source_us);
int pil_output_validate(const uint8_t *payload);
int pil_frame_validate(const pil_frame_t *frame);
/* Packets include the trailing zero. Decode commits only on success.
 * Encode leaves output/length unchanged on failure. Buffers must not overlap.
 * This is a stateless codec, NOT session, lease or clock qualification. */
int pil_encode(const pil_frame_t *frame, uint8_t *out, size_t capacity,
               size_t *length);
int pil_decode(const uint8_t *packet, size_t length, pil_frame_t *frame);

#endif

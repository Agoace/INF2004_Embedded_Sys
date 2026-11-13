/* Separate stopped-USB D1 envelope; never a frozen PIL v1 extension. */
#ifndef PIL_GATE1_DUMP_H
#define PIL_GATE1_DUMP_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PIL_GATE1_D1_MAGIC UINT32_C(0x31444750)
#define PIL_GATE1_D1_BODY 132u
#define PIL_GATE1_D1_WIRE 135u
#define PIL_GATE1_D1_PAYLOAD 104u

enum pil_gate1_d1_kind {
    PIL_GATE1_D1_CONTEXT = 1, PIL_GATE1_D1_COUNTS = 2,
    PIL_GATE1_D1_EXTREMUM = 3, PIL_GATE1_D1_TRACE = 4,
    PIL_GATE1_D1_MANIFEST = 5
};

typedef struct {
    uint16_t kind;
    uint32_t sequence, nonce, session, length;
    uint8_t payload[PIL_GATE1_D1_PAYLOAD];
} pil_gate1_record_t;

/* Packet includes BOTH delimiters; max decoded body132, not v1's128.
 * These functions validate only the D1 envelope, NOT run qualification.
 * Failed calls leave destinations/length/chain unchanged. No overlap.
 * Chain starts0; includes preceding decoded header+payload, no record CRC. */
bool pil_gate1_dump_encode(const pil_gate1_record_t *record, uint8_t *out,
                          size_t capacity, size_t *length);
bool pil_gate1_dump_decode(const uint8_t *packet, size_t length,
                          pil_gate1_record_t *record);
bool pil_gate1_dump_chain(uint32_t *crc, const pil_gate1_record_t *record);
#endif

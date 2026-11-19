#ifndef KR1_ROW_PACK_H
#define KR1_ROW_PACK_H
/*
 * CR1 KR1 RAM row: private 15-byte storage for one unchanged 16-byte KR1 row.
 * RAM only. robot/log, live kr1, USB text and decoders keep the 16-byte row.
 *
 * Input (16 B, little-endian bit order, vehicle_log.h):
 *   time 0..15, state..echo-low 16..106, echo bits 14..15 at 107..108,
 *   servo/events/flag0 109..124, flags bits 1..3 at 125..127.
 * Stored (15 B = 120 bits):
 *   0..12   d = (time16 - index) mod 65536, d < 8192 (index-anchored time)
 *   13..103 input bits 16..106 (state, module, raw, position, speeds, echo 0..13)
 *   104..119 input bits 109..124 (servo, events, flag bit 0)
 * Decode: time16 = (index + d) mod 65536, the only value in
 * [index, index + 8191] with that offset. Echo bits 14..15 and flags bits 1..3
 * are written as 0; no producer sets them.
 *
 * kr1_row_pack refuses (returns false, writes nothing) when d >= 8192 or a
 * zero-only bit is set. The caller counts that as a lost row. A stored row
 * therefore always unpacks to the exact admitted 16 bytes.
 *
 * Straight-line 32-bit shifts only: no loop, no division, no kernel call.
 * Fixed byte order, so host and target give the same bytes.
 */
#include <stdbool.h>
#include <stdint.h>

#define KR1_ROW_BYTES 16u
#define KR1_ROW_STORED_BYTES 15u
#define KR1_ROW_TIME_WINDOW 8192u /* 13-bit index-anchored offset */
/* Word-3 bits that must be zero: echo 14..15 (input 107..108), flags 1..3
 * (input 125..127). */
#define KR1_ROW_ZERO_W3 0xE0001800u

static inline uint32_t kr1_row_get32(const uint8_t *b)
{
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
           ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

static inline void kr1_row_put32(uint8_t *b, uint32_t v)
{
    b[0] = (uint8_t)v;
    b[1] = (uint8_t)(v >> 8);
    b[2] = (uint8_t)(v >> 16);
    b[3] = (uint8_t)(v >> 24);
}

static inline bool kr1_row_pack(uint8_t out[KR1_ROW_STORED_BYTES],
                                const uint8_t in[KR1_ROW_BYTES],
                                uint32_t index)
{
    const uint32_t w0 = kr1_row_get32(in);
    const uint32_t w1 = kr1_row_get32(in + 4);
    const uint32_t w2 = kr1_row_get32(in + 8);
    const uint32_t w3 = kr1_row_get32(in + 12);
    const uint32_t d = (w0 - index) & 0xFFFFu;
    if (d >= KR1_ROW_TIME_WINDOW || (w3 & KR1_ROW_ZERO_W3) != 0u) {
        return false;
    }
    const uint32_t low = w3 & 0x7FFu; /* input bits 96..106 */
    const uint32_t o0 = d | ((w0 & 0xFFFF0000u) >> 3) | (w1 << 29);
    const uint32_t o1 = (w1 >> 3) | (w2 << 29);
    const uint32_t o2 = (w2 >> 3) | (low << 29);
    const uint32_t o3 = (low >> 3) | ((w3 & 0x1FFFE000u) >> 5);
    kr1_row_put32(out, o0);
    kr1_row_put32(out + 4, o1);
    kr1_row_put32(out + 8, o2);
    out[12] = (uint8_t)o3;
    out[13] = (uint8_t)(o3 >> 8);
    out[14] = (uint8_t)(o3 >> 16);
    return true;
}

static inline void kr1_row_unpack(uint8_t out[KR1_ROW_BYTES],
                                  const uint8_t in[KR1_ROW_STORED_BYTES],
                                  uint32_t index)
{
    const uint32_t o0 = kr1_row_get32(in);
    const uint32_t o1 = kr1_row_get32(in + 4);
    const uint32_t o2 = kr1_row_get32(in + 8);
    const uint32_t o3 = (uint32_t)in[12] | ((uint32_t)in[13] << 8) |
                        ((uint32_t)in[14] << 16);
    const uint32_t time16 = (index + (o0 & (KR1_ROW_TIME_WINDOW - 1u))) & 0xFFFFu;
    kr1_row_put32(out, time16 | ((o0 << 3) & 0xFFFF0000u));
    kr1_row_put32(out + 4, (o0 >> 29) | (o1 << 3));
    kr1_row_put32(out + 8, (o1 >> 29) | (o2 << 3));
    kr1_row_put32(out + 12, (((o2 >> 29) | (o3 << 3)) & 0x7FFu) |
                                ((o3 << 5) & 0x1FFFE000u));
}

#endif

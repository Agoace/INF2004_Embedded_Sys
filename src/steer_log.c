#include "steer_log.h"

/* Entry field positions and widths, in bits. */
#define POS_BIT 0u
#define LEFT_BIT 8u
#define RIGHT_BIT 16u
#define INFO_BIT 24u
#define RAW_BIT 32u /* three 12-bit fields */
#define RAW_WIDTH 12u
#define COUNT_L_BIT 68u
#define COUNT_R_BIT 78u
#define COUNT_WIDTH 10u
_Static_assert(COUNT_R_BIT + COUNT_WIDTH == SL_ENTRY_BYTES * 8u, "entry is 88 bits");

static void put_bits(uint8_t *buf, unsigned int pos, unsigned int width, uint32_t value) {
    for (unsigned int i = 0u; i < width; ++i) {
        const unsigned int bit = pos + i;
        const uint8_t mask = (uint8_t)(1u << (bit & 7u));
        if ((value >> i) & 1u) {
            buf[bit >> 3] = (uint8_t)(buf[bit >> 3] | mask);
        } else {
            buf[bit >> 3] = (uint8_t)(buf[bit >> 3] & (uint8_t)~mask);
        }
    }
}

static uint32_t get_bits(const uint8_t *buf, unsigned int pos, unsigned int width) {
    uint32_t value = 0u;
    for (unsigned int i = 0u; i < width; ++i) {
        const unsigned int bit = pos + i;
        if ((buf[bit >> 3] >> (bit & 7u)) & 1u) {
            value |= 1u << i;
        }
    }
    return value;
}

/* Two's complement field of `width` bits to a signed value. */
static int32_t sign_extend(uint32_t value, unsigned int width) {
    const uint32_t sign = 1u << (width - 1u);
    return (int32_t)(value ^ sign) - (int32_t)sign;
}

static uint32_t field_mask(unsigned int width) {
    return (1u << width) - 1u;
}

static uint32_t counts_field(int16_t counts) {
    const int32_t c = counts > SL_COUNTS_MAX    ? SL_COUNTS_MAX
                      : counts < -SL_COUNTS_MAX ? -SL_COUNTS_MAX
                                                : counts;
    return (uint32_t)c & field_mask(COUNT_WIDTH);
}

void sl_pack_entry(const sl_entry_t *e, uint8_t out[SL_ENTRY_BYTES]) {
    put_bits(out, POS_BIT, 8u, (uint8_t)e->position);
    put_bits(out, LEFT_BIT, 8u, (uint8_t)e->left_pct);
    put_bits(out, RIGHT_BIT, 8u, (uint8_t)e->right_pct);
    put_bits(out, INFO_BIT, 8u, e->info);
    for (unsigned int s = 0u; s < 3u; ++s) {
        put_bits(out, RAW_BIT + s * RAW_WIDTH, RAW_WIDTH, e->raw[s] & SL_RAW_MASK);
    }
    put_bits(out, COUNT_L_BIT, COUNT_WIDTH, counts_field(e->left_counts));
    put_bits(out, COUNT_R_BIT, COUNT_WIDTH, counts_field(e->right_counts));
}

void sl_unpack_entry(const uint8_t in[SL_ENTRY_BYTES], sl_entry_t *e) {
    e->position = (int8_t)sign_extend(get_bits(in, POS_BIT, 8u), 8u);
    e->left_pct = (int8_t)sign_extend(get_bits(in, LEFT_BIT, 8u), 8u);
    e->right_pct = (int8_t)sign_extend(get_bits(in, RIGHT_BIT, 8u), 8u);
    e->info = (uint8_t)get_bits(in, INFO_BIT, 8u);
    for (unsigned int s = 0u; s < 3u; ++s) {
        e->raw[s] = (uint16_t)get_bits(in, RAW_BIT + s * RAW_WIDTH, RAW_WIDTH);
    }
    e->left_counts = (int16_t)sign_extend(get_bits(in, COUNT_L_BIT, COUNT_WIDTH), COUNT_WIDTH);
    e->right_counts = (int16_t)sign_extend(get_bits(in, COUNT_R_BIT, COUNT_WIDTH), COUNT_WIDTH);
}

static void put_u16(uint8_t *out, uint16_t value) {
    out[0] = (uint8_t)(value & 0xFFu);
    out[1] = (uint8_t)(value >> 8);
}

size_t sl_write_header(uint8_t *out, unsigned int type, uint16_t boot_id, uint16_t seq) {
    out[0] = (uint8_t)((type & 0x0Fu) | (SL_VERSION << 4));
    put_u16(&out[1], boot_id);
    put_u16(&out[3], seq);
    return SL_HEADER_BYTES;
}

size_t sl_write_log_header(uint8_t *out, uint8_t run, uint8_t flags, uint16_t first,
                           uint8_t count) {
    out[0] = run;
    out[1] = flags;
    put_u16(&out[2], first);
    out[4] = count;
    return SL_LOG_HEADER_BYTES;
}

/* Bypass record field positions and widths, in bits. */
#define BP_TIME_BIT 0u
#define BP_TIME_WIDTH 13u
#define BP_SERVO_BIT 13u
#define BP_ANGLE_WIDTH 6u
#define BP_READING_BIT 19u
#define BP_READING_WIDTH 11u
#define BP_STATE_BIT 30u
#define BP_STATE_WIDTH 2u
#define BP_STEER_BIT 32u
#define BP_PHASE_BIT 38u
#define BP_PHASE_WIDTH 2u
#define BP_ANGLE_BIAS 30
_Static_assert(BP_PHASE_BIT + BP_PHASE_WIDTH == SL_BYPASS_BYTES * 8u, "bypass record is 40 bits");

static uint32_t angle_field(int8_t deg) {
    const int a = deg > BP_ANGLE_BIAS ? BP_ANGLE_BIAS : deg < -BP_ANGLE_BIAS ? -BP_ANGLE_BIAS : deg;
    return (uint32_t)(a + BP_ANGLE_BIAS);
}

void sl_pack_bypass(const sl_bypass_t *b, uint8_t out[SL_BYPASS_BYTES]) {
    const uint32_t t = b->time_10ms > SL_BYPASS_TIME_MAX ? SL_BYPASS_TIME_MAX : b->time_10ms;
    put_bits(out, BP_TIME_BIT, BP_TIME_WIDTH, t);
    put_bits(out, BP_SERVO_BIT, BP_ANGLE_WIDTH, angle_field(b->servo_deg));
    put_bits(out, BP_READING_BIT, BP_READING_WIDTH, b->reading & field_mask(BP_READING_WIDTH));
    put_bits(out, BP_STATE_BIT, BP_STATE_WIDTH, b->state & field_mask(BP_STATE_WIDTH));
    put_bits(out, BP_STEER_BIT, BP_ANGLE_WIDTH, angle_field(b->steer_deg));
    put_bits(out, BP_PHASE_BIT, BP_PHASE_WIDTH, b->phase & field_mask(BP_PHASE_WIDTH));
}

void sl_unpack_bypass(const uint8_t in[SL_BYPASS_BYTES], sl_bypass_t *b) {
    b->time_10ms = (uint16_t)get_bits(in, BP_TIME_BIT, BP_TIME_WIDTH);
    b->servo_deg = (int8_t)((int32_t)get_bits(in, BP_SERVO_BIT, BP_ANGLE_WIDTH) - BP_ANGLE_BIAS);
    b->reading = (uint16_t)get_bits(in, BP_READING_BIT, BP_READING_WIDTH);
    b->state = (uint8_t)get_bits(in, BP_STATE_BIT, BP_STATE_WIDTH);
    b->steer_deg = (int8_t)((int32_t)get_bits(in, BP_STEER_BIT, BP_ANGLE_WIDTH) - BP_ANGLE_BIAS);
    b->phase = (uint8_t)get_bits(in, BP_PHASE_BIT, BP_PHASE_WIDTH);
}

size_t sl_write_bypass_header(uint8_t *out, uint8_t run, uint16_t first, uint8_t count) {
    out[0] = run;
    put_u16(&out[1], first);
    out[3] = count;
    return SL_BYPASS_HEADER_BYTES;
}

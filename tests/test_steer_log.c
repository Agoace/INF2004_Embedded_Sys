/* Host test for src/steer_log.c: entry bit layout, signs, clamping, headers.
 *
 *   cc -std=c11 -Wall -Wextra -Wconversion -Isrc tests/test_steer_log.c \
 *      src/steer_log.c -o build/test_steer_log
 */
#include <stdio.h>
#include <string.h>

#include "steer_log.h"

static int failures;

#define CHECK(cond, ...)                                  \
    do {                                                  \
        if (!(cond)) {                                    \
            printf("FAIL line %d: ", __LINE__);           \
            printf(__VA_ARGS__);                          \
            printf("\n");                                 \
            ++failures;                                   \
        }                                                 \
    } while (0)

/* The same vector is decoded by scripts/steer_log_receiver.py --self-test. */
static const uint8_t VECTOR[SL_ENTRY_BYTES] = {0xDB, 0x0D, 0xEC, 0xA5, 0xB1, 0xC0,
                                               0x8F, 0xFF, 0x0F, 0xFF, 0x7F};

/* Bypass record: 12.34 s, servo -25, NO_RETURN, AROUND, steer +17, DRIVE. Also
 * decoded by the receiver self-test. */
static const uint8_t BYPASS_VECTOR[SL_BYPASS_BYTES] = {0xD2, 0xA4, 0x00, 0x70, 0x2F};

static int same(const sl_entry_t *a, const sl_entry_t *b) {
    return a->position == b->position && a->left_pct == b->left_pct &&
           a->right_pct == b->right_pct && a->info == b->info && a->raw[0] == b->raw[0] &&
           a->raw[1] == b->raw[1] && a->raw[2] == b->raw[2] &&
           a->left_counts == b->left_counts && a->right_counts == b->right_counts;
}

int main(void) {
    const sl_entry_t e = {
        .position = -37, .left_pct = 13, .right_pct = -20, .info = 0xA5u,
        .raw = {177u, 2300u, 4095u}, .left_counts = -16, .right_counts = 511,
    };
    uint8_t buf[SL_ENTRY_BYTES];
    memset(buf, 0xFF, sizeof buf); /* old content must be overwritten */
    sl_pack_entry(&e, buf);
    CHECK(memcmp(buf, VECTOR, sizeof buf) == 0, "packed bytes differ from the vector");

    sl_entry_t back;
    sl_unpack_entry(VECTOR, &back);
    CHECK(same(&e, &back), "vector decodes to the entry");

    /* Extremes round-trip. */
    const sl_entry_t ends[] = {
        {.position = -100, .left_pct = -100, .right_pct = 100, .info = 0u,
         .raw = {0u, 0u, 0u}, .left_counts = -511, .right_counts = -511},
        {.position = 100, .left_pct = 100, .right_pct = -100, .info = 0xFFu,
         .raw = {4095u, 1u, 2048u}, .left_counts = 511, .right_counts = 0},
        {.position = 0, .left_pct = 0, .right_pct = 0, .info = 0x5Au,
         .raw = {1u, 4094u, 2047u}, .left_counts = -1, .right_counts = 1},
    };
    for (unsigned int i = 0u; i < sizeof ends / sizeof ends[0]; ++i) {
        memset(buf, 0, sizeof buf);
        sl_pack_entry(&ends[i], buf);
        sl_unpack_entry(buf, &back);
        CHECK(same(&ends[i], &back), "round trip %u", i);
    }

    /* Counts are clamped to +/-511; raw keeps 12 bits only. */
    {
        const sl_entry_t big = {.raw = {0x1FFFu, 0u, 0u}, .left_counts = 900,
                                .right_counts = -900};
        sl_pack_entry(&big, buf);
        sl_unpack_entry(buf, &back);
        CHECK(back.left_counts == 511 && back.right_counts == -511, "clamp: %d %d",
              back.left_counts, back.right_counts);
        CHECK(back.raw[0] == 0x0FFFu && back.raw[1] == 0u, "raw 12 bits: %u %u",
              back.raw[0], back.raw[1]);
    }

    /* Bypass records. */
    {
        const sl_bypass_t b = {.time_10ms = 1234u, .servo_deg = -25, .reading = 0x0600u,
                               .state = 1u, .steer_deg = 17};
        uint8_t r[SL_BYPASS_BYTES];
        memset(r, 0xFF, sizeof r);
        sl_pack_bypass(&b, r);
        CHECK(memcmp(r, BYPASS_VECTOR, sizeof r) == 0,
              "bypass bytes %02X %02X %02X %02X %02X", r[0], r[1], r[2], r[3], r[4]);
        sl_bypass_t back_b;
        sl_unpack_bypass(BYPASS_VECTOR, &back_b);
        CHECK(back_b.time_10ms == 1234u && back_b.servo_deg == -25 &&
                  back_b.reading == 0x0600u && back_b.state == 1u && back_b.steer_deg == 17 &&
                  back_b.phase == 0u,
              "bypass vector decodes");

        /* Ends: clamps, the streak bits dropped. */
        const sl_bypass_t far = {.time_10ms = 9000u, .servo_deg = 31, .reading = 0x39FFu,
                                  .state = 3u, .steer_deg = -40, .phase = 6u};
        sl_pack_bypass(&far, r);
        sl_unpack_bypass(r, &back_b);
        CHECK(back_b.time_10ms == SL_BYPASS_TIME_MAX && back_b.servo_deg == 30 &&
                  back_b.reading == 0x01FFu && back_b.state == 3u && back_b.steer_deg == -30 &&
                  back_b.phase == 2u,
              "bypass ends: t %u servo %d reading %03X state %u steer %d phase %u",
              back_b.time_10ms, back_b.servo_deg, back_b.reading, back_b.state,
              back_b.steer_deg, back_b.phase);
        CHECK((r[4] & 0xC0u) == 0x80u, "bypass phase in bits 38-39");

        uint8_t h[SL_BYPASS_HEADER_BYTES];
        CHECK(sl_write_bypass_header(h, 9u, 0x0102u, 40u) == SL_BYPASS_HEADER_BYTES &&
                  h[0] == 9u && h[1] == 0x02u && h[2] == 0x01u && h[3] == 40u,
              "bypass header");
    }

    /* Headers. */
    {
        uint8_t h[SL_HEADER_BYTES + SL_LOG_HEADER_BYTES];
        CHECK(sl_write_header(h, SL_TYPE_LOG, 0xBEEFu, 0x1234u) == SL_HEADER_BYTES,
              "header size");
        CHECK(h[0] == (SL_TYPE_LOG | (SL_VERSION << 4)) && h[1] == 0xEFu && h[2] == 0xBEu &&
                  h[3] == 0x34u && h[4] == 0x12u,
              "header bytes %02X %02X %02X %02X %02X", h[0], h[1], h[2], h[3], h[4]);
        CHECK(sl_write_log_header(&h[5], 7u, SL_FLAG_REPLAY | SL_FLAG_REPLAY_END, 0x0190u,
                                  20u) == SL_LOG_HEADER_BYTES,
              "log header size");
        CHECK(h[5] == 7u && h[6] == 3u && h[7] == 0x90u && h[8] == 0x01u && h[9] == 20u,
              "log header bytes");
    }

    if (failures == 0) {
        printf("test_steer_log: all tests passed\n");
        return 0;
    }
    printf("test_steer_log: %d failure(s)\n", failures);
    return 1;
}

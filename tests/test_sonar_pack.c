/*
 * Host tests for sonar_pack.h: the 10-word sonar board and history layout.
 */
#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "../app_program/subsystem5/sonar_pack.h"

#define ANGLE_MIN (-30)
#define ANGLE_MAX 30

/* Field-by-field equality; padding bytes are not part of the value. */
static int same(const sonar_snapshot_t *a, const sonar_snapshot_t *b)
{
    return a->generation == b->generation && a->id == b->id &&
           a->ping_sequence == b->ping_sequence &&
           a->result_sequence == b->result_sequence &&
           a->trigger_ms == b->trigger_ms &&
           a->timestamp_ms == b->timestamp_ms &&
           a->deadline_ms == b->deadline_ms &&
           a->ping_generation == b->ping_generation &&
           a->ping_id == b->ping_id && a->ping_angle == b->ping_angle &&
           a->echo == b->echo && a->angle == b->angle &&
           a->status == b->status && a->pending == b->pending &&
           a->valid == b->valid;
}

static void round_trip(const sonar_snapshot_t *value)
{
    uint32_t words[SONAR_PACK_WORDS];
    sonar_snapshot_t back;

    memset(words, 0xA5, sizeof(words));
    memset(&back, 0x5A, sizeof(back));
    assert(sonar_pack(value, words));
    sonar_unpack(words, &back);
    assert(same(value, &back));
    /* Angle codes 61-63 are never stored. */
    assert(((words[SONAR_PACK_SMALL_WORD] >> SONAR_PACK_ANGLE_SHIFT) &
            SONAR_PACK_ANGLE_MASK) <= SONAR_PACK_ANGLE_CODE_MAX);
    assert(((words[SONAR_PACK_SMALL_WORD] >> SONAR_PACK_PING_ANGLE_SHIFT) &
            SONAR_PACK_ANGLE_MASK) <= SONAR_PACK_ANGLE_CODE_MAX);
}

static void test_size(void)
{
    assert(SONAR_PACK_WORDS == 10u);
    assert(SONAR_PACK_WORDS * sizeof(uint32_t) == 40u);
}

/* All angle, ping angle, status, pending and valid combinations. */
static void test_small_fields(void)
{
    static const ultrasonic_result_t echoes[] = {0u, 0x3FFFu, 0xFFFFu,
                                                 0x0200u};
    sonar_snapshot_t v;

    memset(&v, 0, sizeof(v));
    for (int a = ANGLE_MIN; a <= ANGLE_MAX; ++a) {
        for (int p = ANGLE_MIN; p <= ANGLE_MAX; ++p) {
            for (unsigned int s = 0u; s <= SONAR_CANCELLED; ++s) {
                for (unsigned int f = 0u; f < 4u; ++f) {
                    v.angle = (int8_t)a;
                    v.ping_angle = (int8_t)p;
                    v.status = (uint8_t)s;
                    v.pending = (f & 1u) != 0u;
                    v.valid = (f & 2u) != 0u;
                    v.echo = echoes[(unsigned int)(a + p + 60) % 4u];
                    round_trip(&v);
                }
            }
        }
    }
}

/* Every 16-bit echo word, with the other small fields at both ends. */
static void test_echo(void)
{
    sonar_snapshot_t v;

    memset(&v, 0, sizeof(v));
    for (uint32_t e = 0u; e <= UINT16_MAX; ++e) {
        v.echo = (ultrasonic_result_t)e;
        v.angle = (e & 1u) ? ANGLE_MAX : ANGLE_MIN;
        v.ping_angle = (e & 1u) ? ANGLE_MIN : ANGLE_MAX;
        v.status = (uint8_t)(e & 3u);
        v.pending = (e & 4u) != 0u;
        v.valid = (e & 8u) != 0u;
        round_trip(&v);
    }
}

/* Every u32 field at the ends and at the wrap, each in its own word. */
static void test_wide_fields(void)
{
    static const uint32_t ends[] = {0u, 1u, 0x7FFFFFFFu, 0x80000000u,
                                    UINT32_MAX - 1u, UINT32_MAX};
    sonar_snapshot_t v;

    for (unsigned int i = 0u; i < sizeof(ends) / sizeof(ends[0]); ++i) {
        for (unsigned int field = 0u; field < 9u; ++field) {
            memset(&v, 0, sizeof(v));
            /* Distinct values elsewhere catch swapped words. */
            v.generation = 0x01010101u;
            v.id = 0x02020202u;
            v.ping_sequence = 0x03030303u;
            v.result_sequence = 0x04040404u;
            v.trigger_ms = 0x05050505u;
            v.timestamp_ms = 0x06060606u;
            v.deadline_ms = 0x07070707u;
            v.ping_generation = 0x08080808u;
            v.ping_id = 0x09090909u;
            uint32_t *const fields[] = {
                &v.generation, &v.id, &v.ping_sequence, &v.result_sequence,
                &v.trigger_ms, &v.timestamp_ms, &v.deadline_ms,
                &v.ping_generation, &v.ping_id};
            *fields[field] = ends[i];
            round_trip(&v);
        }
    }
}

/* A zero snapshot packs to all-zero words except the angle offsets. */
static void test_zero_layout(void)
{
    sonar_snapshot_t v;
    uint32_t words[SONAR_PACK_WORDS];

    memset(&v, 0, sizeof(v));
    assert(sonar_pack(&v, words));
    for (unsigned int i = 0u; i < SONAR_PACK_SMALL_WORD; ++i) {
        assert(words[i] == 0u);
    }
    assert(words[SONAR_PACK_SMALL_WORD] ==
           ((30u << SONAR_PACK_ANGLE_SHIFT) |
            (30u << SONAR_PACK_PING_ANGLE_SHIFT)));
}

/* Out-of-range input: false, and the words are not changed. */
static void test_reject(void)
{
    static const int8_t bad_angles[] = {-128, -31, 31, 127};
    uint32_t words[SONAR_PACK_WORDS];
    uint32_t before[SONAR_PACK_WORDS];
    sonar_snapshot_t v;

    memset(before, 0xC3, sizeof(before));
    for (unsigned int i = 0u; i < 4u; ++i) {
        memset(&v, 0, sizeof(v));
        v.angle = bad_angles[i];
        memcpy(words, before, sizeof(words));
        assert(!sonar_pack(&v, words));
        assert(memcmp(words, before, sizeof(words)) == 0);
        memset(&v, 0, sizeof(v));
        v.ping_angle = bad_angles[i];
        assert(!sonar_pack(&v, words));
        assert(memcmp(words, before, sizeof(words)) == 0);
    }
    for (unsigned int s = SONAR_CANCELLED + 1u; s <= UINT8_MAX; ++s) {
        memset(&v, 0, sizeof(v));
        v.status = (uint8_t)s;
        assert(!sonar_pack(&v, words));
        assert(memcmp(words, before, sizeof(words)) == 0);
    }
    assert(!sonar_pack(NULL, words));
}

int main(void)
{
    test_size();
    test_small_fields();
    test_echo();
    test_wide_fields();
    test_zero_layout();
    test_reject();
    return 0;
}

/* Compact robot board: exact ranges, local decode and torn publication. */
#include <assert.h>
#include <limits.h>
#include <string.h>
#include "../app_program/subsystem2/motion_pack.h"
#include "../app_program/subsystem2/motor_ctl_robot.h"

static int interrupt_at = -1;
static uint32_t *shared_words;
static motion_packed_t replacement;
static void read_hook(const uint32_t *sequence, unsigned int index);
#define APP_BOARD_READ_HOOK(sequence, index) read_hook(sequence, index)
#include "../app_program/platform/app_board.h"

static void read_hook(const uint32_t *sequence, unsigned int index)
{
    if (interrupt_at >= 0 && index == (unsigned int)interrupt_at) {
        interrupt_at = -1;
        app_board_publish((uint32_t *)sequence, shared_words, &replacement,
                          sizeof(replacement));
    }
}

static uint16_t pulse(int angle)
{
    const int offset = angle >= 0 ? (angle * 111 + 5) / 10
                                 : -((-angle * 111 + 5) / 10);
    return (uint16_t)(1500 + offset);
}

static motion_status_t sample(void)
{
    const motion_status_t v = {
        .counts = {INT32_MIN, INT32_MAX},
        .travel_mm = {INT32_MAX, INT32_MIN},
        .speed_mm_s = {INT16_MIN, INT16_MAX},
        .duty_permille = {-800, 800},
        .heading_cdeg = INT32_MIN,
        .servo_us = 1500u,
        .servo_deg = 0,
        .servo_settled = true,
        .faults = 15u,
        .ticks = UINT32_MAX,
        .tick_max_us = 0u,
        .gap_min_us = UINT32_MAX,
        .gap_max_us = 0u,
        .late_wakes = UINT32_MAX,
        .missed_wakes = 0u,
        .watchdog_armed = true
    };
    return v;
}

/* Compare members, not C struct padding bytes. */
static void equal_status(const motion_status_t *a, const motion_status_t *b)
{
    for (unsigned int i = 0u; i < MC_WHEELS; ++i) {
        assert(a->counts[i] == b->counts[i]);
        assert(a->travel_mm[i] == b->travel_mm[i]);
        assert(a->speed_mm_s[i] == b->speed_mm_s[i]);
        assert(a->duty_permille[i] == b->duty_permille[i]);
    }
    assert(a->heading_cdeg == b->heading_cdeg);
    assert(a->servo_us == b->servo_us && a->servo_deg == b->servo_deg);
    assert(a->servo_settled == b->servo_settled && a->faults == b->faults);
    assert(a->ticks == b->ticks && a->tick_max_us == b->tick_max_us);
    assert(a->gap_min_us == b->gap_min_us);
    assert(a->gap_max_us == b->gap_max_us);
    assert(a->late_wakes == b->late_wakes);
    assert(a->missed_wakes == b->missed_wakes);
    assert(a->watchdog_armed == b->watchdog_armed);
}

static void round_trip(const motion_status_t *v, uint32_t time)
{
    motion_packed_t packed;
    memset(&packed, 0xa5, sizeof(packed));
    assert(motion_pack(&packed, v, time));
    assert((packed.words[13] & UINT32_C(0xe0000000)) == 0u);
    assert((packed.words[14] & UINT32_C(0xffffe000)) == 0u);
    motion_status_t result;
    uint32_t us = ~time;
    assert(motion_unpack(&result, &us, &packed));
    equal_status(&result, v);
    assert(us == time);
}

static void ranges(void)
{
    const mc_config_t cfg = MC_ROBOT_CONFIG;
    assert(cfg.max_permille == 800u && cfg.servo_centre_us == 1500u);
    assert(cfg.servo_min_us == 1100u && cfg.servo_max_us == 1900u);
    assert(cfg.servo_us_per_deg_x10 == 111u);
    assert(sizeof(motion_packed_t) == 60u);
    motion_status_t v = sample();
    round_trip(&v, UINT32_MAX);
    v.counts[0] = INT32_MAX;
    v.counts[1] = INT32_MIN;
    v.travel_mm[0] = INT32_MIN;
    v.travel_mm[1] = INT32_MAX;
    v.heading_cdeg = INT32_MAX;
    v.ticks = 0u;
    v.tick_max_us = UINT32_MAX;
    v.gap_min_us = 0u;
    v.gap_max_us = UINT32_MAX;
    v.late_wakes = 0u;
    v.missed_wakes = UINT32_MAX;
    round_trip(&v, 0u);
    for (int duty = -800; duty <= 800; ++duty) {
        v.duty_permille[0] = (int16_t)duty;
        v.duty_permille[1] = (int16_t)-duty;
        round_trip(&v, (uint32_t)(duty + 800));
    }
    for (int angle = -36; angle <= 36; ++angle) {
        v.servo_deg = (int8_t)angle;
        /* Deliberately independent pulse and angle fields. */
        v.servo_us = pulse(-angle);
        for (unsigned int flags = 0u; flags < 64u; ++flags) {
            v.faults = (uint8_t)(flags & 15u);
            v.servo_settled = (flags & 16u) != 0u;
            v.watchdog_armed = (flags & 32u) != 0u;
            round_trip(&v, UINT32_MAX - flags);
        }
    }
    for (int speed = INT16_MIN; speed <= INT16_MAX; ++speed) {
        v.speed_mm_s[0] = (int16_t)speed;
        v.speed_mm_s[1] = (int16_t)(-1 - speed);
        round_trip(&v, (uint32_t)(speed + 32768));
    }
    v.counts[0] = -1;
    v.counts[1] = 0;
    v.travel_mm[0] = 1;
    v.travel_mm[1] = -1;
    v.heading_cdeg = 0;
    round_trip(&v, 1u);
}

static void invalid(void)
{
    motion_status_t v = sample();
    motion_packed_t original, output;
    assert(motion_pack(&original, &v, 123u));
    output = original;
    assert(!motion_pack(NULL, &v, 0u));
    assert(!motion_pack(&output, NULL, 0u));
    for (unsigned int field = 0u; field < 8u; ++field) {
        v = sample();
        switch (field) {
        case 0u: v.duty_permille[0] = -801; break;
        case 1u: v.duty_permille[1] = 801; break;
        case 2u: v.servo_deg = -37; break;
        case 3u: v.servo_deg = 37; break;
        case 4u: v.servo_us = 0u; break;
        case 5u: v.servo_us = UINT16_MAX; break;
        case 6u: v.servo_us = 1501u; break;
        default: v.faults = 16u; break;
        }
        assert(!motion_pack(&output, &v, 0u));
        assert(memcmp(&output, &original, sizeof(output)) == 0);
    }
    const uint32_t bad13[] = {
        UINT32_C(1) << 29u, 1601u, 1601u << 11u, 73u << 22u
    };
    const uint32_t bad14[] = {UINT32_C(1) << 13u, 73u};
    v = sample();
    uint32_t us = 456u;
    assert(!motion_unpack(NULL, &us, &original));
    assert(!motion_unpack(&v, NULL, &original));
    assert(!motion_unpack(&v, &us, NULL));
    for (unsigned int i = 0u; i < 6u; ++i) {
        output = original;
        if (i < 4u) {
            output.words[13] = bad13[i];
        } else {
            output.words[14] = bad14[i - 4u];
        }
        const motion_status_t saved = v;
        assert(!motion_unpack(&v, &us, &output));
        equal_status(&v, &saved);
        assert(us == 456u);
    }
}

static void torn_reads(void)
{
    APP_BOARD(motion_packed_t) board = {0};
    assert(sizeof(board) == 64u);
    shared_words = board.words;
    motion_status_t input = sample();
    motion_packed_t first, scratch;
    assert(motion_pack(&first, &input, UINT32_MAX));
    input.counts[0] = 456;
    assert(motion_pack(&replacement, &input, 0u));
    for (int i = 0; i < (int)MOTION_PACK_WORDS; ++i) {
        app_board_publish(&board.sequence, board.words, &first, sizeof(first));
        motion_status_t cached = sample();
        const motion_status_t saved = cached;
        uint32_t us = 123u;
        interrupt_at = i;
        const bool stable = app_board_try_read(&board.sequence, board.words,
                                               &scratch, sizeof(scratch));
        if (stable) {
            assert(motion_unpack(&cached, &us, &scratch));
        }
        assert(!stable && interrupt_at == -1);
        equal_status(&cached, &saved);
        assert(us == 123u);
        assert(app_board_try_read(&board.sequence, board.words, &scratch,
                                  sizeof(scratch)));
        assert(motion_unpack(&cached, &us, &scratch));
        equal_status(&cached, &input);
        assert(us == 0u);
    }
}

int main(void)
{
    ranges();
    invalid();
    torn_reads();
    return 0;
}

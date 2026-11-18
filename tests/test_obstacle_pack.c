/*
 * Host tests for obstacle_pack.h: the 6-word obstacle board layout.
 */
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <stdbool.h>

static const uint32_t *tear_sequence;
static unsigned int tear_index;
static unsigned int tear_count;
void obstacle_pack_test_read_hook(const uint32_t *sequence, unsigned int index)
{
    if (sequence == tear_sequence && index == tear_index) {
        /* A whole writer publication between two word loads. */
        __atomic_store_n((uint32_t *)sequence,
                         __atomic_load_n(sequence, __ATOMIC_SEQ_CST) + 2u,
                         __ATOMIC_SEQ_CST);
        tear_sequence = NULL;
        ++tear_count;
    }
}
#define APP_BOARD_READ_HOOK(s, i) obstacle_pack_test_read_hook((s), (i))
#include "../app_program/vehicle/obstacle_pack.h"
#include "../app_program/vehicle/obstacle_report.h"

static const uint8_t deg_codes[] = {0u, 5u, 10u, 15u, 20u, 25u, 30u, 255u};

static int same(const rm_obstacle_t *a, const rm_obstacle_t *b)
{
    return a->flags == b->flags && a->decision == b->decision &&
           a->closest_angle_deg == b->closest_angle_deg &&
           a->scan_id == b->scan_id && a->uptime_ms == b->uptime_ms &&
           a->closest_mm == b->closest_mm &&
           a->profile_word == b->profile_word &&
           a->right_free_step == b->right_free_step &&
           a->left_free_step == b->left_free_step &&
           a->right_clear_deg == b->right_clear_deg &&
           a->left_clear_deg == b->left_clear_deg &&
           a->boot_id == b->boot_id && a->width_mm == b->width_mm &&
           a->left_clear_mm == b->left_clear_mm &&
           a->right_clear_mm == b->right_clear_mm &&
           a->geometry_flags == b->geometry_flags;
}

static void round_trip(const rm_obstacle_t *value)
{
    uint32_t words[OBSTACLE_PACK_WORDS];
    rm_obstacle_t back;

    memset(words, 0xA5, sizeof(words));
    memset(&back, 0x5A, sizeof(back));
    assert(obstacle_pack(value, words));
    /* Unused bits are zero. */
    assert((words[3] & ~OBSTACLE_PACK_W3_USED) == 0u);
    assert((words[4] & ~OBSTACLE_PACK_W4_USED) == 0u);
    assert((words[5] & ~OBSTACLE_PACK_W5_USED) == 0u);
    obstacle_unpack(words, &back);
    assert(same(value, &back));
}

static rm_obstacle_t base(void)
{
    rm_obstacle_t v;

    memset(&v, 0, sizeof(v));
    v.scan_id = 0x01020304u;
    v.uptime_ms = 0x05060708u;
    v.boot_id = 0x090A0B0Cu;
    v.width_mm = RM_OBSTACLE_GEOMETRY_NONE;
    v.left_clear_mm = RM_OBSTACLE_GEOMETRY_NONE;
    v.right_clear_mm = RM_OBSTACLE_GEOMETRY_NONE;
    v.right_clear_deg = 255u;
    v.left_clear_deg = 255u;
    return v;
}

static void test_size(void)
{
    assert(OBSTACLE_PACK_WORDS == 6u);
    assert(sizeof(obstacle_board_t) == 28u);
}

/* Every value of every small field; the others stay at their base. */
static void test_small_fields(void)
{
    rm_obstacle_t v;

    for (unsigned int x = 0u; x <= UINT8_MAX; ++x) {
        v = base(); v.flags = (uint8_t)x; round_trip(&v);
    }
    for (unsigned int x = 0u; x <= 2u; ++x) {
        v = base(); v.decision = (uint8_t)x; round_trip(&v);
    }
    for (int x = -30; x <= 30; ++x) {
        v = base(); v.closest_angle_deg = (int8_t)x; round_trip(&v);
    }
    for (unsigned int x = 0u; x <= 511u; ++x) {
        v = base(); v.closest_mm = (uint16_t)x; round_trip(&v);
    }
    for (unsigned int x = 0u; x <= 0x7FFFu; ++x) {
        v = base(); v.profile_word = (uint16_t)x; round_trip(&v);
    }
    for (unsigned int r = 0u; r <= 7u; ++r) {
        for (unsigned int l = 0u; l <= 7u; ++l) {
            for (unsigned int d = 0u; d < 64u; ++d) {
                v = base();
                v.right_free_step = (uint8_t)r;
                v.left_free_step = (uint8_t)l;
                v.right_clear_deg = deg_codes[d & 7u];
                v.left_clear_deg = deg_codes[d >> 3u];
                round_trip(&v);
            }
        }
    }
    for (unsigned int x = 0u; x <= 0x1Fu; ++x) {
        v = base(); v.geometry_flags = (uint8_t)x; round_trip(&v);
    }
    for (unsigned int x = 0u; x <= OBSTACLE_PACK_GEOMETRY_MAX; ++x) {
        v = base(); v.width_mm = (uint16_t)x; round_trip(&v);
        v = base(); v.left_clear_mm = (uint16_t)x; round_trip(&v);
        v = base(); v.right_clear_mm = (uint16_t)x; round_trip(&v);
    }
}

static void test_wide_fields(void)
{
    static const uint32_t ends[] = {0u, 1u, 0x7FFFFFFFu, 0x80000000u,
                                    UINT32_MAX};
    rm_obstacle_t v;

    for (unsigned int i = 0u; i < sizeof(ends) / sizeof(ends[0]); ++i) {
        v = base(); v.scan_id = ends[i]; round_trip(&v);
        v = base(); v.uptime_ms = ends[i]; round_trip(&v);
        v = base(); v.boot_id = ends[i]; round_trip(&v);
    }
    /* All small fields at their maximum together. */
    v = base();
    v.flags = UINT8_MAX; v.decision = 2u; v.closest_angle_deg = 30;
    v.closest_mm = 511u; v.profile_word = 0x7FFFu;
    v.right_free_step = 7u; v.left_free_step = 7u;
    v.geometry_flags = 0x1Fu; v.width_mm = OBSTACLE_PACK_GEOMETRY_MAX;
    v.left_clear_mm = OBSTACLE_PACK_GEOMETRY_MAX;
    v.right_clear_mm = OBSTACLE_PACK_GEOMETRY_MAX;
    round_trip(&v);
}

static void expect_reject(const rm_obstacle_t *v)
{
    uint32_t words[OBSTACLE_PACK_WORDS];
    uint32_t before[OBSTACLE_PACK_WORDS];

    memset(before, 0xC3, sizeof(before));
    memcpy(words, before, sizeof(words));
    assert(!obstacle_pack(v, words));
    assert(memcmp(words, before, sizeof(words)) == 0);
}

static void test_reject(void)
{
    rm_obstacle_t v;

    v = base(); v.decision = 3u; expect_reject(&v);
    v = base(); v.closest_angle_deg = -31; expect_reject(&v);
    v = base(); v.closest_angle_deg = 31; expect_reject(&v);
    v = base(); v.closest_mm = 512u; expect_reject(&v);
    v = base(); v.profile_word = 0x8000u; expect_reject(&v);
    v = base(); v.right_free_step = 8u; expect_reject(&v);
    v = base(); v.left_free_step = 8u; expect_reject(&v);
    v = base(); v.right_clear_deg = 3u; expect_reject(&v);
    v = base(); v.left_clear_deg = 35u; expect_reject(&v);
    v = base(); v.geometry_flags = 0x20u; expect_reject(&v);
    v = base(); v.width_mm = OBSTACLE_PACK_GEOMETRY_MAX + 1u;
    expect_reject(&v);
    v = base(); v.left_clear_mm = UINT16_MAX - 1u; expect_reject(&v);
    v = base(); v.right_clear_mm = 2000u; expect_reject(&v);
    assert(!obstacle_pack(NULL, (uint32_t[OBSTACLE_PACK_WORDS]){0}));
}

/* Reports made from real profiles always pack: geometry range sweep. */
static void check_report(const obstacle_profile_t *profile, uint32_t id)
{
    const obstacle_profile_word_t word = obstacle_profile_build(profile);
    const obstacle_side_t side = obstacle_profile_side(word, profile);
    rm_obstacle_t report;

    memset(&report, 0, sizeof(report));
    obstacle_report_make(profile, word, side, id, id * 7u, &report);
    assert(report.width_mm <= OBSTACLE_PACK_GEOMETRY_MAX ||
           report.width_mm == RM_OBSTACLE_GEOMETRY_NONE);
    round_trip(&report);
}

static obstacle_scan_point_t point(unsigned int status, unsigned int mm)
{
    return (obstacle_scan_point_t)((status << ULTRASONIC_STATUS_SHIFT) |
                                   (mm & ULTRASONIC_DISTANCE_MASK));
}

static void test_report_range(void)
{
    obstacle_profile_t profile;
    uint32_t id = 1u;

    /* Extremes: every edge pair at 511 mm, open outside. */
    for (unsigned int first = 0u; first < OBSTACLE_SCAN_POINTS; ++first) {
        for (unsigned int last = first; last < OBSTACLE_SCAN_POINTS; ++last) {
            for (unsigned int i = 0u; i < OBSTACLE_SCAN_POINTS; ++i) {
                profile.point[i] =
                    (i >= first && i <= last)
                        ? point(ULTRASONIC_STATUS_VALID, 511u)
                        : point(ULTRASONIC_STATUS_NO_RETURN, 0u);
            }
            check_report(&profile, id++);
            profile.point[first] = point(ULTRASONIC_STATUS_VALID, 361u);
            check_report(&profile, id++);
        }
    }
    /* Random profiles over all statuses and distances. */
    srand(7350u);
    for (unsigned int n = 0u; n < 200000u; ++n) {
        for (unsigned int i = 0u; i < OBSTACLE_SCAN_POINTS; ++i) {
            const unsigned int r = (unsigned int)rand();
            profile.point[i] = point(r & 3u, (r >> 2u) % 512u);
        }
        check_report(&profile, id++);
    }
}

/* Board: torn read after each payload word, then a clean read. */
static void test_board(void)
{
    obstacle_board_t board;
    rm_obstacle_t v = base(), out;

    memset(&board, 0, sizeof(board));
    assert(!obstacle_board_try_read(&board, &out));
    assert(board.sequence == 0u);
    for (unsigned int k = 0u; k < OBSTACLE_PACK_WORDS; ++k) {
        v.scan_id = 100u + k;
        v.closest_mm = (uint16_t)(k * 50u);
        assert(obstacle_board_publish(&board, &v));
        tear_sequence = &board.sequence;
        tear_index = k;
        memset(&out, 0x77, sizeof(out));
        const rm_obstacle_t saved = out;
        assert(!obstacle_board_try_read(&board, &out));
        assert(tear_count == k + 1u);
        assert(memcmp(&out, &saved, sizeof(out)) == 0);
        assert(obstacle_board_try_read(&board, &out) && same(&out, &v));
    }
    /* Rejected input leaves the board unchanged. */
    const uint32_t sequence = board.sequence;
    rm_obstacle_t bad = v;
    bad.decision = 3u;
    assert(!obstacle_board_publish(&board, &bad));
    assert(board.sequence == sequence);
    assert(obstacle_board_try_read(&board, &out) && same(&out, &v));
}

int main(void)
{
    test_size();
    test_small_fields();
    test_wide_fields();
    test_reject();
    test_report_range();
    test_board();
    return 0;
}

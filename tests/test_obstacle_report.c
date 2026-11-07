/*
 * Host tests for obstacle_report.c: scan points and PROFILE word to report.
 */
#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "../app_program/vehicle/obstacle_report.h"

#define POINT_COUNT OBSTACLE_SCAN_POINTS
#define NO_RETURN_POINT                                                        \
    ((obstacle_scan_point_t)(ULTRASONIC_STATUS_NO_RETURN                       \
                             << ULTRASONIC_STATUS_SHIFT))

/* A VALID point at distance mm. */
static obstacle_scan_point_t valid_point(unsigned int mm)
{
    return (obstacle_scan_point_t)mm;
}

/* A profile where every point sees past the obstacle. */
static void open_profile(obstacle_profile_t *profile)
{
    for (unsigned int i = 0u; i < POINT_COUNT; ++i) {
        profile->point[i] = NO_RETURN_POINT;
    }
}

static void test_closest_index(void)
{
    obstacle_profile_t profile;

    open_profile(&profile);
    assert(obstacle_closest_index(&profile) == OBSTACLE_REPORT_NO_POINT);
    profile.point[9] = valid_point(250u);
    assert(obstacle_closest_index(&profile) == 9u);
    profile.point[2] = valid_point(250u); /* equal: nearer to centre wins */
    assert(obstacle_closest_index(&profile) == 9u);
    profile.point[4] = valid_point(250u);
    assert(obstacle_closest_index(&profile) == 4u);
    profile.point[8] = valid_point(250u); /* 2 and 2 steps: right wins */
    assert(obstacle_closest_index(&profile) == 4u);
    profile.point[0] = valid_point(100u); /* nearer value wins over centre */
    assert(obstacle_closest_index(&profile) == 0u);
    profile.point[0] = valid_point(ULTRASONIC_DISTANCE_MAX_MM); /* not real */
    assert(obstacle_closest_index(&profile) == 4u);
    profile.point[12] = valid_point(0u); /* TOO_NEAR is VALID */
    assert(obstacle_closest_index(&profile) == 12u);
}

static void test_make_left_detour(void)
{
    obstacle_profile_t profile;
    rm_obstacle_t report;
    obstacle_profile_word_t word;
    obstacle_side_t side;

    open_profile(&profile);
    profile.point[6] = valid_point(200u);
    profile.point[5] = valid_point(210u);
    profile.point[7] = valid_point(205u);
    word = obstacle_profile_build(&profile);
    side = obstacle_profile_side(word, &profile);
    obstacle_report_make(&profile, word, side, 3u, 4567u, &report);
    assert(report.scan_id == 3u && report.uptime_ms == 4567u);
    assert(report.closest_mm == 200u && report.closest_angle_deg == 0);
    assert(report.flags == RM_OBSTACLE_CLOSEST_VALID);
    assert(report.profile_word == word && report.decision == (uint8_t)side);
    assert(report.right_free_step == obstacle_profile_right(word));
    assert(report.left_free_step == obstacle_profile_left(word));
    assert(report.right_free_step == 2u && report.left_free_step == 2u);
    assert(report.right_clear_deg == 10u && report.left_clear_deg == 10u);
    assert(report.decision == (uint8_t)OBSTACLE_SIDE_LEFT); /* full tie: left */
}

static void test_make_blocked(void)
{
    obstacle_profile_t profile;
    rm_obstacle_t report;
    obstacle_profile_word_t word;

    for (unsigned int i = 0u; i < POINT_COUNT; ++i) {
        profile.point[i] = valid_point(100u); /* a wall: nothing is free */
    }
    profile.point[1] = valid_point(0u); /* too near, at 25 degrees right */
    word = obstacle_profile_build(&profile);
    obstacle_report_make(&profile, word, obstacle_profile_side(word, &profile),
                         9u, 1u, &report);
    assert(report.flags == (RM_OBSTACLE_CLOSEST_VALID | RM_OBSTACLE_TOO_NEAR |
                            RM_OBSTACLE_LEFT_BLOCKED |
                            RM_OBSTACLE_RIGHT_BLOCKED));
    assert(report.closest_mm == 0u && report.closest_angle_deg == -25);
    assert(report.right_clear_deg == OBSTACLE_REPORT_BLOCKED_DEG);
    assert(report.left_clear_deg == OBSTACLE_REPORT_BLOCKED_DEG);
    assert(report.decision == (uint8_t)OBSTACLE_SIDE_NONE);
}

static void test_make_nothing_seen(void)
{
    obstacle_profile_t profile;
    rm_obstacle_t report;
    obstacle_profile_word_t word;

    open_profile(&profile);
    word = obstacle_profile_build(&profile);
    obstacle_report_make(&profile, word, obstacle_profile_side(word, &profile),
                         1u, 2u, &report);
    assert(report.flags == 0u && report.closest_mm == 511u);
    assert(report.closest_angle_deg == 0);
    assert(report.right_free_step == 0u && report.left_free_step == 0u);
    assert(report.right_clear_deg == 0u && report.left_clear_deg == 0u);
}

/* Every word the profile builder can make must report matching fields. */
static void test_exhaustive_words(void)
{
    obstacle_profile_t profile;
    rm_obstacle_t report;

    open_profile(&profile);
    for (unsigned int word = 0u; word <= 0x7fffu; ++word) {
        obstacle_report_make(&profile, (obstacle_profile_word_t)word,
                             OBSTACLE_SIDE_RIGHT, 1u, 0u, &report);
        assert(report.closest_mm == (word & 0x1ffu));
        assert(report.right_free_step == ((word >> 9) & 7u));
        assert(report.left_free_step == ((word >> 12) & 7u));
        assert(((report.flags & RM_OBSTACLE_RIGHT_BLOCKED) != 0u) ==
               (report.right_free_step == 7u));
        assert(((report.flags & RM_OBSTACLE_LEFT_BLOCKED) != 0u) ==
               (report.left_free_step == 7u));
        assert((report.right_free_step == 7u) ==
               (report.right_clear_deg == 255u));
        assert((report.right_free_step == 7u) ||
               (report.right_clear_deg == report.right_free_step * 5u));
    }
}

int main(void)
{
    test_closest_index();
    test_make_left_detour();
    test_make_blocked();
    test_make_nothing_seen();
    test_exhaustive_words();
    return 0;
}

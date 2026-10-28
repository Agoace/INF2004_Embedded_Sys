/*
 * Host unit test for src/obstacle_profile.c. It needs no Pico hardware.
 *
 *   cc -std=c11 -Wall -Wextra -Wconversion -Isrc \
 *      tests/test_obstacle_profile.c src/obstacle_profile.c \
 *      -o build/test_obstacle_profile
 *   ./build/test_obstacle_profile
 */
#include <stdio.h>

#include "obstacle_profile.h"

static int failures;

/* Profile values: a distance in mm, or one of these markers. */
#define NR -1     /* NO_RETURN */
#define ND -2     /* no data */
#define TN 0      /* TOO_NEAR */

static obstacle_profile_t make(const int v[OBSTACLE_SCAN_POINTS]) {
    obstacle_profile_t p;
    for (unsigned int i = 0; i < OBSTACLE_SCAN_POINTS; ++i) {
        if (v[i] == NR) {
            p.point[i] = (obstacle_scan_point_t)(ULTRASONIC_STATUS_NO_RETURN
                                                 << ULTRASONIC_STATUS_SHIFT);
        } else if (v[i] == ND) {
            p.point[i] = OBSTACLE_SCAN_POINT_EMPTY;
        } else {
            p.point[i] = (obstacle_scan_point_t)v[i];
        }
    }
    return p;
}

static void expect(const char *name, const int v[OBSTACLE_SCAN_POINTS],
                   unsigned int distance, unsigned int right,
                   unsigned int left, obstacle_side_t side) {
    const obstacle_profile_t p = make(v);
    const obstacle_profile_word_t w = obstacle_profile_build(&p);
    const obstacle_side_t got_side = obstacle_profile_side(w, &p);

    if (obstacle_profile_distance(w) != distance ||
        obstacle_profile_right(w) != right || obstacle_profile_left(w) != left ||
        got_side != side || (w & 0x8000u) != 0u) {
        printf("FAIL %s: word 0x%04X -> distance %u right %u left %u side %d; "
               "want %u %u %u %d\n",
               name, w, obstacle_profile_distance(w), obstacle_profile_right(w),
               obstacle_profile_left(w), got_side, distance, right, left, side);
        ++failures;
    }
}

#define B OBSTACLE_PROFILE_BLOCKED
#define RIGHT OBSTACLE_SIDE_RIGHT
#define LEFT OBSTACLE_SIDE_LEFT
#define NONE OBSTACLE_SIDE_NONE

int main(void) {
    /*                      idx: 0    1    2    3    4    5    6    7    8    9   10   11   12
     *                      deg: 30R  25R  20R  15R  10R  5R   0    5L  10L  15L  20L  25L  30L */

    /* Bench run 1 (2026-10-27): box ahead at about 165 mm. */
    expect("run 1", (const int[]){NR, 187, 175, 166, 166, 162, 166, 167, 171, 183, 196, NR, NR},
           162u, 6u, 5u, LEFT);

    expect("open space", (const int[]){NR, NR, NR, NR, NR, NR, NR, NR, NR, NR, NR, NR, NR},
           511u, 0u, 0u, LEFT);

    expect("obstacle on the right", (const int[]){250, 250, 255, 260, 270, 280, NR, NR, NR, NR, NR, NR, NR},
           250u, B, 0u, LEFT);

    expect("obstacle on the left", (const int[]){NR, NR, NR, NR, 300, 270, 260, 255, 250, 250, 250, 250, 250},
           250u, 3u, B, RIGHT);

    expect("wall: blocked both sides", (const int[]){300, 290, 280, 270, 265, 260, 258, 260, 265, 270, 280, 290, 300},
           258u, B, B, NONE);

    expect("gap behind the edge counts as free", (const int[]){480, 470, 260, 258, 258, 258, 258, 258, 258, 258, 260, 270, 280},
           258u, 5u, B, RIGHT);

    expect("one stray NO_RETURN is not an edge", (const int[]){300, 290, 280, 270, NR, 260, 258, 260, 265, 270, 280, 290, 300},
           258u, B, B, NONE);

    expect("outermost point may be free alone", (const int[]){NR, 290, 280, 270, 265, 260, 258, 260, 265, 270, 280, 290, 300},
           258u, 6u, B, RIGHT);

    expect("no data is never free", (const int[]){ND, ND, ND, ND, ND, 260, 258, 260, ND, ND, ND, ND, ND},
           258u, B, B, NONE);

    expect("TOO_NEAR is the nearest", (const int[]){NR, NR, 300, 200, 100, TN, TN, TN, 100, 200, 300, 400, NR},
           0u, 3u, 3u, LEFT);

    expect("tie goes to the side with more room at 30 deg", (const int[]){400, NR, NR, 300, 280, 270, 260, 270, 280, 300, NR, NR, NR},
           260u, 4u, 4u, LEFT);

    expect("tie, more room on the right", (const int[]){NR, NR, NR, 300, 280, 270, 260, 270, 280, 300, NR, NR, 450},
           260u, 4u, 4u, RIGHT);

    if (failures == 0) {
        printf("test_obstacle_profile: all tests passed\n");
        return 0;
    }
    printf("test_obstacle_profile: %d failure(s)\n", failures);
    return 1;
}

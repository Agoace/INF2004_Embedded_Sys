#ifndef OBSTACLE_PROFILE_H
#define OBSTACLE_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

#include "obstacle_scan.h"

/*
 * Profile analysis: turns a 13-point scan into the PROFILE event for the
 * Vehicle Controller. It does not estimate the obstacle width.
 *
 * For each side, the analysis starts at straight ahead and steps outward
 * until the beam is past the obstacle. The result per side is the "free
 * step": the number of 5-degree steps from straight ahead to the first
 * free point. A smaller free step is a shorter way around the obstacle.
 *
 * Pure logic: no hardware or RTOS calls. See docs/OBSTACLE_SCAN.md.
 */

/* A VALID point at least this much farther than the obstacle is free:
 * the beam sees past the obstacle. Corner echoes in run 1 were only
 * 20-35 mm farther than the face. */
#define OBSTACLE_PROFILE_GAP_MM 150u

/* Free step per side: 3 bits. 0-6 = free at 0-30 degrees; 7 = blocked. */
#define OBSTACLE_PROFILE_STEPS 7u
#define OBSTACLE_PROFILE_BLOCKED 7u

/*
 * PROFILE event: one 16-bit word.
 *
 *   bits 0-8    obstacle distance: nearest VALID point, ultrasonic distance
 *               code (0 = TOO_NEAR). 511 when no point is VALID.
 *   bits 9-11   right free step (0-6, 7 = blocked)
 *   bits 12-14  left free step  (0-6, 7 = blocked)
 *   bit 15      spare, always 0
 */
typedef uint16_t obstacle_profile_word_t;

#define OBSTACLE_PROFILE_DISTANCE_SHIFT 0u
#define OBSTACLE_PROFILE_DISTANCE_MASK 0x01FFu
#define OBSTACLE_PROFILE_RIGHT_SHIFT 9u
#define OBSTACLE_PROFILE_RIGHT_MASK 0x0E00u
#define OBSTACLE_PROFILE_LEFT_SHIFT 12u
#define OBSTACLE_PROFILE_LEFT_MASK 0x7000u

typedef enum {
    OBSTACLE_SIDE_NONE = 0, /* Both sides blocked: reverse and rescan. */
    OBSTACLE_SIDE_RIGHT = 1,
    OBSTACLE_SIDE_LEFT = 2,
} obstacle_side_t;

static inline uint16_t obstacle_profile_distance(obstacle_profile_word_t w) {
    return (uint16_t)((w & OBSTACLE_PROFILE_DISTANCE_MASK) >>
                      OBSTACLE_PROFILE_DISTANCE_SHIFT);
}

static inline unsigned int obstacle_profile_right(obstacle_profile_word_t w) {
    return (w & OBSTACLE_PROFILE_RIGHT_MASK) >> OBSTACLE_PROFILE_RIGHT_SHIFT;
}

static inline unsigned int obstacle_profile_left(obstacle_profile_word_t w) {
    return (w & OBSTACLE_PROFILE_LEFT_MASK) >> OBSTACLE_PROFILE_LEFT_SHIFT;
}

/* Build the PROFILE word from a finished scan. */
obstacle_profile_word_t obstacle_profile_build(const obstacle_profile_t *profile);

/*
 * The clearer side: the smaller free step. On a tie, the side whose
 * outermost point has more room; on a full tie, LEFT.
 * NONE when both sides are blocked.
 */
obstacle_side_t obstacle_profile_side(obstacle_profile_word_t w,
                                      const obstacle_profile_t *profile);

#endif

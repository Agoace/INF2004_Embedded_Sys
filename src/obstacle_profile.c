#include "obstacle_profile.h"

/* Point index for a side and a step. Right steps go down from the centre. */
static unsigned int point_index(bool left, unsigned int step) {
    return left ? OBSTACLE_SCAN_CENTRE_INDEX + step
                : OBSTACLE_SCAN_CENTRE_INDEX - step;
}

static bool is_free(obstacle_scan_point_t point, uint16_t obstacle_mm) {
    const ultrasonic_status_t status = ultrasonic_result_status(point);
    if (status == ULTRASONIC_STATUS_NO_RETURN) {
        return true;
    }
    if (status != ULTRASONIC_STATUS_VALID) {
        return false; /* No data is never free. */
    }
    return ultrasonic_result_distance(point) >=
           (uint32_t)obstacle_mm + OBSTACLE_PROFILE_GAP_MM;
}

/* First step where this point and the next one outward are both free.
 * The outermost point may be free alone. */
static unsigned int free_step(const obstacle_profile_t *profile, bool left,
                              uint16_t obstacle_mm) {
    for (unsigned int step = 0u; step < OBSTACLE_PROFILE_STEPS; ++step) {
        if (!is_free(profile->point[point_index(left, step)], obstacle_mm)) {
            continue;
        }
        if (step + 1u == OBSTACLE_PROFILE_STEPS ||
            is_free(profile->point[point_index(left, step + 1u)], obstacle_mm)) {
            return step;
        }
    }
    return OBSTACLE_PROFILE_BLOCKED;
}

obstacle_profile_word_t obstacle_profile_build(const obstacle_profile_t *profile) {
    uint16_t nearest = ULTRASONIC_DISTANCE_MAX_MM;
    for (unsigned int i = 0u; i < OBSTACLE_SCAN_POINTS; ++i) {
        const obstacle_scan_point_t p = profile->point[i];
        if (ultrasonic_result_status(p) == ULTRASONIC_STATUS_VALID &&
            ultrasonic_result_distance(p) < nearest) {
            nearest = ultrasonic_result_distance(p);
        }
    }

    const unsigned int right = free_step(profile, false, nearest);
    const unsigned int left = free_step(profile, true, nearest);

    return (obstacle_profile_word_t)(
        ((uint32_t)nearest << OBSTACLE_PROFILE_DISTANCE_SHIFT) |
        ((uint32_t)right << OBSTACLE_PROFILE_RIGHT_SHIFT) |
        ((uint32_t)left << OBSTACLE_PROFILE_LEFT_SHIFT));
}

/* Room at the outermost point of a side: NO_RETURN is the most room, then
 * a larger distance. No data is the least room. */
static uint16_t outer_room(const obstacle_profile_t *profile, bool left) {
    const obstacle_scan_point_t p =
        profile->point[point_index(left, OBSTACLE_PROFILE_STEPS - 1u)];
    const ultrasonic_status_t status = ultrasonic_result_status(p);
    if (status == ULTRASONIC_STATUS_NO_RETURN) {
        return ULTRASONIC_DISTANCE_MAX_MM + 1u;
    }
    if (status == ULTRASONIC_STATUS_VALID) {
        return ultrasonic_result_distance(p);
    }
    return 0u;
}

obstacle_side_t obstacle_profile_side(obstacle_profile_word_t w,
                                      const obstacle_profile_t *profile) {
    const unsigned int right = obstacle_profile_right(w);
    const unsigned int left = obstacle_profile_left(w);

    if (right == OBSTACLE_PROFILE_BLOCKED && left == OBSTACLE_PROFILE_BLOCKED) {
        return OBSTACLE_SIDE_NONE;
    }
    if (right != left) {
        return right < left ? OBSTACLE_SIDE_RIGHT : OBSTACLE_SIDE_LEFT;
    }
    return outer_room(profile, false) > outer_room(profile, true)
               ? OBSTACLE_SIDE_RIGHT
               : OBSTACLE_SIDE_LEFT;
}

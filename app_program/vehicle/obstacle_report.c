/*
 * obstacle_report.c - conversion of a finished obstacle scan to a report.
 *
 * See obstacle_report.h. This file has no state and no dependencies on
 * hardware, the kernel or the network.
 */
#include "obstacle_report.h"
#include <math.h>

#define REPORT_RADIANS_PER_DEGREE 0.017453292519943295f

/* Geometry is observational only. It never changes the legacy PROFILE. */
static uint16_t rounded_mm(float value)
{
    if (!isfinite(value) || value < 0.0f || value >= 65534.5f) {
        return RM_OBSTACLE_GEOMETRY_NONE;
    }
    return (uint16_t)(value + 0.5f);
}

static float angle(unsigned int index)
{
    return (float)obstacle_scan_angle_deg(index) * REPORT_RADIANS_PER_DEGREE;
}

static uint16_t observed_clearance(const obstacle_profile_t *profile,
                                    unsigned int edge, int direction,
                                    uint16_t nearest)
{
    const float radius =
        (float)ultrasonic_result_distance(profile->point[edge]);
    const float x = radius * cosf(angle(edge));
    const float y = radius * sinf(angle(edge));
    for (int i = (int)edge + direction;
         i >= 0 && i < (int)OBSTACLE_SCAN_POINTS; i += direction) {
        const obstacle_scan_point_t p = profile->point[(unsigned int)i];
        const ultrasonic_status_t status = ultrasonic_result_status(p);
        if (status != ULTRASONIC_STATUS_VALID &&
            status != ULTRASONIC_STATUS_NO_RETURN) {
            return RM_OBSTACLE_GEOMETRY_NONE; /* Unknown is never free. */
        }
        if (status == ULTRASONIC_STATUS_VALID &&
            ultrasonic_result_distance(p) <
                (uint32_t)nearest + OBSTACLE_PROFILE_GAP_MM) {
            return RM_OBSTACLE_GEOMETRY_NONE;
        }
        const float ray_angle = angle((unsigned int)i);
        if (status == ULTRASONIC_STATUS_VALID &&
            (float)ultrasonic_result_distance(p) * cosf(ray_angle) < x) {
            return RM_OBSTACLE_GEOMETRY_NONE;
        }
        return rounded_mm((float)direction * (x * tanf(ray_angle) - y));
    }
    return RM_OBSTACLE_GEOMETRY_NONE; /* Edge touches field-of-view limit. */
}

static void geometry(const obstacle_profile_t *profile, uint16_t nearest,
                      rm_obstacle_t *out)
{
    out->width_mm = RM_OBSTACLE_GEOMETRY_NONE;
    out->left_clear_mm = RM_OBSTACLE_GEOMETRY_NONE;
    out->right_clear_mm = RM_OBSTACLE_GEOMETRY_NONE;
    if (nearest == 0u || nearest >= ULTRASONIC_DISTANCE_MAX_MM) { return; }
    unsigned int first = OBSTACLE_SCAN_POINTS, last = 0u;
    for (unsigned int i = 0u; i < OBSTACLE_SCAN_POINTS; ++i) {
        const obstacle_scan_point_t p = profile->point[i];
        if (ultrasonic_result_status(p) == ULTRASONIC_STATUS_VALID &&
            ultrasonic_result_distance(p) > 0u &&
            ultrasonic_result_distance(p) <=
                (uint32_t)nearest + OBSTACLE_PROFILE_GAP_MM) {
            if (first == OBSTACLE_SCAN_POINTS) { first = i; }
            last = i;
        }
    }
    if (first == OBSTACLE_SCAN_POINTS) { return; }
    if (first == 0u) { out->geometry_flags |= RM_OBSTACLE_RIGHT_CLIPPED; }
    if (last == OBSTACLE_SCAN_POINTS - 1u) {
        out->geometry_flags |= RM_OBSTACLE_LEFT_CLIPPED;
    }
    if (last > first) {
        const float right =
            (float)ultrasonic_result_distance(profile->point[first]);
        const float left =
            (float)ultrasonic_result_distance(profile->point[last]);
        /* Law of cosines; clamp roundoff at zero before square root. */
        const float square = left * left + right * right -
            2.0f * left * right * cosf(angle(last) - angle(first));
        out->width_mm = rounded_mm(sqrtf(square > 0.0f ? square : 0.0f));
        if (out->width_mm != RM_OBSTACLE_GEOMETRY_NONE) {
            out->geometry_flags |= RM_OBSTACLE_WIDTH_VALID;
        }
    }
    out->left_clear_mm = observed_clearance(profile, last, 1, nearest);
    out->right_clear_mm = observed_clearance(profile, first, -1, nearest);
    if (out->left_clear_mm != RM_OBSTACLE_GEOMETRY_NONE) {
        out->geometry_flags |= RM_OBSTACLE_LEFT_CLEAR_VALID;
    }
    if (out->right_clear_mm != RM_OBSTACLE_GEOMETRY_NONE) {
        out->geometry_flags |= RM_OBSTACLE_RIGHT_CLEAR_VALID;
    }
}

/* A VALID point holds a real distance (0 is TOO_NEAR, which is VALID). */
static bool point_is_valid(obstacle_scan_point_t point)
{
    return ultrasonic_result_status(point) == ULTRASONIC_STATUS_VALID;
}

/* Distance in steps from straight ahead (always 0 or more). */
static unsigned int steps_from_centre(unsigned int index)
{
    return (index >= OBSTACLE_SCAN_CENTRE_INDEX)
               ? (index - OBSTACLE_SCAN_CENTRE_INDEX)
               : (OBSTACLE_SCAN_CENTRE_INDEX - index);
}

/* Clearance angle of one side from its free step (7 means blocked). */
static uint8_t clearance_deg(unsigned int free_step)
{
    uint8_t degrees = OBSTACLE_REPORT_BLOCKED_DEG;

    if (free_step < OBSTACLE_PROFILE_BLOCKED) {
        degrees = (uint8_t)(free_step * (unsigned int)OBSTACLE_SCAN_STEP_DEG);
    }
    return degrees;
}

/*
 * Find the nearest VALID point. The scan uses "nearest" as the distance of
 * the PROFILE word, so the same rule is used here.
 */
uint8_t obstacle_closest_index(const obstacle_profile_t *profile)
{
    uint8_t best = OBSTACLE_REPORT_NO_POINT;
    uint16_t best_mm = ULTRASONIC_DISTANCE_MAX_MM;

    for (unsigned int i = 0u; i < OBSTACLE_SCAN_POINTS; ++i) {
        const obstacle_scan_point_t point = profile->point[i];
        const uint16_t mm = ultrasonic_result_distance(point);
        bool better = false;

        if (!point_is_valid(point) || (mm >= ULTRASONIC_DISTANCE_MAX_MM)) {
            continue;
        }
        if (best == OBSTACLE_REPORT_NO_POINT) {
            better = true;
        } else if (mm < best_mm) {
            better = true;
        } else if ((mm == best_mm) &&
                   (steps_from_centre(i) < steps_from_centre(best))) {
            better = true;
        }
        if (better) {
            best = (uint8_t)i;
            best_mm = mm;
        }
    }
    return best;
}

/*
 * Build the report. The distance comes from the PROFILE word so it always
 * agrees with the decision. The angle comes from the nearest scan point.
 */
void obstacle_report_make(const obstacle_profile_t *profile,
                          obstacle_profile_word_t word, obstacle_side_t side,
                          uint32_t scan_id, uint32_t uptime_ms,
                          rm_obstacle_t *out)
{
    const unsigned int right = obstacle_profile_right(word);
    const unsigned int left = obstacle_profile_left(word);
    const uint16_t distance = obstacle_profile_distance(word);
    const uint8_t closest = obstacle_closest_index(profile);

    *out = (rm_obstacle_t){0};
    geometry(profile, distance, out);
    out->scan_id = scan_id;
    out->uptime_ms = uptime_ms;
    out->decision = (uint8_t)side;
    out->closest_mm = distance;
    out->profile_word = word;
    out->right_free_step = (uint8_t)right;
    out->left_free_step = (uint8_t)left;
    out->right_clear_deg = clearance_deg(right);
    out->left_clear_deg = clearance_deg(left);
    if (right >= OBSTACLE_PROFILE_BLOCKED) {
        out->flags |= RM_OBSTACLE_RIGHT_BLOCKED;
    }
    if (left >= OBSTACLE_PROFILE_BLOCKED) {
        out->flags |= RM_OBSTACLE_LEFT_BLOCKED;
    }
    if (closest != OBSTACLE_REPORT_NO_POINT) {
        out->flags |= RM_OBSTACLE_CLOSEST_VALID;
        out->closest_angle_deg = (int8_t)obstacle_scan_angle_deg(closest);
        if (distance == ULTRASONIC_DISTANCE_TOO_NEAR) {
            out->flags |= RM_OBSTACLE_TOO_NEAR;
        }
    }
}

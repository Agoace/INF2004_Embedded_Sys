#include "obstacle.h"

static unsigned int get_field(const obstacle_monitor_t *monitor,
                              unsigned int shift) {
    return ((unsigned int)monitor->bits >> shift) & OBSTACLE_FIELD_MASK;
}

static void set_field(obstacle_monitor_t *monitor, unsigned int shift,
                      unsigned int value) {
    const unsigned int mask = OBSTACLE_FIELD_MASK << shift;
    monitor->bits = (uint16_t)((monitor->bits & ~mask) |
                               ((value & OBSTACLE_FIELD_MASK) << shift));
}

/* Add 1 to a count when the condition is true, saturating at the confirm
 * count. Reset the count when the condition is false. Return the count. */
static unsigned int update_count(obstacle_monitor_t *monitor,
                                 unsigned int shift, bool condition) {
    unsigned int count = get_field(monitor, shift);
    if (!condition) {
        count = 0u;
    } else if (count < OBSTACLE_CONFIRM_COUNT) {
        ++count;
    }
    set_field(monitor, shift, count);
    return count;
}

static obstacle_event_word_t pack(obstacle_event_t event, uint16_t distance) {
    return (obstacle_event_word_t)(
        ((uint32_t)distance << OBSTACLE_EVENT_DISTANCE_SHIFT) |
        ((uint32_t)event << OBSTACLE_EVENT_SHIFT));
}

/* Set the state and clear every count. */
static void enter(obstacle_monitor_t *monitor, obstacle_state_t state) {
    monitor->bits = (uint16_t)((unsigned int)state << OBSTACLE_STATE_SHIFT);
}

void obstacle_init(obstacle_monitor_t *monitor) {
    enter(monitor, OBSTACLE_MONITORING);
}

obstacle_event_word_t obstacle_update(obstacle_monitor_t *monitor,
                                      ultrasonic_result_t result) {
    const obstacle_state_t state = obstacle_state(monitor);

    if (state == OBSTACLE_SENSOR_FAULT) {
        return pack(OBSTACLE_EVENT_NONE, 0u);
    }

    if (ultrasonic_result_fault(result)) {
        enter(monitor, OBSTACLE_SENSOR_FAULT);
        return pack(OBSTACLE_EVENT_SENSOR_FAULT, 0u);
    }

    const ultrasonic_status_t status = ultrasonic_result_status(result);
    const uint16_t distance = ultrasonic_result_distance(result);

    if (status != ULTRASONIC_STATUS_VALID &&
        status != ULTRASONIC_STATUS_NO_RETURN) {
        /* A single NO_RISE or STUCK_HIGH below the fault streak, or no
         * result yet: no evidence either way. Keep the counts. */
        return pack(OBSTACLE_EVENT_NONE, 0u);
    }

    /* TOO_NEAR is distance code 0, so it is inside both ranges. */
    const bool valid = status == ULTRASONIC_STATUS_VALID;
    const bool inside_stop = valid && distance < OBSTACLE_STOP_MM;
    const bool inside_near = valid && distance < OBSTACLE_NEAR_MM;
    const bool beyond_stop =
        valid && distance >= OBSTACLE_STOP_MM + OBSTACLE_HYSTERESIS_MM;
    /* No echo is unknown, not clear. It can end APPROACHING, but only a
     * valid echo can release a STOP (beyond_stop needs valid). */
    const bool beyond_near =
        !valid || distance >= OBSTACLE_NEAR_MM + OBSTACLE_HYSTERESIS_MM;

    /* Consecutive-reading counts. A reading in a dead band resets them. */
    const unsigned int stop_count =
        update_count(monitor, OBSTACLE_STOP_COUNT_SHIFT, inside_stop);
    const unsigned int near_count =
        update_count(monitor, OBSTACLE_NEAR_COUNT_SHIFT, inside_near);
    const unsigned int clear_count =
        update_count(monitor, OBSTACLE_CLEAR_COUNT_SHIFT, beyond_near);
    const unsigned int release_count =
        update_count(monitor, OBSTACLE_RELEASE_COUNT_SHIFT, beyond_stop);

    obstacle_event_t event = OBSTACLE_EVENT_NONE;
    obstacle_state_t next = state;

    if (state != OBSTACLE_CONFIRMED && stop_count >= OBSTACLE_CONFIRM_COUNT) {
        next = OBSTACLE_CONFIRMED;
        event = OBSTACLE_EVENT_STOP;
    } else if (state == OBSTACLE_CONFIRMED &&
               release_count >= OBSTACLE_CONFIRM_COUNT) {
        /* Leave STOP one step at a time: CLEAR needs its own confirmation. */
        next = OBSTACLE_APPROACHING;
        event = OBSTACLE_EVENT_NEAR;
    } else if (state == OBSTACLE_MONITORING &&
               near_count >= OBSTACLE_CONFIRM_COUNT) {
        next = OBSTACLE_APPROACHING;
        event = OBSTACLE_EVENT_NEAR;
    } else if (state == OBSTACLE_APPROACHING &&
               clear_count >= OBSTACLE_CONFIRM_COUNT) {
        next = OBSTACLE_MONITORING;
        event = OBSTACLE_EVENT_CLEAR;
    }

    /* Keep the counts across a change, so the next state can confirm on
     * the next reading when the target keeps moving the same way. */
    set_field(monitor, OBSTACLE_STATE_SHIFT, (unsigned int)next);
    return pack(event, event == OBSTACLE_EVENT_NONE ? 0u : distance);
}

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdbool.h>
#include <stdint.h>

#include "ultrasonic.h"

/*
 * Forward obstacle watch for the Obstacle Scan Task.
 *
 * obstacle_update() takes one packed ultrasonic result and returns an event
 * for the Vehicle Controller. The module has no hardware or RTOS calls, so
 * the same logic runs on the Pico and in the host unit test.
 *
 * One task owns an obstacle_monitor_t. Do not share it between tasks; share
 * only the returned event word. See docs/OBSTACLE_MONITOR.md.
 */

/* Measured echo distance from the sensor to the robot front (run 7).
 * Confirm it with a ruler. */
#define OBSTACLE_SENSOR_TO_FRONT_MM 58u
/* Design review: stop when an obstacle is less than 20 cm from the front. */
#define OBSTACLE_STOP_FROM_FRONT_MM 200u
/* Thresholds at the sensor face. A reading below the value is inside. */
#define OBSTACLE_STOP_MM (OBSTACLE_SENSOR_TO_FRONT_MM + OBSTACLE_STOP_FROM_FRONT_MM)
#define OBSTACLE_NEAR_MM 400u
/* Dead band. To leave a range, a reading must be this far past the
 * threshold. It stops a target on a threshold from changing the state on
 * every reading (one sensor step is about 4 mm). */
#define OBSTACLE_HYSTERESIS_MM 20u
/* Consecutive readings that confirm a change of state. */
#define OBSTACLE_CONFIRM_COUNT 2u

/* Monitor state: 2 bits. Names follow the Week 8 Spatial Optimizer states. */
typedef enum {
    OBSTACLE_MONITORING = 0,   /* Nothing confirmed within the NEAR range. */
    OBSTACLE_APPROACHING = 1,  /* Obstacle confirmed within the NEAR range. */
    OBSTACLE_CONFIRMED = 2,    /* Obstacle confirmed within the STOP range. */
    OBSTACLE_SENSOR_FAULT = 3, /* Latched until obstacle_init(). */
} obstacle_state_t;

/* Event: 3 bits. An event is sent only when the state changes. */
typedef enum {
    OBSTACLE_EVENT_NONE = 0,
    OBSTACLE_EVENT_CLEAR = 1,        /* OBST_CLEAR: entered MONITORING. */
    OBSTACLE_EVENT_NEAR = 2,         /* OBST_NEAR: entered APPROACHING. */
    OBSTACLE_EVENT_STOP = 3,         /* OBST_STOP: entered CONFIRMED. */
    OBSTACLE_EVENT_SENSOR_FAULT = 4, /* SENSOR_FAULT: entered SENSOR_FAULT. */
} obstacle_event_t;

/*
 * Packed event: one 16-bit word.
 *
 *   bits 0-8    ultrasonic distance code of the reading that caused the
 *               event (9 bits; 0 = TOO_NEAR for STOP, 0 for CLEAR by
 *               NO_RETURN and for SENSOR_FAULT)
 *   bits 9-11   event (3 bits)
 *   bits 12-15  spare, always 0
 */
typedef uint16_t obstacle_event_word_t;

#define OBSTACLE_EVENT_DISTANCE_SHIFT 0u
#define OBSTACLE_EVENT_DISTANCE_MASK 0x01FFu
#define OBSTACLE_EVENT_SHIFT 9u
#define OBSTACLE_EVENT_MASK 0x0E00u

static inline obstacle_event_t obstacle_event(obstacle_event_word_t w) {
    return (obstacle_event_t)((w & OBSTACLE_EVENT_MASK) >> OBSTACLE_EVENT_SHIFT);
}

static inline uint16_t obstacle_event_distance(obstacle_event_word_t w) {
    return (uint16_t)((w & OBSTACLE_EVENT_DISTANCE_MASK) >>
                      OBSTACLE_EVENT_DISTANCE_SHIFT);
}

/*
 * Monitor state, private to the owning task: one 16-bit word.
 * Each count is 2 bits and saturates at OBSTACLE_CONFIRM_COUNT.
 *
 *   bits 0-1    state (obstacle_state_t)
 *   bits 2-3    stop count: valid echoes inside the STOP range
 *   bits 4-5    near count: valid echoes inside the NEAR range
 *   bits 6-7    clear count: beyond NEAR + dead band, or no echo
 *   bits 8-9    release count: valid echoes beyond STOP + dead band
 *   bits 10-15  spare, always 0
 */
typedef struct {
    uint16_t bits;
} obstacle_monitor_t;

#define OBSTACLE_FIELD_MASK 0x3u
#define OBSTACLE_STATE_SHIFT 0u
#define OBSTACLE_STOP_COUNT_SHIFT 2u
#define OBSTACLE_NEAR_COUNT_SHIFT 4u
#define OBSTACLE_CLEAR_COUNT_SHIFT 6u
#define OBSTACLE_RELEASE_COUNT_SHIFT 8u

void obstacle_init(obstacle_monitor_t *monitor);

/* Process one ultrasonic result. Returns OBSTACLE_EVENT_NONE in bits 9-11
 * when the state does not change. */
obstacle_event_word_t obstacle_update(obstacle_monitor_t *monitor,
                                      ultrasonic_result_t result);

static inline obstacle_state_t obstacle_state(const obstacle_monitor_t *monitor) {
    return (obstacle_state_t)((monitor->bits >> OBSTACLE_STATE_SHIFT) &
                              OBSTACLE_FIELD_MASK);
}

#endif

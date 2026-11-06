#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Non-blocking HC-SR04+ driver for the Spatial Optimizer.
 *
 * The driver does not depend on an RTOS. A GPIO interrupt records the echo
 * edge times. The caller starts a measurement with ultrasonic_trigger() and
 * then calls ultrasonic_service() from task or main-loop context. The service
 * call converts the raw timing into a bounded result and publishes it.
 *
 * Raw timer values stay in this module. Only the packed result leaves it.
 * See docs/ULTRASONIC_DRIVER.md for the representation table.
 */

/*
 * Result status: 2 bits.
 *
 * NO_RISE and STUCK_HIGH are sensor faults and increase the failure streak.
 * NO_RETURN shows a working sensor that heard no echo in range; it resets
 * the streak. Treat every status except VALID as "unknown" for obstacle
 * decisions. Never treat NO_RETURN as clear.
 */
typedef enum {
    ULTRASONIC_STATUS_VALID = 0,      /* An echo returned from 0-511 mm. */
    ULTRASONIC_STATUS_NO_RISE = 1,    /* Fault: Echo did not go HIGH after
                                         the trigger. */
    ULTRASONIC_STATUS_STUCK_HIGH = 2, /* Fault: Echo was HIGH before the
                                         trigger. */
    ULTRASONIC_STATUS_NO_RETURN = 3,  /* Sensor pinged, but no echo returned
                                         from 0-511 mm. */
} ultrasonic_status_t;

/*
 * Distance code: 9 bits.
 *   0       VALID only: TOO_NEAR, an echo from less than 60 mm. Something is
 *           there, but the exact distance is not reliable.
 *   1-59    not used
 *   60-511  distance in mm
 * An echo from more than 511 mm is reported as NO_RETURN.
 *
 * Below about 40 mm the HC-SR04+ still rings from its own burst and reports
 * wrong distances. The 60 mm floor keeps those values out of the field.
 */
#define ULTRASONIC_DISTANCE_MIN_MM 60u
#define ULTRASONIC_DISTANCE_MAX_MM 511u
#define ULTRASONIC_DISTANCE_TOO_NEAR 0u
/* Distance code when the status is not VALID. The value has no meaning. */
#define ULTRASONIC_DISTANCE_NONE 0u

/* Consecutive NO_RISE or STUCK_HIGH results. Saturates at this value. */
#define ULTRASONIC_FAULT_STREAK 5u

/*
 * Packed result: one 16-bit word.
 *
 *   bits 0-8    distance code (9 bits)
 *   bits 9-10   status (2 bits)
 *   bits 11-13  fault streak, 0-5 (3 bits)
 *   bits 14-15  spare, always 0
 *
 * Use the masks and shifts, not C bit-fields, so the layout is fixed.
 */
typedef uint16_t ultrasonic_result_t;

#define ULTRASONIC_DISTANCE_SHIFT 0u
#define ULTRASONIC_DISTANCE_MASK 0x01FFu
#define ULTRASONIC_STATUS_SHIFT 9u
#define ULTRASONIC_STATUS_MASK 0x0600u
#define ULTRASONIC_STREAK_SHIFT 11u
#define ULTRASONIC_STREAK_MASK 0x3800u

static inline uint16_t ultrasonic_result_distance(ultrasonic_result_t r) {
    return (uint16_t)((r & ULTRASONIC_DISTANCE_MASK) >> ULTRASONIC_DISTANCE_SHIFT);
}

static inline ultrasonic_status_t ultrasonic_result_status(ultrasonic_result_t r) {
    return (ultrasonic_status_t)((r & ULTRASONIC_STATUS_MASK) >> ULTRASONIC_STATUS_SHIFT);
}

static inline uint8_t ultrasonic_result_streak(ultrasonic_result_t r) {
    return (uint8_t)((r & ULTRASONIC_STREAK_MASK) >> ULTRASONIC_STREAK_SHIFT);
}

/*
 * Word before the first measurement: NO_RISE with streak 0. A real fault
 * always has a streak of 1 or more, so this word cannot come from a
 * measurement.
 */
#define ULTRASONIC_RESULT_NONE \
    ((ultrasonic_result_t)(ULTRASONIC_STATUS_NO_RISE << ULTRASONIC_STATUS_SHIFT))

static inline bool ultrasonic_result_is_none(ultrasonic_result_t r) {
    return r == ULTRASONIC_RESULT_NONE;
}

/* True for a VALID echo from less than ULTRASONIC_DISTANCE_MIN_MM. */
static inline bool ultrasonic_result_too_near(ultrasonic_result_t r) {
    return ultrasonic_result_status(r) == ULTRASONIC_STATUS_VALID &&
           ultrasonic_result_distance(r) == ULTRASONIC_DISTANCE_TOO_NEAR;
}

/* True after ULTRASONIC_FAULT_STREAK sensor faults in a row. */
static inline bool ultrasonic_result_fault(ultrasonic_result_t r) {
    return ultrasonic_result_streak(r) >= ULTRASONIC_FAULT_STREAK;
}

#endif

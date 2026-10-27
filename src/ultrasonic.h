#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

#include "pico/types.h"

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

/* Distance code: 9 bits, 0-511 mm. Every code is a distance.
 * An echo from more than 511 mm is reported as NO_RETURN. */
#define ULTRASONIC_DISTANCE_MAX_MM 511u
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

/* True after ULTRASONIC_FAULT_STREAK sensor faults in a row. */
static inline bool ultrasonic_result_fault(ultrasonic_result_t r) {
    return ultrasonic_result_streak(r) >= ULTRASONIC_FAULT_STREAK;
}

/*
 * Optional hook. The echo interrupt calls it after the falling edge.
 * An RTOS build can release sem_echo here. Keep it short and ISR-safe.
 * It is not called for a timeout; ultrasonic_service() detects timeouts.
 */
typedef void (*ultrasonic_echo_hook_t)(void);

/*
 * Configure the trigger and echo pins and install the echo interrupt on the
 * calling core. Call once, from the core that will call ultrasonic_service().
 */
void ultrasonic_init(uint trigger_pin, uint echo_pin, ultrasonic_echo_hook_t hook);

/*
 * Start one measurement. Returns false and does nothing if a measurement is
 * still in progress or the minimum interval since the last trigger has not
 * passed. If Echo is already HIGH, publishes STUCK_HIGH at once and returns
 * true; ultrasonic_busy() is then false.
 */
bool ultrasonic_trigger(void);

/*
 * Finish the current measurement if it is complete or has timed out.
 * Returns true when it publishes a new result. Never blocks.
 */
bool ultrasonic_service(void);

/* True while a measurement waits for its echo or timeout. */
bool ultrasonic_busy(void);

/* Latest published result. Safe to call from any context. */
ultrasonic_result_t ultrasonic_latest(void);

/*
 * Raw timing of the latest measurement, for diagnostics only.
 * Do not share these values with other tasks.
 */
typedef struct {
    uint32_t rise_latency_us; /* Trigger start to Echo rising edge. */
    uint32_t pulse_us;        /* Echo HIGH time. 0 if not measured. */
    uint32_t distance_mm;     /* Unbounded distance, also for an echo
                                 beyond 511 mm. 0 if no echo ended. */
} ultrasonic_debug_t;

ultrasonic_debug_t ultrasonic_debug_latest(void);

#endif

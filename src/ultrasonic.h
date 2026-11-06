#ifndef ULTRASONIC_DRIVER_H
#define ULTRASONIC_DRIVER_H

#include "../app_program/subsystem5/ultrasonic.h"

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
void ultrasonic_init(unsigned int trigger_pin, unsigned int echo_pin,
                     ultrasonic_echo_hook_t hook);

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

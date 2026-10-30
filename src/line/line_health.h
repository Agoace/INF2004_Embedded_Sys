/**
 * @file line_health.h
 * @brief Runtime line-sensor health: a railed or stuck sensor.
 *
 * The YX calibration only checks contrast while sweeping. The integrated
 * firmware also needs a running check, otherwise swapping the follow source
 * silently loses the fault that the old fixed-threshold detector provided
 * (RT-0031 B3).
 *
 * Thresholds are the ones the previous detector used: a reading at or beyond
 * the ADC rails, held for the fault time, marks that sensor faulty. The mask
 * clears as soon as the sensor is back inside the rails.
 *
 * Pure logic, no hardware: host-tested by tests/line.
 */
#ifndef LINE_HEALTH_H
#define LINE_HEALTH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LINE_HEALTH_SENSORS 3u

/** Raw 12-bit ADC rails. */
#define LINE_HEALTH_RAIL_LOW   10u
#define LINE_HEALTH_RAIL_HIGH  4085u

/** Time a sensor must sit on a rail before it counts as faulty. */
#define LINE_HEALTH_FAULT_MS   100u

typedef struct
{
    uint16_t low_ms[LINE_HEALTH_SENSORS];
    uint16_t high_ms[LINE_HEALTH_SENSORS];
    uint8_t  fault_mask;    /* one bit per sensor, 0 = healthy */
} line_health_t;

void line_health_init(line_health_t * p_state);

/**
 * @brief Fold one reading in.
 *
 * @param raw   Three raw 12-bit readings in physical L/C/R order.
 * @param dt_ms Elapsed time since the previous call.
 * @return The current fault mask, one bit per sensor.
 */
uint8_t line_health_update(line_health_t * p_state,
                           const uint16_t raw[LINE_HEALTH_SENSORS],
                           uint32_t dt_ms);

static inline bool
line_health_faulted (const line_health_t * p_state)
{
    return (NULL != p_state) && (0u != p_state->fault_mask);
}

#endif /* LINE_HEALTH_H */

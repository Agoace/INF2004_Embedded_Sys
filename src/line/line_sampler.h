/**
 * @file line_sampler.h
 * @brief 1 kHz cadence decision and overrun accounting for the Line producer.
 *
 * The Line producer must sample at 1 kHz, report how often it slips, and never
 * replay a stale slot (RT-0031 B5). This keeps that arithmetic out of the board
 * code so the host tests can prove it.
 *
 * A late call reports the slots it missed in `overruns` and resynchronises to
 * the next whole period, so a stalled loop produces one fresh sample plus a
 * recorded gap instead of a burst of back-dated ones.
 *
 * Pure logic, no hardware: host-tested by tests/line.
 */
#ifndef LINE_SAMPLER_H
#define LINE_SAMPLER_H

#include <stdbool.h>
#include <stdint.h>

/** Nominal Line sampling period. */
#define LINE_SAMPLER_PERIOD_MS 1u

typedef struct
{
    uint32_t next_ms;       /* next due time                                */
    uint32_t samples;       /* samples taken                                */
    uint32_t overruns;      /* sample slots missed                          */
    uint32_t max_late_ms;   /* worst lateness seen, for the diagnostic      */
} line_sampler_t;

/** Start the cadence; the first call to line_sampler_due() is immediately due. */
void line_sampler_init(line_sampler_t * p_state, uint32_t now_ms);

/**
 * @brief Is a sample due, and charge any lateness?
 *
 * @return true when the caller should read the ADC and publish once.
 */
bool line_sampler_due(line_sampler_t * p_state, uint32_t now_ms);

/** Drop any pending lateness without counting it, e.g. after a deliberate stop. */
void line_sampler_resync(line_sampler_t * p_state, uint32_t now_ms);

#endif /* LINE_SAMPLER_H */

/**
 * @file line_sampler.c
 * @brief 1 kHz cadence with overrun accounting.
 */
#include "line_sampler.h"

#include <stddef.h>

void
line_sampler_init (line_sampler_t * p_state, uint32_t now_ms)
{
    if (NULL == p_state)
    {
        return;
    }

    p_state->next_ms     = now_ms;
    p_state->samples     = 0u;
    p_state->overruns    = 0u;
    p_state->max_late_ms = 0u;
}

bool
line_sampler_due (line_sampler_t * p_state, uint32_t now_ms)
{
    if (NULL == p_state)
    {
        return false;
    }

    /* Not yet due. The cast keeps the wrap-around comparison correct. */
    if ((int32_t) (now_ms - p_state->next_ms) < 0)
    {
        return false;
    }

    uint32_t const late_ms = now_ms - p_state->next_ms;

    if (late_ms > 0u)
    {
        /* Every millisecond we are past the deadline is a slot we did not
         * sample. Count them, and remember the worst case for the report. */
        p_state->overruns += late_ms;

        if (late_ms > p_state->max_late_ms)
        {
            p_state->max_late_ms = late_ms;
        }
    }

    p_state->samples++;

    /* Resynchronise to the next whole period. Never replay the missed slots. */
    p_state->next_ms = now_ms + LINE_SAMPLER_PERIOD_MS;

    return true;
}

void
line_sampler_resync (line_sampler_t * p_state, uint32_t now_ms)
{
    if (NULL == p_state)
    {
        return;
    }

    p_state->next_ms = now_ms + LINE_SAMPLER_PERIOD_MS;
}

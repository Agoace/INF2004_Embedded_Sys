/**
 * @file line_spin.c
 * @brief Sweep sequencing for calibration and search.
 */
#include "line_spin.h"

#include <stddef.h>

void
line_spin_init (line_spin_t * p_state)
{
    if (NULL == p_state)
    {
        return;
    }

    p_state->mode         = LINE_SPIN_MODE_SEARCH;
    p_state->phase        = LINE_SPIN_PHASE_IDLE;
    p_state->first_sign   = -1;
    p_state->segment_sent = false;
    p_state->timed_out    = false;
    p_state->start_ms     = 0u;
}

void
line_spin_start (line_spin_t * p_state, line_spin_mode_t mode,
                 line_side_t first_side, uint32_t now_ms)
{
    if (NULL == p_state)
    {
        return;
    }

    line_spin_init(p_state);

    p_state->mode       = mode;
    p_state->start_ms   = now_ms;
    p_state->first_sign = (LINE_SIDE_RIGHT == first_side) ? 1 : -1;
    p_state->phase      = LINE_SPIN_PHASE_TO_FIRST;
}

bool
line_spin_collecting (const line_spin_t * p_state)
{
    if (NULL == p_state)
    {
        return false;
    }

    return (LINE_SPIN_PHASE_TO_FIRST == p_state->phase) ||
           (LINE_SPIN_PHASE_TO_SECOND == p_state->phase);
}

bool
line_spin_running (const line_spin_t * p_state)
{
    if (NULL == p_state)
    {
        return false;
    }

    /* Only the three motion phases. Once the machine reaches EVALUATE it has
     * stopped sweeping and is waiting for the caller's outcome, so the caller
     * must stop stepping and call line_spin_finish() instead. */
    return (LINE_SPIN_PHASE_TO_FIRST == p_state->phase) ||
           (LINE_SPIN_PHASE_TO_SECOND == p_state->phase) ||
           (LINE_SPIN_PHASE_RETURN == p_state->phase);
}

bool
line_spin_awaiting_result (const line_spin_t * p_state)
{
    return (NULL != p_state) && (LINE_SPIN_PHASE_EVALUATE == p_state->phase);
}

void
line_spin_finish (line_spin_t * p_state)
{
    if (NULL == p_state)
    {
        return;
    }

    p_state->phase        = LINE_SPIN_PHASE_DONE;
    p_state->segment_sent = false;
}

/** Signed rotation for one phase, in chassis degrees. */
static int16_t
segment_degrees (const line_spin_t * p_state)
{
    switch (p_state->phase)
    {
        case LINE_SPIN_PHASE_TO_FIRST:
        {
            return (int16_t) (p_state->first_sign * LINE_SPIN_SWEEP_DEG);
        }

        case LINE_SPIN_PHASE_TO_SECOND:
        {
            /* Through the start, to the opposite extreme. */
            return (int16_t) (-2 * p_state->first_sign * LINE_SPIN_SWEEP_DEG);
        }

        case LINE_SPIN_PHASE_RETURN:
        {
            return (int16_t) (p_state->first_sign * LINE_SPIN_SWEEP_DEG);
        }

        default:
        {
            return 0;
        }
    }
}

line_spin_cmd_t
line_spin_step (line_spin_t * p_state, bool motion_idle, bool line_centred,
                uint32_t now_ms, int16_t * p_degrees)
{
    if ((NULL == p_state) || (NULL == p_degrees))
    {
        return LINE_SPIN_CMD_NONE;
    }

    if ((LINE_SPIN_PHASE_IDLE == p_state->phase) ||
        (LINE_SPIN_PHASE_DONE == p_state->phase) ||
        (LINE_SPIN_PHASE_EVALUATE == p_state->phase))
    {
        return LINE_SPIN_CMD_NONE;
    }

    /* The whole sequence is bounded, so a sweep that never sees the line still
     * ends and lets the caller fault instead of spinning forever. */
    if ((now_ms - p_state->start_ms) >= LINE_SPIN_TIMEOUT_MS)
    {
        p_state->timed_out = true;
        p_state->phase     = LINE_SPIN_PHASE_DONE;

        return LINE_SPIN_CMD_TIMEOUT;
    }

    if (!p_state->segment_sent)
    {
        if (!motion_idle)
        {
            return LINE_SPIN_CMD_NONE;
        }

        *p_degrees            = segment_degrees(p_state);
        p_state->segment_sent = true;

        return LINE_SPIN_CMD_ROTATE;
    }

    /* Returning: if the line is already back under the centre sensor there is
     * no reason to finish the rotation. Stop, then evaluate. */
    if ((LINE_SPIN_PHASE_RETURN == p_state->phase) && line_centred)
    {
        p_state->segment_sent = false;
        p_state->phase        = LINE_SPIN_PHASE_EVALUATE;

        return LINE_SPIN_CMD_STOP;
    }

    if (!motion_idle)
    {
        return LINE_SPIN_CMD_NONE;
    }

    /* The segment finished. */
    p_state->segment_sent = false;

    switch (p_state->phase)
    {
        case LINE_SPIN_PHASE_TO_FIRST:
        {
            p_state->phase = LINE_SPIN_PHASE_TO_SECOND;
            break;
        }

        case LINE_SPIN_PHASE_TO_SECOND:
        {
            /* The crossings are complete: head back toward the start. */
            p_state->phase = LINE_SPIN_PHASE_RETURN;
            break;
        }

        case LINE_SPIN_PHASE_RETURN:
        {
            /* Rotationally back at the start: the caller decides the outcome. */
            p_state->phase = LINE_SPIN_PHASE_EVALUATE;
            break;
        }

        default:
        {
            p_state->phase = LINE_SPIN_PHASE_EVALUATE;
            break;
        }
    }

    return LINE_SPIN_CMD_NONE;
}

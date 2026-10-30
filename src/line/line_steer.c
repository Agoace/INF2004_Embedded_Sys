/**
 * @file line_steer.c
 * @brief Position to signed-steer mapping, with the upstream P+D and calm logic.
 */
#include "line_steer.h"

#include <stddef.h>

/* Settings are stored in signed 16-bit fields but must never go negative. */
#define LINE_STEER_MAX_GAIN 255

static int16_t
clamp_gain (int16_t value)
{
    if (value < 0)
    {
        return 0;
    }

    return (value > LINE_STEER_MAX_GAIN) ? LINE_STEER_MAX_GAIN : value;
}

/**
 * Calm mode, forked unchanged from upstream.
 *
 * The robot is calm at the start, after a trigger (junction, line search) and
 * while |position| is at or above the enter threshold. It returns to the fast
 * settings after |position| has stayed at or below the near threshold for the
 * hold time.
 */
static bool
calm_update (line_steer_state_t * p_state, int8_t position, bool trigger,
             uint32_t now_ms)
{
    int const magnitude = (position < 0) ? -((int) position) : ((int) position);

    if (trigger || (magnitude >= LINE_STEER_CALM_ENTER))
    {
        p_state->calm = true;
        p_state->near = false;

        return true;
    }

    if (!p_state->calm)
    {
        return false;
    }

    if (magnitude > LINE_STEER_CALM_NEAR)
    {
        p_state->near = false;

        return true;
    }

    if (!p_state->near)
    {
        p_state->near    = true;
        p_state->near_ms = now_ms;
    }

    if ((now_ms - p_state->near_ms) >= LINE_STEER_CALM_HOLD_MS)
    {
        p_state->calm = false;
        p_state->near = false;
    }

    return p_state->calm;
}

void
line_steer_init (line_steer_state_t * p_state,
                 const line_steer_config_t * p_cfg)
{
    if ((NULL == p_state) || (NULL == p_cfg))
    {
        return;
    }

    p_state->cfg.gain_pct = clamp_gain(p_cfg->gain_pct);
    p_state->cfg.kd_pct   = clamp_gain(p_cfg->kd_pct);

    line_steer_reset(p_state);
}

void
line_steer_reset (line_steer_state_t * p_state)
{
    if (NULL == p_state)
    {
        return;
    }

    p_state->previous     = 0;
    p_state->has_previous = false;
    p_state->calm         = true;
    p_state->near         = false;
    p_state->near_ms      = 0u;
}

line_steer_output_t
line_steer_update (line_steer_state_t * p_state, int8_t position, bool trigger,
                   uint32_t now_ms)
{
    line_steer_output_t out;

    out.steer    = 0;
    out.straight = false;
    out.calm     = false;

    if (NULL == p_state)
    {
        return out;
    }

    int32_t change = 0;

    if (p_state->has_previous)
    {
        change = ((int32_t) position) - ((int32_t) p_state->previous);

        if (change > LINE_STEER_MAX_CHANGE)
        {
            change = LINE_STEER_MAX_CHANGE;
        }
        else if (change < -LINE_STEER_MAX_CHANGE)
        {
            change = -LINE_STEER_MAX_CHANGE;
        }
    }

    int32_t const turn = (((int32_t) p_state->cfg.gain_pct) * position) +
                         (((int32_t) p_state->cfg.kd_pct) * change);

    /* steer = round(127 * turn / 10000), rounded away from zero. */
    int32_t steer;

    if (turn >= 0)
    {
        steer = (((int32_t) LINE_STEER_FULL * turn) + 5000) / 10000;
    }
    else
    {
        steer = -((((int32_t) LINE_STEER_FULL * (-turn)) + 5000) / 10000);
    }

    if (steer > LINE_STEER_FULL)
    {
        steer = LINE_STEER_FULL;
    }
    else if (steer < -LINE_STEER_FULL)
    {
        steer = -LINE_STEER_FULL;
    }

    p_state->previous     = position;
    p_state->has_previous = true;

    out.steer = (int8_t) steer;
    out.calm  = calm_update(p_state, position, trigger, now_ms);

    return out;
}

line_steer_output_t
line_steer_straight (line_steer_state_t * p_state, uint32_t now_ms)
{
    line_steer_output_t out;

    out.steer    = 0;
    out.straight = true;
    out.calm     = true;

    if (NULL == p_state)
    {
        return out;
    }

    /* Forgetting the D term here means the next update after the junction has
     * no stored kick, which is what upstream's double reset achieves. */
    line_steer_reset(p_state);

    out.calm = calm_update(p_state, 0, true, now_ms);

    return out;
}

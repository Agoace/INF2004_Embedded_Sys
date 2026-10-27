/**
 * @file manager_button.c
 * @brief See manager_button.h.
 */
#include "manager_button.h"

#include <stddef.h>

void
manager_button_init (manager_button_state_t * p_state, bool down,
                     uint32_t now_ms)
{
    if (NULL == p_state)
    {
        return;
    }

    p_state->down    = down;
    p_state->edge_ms = now_ms;
}

bool
manager_button_edge (manager_button_state_t * p_state, bool down,
                     uint32_t now_ms)
{
    if (NULL == p_state)
    {
        return false;
    }

    /* A repeated level is bounce, or an edge the bank coalesced. */
    if (down == p_state->down)
    {
        return false;
    }

    p_state->down = down;

    if (down)
    {
        p_state->edge_ms = now_ms;
        return false;
    }

    uint32_t const held = now_ms - p_state->edge_ms;

    return (held >= MANAGER_BUTTON_DEBOUNCE_MS) &&
           (held <= MANAGER_BUTTON_SHORT_MAX_MS);
}

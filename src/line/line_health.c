/**
 * @file line_health.c
 * @brief Railed or stuck sensor detection.
 */
#include "line_health.h"

#include <stddef.h>

static uint16_t
rail_ms (uint16_t charge_ms, uint32_t dt_ms)
{
    uint32_t next = (uint32_t) charge_ms + dt_ms;

    if (next > LINE_HEALTH_FAULT_MS)
    {
        next = LINE_HEALTH_FAULT_MS;
    }

    return (uint16_t) next;
}

void
line_health_init (line_health_t * p_state)
{
    if (NULL == p_state)
    {
        return;
    }

    for (uint32_t s = 0u; s < LINE_HEALTH_SENSORS; s++)
    {
        p_state->low_ms[s]  = 0u;
        p_state->high_ms[s] = 0u;
    }

    p_state->fault_mask = 0u;
}

uint8_t
line_health_update (line_health_t * p_state,
                    const uint16_t raw[LINE_HEALTH_SENSORS], uint32_t dt_ms)
{
    if ((NULL == p_state) || (NULL == raw))
    {
        return 0u;
    }

    uint8_t mask = 0u;

    for (uint32_t s = 0u; s < LINE_HEALTH_SENSORS; s++)
    {
        if (raw[s] <= LINE_HEALTH_RAIL_LOW)
        {
            p_state->low_ms[s]  = rail_ms(p_state->low_ms[s], dt_ms);
            p_state->high_ms[s] = 0u;
        }
        else if (raw[s] >= LINE_HEALTH_RAIL_HIGH)
        {
            p_state->high_ms[s] = rail_ms(p_state->high_ms[s], dt_ms);
            p_state->low_ms[s]  = 0u;
        }
        else
        {
            p_state->low_ms[s]  = 0u;
            p_state->high_ms[s] = 0u;
        }

        if ((p_state->low_ms[s] >= LINE_HEALTH_FAULT_MS) ||
            (p_state->high_ms[s] >= LINE_HEALTH_FAULT_MS))
        {
            mask |= (uint8_t) (1u << s);
        }
    }

    p_state->fault_mask = mask;

    return mask;
}

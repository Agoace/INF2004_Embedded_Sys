/**
 * @file line_types.c
 * @brief Packing and unpacking for the Line observation word.
 */
#include "line_types.h"

#include <stddef.h>

/** Position field range. The pack helper clamps into it. */
#define LINE_POSITION_MIN (-100)
#define LINE_POSITION_MAX (100)

line_observation_t
line_obs_pack (const line_obs_fields_t * p_fields)
{
    if (NULL == p_fields)
    {
        return 0u;
    }

    int32_t position = p_fields->position;

    if (position < LINE_POSITION_MIN)
    {
        position = LINE_POSITION_MIN;
    }
    else if (position > LINE_POSITION_MAX)
    {
        position = LINE_POSITION_MAX;
    }

    uint32_t word = ((uint32_t) ((uint8_t) (int8_t) position)
                     << LINE_OBS_POSITION_SHIFT) & LINE_OBS_POSITION_MASK;

    if (p_fields->seen)
    {
        word |= LINE_OBS_SEEN;
    }

    word |= (((uint32_t) p_fields->side) << LINE_OBS_SIDE_SHIFT) &
            LINE_OBS_SIDE_MASK;

    if (p_fields->junction)
    {
        word |= LINE_OBS_JUNCTION;
    }

    if (p_fields->junction_hold)
    {
        word |= LINE_OBS_JUNCTION_HOLD;
    }

    if (p_fields->junction_travel_done)
    {
        word |= LINE_OBS_JUNCTION_TRAVEL_DONE;
    }

    if (p_fields->sensor_fault)
    {
        word |= LINE_OBS_SENSOR_FAULT;
    }

    if (p_fields->calibrated)
    {
        word |= LINE_OBS_CALIBRATED;
    }

    return word;
}

void
line_obs_unpack (line_observation_t word, line_obs_fields_t * p_fields)
{
    if (NULL == p_fields)
    {
        return;
    }

    p_fields->position      = line_obs_position(word);
    p_fields->side          = (uint8_t) line_obs_side(word);
    p_fields->seen          = line_obs_seen(word);
    p_fields->junction      = line_obs_junction(word);
    p_fields->junction_hold = line_obs_junction_hold(word);
    p_fields->junction_travel_done = line_obs_junction_travel_done(word);
    p_fields->sensor_fault  = line_obs_sensor_fault(word);
    p_fields->calibrated    = line_obs_calibrated(word);
}

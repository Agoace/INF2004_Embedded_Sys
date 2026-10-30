/**
 * @file line_producer.c
 * @brief Sensor sampling, observation packing and calibration ownership.
 */
#include "line_producer.h"

#include <stddef.h>
#include <string.h>

/* The two result enums must agree, because the producer and the Manager speak
 * through line_types.h but the learner keeps its own names. */
_Static_assert((int) LP_CALIB_OK == (int) LINE_CALIB_OK,
               "LP_CALIB_OK must match LINE_CALIB_OK");
_Static_assert((int) LP_CALIB_NO_DATA == (int) LINE_CALIB_NO_DATA,
               "LP_CALIB_NO_DATA must match LINE_CALIB_NO_DATA");
_Static_assert((int) LP_CALIB_LOW_CONTRAST == (int) LINE_CALIB_LOW_CONTRAST,
               "LP_CALIB_LOW_CONTRAST must match LINE_CALIB_LOW_CONTRAST");

void
line_producer_init (line_producer_t * p_producer, uint32_t now_ms)
{
    if (NULL == p_producer)
    {
        return;
    }

    memset(p_producer, 0, sizeof(*p_producer));

    lp_init(&p_producer->lp);
    line_health_init(&p_producer->health);
    line_sampler_init(&p_producer->sampler, now_ms);

    p_producer->last_sample_ms = now_ms;
}

void
line_producer_step (line_producer_t * p_producer, uint32_t now_ms,
                    line_producer_out_t * p_out)
{
    if ((NULL == p_producer) || (NULL == p_out))
    {
        return;
    }

    p_out->got_observation = false;
    p_out->has_event       = false;
    p_out->overruns        = p_producer->sampler.overruns;

    if (!line_sampler_due(&p_producer->sampler, now_ms))
    {
        return;
    }

    uint16_t raw[LINE_ADC_SENSORS];

    line_adc_read(raw);

    for (uint32_t s = 0u; s < LINE_ADC_SENSORS; s++)
    {
        p_producer->raw_last[s] = raw[s];
    }

    uint32_t dt_ms = LINE_SAMPLER_PERIOD_MS;

    if (p_producer->have_sample)
    {
        dt_ms = now_ms - p_producer->last_sample_ms;

        if (0u == dt_ms)
        {
            dt_ms = 1u;
        }
    }

    p_producer->last_sample_ms = now_ms;
    p_producer->have_sample    = true;

    /* Health first: a railed sensor makes this sample untrustworthy, and the
     * Manager faults on it. */
    p_producer->fault_mask =
        line_health_update(&p_producer->health, raw, dt_ms);

    lp_event_t const edge = lp_update(&p_producer->lp, raw, now_ms);

    if (LP_EVENT_NONE != edge)
    {
        if (LP_EVENT_FOUND == edge)
        {
            /* A search is not drift: restart the heading once the line is back,
             * as upstream does when it ends a search. */
            p_producer->heading_counts = 0;
        }

        /* The learner reports a line-local edge; the Manager speaks
         * line_event_id_t, so translate once, here. */
        p_out->event.id = (uint8_t) ((LP_EVENT_LOST == edge) ? LINE_EVT_LOST
                                                             : LINE_EVT_FOUND);
        p_out->event.side     = (uint8_t) lp_last_side(&p_producer->lp);
        p_out->event.reserved = 0u;
        p_out->has_event      = true;
    }

    line_producer_observation(p_producer, &p_out->observation);
    p_out->got_observation = true;
}

void
line_producer_observation (const line_producer_t * p_producer,
                           line_observation_t * p_obs)
{
    if ((NULL == p_producer) || (NULL == p_obs))
    {
        return;
    }

    line_obs_fields_t fields;

    memset(&fields, 0, sizeof(fields));

    fields.position      = p_producer->lp.position;
    fields.side          = (uint8_t) lp_last_side(&p_producer->lp);
    fields.seen          = lp_line_seen(&p_producer->lp);
    fields.junction      = (0u != (p_producer->lp.flags & LP_FLAG_JUNCTION));
    fields.junction_hold = (0u != (p_producer->lp.flags &
                                   LP_FLAG_JUNCTION_HOLD));
    fields.junction_travel_done =
        p_producer->junction_active &&
        ((p_producer->mean_counts - p_producer->junction_from) >=
         (int32_t) LINE_JUNCTION_STRAIGHT_COUNTS);
    fields.sensor_fault  = (0u != p_producer->fault_mask);
    fields.calibrated    = lp_calibrated(&p_producer->lp);

    *p_obs = line_obs_pack(&fields);
}

void
line_producer_update_travel (line_producer_t * p_producer, int32_t left_counts,
                             int32_t right_counts)
{
    if (NULL == p_producer)
    {
        return;
    }

    if (p_producer->have_counts)
    {
        int32_t const d_left  = left_counts - p_producer->last_left_counts;
        int32_t const d_right = right_counts - p_producer->last_right_counts;

        /* Upstream accumulates the left-minus-right heading and applies the
         * threshold to the total, so a slow drift of a couple of counts per
         * sample still produces a hint instead of being discarded every time
         * (RT-0038). */
        p_producer->heading_counts += d_left - d_right;

        if (0u != (p_producer->lp.flags & LP_FLAG_CENTRED))
        {
            /* Centred: the drift so far is history, exactly as upstream clears
             * it. */
            p_producer->heading_counts = 0;
        }

        /* The left wheel ahead means the chassis is turning right, so the line
         * moves left: the convention lp_set_hint() documents. The comparison is
         * strict, as upstream's is. */
        if (p_producer->heading_counts > (int32_t) LINE_HINT_COUNTS)
        {
            lp_set_hint(&p_producer->lp, LINE_SIDE_LEFT);
        }
        else if (p_producer->heading_counts < -(int32_t) LINE_HINT_COUNTS)
        {
            lp_set_hint(&p_producer->lp, LINE_SIDE_RIGHT);
        }
        else
        {
            lp_set_hint(&p_producer->lp, LINE_SIDE_UNKNOWN);
        }
    }

    p_producer->last_left_counts  = left_counts;
    p_producer->last_right_counts = right_counts;
    p_producer->mean_counts       = (left_counts + right_counts) / 2;
    p_producer->have_counts       = true;

    /* Junction budget: signed mean travel since the hold's rising edge, so
     * reversing during a junction cannot spend it early. */
    bool const hold = (0u != (p_producer->lp.flags & LP_FLAG_JUNCTION_HOLD));

    if (hold && !p_producer->junction_active)
    {
        p_producer->junction_from = p_producer->mean_counts;
    }

    p_producer->junction_active = hold;
}

void
line_producer_calib_begin (line_producer_t * p_producer)
{
    if (NULL == p_producer)
    {
        return;
    }

    /* Discards the usable calibration: a failed or cancelled sweep must never
     * leave the old endpoints live (RT-0031 B4). */
    lp_calib_begin(&p_producer->lp);

    /* A new run starts with a clean heading. YX declares `heading = 0` at the
     * start of every follow run, so the spin that follows must not inherit a
     * heading from a previous run (RT-0040). lp_calib_begin() has already
     * cleared the hint itself. */
    p_producer->heading_counts = 0;
}

void
line_producer_calib_add (line_producer_t * p_producer,
                         const uint16_t raw[LINE_ADC_SENSORS])
{
    if ((NULL == p_producer) || (NULL == raw))
    {
        return;
    }

    lp_calib_add(&p_producer->lp, raw);
}

const uint16_t *
line_producer_last_raw (const line_producer_t * p_producer)
{
    return (NULL == p_producer) ? NULL : p_producer->raw_last;
}

void
line_producer_calib_finish (line_producer_t * p_producer,
                            line_calib_event_t * p_out)
{    if ((NULL == p_producer) || (NULL == p_out))
    {
        return;
    }

    unsigned int bad_mask = 0u;
    lp_calib_result_t const result = lp_calib_end(&p_producer->lp, &bad_mask);

    /* The sweep turned the chassis, and that is not drift: hand the run a clean
     * heading and no hint, so the first FOLLOW sample cannot inherit the spin.
     * This is the transition YX covers with `heading = 0` after its calibration
     * spin (RT-0040). The FOUND reset in line_producer_step() is unaffected. */
    p_producer->heading_counts = 0;
    lp_set_hint(&p_producer->lp, LINE_SIDE_UNKNOWN);

    memset(p_out, 0, sizeof(*p_out));

    p_out->result          = (uint8_t) result;
    p_out->bad_mask        = (uint8_t) (bad_mask & 0x7u);
    p_out->line_reads_high = lp_line_reads_high(&p_producer->lp) ? 1u : 0u;
}

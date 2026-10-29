/**
 * @file imu_feed.c
 * @brief Encoder travel to IMU distance conversion.
 */
#include "imu_feed.h"

#include "motion_config.h"

/**
 * Denominator for the distance conversion, in the same scaled unit as the
 * numerator:
 *
 *   distance_mm = (d_left + d_right) * CIRCUMFERENCE_UM
 *                 / (2 * COUNTS_PER_REV * 1000)
 *
 * The factor 2 is the wheel averaging and the 1000 converts micrometres to
 * millimetres, so the whole conversion stays in integers and keeps an exact
 * remainder.
 */
#define FEED_DEN \
    (((int64_t) 2) * ((int64_t) MOTION_COUNTS_PER_REV) * 1000)

/** Clamp margin so a corrupt delta can never overflow the int16 field. */
#define FEED_MM_LIMIT 30000

void
imu_feed_init (imu_feed_t * feed)
{
    if (NULL != feed)
    {
        feed->remainder = 0;
    }
}

int16_t
imu_feed_step (imu_feed_t * feed, int16_t d_left, int16_t d_right,
               bool reverse, imu_encoder_input_t * out)
{
    if (NULL == out)
    {
        return 0;
    }

    /* The raw deltas are always published, so the diagnostic shows real wheel
     * motion even while the distance is gated. */
    out->encoder_left_delta  = d_left;
    out->encoder_right_delta = d_right;

    /* RT-0023: reversing must also freeze the terrain classifier. Zeroing the
     * distance alone is not enough, because the pitch-only dwell counters could
     * still start an episode or drive an active one to its crest. */
    out->terrain_pause = reverse;

    if (reverse)
    {
        /* Reverse recovery must not start or modify a hump episode. Drop the
         * carry too, so forward travel cannot resume from a reverse offset. */
        if (NULL != feed)
        {
            feed->remainder = 0;
        }

        out->distance_delta_mm = 0;

        return 0;
    }

    int32_t carry = 0;

    if (NULL != feed)
    {
        carry = feed->remainder;
    }

    int64_t const numerator = (((int64_t) d_left) + ((int64_t) d_right)) *
                              ((int64_t) MOTION_CIRCUMFERENCE_UM) + carry;
    int64_t const whole = numerator / FEED_DEN;

    if (NULL != feed)
    {
        feed->remainder = (int32_t) (numerator - (whole * FEED_DEN));
    }

    int32_t mm = (int32_t) whole;

    if (mm > FEED_MM_LIMIT)
    {
        mm = FEED_MM_LIMIT;
    }
    else if (mm < -FEED_MM_LIMIT)
    {
        mm = -FEED_MM_LIMIT;
    }

    out->distance_delta_mm = (int16_t) mm;

    return (int16_t) mm;
}

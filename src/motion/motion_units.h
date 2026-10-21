/**
 * @file motion_units.h
 * @brief Integer unit conversions shared by the motion core and other tasks.
 *        All results depend on MOTION_WHEEL_DIAMETER_MM (MEASURE).
 */
#ifndef MOTION_UNITS_H
#define MOTION_UNITS_H

#include <stdint.h>

#include "motion_config.h"

/** Signed encoder counts -> millimetres (rounded toward zero). */
static inline int32_t
motion_counts_to_mm (int32_t counts)
{
    return (int32_t) (((int64_t) counts * MOTION_CIRCUMFERENCE_UM) /
                      ((int64_t) MOTION_COUNTS_PER_REV * 1000));
}

/** Signed millimetres -> encoder counts (rounded toward zero). */
static inline int32_t
motion_mm_to_counts (int32_t mm)
{
    return (int32_t) (((int64_t) mm * MOTION_COUNTS_PER_REV * 1000) /
                      MOTION_CIRCUMFERENCE_UM);
}

/** Speed mm/s -> counts per 20 ms tick in Q8 (256 = 1 count/tick). */
static inline int32_t
motion_mm_s_to_cpt_q8 (int32_t mm_s)
{
    return (int32_t) (((int64_t) mm_s * MOTION_COUNTS_PER_REV * 1000 *
                       MOTION_TICK_MS * 256) /
                      ((int64_t) MOTION_CIRCUMFERENCE_UM * 1000));
}

/** Counts per tick -> mm/s. */
static inline int32_t
motion_cpt_to_mm_s (int32_t cpt)
{
    return (int32_t) (((int64_t) cpt * MOTION_CIRCUMFERENCE_UM) /
                      ((int64_t) MOTION_COUNTS_PER_REV * MOTION_TICK_MS));
}

/** Wheel travel (counts, per wheel) for an in-place rotation of deg. */
static inline int32_t
motion_rotate_counts (int32_t deg)
{
    /* arc = deg/360 * pi * W ; counts = arc / (pi * D) * CPR */
    return (int32_t) (((int64_t) deg * MOTION_WHEELBASE_MM *
                       MOTION_COUNTS_PER_REV) /
                      (360 * (int64_t) MOTION_WHEEL_DIAMETER_MM));
}

/** Outer-wheel travel (counts) for a pivot turn of deg about the inner wheel. */
static inline int32_t
motion_pivot_counts (int32_t deg)
{
    return 2 * motion_rotate_counts(deg);
}

#endif /* MOTION_UNITS_H */

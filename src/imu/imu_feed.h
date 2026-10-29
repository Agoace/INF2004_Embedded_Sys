/**
 * @file imu_feed.h
 * @brief Turn Motion's encoder travel into the IMU's distance input.
 *
 * Motion owns the encoder interrupt and the counts, so the integrated firmware
 * reads travel through `motion_consume_travel()` and converts it here. Keeping
 * the arithmetic in this file, away from the board code, is what lets the host
 * tests prove the conversion and the reverse gate.
 *
 * Units: the geometry is shared with Motion rather than duplicated, so
 * MOTION_WHEEL_DIAMETER_MM (65 mm) and MOTION_COUNTS_PER_REV (2535) have one
 * definition.
 *
 * Distance per cycle is the signed average of the two wheel travels:
 *
 *     distance_mm = ((d_left + d_right) / 2) * circumference / counts_per_rev
 *
 * A sub-millimetre remainder is carried between cycles. Without it, low-speed
 * travel would truncate to zero every 13 ms cycle and the hump height would
 * never grow.
 */
#ifndef IMU_FEED_H
#define IMU_FEED_H

#include <stdbool.h>
#include <stdint.h>

#include "imu_types.h"

typedef struct
{
    /** Sub-millimetre carry, in units of (micrometre * FEED_DEN). */
    int32_t remainder;
} imu_feed_t;

/** Reset the carry. */
void imu_feed_init(imu_feed_t * feed);

/**
 * @brief Convert one Motion travel delta into one IMU encoder input.
 *
 * @param feed    Carry state.
 * @param d_left  Signed left count delta since the previous call.
 * @param d_right Signed right count delta since the previous call.
 * @param reverse True while the chassis is reversing, i.e. in a recovery.
 *                Reverse travel must not start or modify a hump episode, so
 *                the published distance is zero and the carry is dropped; the
 *                raw deltas are still published for the diagnostic.
 * @param out     Input to hand to imu_publish_encoder_input().
 *
 * @return Signed millimetres emitted this cycle, zero while reversing.
 */
int16_t imu_feed_step(imu_feed_t * feed, int16_t d_left, int16_t d_right,
                      bool reverse, imu_encoder_input_t * out);

#endif /* IMU_FEED_H */

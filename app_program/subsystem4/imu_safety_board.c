/* CR1 IMU safety board pack/unpack. See imu_safety_board.h. */
#include "imu_safety_board.h"
#include <string.h>

void imu_safety_board_pack(imu_safety_board_t *out,
                           const imu_safety_report_t *report)
{
    const imu_safety_output_t *v = &report->value;
    const uint32_t clear = v->clear_ms > IMU_SAFETY_W2_CLEAR_MASK
                               ? IMU_SAFETY_W2_CLEAR_MASK : v->clear_ms;

    out->word[0] = v->observation_ms;
    out->word[1] = report->applied_request;
    out->word[2] =
        (clear << IMU_SAFETY_W2_CLEAR_SHIFT) |
        (((uint32_t)v->latched_trigger & IMU_SAFETY_W2_LATCHED_MASK)
         << IMU_SAFETY_W2_LATCHED_SHIFT) |
        (((uint32_t)v->flags & IMU_SAFETY_W2_FLAGS_MASK)
         << IMU_SAFETY_W2_FLAGS_SHIFT) |
        (((uint32_t)v->clear_blockers & IMU_SAFETY_W2_BLOCK_MASK)
         << IMU_SAFETY_W2_BLOCK_SHIFT);
}

void imu_safety_board_unpack(imu_safety_report_t *out,
                             const imu_safety_board_t *board)
{
    const uint32_t w2 = board->word[2];

    memset(out, 0, sizeof(*out));
    out->value.observation_ms = board->word[0];
    out->applied_request = board->word[1];
    out->value.clear_ms = (w2 >> IMU_SAFETY_W2_CLEAR_SHIFT) &
                          IMU_SAFETY_W2_CLEAR_MASK;
    out->value.latched_trigger = (uint8_t)((w2 >> IMU_SAFETY_W2_LATCHED_SHIFT) &
                                           IMU_SAFETY_W2_LATCHED_MASK);
    out->value.flags = (uint8_t)((w2 >> IMU_SAFETY_W2_FLAGS_SHIFT) &
                                 IMU_SAFETY_W2_FLAGS_MASK);
    out->value.clear_blockers = (uint8_t)((w2 >> IMU_SAFETY_W2_BLOCK_SHIFT) &
                                          IMU_SAFETY_W2_BLOCK_MASK);
}

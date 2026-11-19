/*
 * CR1 shared IMU safety board: 3 words (12 B), masks and shifts only.
 * imu_safety.c/.h (teammate port) do not change: the IMU task packs its
 * imu_safety_report_t, and imu_safety_try_read() unpacks into the same type.
 *
 * Carried exactly, because a reader uses them (imu_gate.c, imu_safety.c
 * evaluate/cause, bench): observation_ms, applied_request, clear_ms,
 * latched_trigger, flags, clear_blockers.
 *
 * NOT carried (no board reader): tilt_ddeg, g_mg, vertical_mg,
 * overturn_samples, lift_samples, current_trigger, stops. They unpack as 0.
 * Read them from imu_safety_t inside the IMU task (the capture tool does).
 *
 *   W0 31..0 observation_ms    W1 31..0 applied_request
 *   W2  9..0 clear_ms (0..1000)   W2 11..10 latched_trigger
 *   W2 19..12 flags               W2 23..20 clear_blockers   W2 31..24 0
 */
#ifndef IMU_SAFETY_BOARD_H
#define IMU_SAFETY_BOARD_H

#include "imu_safety.h"

#define IMU_SAFETY_BOARD_WORDS 3u
typedef struct {
    uint32_t word[IMU_SAFETY_BOARD_WORDS];
} imu_safety_board_t;

#define IMU_SAFETY_W2_CLEAR_SHIFT 0u
#define IMU_SAFETY_W2_CLEAR_MASK 0x3FFu
#define IMU_SAFETY_W2_LATCHED_SHIFT 10u
#define IMU_SAFETY_W2_LATCHED_MASK 0x3u
#define IMU_SAFETY_W2_FLAGS_SHIFT 12u
#define IMU_SAFETY_W2_FLAGS_MASK 0xFFu
#define IMU_SAFETY_W2_BLOCK_SHIFT 20u
#define IMU_SAFETY_W2_BLOCK_MASK 0xFu

_Static_assert(IMU_SAFETY_CLEAR_MS <= IMU_SAFETY_W2_CLEAR_MASK,
               "clear_ms field too small");
_Static_assert(IMU_SAFETY_IMU_UNAVAILABLE <= IMU_SAFETY_W2_LATCHED_MASK,
               "trigger field too small");
_Static_assert((IMU_SAFETY_BLOCK_NOT_UPRIGHT | IMU_SAFETY_BLOCK_NOT_NEAR_ONE_G |
                IMU_SAFETY_BLOCK_WHEELS_MOVING | IMU_SAFETY_BLOCK_GAP) <=
                   IMU_SAFETY_W2_BLOCK_MASK,
               "blocker field too small");

/* Writer: one report to board words. clear_ms saturates at the field. */
void imu_safety_board_pack(imu_safety_board_t *out,
                           const imu_safety_report_t *report);
/* Reader: board words to a report; the not-carried fields are 0. */
void imu_safety_board_unpack(imu_safety_report_t *out,
                             const imu_safety_board_t *board);

#endif

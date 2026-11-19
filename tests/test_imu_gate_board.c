/* CR1 safety board: run every test_imu_gate.c vector with the gate fed a
 * board copy (pack, then unpack) instead of the report. */
#include <stddef.h>
#include "../app_program/subsystem4/imu_safety_board.h"
#include "../app_program/vehicle/imu_gate.h"

static imu_safety_report_t board_copy;

static const imu_safety_report_t *through_board(const imu_safety_report_t *r)
{
    if (r == NULL) {
        return NULL;
    }
    imu_safety_board_t b;
    imu_safety_board_pack(&b, r);
    imu_safety_board_unpack(&board_copy, &b);
    return &board_copy;
}

#define ig_step(g, r, now, active) ig_step((g), through_board(r), (now), \
                                           (active))
#include "test_imu_gate.c"

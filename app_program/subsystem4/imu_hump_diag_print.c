/* USB only, once after STOP and once per GP20 DUMP. */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "../app.h"
#include "imu_hump_diag.h"

void imu_hump_diag_board_print(const imu_hump_diag_board_t *b)
{
    imu_hump_diag_board_values_t v;

    imu_hump_diag_board_unpack(&v, b);
    tm_printf((UB *)"[S4_HUMP_DIAG] epoch %lu valid %u peak_valid %u "
              "peak_ms %lu travel_mm %ld corrected_0.1mm %ld "
              "filtered_0.1mm %ld speed_mm_s %ld yaw_mrad_s %ld "
              "yaw_accel_mrad_s2 %ld lateral_mg %ld "
              "win_yaw_mrad_s %ld win_yaw_accel_mrad_s2 %ld "
              "win_lateral_mg %ld gate %02x win_gate %02x left %u "
              "line_samples %lu line_lateral_mg %ld "
              "line_yaw_accel_mrad_s2 %ld\n",
              (unsigned long)v.run_epoch, (unsigned int)v.valid,
              (unsigned int)v.peak_valid, (unsigned long)v.ms,
              v.travel_mm, v.corrected_01mm, v.filtered_01mm, v.speed_mm_s,
              v.yaw_mrad_s, v.yaw_accel_mrad_s2, v.lateral_mg,
              v.win_yaw_mrad_s, v.win_yaw_accel_mrad_s2, v.win_lateral_mg,
              (unsigned int)v.gate, (unsigned int)v.win_gate,
              (unsigned int)v.left, (unsigned long)v.line_samples,
              v.line_lateral_mg, v.line_yaw_accel_mrad_s2);
}

void imu_hump_diag_print(const imu_hump_diag_report_t *r)
{
    imu_hump_diag_board_t b;

    imu_hump_diag_pack(&b, r);
    imu_hump_diag_board_print(&b);
}

/* USB only, once after STOP and once per GP20 DUMP. */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "../app.h"
#include "imu_hump_diag.h"
#include <math.h>
#include <limits.h>

/* No printf float support; saturate before conversion. */
static long scaled(float value, float scale)
{
    const float x = value * scale;

    if (!isfinite(x)) {
        return 0L;
    }
    if (x >= 2147483520.0f) {
        return (long)INT32_MAX;
    }
    if (x <= -2147483648.0f) {
        return (long)INT32_MIN;
    }
    return (long)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

void imu_hump_diag_print(const imu_hump_diag_report_t *r)
{
    const imu_hump_peak_diag_t *p = &r->peak;

    tm_printf((UB *)"[S4_HUMP_DIAG] epoch %lu valid %u peak_valid %u "
              "peak_ms %lu travel_mm %ld corrected_0.1mm %ld "
              "filtered_0.1mm %ld speed_mm_s %ld yaw_mrad_s %ld "
              "yaw_accel_mrad_s2 %ld lateral_mg %ld "
              "win_yaw_mrad_s %ld win_yaw_accel_mrad_s2 %ld "
              "win_lateral_mg %ld gate %02x win_gate %02x left %u "
              "line_samples %lu line_lateral_mg %ld "
              "line_yaw_accel_mrad_s2 %ld\n",
              (unsigned long)r->run_epoch, (unsigned int)r->valid,
              (unsigned int)p->valid, (unsigned long)p->ms,
              scaled(p->travel_mm, 1.0f), scaled(p->corrected_mm, 10.0f),
              scaled(p->filtered_mm, 10.0f), scaled(p->speed_mm_s, 1.0f),
              scaled(p->yaw_rad_s, 1000.0f),
              scaled(p->yaw_accel_rad_s2, 1000.0f),
              scaled(p->lateral_mg, 1.0f),
              scaled(p->window_yaw_rad_s, 1000.0f),
              scaled(p->window_yaw_accel_rad_s2, 1000.0f),
              scaled(p->window_lateral_mg, 1.0f),
              (unsigned int)p->gate, (unsigned int)p->window_gate,
              (unsigned int)p->left, (unsigned long)r->line_samples,
              scaled(r->line_lateral_mg, 1.0f),
              scaled(r->line_yaw_accel_rad_s2, 1000.0f));
}

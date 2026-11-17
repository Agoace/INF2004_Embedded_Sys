/*
 * LINE_KD=<0..85> only: force-included (-include) into vehicle_task.o, so
 * vehicle_task.c and vehicle_robot_config.h do not change. The default 85
 * adds nothing (ordinary images stay byte-identical).
 *
 * Why: school run 2026-11-17 (LINE_GAP + MOTION_SAME_TICK), on straights a
 * 2-3 mm drift flips the pair position 0 -> 32 -> 50 in 40 ms; the D term
 * (kd_pct 85, change clipped to STEER_MAX_CHANGE 25) almost doubles the
 * kick. This switch changes only kd_pct; the clip and all other LINE
 * settings are unchanged. See docs/s3-line-barcode/LINE.md.
 */
#ifndef LINE_KD_H
#define LINE_KD_H
#include "vehicle_robot_config.h"

#ifndef LINE_KD
#error "LINE_KD must be defined (0..85)"
#endif
#if LINE_KD < 0 || LINE_KD > 85
#error "LINE_KD must be an integer from 0 to 85"
#endif

/* Copy the reference settings; change only the D gain. */
static inline const lt_config_t *line_kd_config(void)
{
    static lt_config_t candidate;
    candidate = LINE_CFG;
    candidate.steer.kd_pct = (uint8_t)LINE_KD;
    return &candidate;
}

/* Both opt-in headers may stack (LINE_KD + LINE_BASE); each wraps the
 * previous LINE_CFG, so replace the macro without a redefinition warning. */
#undef LINE_CFG
#define LINE_CFG (*line_kd_config())
#endif

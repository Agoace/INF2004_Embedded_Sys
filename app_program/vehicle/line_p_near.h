/* Opt-in vehicle-only configuration. Not included by default builds. */
#ifndef LINE_P_NEAR_H
#define LINE_P_NEAR_H
#include "vehicle_robot_config.h"

#ifndef LINE_P_NEAR
#define LINE_P_NEAR 85
#endif
#ifndef LINE_BASE
#define LINE_BASE 146
#endif
#if LINE_BASE <= 82 || LINE_BASE > 378
#error "LINE_BASE must be an integer greater than 82 and at most 378"
#endif
#if LINE_P_NEAR < 0 || LINE_P_NEAR > 85
#error "LINE_P_NEAR must be an integer from 0 to 85"
#endif

/* Copy the reference settings; change only the selected P gain/base speed.
 * The extra gain on position excess beyond 50 is unchanged, so the
 * requested correction has no discontinuity at either boundary. */
static inline const lt_config_t *line_p_near_config(void)
{
    static lt_config_t candidate;
    candidate = LINE_CFG;
    candidate.steer.gain_pct = LINE_P_NEAR;
    candidate.steer.base_mm_s = LINE_BASE;
    return &candidate;
}
#define LINE_CFG (*line_p_near_config())
#endif

/* Live-only: a claimed LEFT/RIGHT owns its bounded centre advance.
 * Do not run lt_step first: its gap loss would cancel the advance. */
#include "../vehicle/junction_line.h"

vsm_outcome_t blt_line_step(void *ctx, const vsm_inputs_t *in,
                            uint32_t now, vsm_drive_t *drive);
vsm_outcome_t blt_line_step(void *ctx, const vsm_inputs_t *in,
                            uint32_t now, vsm_drive_t *drive)
{
    jl_t *line = ctx;
    ja_t *s = &line->action;
    if (s->phase != JA_BAR_CLEAR ||
        (s->result.command != JA_LEFT && s->result.command != JA_RIGHT)) {
        return jl_step(ctx, in, now, drive);
    }
    (void)ja_step(s, in, &line->sample, now, drive);
    if (s->phase == JA_FAILED) {
        return vsm_outcome(VSM_OUT_FAILED,
                            (uint16_t)(JA_FAIL_BASE + s->result.reason));
    }
    return vsm_outcome(VSM_OUT_RUNNING, 0u);
}

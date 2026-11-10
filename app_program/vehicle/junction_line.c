#include "junction_line.h"

void jl_init(jl_t *line, lt_t *original, const ja_config_t *config)
{
    *line = (jl_t){.line = original};
    ja_init(&line->action, config, 0u);
}

void jl_start(void *ctx, uint16_t payload, uint32_t now)
{
    jl_t *line = ctx;
    lt_start(line->line, payload, now);
    ja_suspend(&line->action);
}

vsm_outcome_t jl_step(void *ctx, const vsm_inputs_t *in, uint32_t now,
                       vsm_drive_t *drive)
{
    jl_t *line = ctx;
    ja_t *action = &line->action;
    if (!ja_active(action)) {
        const vsm_outcome_t out = lt_step(line->line, in, now, drive);
        if (vsm_outcome_type(out) != VSM_OUT_RUNNING) {
            ja_suspend(action);
            return out;
        }
    }
    if (ja_step(action, in, &line->sample, now, drive)) {
        if (action->phase == JA_FAILED) {
            return vsm_outcome(VSM_OUT_FAILED,
                (uint16_t)(JA_FAIL_BASE + action->result.reason));
        }
        if (action->phase == JA_DONE) {
            lt_start(line->line, 0u, now);
        }
    }
    return vsm_outcome(VSM_OUT_RUNNING, 0u);
}

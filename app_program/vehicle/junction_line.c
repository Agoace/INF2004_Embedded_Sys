#include "junction_line.h"

void jl_init(jl_t *line, lt_t *original, int16_t search_speed)
{
    *line = (jl_t){.line = original};
    ja_init(&line->action, search_speed, 0u);
}

void jl_start(void *ctx, uint16_t payload, uint32_t now)
{
    jl_t *line = ctx;
    lt_start(line->line, payload, now);
    ja_suspend(&line->action);
}

static vsm_outcome_t failed(const ja_t *action)
{
    return vsm_outcome(VSM_OUT_FAILED,
        (uint16_t)(JA_FAIL_BASE + action->result.reason));
}

vsm_outcome_t jl_step(void *ctx, const vsm_inputs_t *in, uint32_t now,
                       vsm_drive_t *drive)
{
    jl_t *line = ctx;
    ja_t *action = &line->action;
    vsm_outcome_t out = vsm_outcome(VSM_OUT_RUNNING, 0u);
    if (!ja_searching(action)) {
        out = lt_step(line->line, in, now, drive);
        if (vsm_outcome_type(out) != VSM_OUT_RUNNING) {
            if (ja_active(action)) {
                const vsm_outcome_type_t type = vsm_outcome_type(out);
                ja_abort(action, in->counts, now,
                    type == VSM_OUT_OBSTACLE ? JA_OBSTACLE :
                    type == VSM_OUT_LINE_LOST ? JA_NO_LINE :
                                               JA_SONAR_FAULT);
                *drive = (vsm_drive_t){0};
                return failed(action);
            }
            /* Natural LINE handoff cancels an unconsumed command now,
             * not only when SEARCH/SCAN eventually ends the run. */
            ja_finish(action, in->counts, now);
            return out;
        }
    }
    const bool owned = ja_step(action, in, &line->sample, now, drive);
    if (action->phase == JA_FAILED) {
        return failed(action);
    }
    if (owned && action->phase == JA_DONE) {
        /* Exactly the normal SEARCH -> LINE intent: no settle/centering
         * pass. The qualified raw edge may precede global FOUND timing. */
        lt_start(line->line, 0u, now);
        vsm_inputs_t resumed = *in;
        resumed.line_event = VC_EVENT_NONE;
        out = lt_step(line->line, &resumed, now, drive);
    }
    return out;
}

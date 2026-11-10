/* LINE adapter: the locked LINE implementation is never edited. */
#ifndef JUNCTION_LINE_H
#define JUNCTION_LINE_H

#include "junction_action.h"
#include "../subsystem3/line_trace.h"

typedef struct {
    lt_t *line;
    ja_t action;
    ja_sample_t sample;
} jl_t;

void jl_init(jl_t *line, lt_t *original, int16_t search_speed_mm_s);
void jl_start(void *ctx, uint16_t payload, uint32_t now_ms);
vsm_outcome_t jl_step(void *ctx, const vsm_inputs_t *in, uint32_t now_ms,
                       vsm_drive_t *drive);
static inline vsm_module_t jl_module(jl_t *line)
{
    return (vsm_module_t){jl_start, jl_step, line};
}

#endif

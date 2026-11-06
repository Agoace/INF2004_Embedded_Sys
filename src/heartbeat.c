#include "heartbeat.h"

void hb_init(hb_t *hb) {
    hb->hook_count = 0u;
    hb->flag_count = 0u;
    hb->ticks = 0u;
    hb->last_tick_us = 0u;
    hb->gap_overruns = 0u;
    hb->isr_overruns = 0u;
    hb->isr_max_us = 0u;
    hb->missed_frames = 0u;
    for (unsigned int i = 0u; i < HB_MAX_FLAGS; ++i) {
        hb->posted[i] = 0u;
        hb->taken[i] = 0u;
        hb->flag_cfg[i] = (hb_flag_cfg_t){0u, 0u};
    }
    for (unsigned int i = 0u; i < HB_MAX_HOOKS; ++i) {
        hb->hooks[i] = (hb_hook_t){0, 0, 0u, 0u};
    }
}

int hb_add_hook(hb_t *hb, uint16_t period_ticks, uint16_t phase, hb_hook_fn fn, void *ctx) {
    /* phase >= period also refuses period 0. */
    if (hb->hook_count >= HB_MAX_HOOKS || phase >= period_ticks || !fn) {
        return -1;
    }
    hb->hooks[hb->hook_count] = (hb_hook_t){fn, ctx, period_ticks, phase};
    return (int)hb->hook_count++;
}

int hb_add_flag(hb_t *hb, uint16_t period_ticks, uint16_t phase) {
    if (hb->flag_count >= HB_MAX_FLAGS || phase >= period_ticks) {
        return -1;
    }
    hb->flag_cfg[hb->flag_count] = (hb_flag_cfg_t){period_ticks, phase};
    return (int)hb->flag_count++;
}

void hb_tick(hb_t *hb, uint32_t now_us) {
    const uint32_t t = hb->ticks;
    if (t != 0u && now_us - hb->last_tick_us > HB_TICK_US + HB_TICK_US / 2u) {
        hb->gap_overruns = hb->gap_overruns + 1u;
    }
    hb->last_tick_us = now_us;
    for (unsigned int i = 0u; i < hb->hook_count; ++i) {
        const hb_hook_t *h = &hb->hooks[i];
        if (t % h->period == h->phase) {
            h->fn(now_us, h->ctx);
        }
    }
    for (unsigned int i = 0u; i < hb->flag_count; ++i) {
        if (t % hb->flag_cfg[i].period == hb->flag_cfg[i].phase) {
            hb->posted[i] = (uint16_t)(hb->posted[i] + 1u);
        }
    }
    hb->ticks = t + 1u;
}

void hb_isr_done(hb_t *hb, uint32_t duration_us) {
    if (duration_us > hb->isr_max_us) {
        hb->isr_max_us = duration_us;
    }
    if (duration_us >= HB_TICK_US) {
        hb->isr_overruns = hb->isr_overruns + 1u;
    }
}

uint16_t hb_take(hb_t *hb, int flag) {
    if (flag < 0 || (unsigned int)flag >= hb->flag_count) {
        return 0u;
    }
    const uint16_t posted = hb->posted[flag]; /* one 16-bit read: atomic */
    const uint16_t pending = (uint16_t)(posted - hb->taken[flag]);
    hb->taken[flag] = posted;
    if (pending > 1u) {
        hb->missed_frames += (uint32_t)(pending - 1u);
    }
    return pending;
}

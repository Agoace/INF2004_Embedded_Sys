/*
 * Heartbeat: the one time base for every module
 * (docs/architecture/ADR_HEARTBEAT.md).
 *
 * A hardware timer calls hb_tick() every HB_TICK_US (1 ms) in interrupt
 * context (src/heartbeat_pico.c). Users connect in two ways:
 *
 *   Hook  (hb_add_hook): a short function that runs INSIDE the interrupt
 *         every period_ticks, at tick numbers where
 *         tick % period == phase. For exact, short work only: ADC sample,
 *         mc_tick(). No printf, no blocking call.
 *   Flag  (hb_add_flag): the interrupt posts the flag every period_ticks;
 *         the main loop takes it with hb_take() and does the work there
 *         (detectors, running module, Vehicle Controller, logging).
 *
 * Flags are lock-free: the interrupt is the only writer of posted[], the
 * main loop the only writer of taken[]; pending = posted - taken
 * (wrapping 16-bit counters). No interrupt-off section is needed.
 *
 * Health counters:
 *   gap_overruns   two ticks more than 1.5 tick periods apart (a tick was
 *                  late or lost).
 *   isr_overruns   one tick's interrupt work took a full tick period or more.
 *   isr_max_us     the longest interrupt work seen (hb_isr_done()).
 *   missed_frames  hb_take() found more than one pending post: the main
 *                  loop was too slow for that flag. Counted per missed post.
 *
 * Pure logic: no hardware calls. Host-tested by tests/test_heartbeat.c.
 */
#ifndef HEARTBEAT_H
#define HEARTBEAT_H

#include <stdbool.h>
#include <stdint.h>

#define HB_TICK_US 1000u
#define HB_MAX_HOOKS 4u
#define HB_MAX_FLAGS 8u

typedef void (*hb_hook_fn)(uint32_t now_us, void *ctx);

typedef struct {
    hb_hook_fn fn;
    void *ctx;
    uint16_t period;
    uint16_t phase;
} hb_hook_t;

typedef struct {
    uint16_t period;
    uint16_t phase;
} hb_flag_cfg_t;

typedef struct {
    /* Set up before the timer starts. */
    hb_hook_t hooks[HB_MAX_HOOKS];
    hb_flag_cfg_t flag_cfg[HB_MAX_FLAGS];
    uint8_t hook_count;
    uint8_t flag_count;
    /* Written by the interrupt only. */
    volatile uint32_t ticks;
    volatile uint32_t last_tick_us;
    volatile uint16_t posted[HB_MAX_FLAGS];
    volatile uint32_t gap_overruns;
    volatile uint32_t isr_overruns;
    volatile uint32_t isr_max_us;
    /* Written by the main loop only. */
    uint16_t taken[HB_MAX_FLAGS];
    uint32_t missed_frames;
} hb_t;

void hb_init(hb_t *hb);

/* Returns the hook number, or -1 if full or period is 0 or phase >= period. */
int hb_add_hook(hb_t *hb, uint16_t period_ticks, uint16_t phase, hb_hook_fn fn, void *ctx);

/* Returns the flag number, or -1 if full or period is 0 or phase >= period. */
int hb_add_flag(hb_t *hb, uint16_t period_ticks, uint16_t phase);

/* Interrupt: one tick. Runs the hooks that are due, then posts the flags. */
void hb_tick(hb_t *hb, uint32_t now_us);

/* Interrupt: record how long this tick's work took. */
void hb_isr_done(hb_t *hb, uint32_t duration_us);

/* Main loop: number of posts since the last take (0 = not due), then
 * clear. More than 1 adds the extra posts to missed_frames. */
uint16_t hb_take(hb_t *hb, int flag);

#endif

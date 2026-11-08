/*
 * Bench test for the heartbeat (docs/s2-motion/HEARTBEAT.md). No motors move.
 *
 * Every second, over USB serial: tick count, tick gap (min/max), longest
 * interrupt work, overrun counters, missed main-loop frames.
 *
 *   GP20  hang the main loop for 3 s. Expected: no reset; ticks continue;
 *         missed_frames goes up (the motor controller's stale stop would
 *         act here).
 *   GP21  stop the heartbeat timer. Expected: watchdog reset after
 *         WATCHDOG_MS; the next boot prints "last reset: watchdog".
 */
#include <stdio.h>

#include "heartbeat.h"
#include "heartbeat_pico.h"
#include "pico/stdlib.h"

#define HANG_BUTTON_PIN 20u
#define STOP_BUTTON_PIN 21u
#define WATCHDOG_MS 50u
#define HANG_MS 3000u

typedef struct {
    uint32_t last_us;
    uint32_t gap_min;
    uint32_t gap_max;
    uint32_t calls;
} gap_probe_t;

static hb_t hb;
static gap_probe_t every_tick;
static gap_probe_t every_10;

/* Hook: runs in the interrupt. Records the gap between its calls. */
static void probe(uint32_t now_us, void *ctx) {
    gap_probe_t *p = ctx;
    if (p->calls != 0u) {
        const uint32_t gap = now_us - p->last_us;
        if (gap < p->gap_min) p->gap_min = gap;
        if (gap > p->gap_max) p->gap_max = gap;
    }
    p->last_us = now_us;
    p->calls++;
}

static void probe_reset(gap_probe_t *p) {
    p->gap_min = UINT32_MAX;
    p->gap_max = 0u;
}

static bool pressed(unsigned int pin, bool *previous) {
    const bool down = !gpio_get(pin);
    const bool edge = down && !*previous;
    *previous = down;
    return edge;
}

int main(void) {
    stdio_init_all();
    const unsigned int pins[2] = {HANG_BUTTON_PIN, STOP_BUTTON_PIN};
    for (unsigned int i = 0u; i < 2u; ++i) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_pull_up(pins[i]);
    }
    sleep_ms(2000);
    printf("\nHeartbeat test. Last reset: %s. GP20: hang main loop %u ms. GP21: stop the "
           "heartbeat (watchdog %u ms).\n",
           heartbeat_watchdog_reset() ? "watchdog" : "power or flash", HANG_MS, WATCHDOG_MS);

    hb_init(&hb);
    probe_reset(&every_tick);
    probe_reset(&every_10);
    (void)hb_add_hook(&hb, 1u, 0u, probe, &every_tick);
    (void)hb_add_hook(&hb, 10u, 0u, probe, &every_10);
    const int f_10ms = hb_add_flag(&hb, 10u, 0u);
    const int f_20ms = hb_add_flag(&hb, 20u, 1u);
    const int f_1s = hb_add_flag(&hb, 1000u, 999u);
    if (!heartbeat_start(&hb, WATCHDOG_MS)) {
        printf("heartbeat_start failed\n");
        for (;;) tight_loop_contents();
    }

    bool prev_hang = false;
    bool prev_stop = false;
    uint32_t frames = 0u;
    for (;;) {
        if (hb_take(&hb, f_10ms) != 0u) {
            frames++; /* the 10 ms decision step would run here */
        }
        if (hb_take(&hb, f_20ms) != 0u) {
            if (pressed(HANG_BUTTON_PIN, &prev_hang)) {
                printf("hang %u ms ...\n", HANG_MS);
                busy_wait_ms(HANG_MS);
                printf("hang done, no reset\n");
            }
            if (pressed(STOP_BUTTON_PIN, &prev_stop)) {
                printf("heartbeat stopped; expect a watchdog reset\n");
                heartbeat_stop();
            }
        }
        if (hb_take(&hb, f_1s) != 0u) {
            printf("ticks %lu frames %lu | gap 1ms %lu..%lu us, 10ms %lu..%lu us | isr max %lu us | "
                   "overrun gap %lu isr %lu | missed %lu\n",
                   (unsigned long)hb.ticks, (unsigned long)frames, (unsigned long)every_tick.gap_min,
                   (unsigned long)every_tick.gap_max, (unsigned long)every_10.gap_min,
                   (unsigned long)every_10.gap_max, (unsigned long)hb.isr_max_us,
                   (unsigned long)hb.gap_overruns, (unsigned long)hb.isr_overruns,
                   (unsigned long)hb.missed_frames);
            probe_reset(&every_tick); /* small race with the hook: bench only */
            probe_reset(&every_10);
        }
    }
}

#include "heartbeat_pico.h"

#include "hardware/watchdog.h"
#include "pico/stdlib.h"

static repeating_timer_t timer;
static hb_t *beat;
static bool watchdog_on;
static bool running;

static bool on_tick(repeating_timer_t *t) {
    (void)t;
    const uint32_t start = time_us_32();
    hb_tick(beat, start);
    if (watchdog_on) {
        watchdog_update();
    }
    hb_isr_done(beat, time_us_32() - start);
    return true;
}

bool heartbeat_start(hb_t *hb, uint32_t watchdog_ms) {
    beat = hb;
    /* Negative delay: start-to-start, so the period does not drift by the
     * handler time. */
    running = add_repeating_timer_us(-(int64_t)HB_TICK_US, on_tick, 0, &timer);
    if (running && watchdog_ms != 0u) {
        watchdog_enable(watchdog_ms, true); /* paused while a debugger halts */
        watchdog_on = true;
    }
    return running;
}

void heartbeat_stop(void) {
    if (running) {
        (void)cancel_repeating_timer(&timer);
        running = false;
    }
}

bool heartbeat_watchdog_reset(void) {
    return watchdog_caused_reboot();
}

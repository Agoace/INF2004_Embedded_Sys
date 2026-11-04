/*
 * Duty map bench on the floor (docs/DUTY_MAP.md).
 *
 * Open loop: both wheels get the same fixed duty, forward then reverse,
 * for each level in LEVELS. The steady speed of each wheel is measured
 * over the last part of each drive. This maps the old duty-based settings
 * (line steering, detour) to wheel speeds for the motor controller.
 *
 * This program writes duty directly (drive.c), so it does not use the
 * motor controller. Safety in its place:
 *   - the heartbeat hook is the only PWM writer; it stops both wheels if
 *     the main loop has not refreshed the command for 100 ms;
 *   - the hardware watchdog (heartbeat) resets the chip if the timer stops;
 *   - stall stop: a wheel at 15 % or more with no count for 500 ms;
 *   - GP21 during the run: stop.
 *
 * Each drive starts from rest with a kick (12 % until 10 counts, at most
 * 250 ms), as in the start policy (docs/CALIBRATION_RESULTS.md); a level
 * below the running minimum then slows the wheel to 0 (reported as 0).
 *
 *   GP21  start the run (after 1 s); press again to stop.
 *   GP20  print the last run again.
 *
 * Space: about 70 cm clear ahead and behind (the high levels).
 */
#include <stdio.h>
#include <stdlib.h>

#include "drive.h"
#include "heartbeat.h"
#include "heartbeat_pico.h"
#include "pico/stdlib.h"
#include "wifi_log.h"

#define SPIN_BUTTON_PIN 20u
#define RUN_BUTTON_PIN 21u
#define WATCHDOG_MS 100u
#define START_DELAY_MS 1000u
#define STALE_MS 100u
#define KICK_PCT 12
#define KICK_COUNTS 10
#define KICK_TIMEOUT_MS 250u
#define STALL_MIN_PCT 15
#define STALL_MS 500u
#define REST_MS 500u
#define COUNTS_PER_M 12414
#define DUMP_ROWS_PER_CHUNK 24u

typedef struct {
    uint8_t pct;
    uint16_t settle_ms; /* after the kick, not measured */
    uint16_t measure_ms;
} level_t;

static const level_t LEVELS[] = {
    {5u, 500u, 500u},   {6u, 500u, 500u},   {7u, 500u, 500u},   {8u, 500u, 500u},
    {9u, 500u, 500u},   {10u, 500u, 500u},  {11u, 500u, 500u},  {12u, 500u, 500u},
    {14u, 500u, 500u},  {16u, 500u, 500u},  {18u, 500u, 500u},  {20u, 500u, 500u},
    {25u, 400u, 400u},  {30u, 400u, 400u},  {35u, 400u, 400u},  {40u, 400u, 400u},
    {45u, 300u, 400u},  {50u, 300u, 400u},  {60u, 300u, 300u},  {70u, 300u, 300u},
    {80u, 300u, 300u},  {90u, 300u, 300u},  {100u, 300u, 300u},
};
#define LEVEL_COUNT (sizeof LEVELS / sizeof LEVELS[0])
#define MAX_ROWS 9000u /* 90 s of 10 ms rows */

typedef struct {
    int8_t d[2]; /* counts in this 10 ms tick, forward positive */
    uint8_t level;
    uint8_t phase; /* phase_t */
} row_t;

typedef enum { PH_KICK = 0, PH_SETTLE = 1, PH_MEASURE = 2, PH_REST = 3 } phase_t;

typedef struct {
    int32_t counts[2]; /* measured window */
    uint16_t ms;
    bool kicked;       /* the kick reached 10 counts on both wheels */
} result_t;

static row_t rows[MAX_ROWS];
static unsigned int row_count;
static result_t results[LEVEL_COUNT][2]; /* [level][0 fwd, 1 rev] */
static hb_t hb;

/* Command for the hook (the only PWM writer). */
static volatile int cmd_pct;
static volatile uint32_t cmd_ms;

static uint32_t now_ms(void) {
    return (uint32_t)(time_us_64() / 1000u);
}

static void pwm_hook(uint32_t now_us, void *ctx) {
    (void)now_us;
    (void)ctx;
    const uint32_t now = now_ms();
    const int pct = (now - cmd_ms > STALE_MS) ? 0 : cmd_pct;
    drive_set(pct, pct);
}

static void command(int pct) {
    cmd_pct = pct;
    cmd_ms = now_ms();
}

static bool pressed(unsigned int pin, bool *previous) {
    const bool down = !gpio_get(pin);
    const bool edge = down && !*previous;
    *previous = down;
    return edge;
}

static void idle_ms(uint32_t ms) {
    const uint32_t end = now_ms() + ms;
    while ((int32_t)(now_ms() - end) < 0) {
        wifi_log_poll(now_ms());
        sleep_ms(2);
    }
}

static long mm_s(int32_t counts, uint16_t ms) {
    /* 64-bit: counts x 1e6 is above 2^31 for more than 2147 counts (run DM1, 80 %). */
    return ms ? (long)((int64_t)counts * 1000000 / ((int64_t)COUNTS_PER_M * ms)) : 0;
}

static void print_summary(const char *why, unsigned int levels_done) {
    printf("\n[duty] stop: %s. levels %u of %u, rows %u, isr max %lu us, overruns gap %lu isr %lu\n", why,
           levels_done, (unsigned int)LEVEL_COUNT, row_count, (unsigned long)hb.isr_max_us,
           (unsigned long)hb.gap_overruns, (unsigned long)hb.isr_overruns);
    printf("[duty] pct  fwd L mm/s  fwd R mm/s  rev L mm/s  rev R mm/s  (window ms; * = kick did not reach 10 counts)\n");
    for (unsigned int i = 0u; i < levels_done && i < LEVEL_COUNT; ++i) {
        const result_t *f = &results[i][0];
        const result_t *r = &results[i][1];
        printf("[duty] %3u  %+6ld      %+6ld      %+6ld      %+6ld      (%u, %u)%s%s\n", LEVELS[i].pct,
               mm_s(f->counts[0], f->ms), mm_s(f->counts[1], f->ms), mm_s(r->counts[0], r->ms),
               mm_s(r->counts[1], r->ms), f->ms, r->ms, f->kicked ? "" : " *fwd", r->kicked ? "" : " *rev");
    }
}

static void print_rows(void) {
    printf("[duty] rows: i level phase dL dR (counts per 10 ms; phase 0 kick 1 settle 2 measure 3 rest; "
           "level index, odd drive = reverse)\n");
    for (unsigned int i = 0u; i < row_count; ++i) {
        printf("DM %u %u %u %d %d\n", i, rows[i].level, rows[i].phase, rows[i].d[0], rows[i].d[1]);
        if ((i + 1u) % DUMP_ROWS_PER_CHUNK == 0u) {
            idle_ms(25u);
        }
    }
    printf("[duty] rows end. Wi-Fi packets dropped so far: %lu\n", (unsigned long)wifi_log_dropped());
}

int main(void) {
    stdio_init_all();
    const unsigned int pins[2] = {SPIN_BUTTON_PIN, RUN_BUTTON_PIN};
    for (unsigned int i = 0u; i < 2u; ++i) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_pull_up(pins[i]);
    }
    drive_init();
    hb_init(&hb);
    (void)hb_add_hook(&hb, 10u, 0u, pwm_hook, 0);
    const int f_10ms = hb_add_flag(&hb, 10u, 1u);
    const int f_20ms = hb_add_flag(&hb, 20u, 2u);
    const bool hb_ok = heartbeat_start(&hb, WATCHDOG_MS);
    wifi_log_init();
    idle_ms(2000u);
    printf("\nDuty map bench (floor, open loop). Last reset: %s. heartbeat %s. %u levels, forward then "
           "reverse. GP21: run (press again: stop). GP20: print again.\n",
           heartbeat_watchdog_reset() ? "watchdog" : "power or flash", hb_ok ? "ok" : "FAILED",
           (unsigned int)LEVEL_COUNT);

    bool prev_run = false, prev_dump = false;
    bool running = false, waiting = false;
    uint32_t t0 = 0u, wait_until = 0u;
    unsigned int level = 0u, drive = 0u; /* drive: 2*level + (0 fwd, 1 rev) */
    phase_t phase = PH_REST;
    int32_t last[2] = {0, 0}, start_counts[2] = {0, 0};
    uint32_t still_ms[2] = {0u, 0u};
    for (;;) {
        const uint32_t now = now_ms();
        wifi_log_poll(now);
        if (hb_take(&hb, f_20ms) != 0u) {
            const bool run_edge = pressed(RUN_BUTTON_PIN, &prev_run);
            const bool dump_edge = pressed(SPIN_BUTTON_PIN, &prev_dump);
            if (!running && !waiting && run_edge) {
                waiting = true;
                wait_until = now + START_DELAY_MS;
                printf("[duty] start in %u ms\n", START_DELAY_MS);
            } else if (!running && !waiting && dump_edge && row_count) {
                print_summary("print again", level);
                print_rows();
            } else if ((running || waiting) && run_edge) {
                command(0);
                running = waiting = false;
                print_summary("GP21 pressed", level);
                print_rows();
            }
        }
        if (hb_take(&hb, f_10ms) == 0u) {
            continue;
        }
        if (waiting && (int32_t)(now - wait_until) >= 0) {
            waiting = false;
            running = true;
            row_count = 0u;
            level = 0u;
            drive = 0u;
            phase = PH_KICK;
            t0 = now;
            drive_counts(&last[0], &last[1]);
            for (unsigned int l = 0u; l < LEVEL_COUNT; ++l) {
                results[l][0] = results[l][1] = (result_t){{0, 0}, 0u, false};
            }
            still_ms[0] = still_ms[1] = 0u;
        }
        if (!running) {
            command(0);
            continue;
        }
        int32_t c[2];
        drive_counts(&c[0], &c[1]);
        const int32_t d[2] = {c[0] - last[0], c[1] - last[1]};
        last[0] = c[0];
        last[1] = c[1];
        const int sign = (drive & 1u) ? -1 : +1;
        const level_t *lv = &LEVELS[level];
        result_t *res = &results[level][drive & 1u];
        const uint32_t in_phase = now - t0;
        const char *why = 0;

        /* Stall stop. */
        const int duty_now = phase == PH_KICK ? KICK_PCT : (phase == PH_REST ? 0 : lv->pct);
        for (unsigned int w = 0u; w < 2u; ++w) {
            if (duty_now >= STALL_MIN_PCT && d[w] == 0) {
                still_ms[w] += 10u;
                if (still_ms[w] >= STALL_MS) why = w ? "right wheel stalled" : "left wheel stalled";
            } else {
                still_ms[w] = 0u;
            }
        }
        if (row_count < MAX_ROWS) {
            rows[row_count++] = (row_t){{(int8_t)(d[0] > 127 ? 127 : d[0] < -127 ? -127 : d[0]),
                                         (int8_t)(d[1] > 127 ? 127 : d[1] < -127 ? -127 : d[1])},
                                        (uint8_t)drive, (uint8_t)phase};
        }
        /* Phase changes. */
        if (phase == PH_KICK) {
            if (in_phase == 0u) {
                start_counts[0] = c[0];
                start_counts[1] = c[1];
            }
            const bool done = labs((long)(c[0] - start_counts[0])) >= KICK_COUNTS &&
                              labs((long)(c[1] - start_counts[1])) >= KICK_COUNTS;
            if (done || in_phase >= KICK_TIMEOUT_MS) {
                res->kicked = done;
                phase = PH_SETTLE;
                t0 = now;
            }
        } else if (phase == PH_SETTLE && in_phase >= lv->settle_ms) {
            phase = PH_MEASURE;
            t0 = now;
            start_counts[0] = c[0];
            start_counts[1] = c[1];
        } else if (phase == PH_MEASURE && in_phase >= lv->measure_ms) {
            res->counts[0] = c[0] - start_counts[0];
            res->counts[1] = c[1] - start_counts[1];
            res->ms = (uint16_t)in_phase;
            phase = PH_REST;
            t0 = now;
        } else if (phase == PH_REST && in_phase >= REST_MS) {
            drive++;
            level = drive / 2u;
            if (level >= LEVEL_COUNT) {
                why = "list done";
            } else {
                phase = PH_KICK;
                t0 = now;
                start_counts[0] = c[0];
                start_counts[1] = c[1];
            }
        }
        if (why) {
            command(0);
            running = false;
            print_summary(why, level >= LEVEL_COUNT ? (unsigned int)LEVEL_COUNT : level);
            print_rows();
            continue;
        }
        const int duty = phase == PH_KICK ? KICK_PCT : (phase == PH_REST ? 0 : LEVELS[level].pct);
        command(sign * duty);
    }
}

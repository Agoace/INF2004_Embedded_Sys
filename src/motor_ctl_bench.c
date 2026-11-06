/*
 * Motor controller bench test on the floor (docs/MOTOR_CONTROLLER.md, step 3).
 *
 * Drives a fixed list of wheel speed steps through the motor controller
 * (src/motor_ctl_pico.c) and records one row per 10 ms tick in RAM. After
 * the run it prints a summary and the rows; with src/wifi_config.h the
 * printf output also goes over Wi-Fi to scripts/steer_log_receiver.py
 * (text_<boot>.txt).
 *
 *   GP21  start the run (after 1 s); press again to stop it (emergency stop).
 *   GP20  print the last run again.
 *
 * Space: about 40 cm clear ahead of and behind the robot, and room to
 * turn on the spot. The net travel of the list is about 0.
 *
 * A run stops early on any latched fault (stall or emergency stop).
 */
#include <stdio.h>
#include <stdlib.h>

#include "heartbeat.h"
#include "heartbeat_pico.h"
#include "motor_ctl_pico.h"
#include "motor_ctl_robot.h"
#include "pico/stdlib.h"
#include "wifi_log.h"

#define SPIN_BUTTON_PIN 20u
#define RUN_BUTTON_PIN 21u
#define WATCHDOG_MS 100u
#define START_DELAY_MS 1000u
#define DUMP_ROWS_PER_CHUNK 16u

typedef struct {
    const char *name;
    uint16_t ms;
    int16_t left;  /* mm/s */
    int16_t right; /* mm/s */
} step_t;

/* Forward and back in pairs, so the robot ends near its start. */
static const step_t STEPS[] = {
    {"rest", 500u, 0, 0},
    {"fwd 150", 1500u, 150, 150},     {"rest", 700u, 0, 0},
    {"rev 150", 1500u, -150, -150},   {"rest", 700u, 0, 0},
    {"fwd 300", 1000u, 300, 300},     {"rest", 700u, 0, 0},
    {"rev 300", 1000u, -300, -300},   {"rest", 700u, 0, 0},
    {"spin L 20", 3000u, -20, 20},    {"rest", 700u, 0, 0}, /* about 60 degrees */
    {"spin R 20", 3000u, 20, -20},    {"rest", 700u, 0, 0},
    {"fwd 40", 2000u, 40, 40},        {"rest", 700u, 0, 0},
    {"rev 40", 2000u, -40, -40},      {"rest", 700u, 0, 0},
};
#define STEP_COUNT (sizeof STEPS / sizeof STEPS[0])
#define MAX_ROWS 2400u /* 24 s; the list is 21.4 s */

typedef struct {
    int32_t count[MC_WHEELS];
    int16_t target[MC_WHEELS];
    int16_t speed[MC_WHEELS];
    int16_t duty[MC_WHEELS];
    uint8_t step;
    uint8_t faults;
} row_t;

static row_t rows[MAX_ROWS];
static unsigned int row_count;
static hb_t hb;
static int f_10ms;
static int f_20ms;

typedef enum { BENCH_IDLE, BENCH_WAIT, BENCH_RUN } bench_mode_t;

static bool pressed(unsigned int pin, bool *previous) {
    const bool down = !gpio_get(pin);
    const bool edge = down && !*previous;
    *previous = down;
    return edge;
}

/* Keep the Wi-Fi going while the main loop waits. */
static void idle_ms(uint32_t ms) {
    const uint32_t end = mcp_now_ms() + ms;
    while ((int32_t)(mcp_now_ms() - end) < 0) {
        wifi_log_poll(mcp_now_ms());
        sleep_ms(2);
    }
}

static void print_summary(const char *why, int32_t heading_end_cdeg) {
    mcp_status_t s;
    mcp_status(&s);
    printf("\n[bench] stop: %s. rows %u, faults 0x%02x, hook max %lu us, heartbeat isr max %lu us, "
           "overruns gap %lu isr %lu, missed %lu\n",
           why, row_count, s.faults, (unsigned long)s.tick_max_us, (unsigned long)hb.isr_max_us,
           (unsigned long)hb.gap_overruns, (unsigned long)hb.isr_overruns, (unsigned long)hb.missed_frames);
    printf("[bench] heading at the end %+ld cdeg\n", (long)heading_end_cdeg);
    printf("[bench] step          target L/R   speed 2nd half L/R   duty 2nd half L/R   heading change\n");
    unsigned int start = 0u;
    while (start < row_count) {
        const uint8_t st = rows[start].step;
        unsigned int end = start;
        while (end < row_count && rows[end].step == st) end++;
        const unsigned int half = start + (end - start) / 2u;
        long sp[2] = {0, 0}, du[2] = {0, 0};
        for (unsigned int r = half; r < end; ++r) {
            for (unsigned int w = 0u; w < MC_WHEELS; ++w) {
                sp[w] += rows[r].speed[w];
                du[w] += rows[r].duty[w];
            }
        }
        const long n = (long)(end - half);
        /* Heading from the counts: (right - left) / counts per degree. */
        const long dl = (long)(rows[end - 1u].count[MC_LEFT] - rows[start].count[MC_LEFT]);
        const long dr = (long)(rows[end - 1u].count[MC_RIGHT] - rows[start].count[MC_RIGHT]);
        const long head_cdeg = (dr - dl) * 10000L / 2405L;
        printf("[bench] %-12s %+5d/%+5d   %+5ld/%+5ld          %+4ld/%+4ld permille  %+6ld cdeg\n",
               STEPS[st].name, STEPS[st].left, STEPS[st].right, n ? sp[0] / n : 0, n ? sp[1] / n : 0,
               n ? du[0] / n : 0, n ? du[1] / n : 0, head_cdeg);
        start = end;
    }
}

static void print_rows(void) {
    printf("[bench] rows: i step tgtL tgtR spdL spdR dutyL dutyR cntL cntR faults (mm/s, permille, counts)\n");
    for (unsigned int r = 0u; r < row_count; ++r) {
        const row_t *x = &rows[r];
        printf("MB %u %u %d %d %d %d %d %d %ld %ld %u\n", r, x->step, x->target[0], x->target[1],
               x->speed[0], x->speed[1], x->duty[0], x->duty[1], (long)x->count[0], (long)x->count[1],
               x->faults);
        if ((r + 1u) % DUMP_ROWS_PER_CHUNK == 0u) {
            idle_ms(25u); /* let the Wi-Fi text buffer (2 KB) drain */
        }
    }
    printf("[bench] rows end. Wi-Fi packets dropped so far: %lu\n", (unsigned long)wifi_log_dropped());
}

int main(void) {
    stdio_init_all();
    const unsigned int pins[2] = {SPIN_BUTTON_PIN, RUN_BUTTON_PIN};
    for (unsigned int i = 0u; i < 2u; ++i) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_pull_up(pins[i]);
    }
    hb_init(&hb);
    static const mc_config_t cfg = MC_ROBOT_CONFIG;
    const bool mc_ok = mcp_init(&hb, &cfg); /* motors 0, servo centre */
    f_10ms = hb_add_flag(&hb, 10u, 1u);     /* phase 1: just after the mc_tick at phase 0 */
    f_20ms = hb_add_flag(&hb, 20u, 2u);
    const bool hb_ok = heartbeat_start(&hb, WATCHDOG_MS);
    wifi_log_init();
    idle_ms(2000u);
    printf("\nMotor controller bench (floor). Last reset: %s. mcp_init %s, heartbeat %s.\n"
           "GP21: run %u steps (press again: stop). GP20: print the last run again.\n",
           heartbeat_watchdog_reset() ? "watchdog" : "power or flash", mc_ok ? "ok" : "FAILED",
           hb_ok ? "ok" : "FAILED", (unsigned int)STEP_COUNT);
    if (!mc_ok || !hb_ok) {
        for (;;) idle_ms(1000u);
    }

    bool prev_run = false, prev_dump = false;
    bench_mode_t mode = BENCH_IDLE;
    uint32_t wait_until = 0u, step_start = 0u;
    unsigned int step = 0u;
    for (;;) {
        const uint32_t now = mcp_now_ms();
        wifi_log_poll(now);
        if (hb_take(&hb, f_20ms) != 0u) {
            const bool run_edge = pressed(RUN_BUTTON_PIN, &prev_run);
            const bool dump_edge = pressed(SPIN_BUTTON_PIN, &prev_dump);
            if (mode == BENCH_IDLE && run_edge) {
                mode = BENCH_WAIT;
                wait_until = now + START_DELAY_MS;
                printf("[bench] start in %u ms\n", START_DELAY_MS);
            } else if (mode == BENCH_IDLE && dump_edge && row_count) {
                print_summary("print again", 0);
                print_rows();
            } else if (mode != BENCH_IDLE && run_edge) {
                mcp_estop();
                mcp_status_t s;
                mcp_status(&s);
                mode = BENCH_IDLE;
                print_summary("GP21 pressed", s.heading_cdeg);
                print_rows();
                mcp_clear_faults();
            }
        }
        if (hb_take(&hb, f_10ms) == 0u) {
            continue;
        }
        if (mode == BENCH_WAIT && (int32_t)(now - wait_until) >= 0) {
            mcp_clear_faults();
            row_count = 0u;
            step = 0u;
            step_start = now;
            mode = BENCH_RUN;
        }
        if (mode != BENCH_RUN) {
            continue;
        }
        while (step < STEP_COUNT && now - step_start >= STEPS[step].ms) {
            step_start += STEPS[step].ms;
            step++;
        }
        mcp_status_t s;
        mcp_status(&s);
        const char *why = 0;
        if (s.faults & MC_FAULT_LATCHED) {
            why = (s.faults & MC_FAULT_STALL_LEFT)    ? "left wheel stalled"
                  : (s.faults & MC_FAULT_STALL_RIGHT) ? "right wheel stalled"
                                                      : "emergency stop";
        } else if (step >= STEP_COUNT) {
            why = "list done";
        } else if (row_count >= MAX_ROWS) {
            why = "row buffer full";
        }
        if (why) {
            mcp_stop();
            mode = BENCH_IDLE;
            print_summary(why, s.heading_cdeg);
            print_rows();
            mcp_clear_faults();
            continue;
        }
        mcp_set_speed(STEPS[step].left, STEPS[step].right);
        row_t *x = &rows[row_count++];
        for (unsigned int w = 0u; w < MC_WHEELS; ++w) {
            x->count[w] = s.counts[w];
            x->speed[w] = s.speed_mm_s[w];
            x->duty[w] = s.duty_permille[w];
        }
        x->target[MC_LEFT] = STEPS[step].left;
        x->target[MC_RIGHT] = STEPS[step].right;
        x->step = (uint8_t)step;
        x->faults = s.faults;
    }
}

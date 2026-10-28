/*
 * Bench test: follow the line with proportional steering (src/line_steer.c).
 *
 *   GP20  spin calibration (src/line_spin.c)
 *   GP21  start line following (after 1 s) / stop
 *
 * A run stops on: GP21, LINE_LOST, a stalled wheel, or RUN_LIMIT_MS.
 * There is no line search yet: LINE_LOST stops the robot.
 *
 * Power: a driving test. Use 5 V VIN with USB disconnected
 * (USB_POWER_SAFETY.md). The run is logged in RAM every 50 ms. Connect USB
 * after the robot stops and open the serial port: the log prints once, then
 * the summary repeats every 5 s.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "drive.h"
#include "hardware/gpio.h"
#include "line_position.h"
#include "line_sensors.h"
#include "line_spin.h"
#include "line_steer.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

#define SPIN_BUTTON_PIN 20
#define RUN_BUTTON_PIN 21

/* Steering settings for this test. */
static const steer_config_t STEER = {
    .base_pct = 25u, /* both wheels when centred */
    .gain_pct = 100u, /* line under an outer sensor: inner wheel stops */
    .kd_pct = 0u,    /* D off; add it if the robot zig-zags */
    .min_pct = 12u,  /* below this a loaded wheel does not turn */
    .max_pct = 50u,
};

#define SAMPLE_PERIOD_US 1000u
#define STEER_PERIOD_MS 10u
#define LOG_PERIOD_MS 50u
#define RUN_LIMIT_MS 20000u
#define START_DELAY_MS 1000u
#define STALL_GRACE_MS 1000u
#define STALL_TIMEOUT_MS 500u
#define BUTTON_POLL_MS 20u
#define REPORT_REPEAT_MS 5000u
#define LOG_ENTRIES (RUN_LIMIT_MS / LOG_PERIOD_MS)

typedef enum {
    STOP_BUTTON,
    STOP_LINE_LOST,
    STOP_STALL_LEFT,
    STOP_STALL_RIGHT,
    STOP_TIME_LIMIT,
} stop_reason_t;

static const char *const STOP_NAMES[] = {
    "GP21 pressed", "LINE_LOST", "left wheel stalled", "right wheel stalled",
    "time limit",
};

typedef struct {
    int8_t position;
    uint8_t left_pct;
    uint8_t right_pct;
    int16_t left_counts;  /* counts in the last 50 ms */
    int16_t right_counts;
} log_entry_t;

/* |position| buckets for the summary. */
#define BUCKETS 5u
static const char *const BUCKET_NAMES[BUCKETS] = {"0", "1-20", "21-50", "51-99", "100"};

static unsigned int bucket_of(int position) {
    const int a = abs(position);
    return a == 0 ? 0u : a <= 20 ? 1u : a <= 50 ? 2u : a < 100 ? 3u : 4u;
}

typedef struct {
    bool valid;
    stop_reason_t reason;
    vc_line_side_t lost_side;
    uint32_t duration_ms;
    int32_t left_counts;
    int32_t right_counts;
    uint32_t steer_updates;
    uint32_t sum_abs_position;
    uint32_t bucket[BUCKETS];
    unsigned int log_count;
    log_entry_t log[LOG_ENTRIES];
} run_t;

static run_t run;
static lp_t lp;

static bool button_pressed(unsigned int pin, bool *previous) {
    const bool down = !gpio_get(pin);
    const bool edge = down && !*previous;
    *previous = down;
    return edge;
}

static void print_summary(void) {
    printf("[run] %lu ms, stopped by %s", (unsigned long)run.duration_ms,
           STOP_NAMES[run.reason]);
    if (run.reason == STOP_LINE_LOST) {
        printf(" (last seen %s)", run.lost_side == VC_LINE_SIDE_LEFT    ? "LEFT"
                                  : run.lost_side == VC_LINE_SIDE_RIGHT ? "RIGHT"
                                                                        : "unknown");
    }
    printf("\n[run] wheels L %ld  R %ld counts (%.2f / %.2f wheel turns)\n",
           (long)run.left_counts, (long)run.right_counts,
           (double)run.left_counts / DRIVE_COUNTS_PER_WHEEL_TURN,
           (double)run.right_counts / DRIVE_COUNTS_PER_WHEEL_TURN);
    printf("[run] calibration (floor/line): L %u/%u  C %u/%u  R %u/%u\n", lp_floor(&lp, 0u),
           lp_line(&lp, 0u), lp_floor(&lp, 1u), lp_line(&lp, 1u), lp_floor(&lp, 2u),
           lp_line(&lp, 2u));
    printf("[run] steering: base %u%% gain %u%% kd %u%% min %u%% max %u%%\n",
           STEER.base_pct, STEER.gain_pct, STEER.kd_pct, STEER.min_pct, STEER.max_pct);
    if (run.steer_updates > 0u) {
        printf("[run] mean |pos| %lu;  |pos| share:",
               (unsigned long)(run.sum_abs_position / run.steer_updates));
        for (unsigned int b = 0u; b < BUCKETS; ++b) {
            printf("  %s: %lu%%", BUCKET_NAMES[b],
                   (unsigned long)(run.bucket[b] * 100u / run.steer_updates));
        }
        printf("\n");
    }
}

static void print_log(void) {
    printf("\n[log] t_ms  pos  dutyL dutyR  countsL countsR (per 50 ms)\n");
    for (unsigned int i = 0u; i < run.log_count; ++i) {
        const log_entry_t *e = &run.log[i];
        printf("[log] %5u %+4d  %3u %3u  %5d %5d\n", i * LOG_PERIOD_MS, e->position,
               e->left_pct, e->right_pct, e->left_counts, e->right_counts);
    }
}

static void follow_line(lp_t *line, steer_t *steer, bool *previous_run) {
    run = (run_t){0};
    steer_reset(steer);
    printf("[run] start\n");

    const uint32_t t0 = to_ms_since_boot(get_absolute_time());
    int32_t start[2];
    drive_counts(&start[0], &start[1]);
    int32_t last_log[2] = {start[0], start[1]};
    int32_t last_count[2] = {start[0], start[1]};
    uint32_t moved_ms[2] = {t0, t0};
    unsigned int duty[2] = {0u, 0u};
    absolute_time_t next_sample = get_absolute_time();
    uint32_t next_steer_ms = t0;
    uint32_t next_log_ms = t0 + LOG_PERIOD_MS;
    uint32_t next_button_ms = t0;
    stop_reason_t reason = STOP_TIME_LIMIT;

    while (true) {
        const uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - t0 >= RUN_LIMIT_MS) {
            reason = STOP_TIME_LIMIT;
            break;
        }

        if (time_reached(next_sample)) {
            next_sample = delayed_by_us(next_sample, SAMPLE_PERIOD_US);
            uint16_t raw[LP_SENSORS];
            line_sensors_read(raw);
            const vc_event_t event = lp_update(line, raw, now);
            if (vc_event_type(event) == VC_EVENT_LINE_LOST) {
                run.lost_side = (vc_line_side_t)vc_event_payload(event);
                reason = STOP_LINE_LOST;
                break;
            }
        }

        if ((int32_t)(now - next_steer_ms) >= 0) {
            next_steer_ms += STEER_PERIOD_MS;
            const steer_word_t w = steer_update(steer, line->position);
            duty[0] = steer_left(w);
            duty[1] = steer_right(w);
            drive_set((int)duty[0], (int)duty[1]);
            ++run.steer_updates;
            run.sum_abs_position += (uint32_t)abs(line->position);
            ++run.bucket[bucket_of(line->position)];
        }

        int32_t c[2];
        drive_counts(&c[0], &c[1]);
        bool stalled[2];
        for (unsigned int w = 0u; w < 2u; ++w) {
            if (c[w] != last_count[w] || duty[w] == 0u) {
                last_count[w] = c[w];
                moved_ms[w] = now;
            }
            stalled[w] = now - t0 >= STALL_GRACE_MS && now - moved_ms[w] >= STALL_TIMEOUT_MS;
        }
        if (stalled[0] || stalled[1]) {
            reason = stalled[0] ? STOP_STALL_LEFT : STOP_STALL_RIGHT;
            break;
        }

        if ((int32_t)(now - next_log_ms) >= 0) {
            next_log_ms += LOG_PERIOD_MS;
            if (run.log_count < LOG_ENTRIES) {
                run.log[run.log_count++] = (log_entry_t){
                    .position = line->position,
                    .left_pct = (uint8_t)duty[0],
                    .right_pct = (uint8_t)duty[1],
                    .left_counts = (int16_t)(c[0] - last_log[0]),
                    .right_counts = (int16_t)(c[1] - last_log[1]),
                };
            }
            last_log[0] = c[0];
            last_log[1] = c[1];
        }

        if ((int32_t)(now - next_button_ms) >= 0) {
            next_button_ms = now + BUTTON_POLL_MS;
            if (button_pressed(RUN_BUTTON_PIN, previous_run)) {
                reason = STOP_BUTTON;
                break;
            }
        }
    }

    drive_stop();
    const uint32_t end = to_ms_since_boot(get_absolute_time());
    int32_t c[2];
    drive_counts(&c[0], &c[1]);
    run.reason = reason;
    run.duration_ms = end - t0;
    run.left_counts = c[0] - start[0];
    run.right_counts = c[1] - start[1];
    run.valid = true;
    print_summary();
}

int main(void) {
    stdio_init_all();
    line_sensors_init();
    drive_init();
    drive_stop();
    const unsigned int buttons[2] = {SPIN_BUTTON_PIN, RUN_BUTTON_PIN};
    for (unsigned int i = 0u; i < 2u; ++i) {
        gpio_init(buttons[i]);
        gpio_set_dir(buttons[i], GPIO_IN);
        gpio_pull_up(buttons[i]);
    }
    sleep_ms(2000);
    printf("\nLine steering test. GP20: spin calibration. GP21: start/stop.\n");

    lp_init(&lp);
    steer_t steer;
    steer_init(&steer, &STEER);

    bool previous_spin = false;
    bool previous_run = false;
    bool was_connected = false;
    uint32_t next_report_ms = 0u;

    while (true) {
        const uint32_t now = to_ms_since_boot(get_absolute_time());

        if (button_pressed(SPIN_BUTTON_PIN, &previous_spin)) {
            (void)line_spin_calibrate(&lp);
            printf("\n");
        }

        if (button_pressed(RUN_BUTTON_PIN, &previous_run)) {
            if (!lp_calibrated(&lp)) {
                printf("[run] Not calibrated. Press GP20 first.\n");
            } else {
                sleep_ms(START_DELAY_MS);
                follow_line(&lp, &steer, &previous_run);
                next_report_ms = to_ms_since_boot(get_absolute_time()) + REPORT_REPEAT_MS;
            }
        }

        /* Print the log once when a terminal connects after a run. */
        const bool connected = stdio_usb_connected();
        if (connected && !was_connected && run.valid) {
            sleep_ms(500);
            print_summary();
            print_log();
            next_report_ms = now + REPORT_REPEAT_MS;
        }
        was_connected = connected;
        if (connected && run.valid && (int32_t)(now - next_report_ms) >= 0) {
            next_report_ms = now + REPORT_REPEAT_MS;
            print_summary();
        }

        sleep_ms(BUTTON_POLL_MS);
    }
}

/*
 * Bench test for the normalised line position (src/line_position.c).
 *
 * The test prints the line position every 100 ms. LINE_LOST and
 * LINE_FOUND events are printed at once. Calibrate first:
 *
 *   GP20  spin calibration (src/line_spin.c): the robot turns on the spot
 *         a quarter wheel turn left and right, then back onto the line
 *   GP21  hand calibration: for 4 s, slide the robot across the line
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "drive.h"
#include "hardware/gpio.h"
#include "line_position.h"
#include "line_sensors.h"
#include "line_spin.h"
#include "pico/stdlib.h"

#define SPIN_BUTTON_PIN 20
#define HAND_BUTTON_PIN 21

#define SAMPLE_PERIOD_US 1000u
#define CALIBRATE_MS 4000u
#define PRINT_PERIOD_MS 100u
#define BUTTON_POLL_MS 20u

/* 21-character bar: '|' at the centre, '^' at the line position. */
static void print_bar(int position) {
    char bar[22];
    for (int i = 0; i < 21; ++i) bar[i] = '-';
    bar[10] = '|';
    bar[(position + 100) / 10] = '^';
    bar[21] = '\0';
    printf("L %s R", bar);
}

static void hand_calibrate(lp_t *lp) {
    printf("\n[calib] Slide the robot slowly across the line, left and right,\n");
    printf("[calib] so every sensor passes over it. %u s...\n", CALIBRATE_MS / 1000u);
    lp_calib_begin(lp);

    absolute_time_t next_sample = get_absolute_time();
    const absolute_time_t end = make_timeout_time_ms(CALIBRATE_MS);
    absolute_time_t next_tick = make_timeout_time_ms(1000u);
    unsigned int seconds_left = CALIBRATE_MS / 1000u;
    while (!time_reached(end)) {
        if (time_reached(next_sample)) {
            next_sample = delayed_by_us(next_sample, SAMPLE_PERIOD_US);
            uint16_t raw[LP_SENSORS];
            line_sensors_read(raw);
            lp_calib_add(lp, raw);
        }
        if (time_reached(next_tick)) {
            next_tick = delayed_by_ms(next_tick, 1000u);
            --seconds_left;
            printf("[calib] %u s  range L %u-%u  C %u-%u  R %u-%u\n", seconds_left,
                   lp->cal_min[0], lp->cal_max[0], lp->cal_min[1], lp->cal_max[1],
                   lp->cal_min[2], lp->cal_max[2]);
        }
    }
    (void)line_calib_end_and_report(lp);
    printf("\n");
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    printf("\nNormalised line position test\n");
    printf("L/C/R = GP28/GP27/GP26 (see line_sensors.c). GP20: spin calibration.\n"
           "GP21: hand calibration.\n");

    line_sensors_init();
    drive_init();
    gpio_init(SPIN_BUTTON_PIN);
    gpio_set_dir(SPIN_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(SPIN_BUTTON_PIN);
    gpio_init(HAND_BUTTON_PIN);
    gpio_set_dir(HAND_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(HAND_BUTTON_PIN);

    lp_t lp;
    lp_init(&lp);
    printf("Not calibrated. Put the robot on the line and press GP20 (spin)\n");
    printf("or GP21 (slide by hand).\n\n");

    bool previous_spin = false;
    bool previous_hand = false;
    uint16_t raw[LP_SENSORS] = {0u, 0u, 0u};
    absolute_time_t next_sample = get_absolute_time();
    absolute_time_t next_print = make_timeout_time_ms(PRINT_PERIOD_MS);
    absolute_time_t next_button_poll = get_absolute_time();

    while (true) {
        if (time_reached(next_sample)) {
            next_sample = delayed_by_us(next_sample, SAMPLE_PERIOD_US);
            line_sensors_read(raw);
            const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            const vc_event_t event = lp_update(&lp, raw, now_ms);
            if (vc_event_type(event) == VC_EVENT_LINE_LOST) {
                const vc_line_side_t side = (vc_line_side_t)vc_event_payload(event);
                printf(">>> LINE_LOST  last seen %s\n",
                       side == VC_LINE_SIDE_LEFT    ? "LEFT"
                       : side == VC_LINE_SIDE_RIGHT ? "RIGHT"
                                                    : "CENTRE (unknown side)");
            } else if (vc_event_type(event) == VC_EVENT_LINE_FOUND) {
                printf(">>> LINE_FOUND\n");
            }
        }

        if (time_reached(next_print)) {
            next_print = delayed_by_ms(next_print, PRINT_PERIOD_MS);
            printf("raw %4u %4u %4u  n %4u %4u %4u  ", raw[0], raw[1], raw[2],
                   lp.norm[0], lp.norm[1], lp.norm[2]);
            if (lp_line_seen(&lp)) {
                printf("pos %+4d  ", lp.position);
                print_bar(lp.position);
                printf("%s\n", lp_junction(&lp) ? "  JUNCTION" : "");
            } else {
                printf("no line\n");
            }
        }

        if (time_reached(next_button_poll)) {
            next_button_poll = make_timeout_time_ms(BUTTON_POLL_MS);
            const bool spin_button = !gpio_get(SPIN_BUTTON_PIN);
            const bool hand_button = !gpio_get(HAND_BUTTON_PIN);
            if ((spin_button && !previous_spin) || (hand_button && !previous_hand)) {
                if (spin_button) {
                    (void)line_spin_calibrate(&lp);
                    printf("\n");
                } else {
                    hand_calibrate(&lp);
                }
                next_sample = get_absolute_time();
                next_print = make_timeout_time_ms(PRINT_PERIOD_MS);
            }
            previous_spin = spin_button;
            previous_hand = hand_button;
        }

        tight_loop_contents();
    }
}

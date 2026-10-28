/*
 * Bench test for the normalised line position (src/line_position.c).
 *
 * 1. Calibration: for 4 s, slide the robot slowly across the line so that
 *    every sensor passes over it (left-right, two or three times).
 * 2. Then the test prints the line position every 100 ms. LINE_LOST and
 *    LINE_FOUND events are printed at once.
 *
 *   GP20  calibrate again
 *   GP21  pause or resume the 100 ms output (events still print)
 *
 * The motors are not driven.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "ir_module_pins.h"
#include "line_position.h"
#include "pico/stdlib.h"

#define CALIBRATE_BUTTON_PIN 20
#define PAUSE_BUTTON_PIN 21

#define SAMPLE_PERIOD_US 1000u
#define SAMPLES_PER_SENSOR 8u
#define CALIBRATE_MS 4000u
#define PRINT_PERIOD_MS 100u
#define BUTTON_POLL_MS 20u

static const unsigned int sensor_gpio[LP_SENSORS] = {
    LINE_IR_LEFT_ANALOG_GPIO,
    LINE_IR_CENTRE_ANALOG_GPIO,
    LINE_IR_RIGHT_ANALOG_GPIO,
};

static void read_sensors(uint16_t raw[LP_SENSORS]) {
    for (unsigned int s = 0u; s < LP_SENSORS; ++s) {
        adc_select_input(sensor_gpio[s] - PICO_ADC_GPIO_MIN);
        uint32_t sum = 0u;
        for (unsigned int i = 0u; i < SAMPLES_PER_SENSOR; ++i) {
            sum += adc_read();
        }
        raw[s] = (uint16_t)(sum / SAMPLES_PER_SENSOR);
    }
}

/* 21-character bar: '|' at the centre, '^' at the line position. */
static void print_bar(int position) {
    char bar[22];
    for (int i = 0; i < 21; ++i) bar[i] = '-';
    bar[10] = '|';
    bar[(position + 100) / 10] = '^';
    bar[21] = '\0';
    printf("L %s R", bar);
}

static void calibrate(lp_t *lp) {
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
            read_sensors(raw);
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

    unsigned int bad = 0u;
    const lp_calib_result_t result = lp_calib_end(lp, &bad);
    if (result == LP_CALIB_OK) {
        printf("[calib] OK. Line reads %s than the floor.\n",
               lp_line_reads_high(lp) ? "HIGHER" : "LOWER");
        for (unsigned int s = 0u; s < LP_SENSORS; ++s) {
            printf("[calib]   %c: floor %4u  line %4u\n", "LCR"[s], lp_floor(lp, s),
                   lp_line(lp, s));
        }
    } else if (result == LP_CALIB_LOW_CONTRAST) {
        printf("[calib] FAILED: range below %u ADC on:%s%s%s. Press GP20 to retry.\n",
               LP_MIN_CONTRAST_ADC, (bad & 1u) ? " L" : "", (bad & 2u) ? " C" : "",
               (bad & 4u) ? " R" : "");
        if (lp_calibrated(lp)) {
            printf("[calib] The previous calibration stays in use.\n");
        }
    } else {
        printf("[calib] FAILED: no data. Press GP20 to retry.\n");
    }
    printf("\n");
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    printf("\nNormalised line position test\n");
    printf("L/C/R = GP%u/GP%u/GP%u. GP20: calibrate again. GP21: pause output.\n",
           LINE_IR_LEFT_ANALOG_GPIO, LINE_IR_CENTRE_ANALOG_GPIO,
           LINE_IR_RIGHT_ANALOG_GPIO);

    adc_init();
    for (unsigned int s = 0u; s < LP_SENSORS; ++s) {
        adc_gpio_init(sensor_gpio[s]);
    }
    gpio_init(CALIBRATE_BUTTON_PIN);
    gpio_set_dir(CALIBRATE_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(CALIBRATE_BUTTON_PIN);
    gpio_init(PAUSE_BUTTON_PIN);
    gpio_set_dir(PAUSE_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(PAUSE_BUTTON_PIN);

    lp_t lp;
    lp_init(&lp);
    calibrate(&lp);

    bool paused = false;
    bool previous_calibrate = false;
    bool previous_pause = false;
    uint16_t raw[LP_SENSORS] = {0u, 0u, 0u};
    absolute_time_t next_sample = get_absolute_time();
    absolute_time_t next_print = make_timeout_time_ms(PRINT_PERIOD_MS);
    absolute_time_t next_button_poll = get_absolute_time();

    while (true) {
        if (time_reached(next_sample)) {
            next_sample = delayed_by_us(next_sample, SAMPLE_PERIOD_US);
            read_sensors(raw);
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

        if (!paused && time_reached(next_print)) {
            next_print = delayed_by_ms(next_print, PRINT_PERIOD_MS);
            printf("raw %4u %4u %4u  n %3u %3u %3u %%  ", raw[0], raw[1], raw[2],
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
            const bool calibrate_button = !gpio_get(CALIBRATE_BUTTON_PIN);
            const bool pause_button = !gpio_get(PAUSE_BUTTON_PIN);
            if (calibrate_button && !previous_calibrate) {
                calibrate(&lp);
                next_sample = get_absolute_time();
                next_print = make_timeout_time_ms(PRINT_PERIOD_MS);
            }
            if (pause_button && !previous_pause) {
                paused = !paused;
                printf("[output] %s\n", paused ? "paused" : "resumed");
            }
            previous_calibrate = calibrate_button;
            previous_pause = pause_button;
        }

        tight_loop_contents();
    }
}

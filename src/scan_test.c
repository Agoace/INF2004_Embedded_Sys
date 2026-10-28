/*
 * Bench test for the servo scan (src/obstacle_scan.c).
 *
 *   GP20  run one scan
 *   GP21  repeat a scan every 2 s on or off
 *
 * The servo moves. Keep fingers away from the sensor mount.
 * The motors are not driven.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "hardware/gpio.h"
#include "obstacle_profile.h"
#include "obstacle_scan.h"
#include "pico/stdlib.h"
#include "ultrasonic.h"

#define ULTRASONIC_ECHO_PIN 2
#define ULTRASONIC_TRIGGER_PIN 3
#define SERVO_PIN 15
#define SCAN_BUTTON_PIN 20
#define REPEAT_BUTTON_PIN 21

#define REPEAT_GAP_MS 2000u
#define BUTTON_POLL_MS 20u
/* One '#' per 20 mm in the bar chart. */
#define BAR_MM 20u

static void print_point(unsigned int index, obstacle_scan_point_t point) {
    const int angle = obstacle_scan_angle_deg(index);
    const ultrasonic_status_t status = ultrasonic_result_status(point);
    const uint16_t distance = ultrasonic_result_distance(point);

    printf("  idx %u  %2d deg %-6s %4u us  ", index, abs(angle),
           angle < 0 ? "right" : angle > 0 ? "left" : "ahead",
           obstacle_scan_pulse_us(index));
    if (status == ULTRASONIC_STATUS_VALID && distance == 0u) {
        printf("TOO_NEAR   #\n");
    } else if (status == ULTRASONIC_STATUS_VALID) {
        printf("%3u mm     ", distance);
        for (unsigned int i = 0; i < (distance + BAR_MM - 1u) / BAR_MM; ++i) {
            putchar('#');
        }
        putchar('\n');
    } else if (status == ULTRASONIC_STATUS_NO_RETURN) {
        printf("NO_RETURN  (nothing within 511 mm)\n");
    } else {
        printf("NO DATA    (both readings failed)\n");
    }
}

static void print_free(const char *name, unsigned int step) {
    if (step == OBSTACLE_PROFILE_BLOCKED) {
        printf("%s blocked", name);
    } else {
        printf("%s free at %u deg", name, step * (unsigned int)OBSTACLE_SCAN_STEP_DEG);
    }
}

static void print_summary(const obstacle_profile_t *profile, uint32_t took_ms) {
    unsigned int nearest = OBSTACLE_SCAN_POINTS;
    uint16_t nearest_mm = 0u;

    for (unsigned int i = 0; i < OBSTACLE_SCAN_POINTS; ++i) {
        const obstacle_scan_point_t p = profile->point[i];
        if (ultrasonic_result_status(p) != ULTRASONIC_STATUS_VALID) {
            continue;
        }
        if (nearest == OBSTACLE_SCAN_POINTS ||
            ultrasonic_result_distance(p) < nearest_mm) {
            nearest = i;
            nearest_mm = ultrasonic_result_distance(p);
        }
    }
    printf("[scan] done in %lu ms. ", (unsigned long)took_ms);
    if (nearest == OBSTACLE_SCAN_POINTS) {
        printf("No echo at any point.\n");
    } else {
        printf("Nearest: idx %u at %u mm.\n", nearest, nearest_mm);
    }

    const obstacle_profile_word_t w = obstacle_profile_build(profile);
    const obstacle_side_t side = obstacle_profile_side(w, profile);
    printf("[profile] word 0x%04X  obstacle %u mm  ", w,
           obstacle_profile_distance(w));
    print_free("right", obstacle_profile_right(w));
    printf("  ");
    print_free("left", obstacle_profile_left(w));
    printf("  -> go %s\n\n", side == OBSTACLE_SIDE_LEFT    ? "LEFT"
                              : side == OBSTACLE_SIDE_RIGHT ? "RIGHT"
                                                            : "NONE (reverse and rescan)");
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    printf("\nServo scan test\n");
    printf("%u points, %d deg steps, %u readings per point (nearest kept).\n",
           OBSTACLE_SCAN_POINTS, OBSTACLE_SCAN_STEP_DEG, OBSTACLE_SCAN_READINGS);
    printf("GP20: one scan. GP21: repeat every %u ms on/off.\n\n",
           REPEAT_GAP_MS);

    gpio_init(SCAN_BUTTON_PIN);
    gpio_set_dir(SCAN_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(SCAN_BUTTON_PIN);
    gpio_init(REPEAT_BUTTON_PIN);
    gpio_set_dir(REPEAT_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(REPEAT_BUTTON_PIN);

    ultrasonic_init(ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PIN, NULL);
    obstacle_scan_init(SERVO_PIN);

    bool repeat = false;
    bool previous_scan = false;
    bool previous_repeat = false;
    absolute_time_t next_button_poll = get_absolute_time();
    absolute_time_t next_repeat = get_absolute_time();
    uint32_t started_ms = 0u;

    while (true) {
        const obstacle_scan_event_t event = obstacle_scan_service();
        if (event == OBSTACLE_SCAN_POINT) {
            const unsigned int done = obstacle_scan_progress();
            print_point(done - 1u, obstacle_scan_profile()->point[done - 1u]);
        } else if (event == OBSTACLE_SCAN_DONE) {
            print_summary(obstacle_scan_profile(),
                          to_ms_since_boot(get_absolute_time()) - started_ms);
            next_repeat = make_timeout_time_ms(REPEAT_GAP_MS);
        } else if (event == OBSTACLE_SCAN_FAULT) {
            printf("[scan] SENSOR FAULT: scan stopped, servo to centre.\n\n");
            next_repeat = make_timeout_time_ms(REPEAT_GAP_MS);
        }

        bool start = false;
        if (time_reached(next_button_poll)) {
            next_button_poll = make_timeout_time_ms(BUTTON_POLL_MS);
            const bool scan_button = !gpio_get(SCAN_BUTTON_PIN);
            const bool repeat_button = !gpio_get(REPEAT_BUTTON_PIN);
            if (scan_button && !previous_scan) {
                start = true;
            }
            if (repeat_button && !previous_repeat) {
                repeat = !repeat;
                printf("[repeat] %s\n", repeat ? "on" : "off");
                next_repeat = get_absolute_time();
            }
            previous_scan = scan_button;
            previous_repeat = repeat_button;
        }
        if (repeat && time_reached(next_repeat)) {
            start = true;
        }

        if (start && obstacle_scan_start()) {
            started_ms = to_ms_since_boot(get_absolute_time());
            printf("[scan] right to left\n");
        }

        sleep_us(200);
    }
}

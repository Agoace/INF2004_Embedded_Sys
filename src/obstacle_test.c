/*
 * Bench test for the forward obstacle watch (src/obstacle.c).
 *
 * The HC-SR04+ is triggered every 100 ms. Each result goes through
 * obstacle_update(). Events are printed at once; a status line is printed
 * every second.
 *
 *   GP21  show every reading on or off
 *   GP20  reset the monitor (also clears a latched SENSOR_FAULT)
 *
 * This test does not drive the motors or the servo.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "obstacle.h"
#include "pico/stdlib.h"
#include "ultrasonic.h"

#define ULTRASONIC_ECHO_PIN 2
#define ULTRASONIC_TRIGGER_PIN 3
#define RESET_BUTTON_PIN 20
#define EVERY_BUTTON_PIN 21

#define MEASURE_PERIOD_US 100000u
#define STATUS_PERIOD_MS 1000u
#define BUTTON_POLL_MS 20u

static const char *state_name(obstacle_state_t state) {
    switch (state) {
    case OBSTACLE_MONITORING:
        return "MONITORING";
    case OBSTACLE_APPROACHING:
        return "APPROACHING";
    case OBSTACLE_CONFIRMED:
        return "CONFIRMED";
    case OBSTACLE_SENSOR_FAULT:
        return "SENSOR_FAULT";
    }
    return "?";
}

static const char *event_name(obstacle_event_t event) {
    switch (event) {
    case OBSTACLE_EVENT_NONE:
        return "NONE";
    case OBSTACLE_EVENT_CLEAR:
        return "OBST_CLEAR";
    case OBSTACLE_EVENT_NEAR:
        return "OBST_NEAR";
    case OBSTACLE_EVENT_STOP:
        return "OBST_STOP";
    case OBSTACLE_EVENT_SENSOR_FAULT:
        return "SENSOR_FAULT";
    }
    return "?";
}

/* Short text for one ultrasonic result, for example "245 mm". */
static void print_reading(ultrasonic_result_t result) {
    const ultrasonic_status_t status = ultrasonic_result_status(result);
    if (ultrasonic_result_too_near(result)) {
        printf("TOO_NEAR");
    } else if (status == ULTRASONIC_STATUS_VALID) {
        printf("%u mm", ultrasonic_result_distance(result));
    } else if (status == ULTRASONIC_STATUS_NO_RETURN) {
        printf("NO_RETURN");
    } else if (ultrasonic_result_is_none(result)) {
        printf("no result");
    } else {
        printf("%s streak %u",
               status == ULTRASONIC_STATUS_NO_RISE ? "NO_RISE" : "STUCK_HIGH",
               ultrasonic_result_streak(result));
    }
}

static void handle(obstacle_monitor_t *monitor, ultrasonic_result_t result,
                   bool every, ultrasonic_result_t *last) {
    const obstacle_event_word_t out = obstacle_update(monitor, result);
    const obstacle_event_t event = obstacle_event(out);
    const double seconds = (double)to_us_since_boot(get_absolute_time()) / 1e6;

    *last = result;
    if (every) {
        printf("  %8.1f s  ", seconds);
        print_reading(result);
        printf("  -> %s\n", state_name(obstacle_state(monitor)));
    }
    if (event != OBSTACLE_EVENT_NONE) {
        printf(">>> %8.1f s  %-12s word 0x%04X  reading ", seconds,
               event_name(event), out);
        print_reading(result);
        printf("  state %s\n", state_name(obstacle_state(monitor)));
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    printf("\nObstacle watch test\n");
    printf("STOP below %u mm at the sensor (%u mm + %u mm robot front). "
           "NEAR below %u mm.\n",
           OBSTACLE_STOP_MM, OBSTACLE_STOP_FROM_FRONT_MM,
           OBSTACLE_SENSOR_TO_FRONT_MM, OBSTACLE_NEAR_MM);
    printf("Leave a range %u mm past its threshold. %u readings in a row "
           "confirm a change.\n",
           OBSTACLE_HYSTERESIS_MM, OBSTACLE_CONFIRM_COUNT);
    printf("GP21: every reading on/off. GP20: reset the monitor.\n\n");

    gpio_init(RESET_BUTTON_PIN);
    gpio_set_dir(RESET_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(RESET_BUTTON_PIN);
    gpio_init(EVERY_BUTTON_PIN);
    gpio_set_dir(EVERY_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(EVERY_BUTTON_PIN);

    ultrasonic_init(ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PIN, NULL);

    obstacle_monitor_t monitor;
    obstacle_init(&monitor);

    bool every = false;
    bool previous_reset = false;
    bool previous_every = false;
    ultrasonic_result_t last = ULTRASONIC_RESULT_NONE;

    absolute_time_t next_trigger = get_absolute_time();
    absolute_time_t next_status = make_timeout_time_ms(STATUS_PERIOD_MS);
    absolute_time_t next_button_poll = get_absolute_time();

    while (true) {
        if (time_reached(next_trigger)) {
            next_trigger = delayed_by_us(next_trigger, MEASURE_PERIOD_US);
            /* A STUCK_HIGH found at trigger time is published at once. */
            if (ultrasonic_trigger() && !ultrasonic_busy()) {
                handle(&monitor, ultrasonic_latest(), every, &last);
            }
        }

        if (ultrasonic_service()) {
            handle(&monitor, ultrasonic_latest(), every, &last);
        }

        if (!every && time_reached(next_status)) {
            next_status = delayed_by_ms(next_status, STATUS_PERIOD_MS);
            printf("    state %-12s last ", state_name(obstacle_state(&monitor)));
            print_reading(last);
            printf("\n");
        }

        if (time_reached(next_button_poll)) {
            next_button_poll = make_timeout_time_ms(BUTTON_POLL_MS);
            const bool reset_button = !gpio_get(RESET_BUTTON_PIN);
            const bool every_button = !gpio_get(EVERY_BUTTON_PIN);

            if (reset_button && !previous_reset) {
                obstacle_init(&monitor);
                printf("[reset] state MONITORING\n");
            }
            if (every_button && !previous_every) {
                every = !every;
                printf("[every] reading output %s\n", every ? "on" : "off");
            }
            previous_reset = reset_button;
            previous_every = every_button;
        }

        sleep_us(200);
    }
}

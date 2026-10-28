#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "ultrasonic.h"

/*
 * HC-SR04+ bench test for the non-blocking ultrasonic driver.
 *
 * The program triggers every 100 ms, the same period as the planned Obstacle
 * Task. Echo times come from the GPIO interrupt, so USB printing does not
 * change the measurement.
 *
 * GP20: record a batch of BATCH_SIZE readings and print statistics.
 * GP21: turn the live per-reading output on or off.
 */

#define ULTRASONIC_ECHO_PIN 2
#define ULTRASONIC_TRIGGER_PIN 3
#define BATCH_BUTTON_PIN 20
#define LIVE_BUTTON_PIN 21

#define MEASURE_PERIOD_US 100000u
#define BUTTON_POLL_MS 20u
#define BATCH_SIZE 50u

typedef struct {
    bool active;
    uint32_t taken;
    uint32_t status_count[4];
    uint32_t far_count; /* NO_RETURN with an echo beyond the field. */
    uint32_t echo_count;
    uint32_t mm_min;
    uint32_t mm_max;
    double mm_sum;
    double mm_sum_sq;
    uint32_t rise_min;
    uint32_t rise_max;
} batch_t;

static const char *status_name(ultrasonic_status_t status) {
    switch (status) {
        case ULTRASONIC_STATUS_VALID:
            return "VALID";
        case ULTRASONIC_STATUS_NO_RISE:
            return "NO_RISE";
        case ULTRASONIC_STATUS_STUCK_HIGH:
            return "STUCK_HIGH";
        case ULTRASONIC_STATUS_NO_RETURN:
            return "NO_RETURN";
        default:
            return "?";
    }
}

static void batch_start(batch_t *batch) {
    *batch = (batch_t){0};
    batch->active = true;
    batch->mm_min = UINT32_MAX;
    batch->rise_min = UINT32_MAX;
    printf("\n[batch] recording %u readings; keep the target still\n",
           BATCH_SIZE);
}

static void batch_add(batch_t *batch, ultrasonic_result_t result,
                      ultrasonic_debug_t debug) {
    const ultrasonic_status_t status = ultrasonic_result_status(result);
    ++batch->status_count[status];
    ++batch->taken;

    /* Statistics use the unbounded diagnostic distance, so a test target
     * beyond the 511 mm field can still be measured. */
    const uint32_t mm = debug.distance_mm;
    if (mm == 0u) {
        return;
    }
    if (status == ULTRASONIC_STATUS_NO_RETURN) {
        ++batch->far_count;
    }
    ++batch->echo_count;
    batch->mm_min = mm < batch->mm_min ? mm : batch->mm_min;
    batch->mm_max = mm > batch->mm_max ? mm : batch->mm_max;
    batch->mm_sum += mm;
    batch->mm_sum_sq += (double)mm * mm;
    batch->rise_min = debug.rise_latency_us < batch->rise_min
                          ? debug.rise_latency_us
                          : batch->rise_min;
    batch->rise_max = debug.rise_latency_us > batch->rise_max
                          ? debug.rise_latency_us
                          : batch->rise_max;
}

static void batch_report(const batch_t *batch) {
    const uint32_t echoes = batch->echo_count;

    printf("[batch] %lu readings: VALID %lu, NO_RETURN %lu "
           "(echo beyond field %lu), NO_RISE %lu, STUCK_HIGH %lu\n",
           (unsigned long)batch->taken,
           (unsigned long)batch->status_count[ULTRASONIC_STATUS_VALID],
           (unsigned long)batch->status_count[ULTRASONIC_STATUS_NO_RETURN],
           (unsigned long)batch->far_count,
           (unsigned long)batch->status_count[ULTRASONIC_STATUS_NO_RISE],
           (unsigned long)batch->status_count[ULTRASONIC_STATUS_STUCK_HIGH]);

    if (echoes == 0u) {
        printf("[batch] no echo measured; distance statistics unavailable\n\n");
        return;
    }

    const double mean = batch->mm_sum / echoes;
    double variance = batch->mm_sum_sq / echoes - mean * mean;
    if (variance < 0.0) {
        variance = 0.0;
    }
    printf("[batch] distance mm (%lu echoes): mean %.1f, std %.1f, "
           "min %lu, max %lu\n",
           (unsigned long)echoes, mean, sqrt(variance), (unsigned long)batch->mm_min,
           (unsigned long)batch->mm_max);
    printf("[batch] trigger-to-echo latency us: min %lu, max %lu\n\n",
           (unsigned long)batch->rise_min, (unsigned long)batch->rise_max);
}

static void print_live(uint32_t sequence, ultrasonic_result_t result,
                       ultrasonic_debug_t debug) {
    const ultrasonic_status_t status = ultrasonic_result_status(result);
    const uint16_t code = ultrasonic_result_distance(result);

    printf("#%lu %-10s word 0x%04X streak %u%s", (unsigned long)sequence,
           status_name(status), result, ultrasonic_result_streak(result),
           ultrasonic_result_fault(result) ? " FAULT" : "");
    if (ultrasonic_result_too_near(result)) {
        printf("  TOO_NEAR (<%u mm, raw %lu mm)  pulse %lu us  rise %lu us",
               ULTRASONIC_DISTANCE_MIN_MM, (unsigned long)debug.distance_mm,
               (unsigned long)debug.pulse_us,
               (unsigned long)debug.rise_latency_us);
    } else if (status == ULTRASONIC_STATUS_VALID) {
        printf("  %u mm  pulse %lu us  rise %lu us", code,
               (unsigned long)debug.pulse_us,
               (unsigned long)debug.rise_latency_us);
    } else if (status == ULTRASONIC_STATUS_NO_RETURN) {
        printf("  distance unknown  rise %lu us",
               (unsigned long)debug.rise_latency_us);
        if (debug.distance_mm != 0u) {
            printf(" (echo at %lu mm, beyond the field)",
                   (unsigned long)debug.distance_mm);
        } else {
            printf(" (no echo before the 3 ms timeout)");
        }
    } else if (status == ULTRASONIC_STATUS_STUCK_HIGH) {
        printf("  distance unknown  (echo HIGH before trigger)");
    } else {
        printf("  distance unknown  (no echo rise; check wiring/power)");
    }
    printf("\n");
}

static void initialise_button(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    initialise_button(BATCH_BUTTON_PIN);
    initialise_button(LIVE_BUTTON_PIN);
    ultrasonic_init(ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PIN, NULL);

    printf("\nHC-SR04+ ultrasonic test (GP3 trigger, GP2 echo)\n");
    printf("Trigger period %u ms. Distance field %u-%u mm; nearer is "
           "TOO_NEAR, farther is NO_RETURN.\n",
           MEASURE_PERIOD_US / 1000u, ULTRASONIC_DISTANCE_MIN_MM,
           ULTRASONIC_DISTANCE_MAX_MM);
    printf("GP20: record %u-reading batch. GP21: live output on/off.\n\n",
           BATCH_SIZE);

    bool live = true;
    bool previous_batch_button = false;
    bool previous_live_button = false;
    uint32_t sequence = 0;
    batch_t batch = {0};

    absolute_time_t next_trigger = get_absolute_time();
    absolute_time_t next_button_poll = get_absolute_time();

    while (true) {
        if (time_reached(next_trigger)) {
            next_trigger = delayed_by_us(next_trigger, MEASURE_PERIOD_US);
            /* A STUCK_HIGH found at trigger time is published at once. */
            if (ultrasonic_trigger() && !ultrasonic_busy()) {
                const ultrasonic_result_t result = ultrasonic_latest();
                const ultrasonic_debug_t debug = ultrasonic_debug_latest();
                ++sequence;
                if (live) {
                    print_live(sequence, result, debug);
                }
                if (batch.active) {
                    batch_add(&batch, result, debug);
                }
            }
        }

        if (ultrasonic_service()) {
            const ultrasonic_result_t result = ultrasonic_latest();
            const ultrasonic_debug_t debug = ultrasonic_debug_latest();
            ++sequence;
            if (live) {
                print_live(sequence, result, debug);
            }
            if (batch.active) {
                batch_add(&batch, result, debug);
            }
        }

        if (batch.active && batch.taken >= BATCH_SIZE) {
            batch.active = false;
            batch_report(&batch);
        }

        if (time_reached(next_button_poll)) {
            next_button_poll = make_timeout_time_ms(BUTTON_POLL_MS);
            const bool batch_button = !gpio_get(BATCH_BUTTON_PIN);
            const bool live_button = !gpio_get(LIVE_BUTTON_PIN);

            if (batch_button && !previous_batch_button && !batch.active) {
                batch_start(&batch);
            }
            if (live_button && !previous_live_button) {
                live = !live;
                printf("[live] output %s\n", live ? "on" : "off");
            }
            previous_batch_button = batch_button;
            previous_live_button = live_button;
        }

        sleep_us(200);
    }
}

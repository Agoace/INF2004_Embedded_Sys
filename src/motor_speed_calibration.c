#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#define MOTOR_1_FORWARD_PIN 8
#define MOTOR_1_REVERSE_PIN 9
#define MOTOR_2_FORWARD_PIN 10
#define MOTOR_2_REVERSE_PIN 11

#define MOTOR_1_ENCODER_A_PIN 16
#define MOTOR_1_ENCODER_B_PIN 17
#define MOTOR_2_ENCODER_A_PIN 4
#define MOTOR_2_ENCODER_B_PIN 5

#define START_BUTTON_PIN 20
#define EMERGENCY_STOP_BUTTON_PIN 21

/* Measured 2535 counts per wheel revolution for both motors by the
 * 50-revolution rotation drift test. See docs/MOTOR_ROTATION_DRIFT.md.
 * This supersedes the earlier 2540 and 2539, which were not independent
 * measurements. RPM values recorded before this change read about 0.197%
 * low. */
#define MOTOR_1_COUNTS_PER_REVOLUTION 2535
#define MOTOR_2_COUNTS_PER_REVOLUTION 2535

#define PWM_WRAP 6249
#define STARTUP_DUTY_PERCENT 10
#define STARTUP_COUNTS 10
#define STARTUP_TIMEOUT_MS 250
#define FIRST_DUTY_PERCENT 10
#define LAST_DUTY_PERCENT 100
#define DUTY_STEP_PERCENT 10
#define SETTLE_TIME_MS 1000
#define MEASUREMENT_TIME_MS 5000
#define PAUSE_TIME_MS 5000
#define POLL_INTERVAL_MS 10
#define RESULT_COUNT \
    (((LAST_DUTY_PERCENT - FIRST_DUTY_PERCENT) / DUTY_STEP_PERCENT) + 1)

typedef struct {
    uint a_pin;
    uint b_pin;
    volatile int32_t count;
    volatile uint8_t previous_state;
} encoder_t;

typedef struct {
    unsigned int duty_percent;
    uint32_t motor_1_counts;
    uint32_t motor_2_counts;
    uint32_t motor_1_rpm_milli;
    uint32_t motor_2_rpm_milli;
} speed_result_t;

static encoder_t motor_1_encoder = {
    .a_pin = MOTOR_1_ENCODER_A_PIN,
    .b_pin = MOTOR_1_ENCODER_B_PIN,
};

static encoder_t motor_2_encoder = {
    .a_pin = MOTOR_2_ENCODER_A_PIN,
    .b_pin = MOTOR_2_ENCODER_B_PIN,
};

static speed_result_t saved_results[RESULT_COUNT];
static unsigned int saved_result_count;
static bool results_valid;

static const int8_t quadrature_table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0,
};

static uint8_t read_encoder_state(const encoder_t *encoder) {
    return (uint8_t)((gpio_get(encoder->a_pin) << 1u) |
                     gpio_get(encoder->b_pin));
}

static void update_encoder(encoder_t *encoder) {
    const uint8_t current_state = read_encoder_state(encoder);
    const uint8_t table_index =
        (uint8_t)((encoder->previous_state << 2u) | current_state);
    encoder->count += quadrature_table[table_index];
    encoder->previous_state = current_state;
}

static void encoder_irq_callback(uint gpio, uint32_t events) {
    (void)events;

    if (gpio == motor_1_encoder.a_pin || gpio == motor_1_encoder.b_pin) {
        update_encoder(&motor_1_encoder);
    } else if (gpio == motor_2_encoder.a_pin ||
               gpio == motor_2_encoder.b_pin) {
        update_encoder(&motor_2_encoder);
    }
}

static void initialise_encoder(encoder_t *encoder, bool set_callback) {
    gpio_init(encoder->a_pin);
    gpio_set_dir(encoder->a_pin, GPIO_IN);
    gpio_disable_pulls(encoder->a_pin);
    gpio_init(encoder->b_pin);
    gpio_set_dir(encoder->b_pin, GPIO_IN);
    gpio_disable_pulls(encoder->b_pin);
    encoder->previous_state = read_encoder_state(encoder);

    const uint32_t events = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    if (set_callback) {
        gpio_set_irq_enabled_with_callback(
            encoder->a_pin, events, true, encoder_irq_callback);
    } else {
        gpio_set_irq_enabled(encoder->a_pin, events, true);
    }
    gpio_set_irq_enabled(encoder->b_pin, events, true);
}

static void reset_encoder(encoder_t *encoder) {
    const uint32_t interrupt_state = save_and_disable_interrupts();
    encoder->count = 0;
    encoder->previous_state = read_encoder_state(encoder);
    restore_interrupts(interrupt_state);
}

static int32_t read_encoder_count(const encoder_t *encoder) {
    const uint32_t interrupt_state = save_and_disable_interrupts();
    const int32_t count = encoder->count;
    restore_interrupts(interrupt_state);
    return count;
}

static uint32_t absolute_count(int32_t count) {
    return (uint32_t)(count < 0 ? -count : count);
}

static void initialise_motor_pair(uint forward_pin, uint reverse_pin) {
    gpio_set_function(forward_pin, GPIO_FUNC_PWM);
    gpio_set_function(reverse_pin, GPIO_FUNC_PWM);

    const uint slice = pwm_gpio_to_slice_num(forward_pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.0f);
    pwm_config_set_wrap(&config, PWM_WRAP);
    pwm_init(slice, &config, true);

    pwm_set_gpio_level(forward_pin, 0);
    pwm_set_gpio_level(reverse_pin, 0);
}

static uint16_t duty_level(unsigned int duty_percent) {
    return (uint16_t)(((PWM_WRAP + 1u) * duty_percent) / 100u);
}

static void set_motor_forward(uint forward_pin, uint reverse_pin,
                              unsigned int duty_percent) {
    pwm_set_gpio_level(reverse_pin, 0);
    pwm_set_gpio_level(forward_pin, duty_level(duty_percent));
}

static void set_both_motors(unsigned int duty_percent) {
    set_motor_forward(MOTOR_1_FORWARD_PIN, MOTOR_1_REVERSE_PIN,
                      duty_percent);
    set_motor_forward(MOTOR_2_FORWARD_PIN, MOTOR_2_REVERSE_PIN,
                      duty_percent);
}

static void stop_all_motors(void) {
    pwm_set_gpio_level(MOTOR_1_FORWARD_PIN, 0);
    pwm_set_gpio_level(MOTOR_1_REVERSE_PIN, 0);
    pwm_set_gpio_level(MOTOR_2_FORWARD_PIN, 0);
    pwm_set_gpio_level(MOTOR_2_REVERSE_PIN, 0);
}

static void initialise_button(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

static bool emergency_stop_pressed(void) {
    return !gpio_get(EMERGENCY_STOP_BUTTON_PIN);
}

static uint32_t counts_to_rpm_milli(uint32_t counts,
                                    uint32_t counts_per_revolution) {
    const uint64_t numerator = (uint64_t)counts * 60000000u;
    const uint64_t denominator =
        (uint64_t)counts_per_revolution * MEASUREMENT_TIME_MS;
    return (uint32_t)(numerator / denominator);
}

static void print_rpm(const char *label, uint32_t rpm_milli) {
    printf("%s%lu.%03lu RPM",
           label,
           (unsigned long)(rpm_milli / 1000u),
           (unsigned long)(rpm_milli % 1000u));
}

static void print_saved_results(void) {
    if (!results_valid) {
        return;
    }

    printf("\nSaved motor speed results:\n");
    printf("Duty | Motor 1 RPM | Motor 2 RPM | Difference\n");
    for (unsigned int index = 0; index < saved_result_count; ++index) {
        const speed_result_t *result = &saved_results[index];
        const int32_t difference =
            (int32_t)result->motor_1_rpm_milli -
            (int32_t)result->motor_2_rpm_milli;

        printf("%3u%% | ", result->duty_percent);
        print_rpm("", result->motor_1_rpm_milli);
        printf(" | ");
        print_rpm("", result->motor_2_rpm_milli);
        if (difference < 0) {
            const uint32_t magnitude = (uint32_t)(-difference);
            printf(" | -%lu.%03lu RPM\n",
                   (unsigned long)(magnitude / 1000u),
                   (unsigned long)(magnitude % 1000u));
        } else {
            printf(" | %lu.%03lu RPM\n",
                   (unsigned long)((uint32_t)difference / 1000u),
                   (unsigned long)((uint32_t)difference % 1000u));
        }
    }
    printf("Results remain in RAM while VIN power stays on.\n");
}

static bool wait_with_emergency_stop(uint32_t duration_ms) {
    for (uint32_t elapsed = 0; elapsed < duration_ms;
         elapsed += POLL_INTERVAL_MS) {
        if (emergency_stop_pressed()) {
            stop_all_motors();
            printf("Emergency stop activated.\n");
            return false;
        }
        sleep_ms(POLL_INTERVAL_MS);
    }
    return true;
}

static bool apply_startup_kick(void) {
    reset_encoder(&motor_1_encoder);
    reset_encoder(&motor_2_encoder);
    set_both_motors(STARTUP_DUTY_PERCENT);

    for (uint32_t elapsed = 0; elapsed < STARTUP_TIMEOUT_MS;
         elapsed += POLL_INTERVAL_MS) {
        if (emergency_stop_pressed()) {
            stop_all_motors();
            printf("Emergency stop activated during startup kick.\n");
            return false;
        }

        const bool motor_1_started =
            absolute_count(read_encoder_count(&motor_1_encoder)) >=
            STARTUP_COUNTS;
        const bool motor_2_started =
            absolute_count(read_encoder_count(&motor_2_encoder)) >=
            STARTUP_COUNTS;
        if (motor_1_started && motor_2_started) {
            return true;
        }
        sleep_ms(POLL_INTERVAL_MS);
    }

    stop_all_motors();
    printf("Startup fault: both motors did not reach %u counts in %u ms.\n",
           STARTUP_COUNTS, STARTUP_TIMEOUT_MS);
    return false;
}

static void run_speed_test(void) {
    results_valid = false;
    saved_result_count = 0;

    for (unsigned int duty = FIRST_DUTY_PERCENT;
         duty <= LAST_DUTY_PERCENT;
         duty += DUTY_STEP_PERCENT) {
        printf("\nPreparing %u%% duty test.\n", duty);
        if (!wait_with_emergency_stop(PAUSE_TIME_MS)) {
            return;
        }

        printf("Applying %u%% startup kick.\n", STARTUP_DUTY_PERCENT);
        if (!apply_startup_kick()) {
            return;
        }

        set_both_motors(duty);
        printf("Settling at %u%% for %u ms.\n", duty, SETTLE_TIME_MS);
        if (!wait_with_emergency_stop(SETTLE_TIME_MS)) {
            return;
        }

        reset_encoder(&motor_1_encoder);
        reset_encoder(&motor_2_encoder);
        printf("Measuring %u%% for %u seconds.\n",
               duty, MEASUREMENT_TIME_MS / 1000);
        if (!wait_with_emergency_stop(MEASUREMENT_TIME_MS)) {
            return;
        }

        stop_all_motors();
        const uint32_t motor_1_counts =
            absolute_count(read_encoder_count(&motor_1_encoder));
        const uint32_t motor_2_counts =
            absolute_count(read_encoder_count(&motor_2_encoder));
        const uint32_t motor_1_rpm_milli = counts_to_rpm_milli(
            motor_1_counts, MOTOR_1_COUNTS_PER_REVOLUTION);
        const uint32_t motor_2_rpm_milli = counts_to_rpm_milli(
            motor_2_counts, MOTOR_2_COUNTS_PER_REVOLUTION);

        speed_result_t *result = &saved_results[saved_result_count++];
        result->duty_percent = duty;
        result->motor_1_counts = motor_1_counts;
        result->motor_2_counts = motor_2_counts;
        result->motor_1_rpm_milli = motor_1_rpm_milli;
        result->motor_2_rpm_milli = motor_2_rpm_milli;

        printf("%u%% result: motor 1=%lu counts, ",
               duty, (unsigned long)motor_1_counts);
        print_rpm("", motor_1_rpm_milli);
        printf("; motor 2=%lu counts, ", (unsigned long)motor_2_counts);
        print_rpm("", motor_2_rpm_milli);
        printf(".\n");
    }

    stop_all_motors();
    results_valid = true;
    print_saved_results();
}

int main(void) {
    stdio_init_all();

    initialise_motor_pair(MOTOR_1_FORWARD_PIN, MOTOR_1_REVERSE_PIN);
    initialise_motor_pair(MOTOR_2_FORWARD_PIN, MOTOR_2_REVERSE_PIN);
    stop_all_motors();

    initialise_encoder(&motor_1_encoder, true);
    initialise_encoder(&motor_2_encoder, false);
    initialise_button(START_BUTTON_PIN);
    initialise_button(EMERGENCY_STOP_BUTTON_PIN);

    sleep_ms(1000);
    printf("\nUnloaded motor speed calibration\n");
    printf("WARNING: power motors from VIN, not Raspberry Pi USB.\n");
    printf("Tap GP20 to test 10%% through 100%% duty.\n");
    printf("GP21 is the emergency stop. Keep both wheels lifted.\n");
    printf("Each level settles for %u second and measures for %u seconds.\n",
           SETTLE_TIME_MS / 1000, MEASUREMENT_TIME_MS / 1000);

    bool previous_start_pressed = false;
    absolute_time_t next_result_report = make_timeout_time_ms(2000);

    while (true) {
        const bool start_pressed = !gpio_get(START_BUTTON_PIN);
        if (start_pressed && !previous_start_pressed) {
            run_speed_test();
            printf("Tap GP20 to run the speed test again.\n");
            next_result_report = make_timeout_time_ms(2000);
            while (!gpio_get(START_BUTTON_PIN)) {
                sleep_ms(POLL_INTERVAL_MS);
            }
            previous_start_pressed = false;
            continue;
        }

        if (results_valid && time_reached(next_result_report)) {
            print_saved_results();
            next_result_report = make_timeout_time_ms(2000);
        }

        previous_start_pressed = start_pressed;
        sleep_ms(POLL_INTERVAL_MS);
    }
}

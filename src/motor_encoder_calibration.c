#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#define MOTOR_1_PWM_PIN 8
#define MOTOR_1_BRAKE_PIN 9
#define MOTOR_2_PWM_PIN 10
#define MOTOR_2_BRAKE_PIN 11

#define ENCODER_1_A_PIN 16
#define ENCODER_1_B_PIN 17
#define ENCODER_2_A_PIN 4
#define ENCODER_2_B_PIN 5

#define MOTOR_1_BUTTON_PIN 20
#define MOTOR_2_BUTTON_PIN 21

#define PWM_WRAP 6249
#define PWM_DUTY_PERCENT 10
#define BUTTON_POLL_MS 20
#define BRAKE_SETTLE_MS 100
/* Measured 2535 counts per wheel revolution for both motors by the
 * 50-revolution rotation drift test. See docs/MOTOR_ROTATION_DRIFT.md.
 * The brake compensation is set so each stop threshold stays at the
 * empirically tuned 2500 counts that this test used before, and the
 * measured 33-40 count braking travel still applies. The corrected value
 * only changes the angular drift calculation. */
#define MOTOR_2_COUNTS_PER_REVOLUTION 2535
#define MOTOR_2_BRAKE_COMPENSATION_COUNTS 35
#define MOTOR_2_TARGET_COUNTS \
    (MOTOR_2_COUNTS_PER_REVOLUTION - MOTOR_2_BRAKE_COMPENSATION_COUNTS)
#define MOTOR_2_TIMEOUT_MS 5000
#define MOTOR_2_TEST_CYCLES 10
#define DIRECTION_PAUSE_MS 500
#define MOTOR_1_COUNTS_PER_REVOLUTION 2535
#define MOTOR_1_BRAKE_COMPENSATION_COUNTS 35
#define MOTOR_1_TARGET_COUNTS \
    (MOTOR_1_COUNTS_PER_REVOLUTION - MOTOR_1_BRAKE_COMPENSATION_COUNTS)
#define MOTOR_1_TIMEOUT_MS 5000
#define MOTOR_1_TEST_CYCLES 10

typedef struct {
    uint a_pin;
    uint b_pin;
    volatile uint32_t a_edges;
    volatile uint32_t b_edges;
    volatile int32_t quadrature_count;
    volatile uint8_t previous_state;
} encoder_state_t;

static encoder_state_t encoder_1 = {
    .a_pin = ENCODER_1_A_PIN,
    .b_pin = ENCODER_1_B_PIN,
};

static encoder_state_t encoder_2 = {
    .a_pin = ENCODER_2_A_PIN,
    .b_pin = ENCODER_2_B_PIN,
};

static const int8_t quadrature_table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0,
};

static uint motor_1_slice;
static uint motor_2_slice;

static uint8_t read_encoder_state(const encoder_state_t *encoder) {
    return (uint8_t)((gpio_get(encoder->a_pin) << 1u) |
                     gpio_get(encoder->b_pin));
}

static void update_encoder(encoder_state_t *encoder, uint changed_gpio) {
    if (changed_gpio == encoder->a_pin) {
        ++encoder->a_edges;
    } else {
        ++encoder->b_edges;
    }

    const uint8_t current_state = read_encoder_state(encoder);
    const uint8_t table_index =
        (uint8_t)((encoder->previous_state << 2u) | current_state);
    encoder->quadrature_count += quadrature_table[table_index];
    encoder->previous_state = current_state;
}

static void encoder_irq_callback(uint gpio, uint32_t events) {
    (void)events;

    if (gpio == encoder_1.a_pin || gpio == encoder_1.b_pin) {
        update_encoder(&encoder_1, gpio);
    } else if (gpio == encoder_2.a_pin || gpio == encoder_2.b_pin) {
        update_encoder(&encoder_2, gpio);
    }
}

static void initialise_encoder(encoder_state_t *encoder, bool set_callback) {
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

static void reset_encoder(encoder_state_t *encoder) {
    const uint32_t interrupt_state = save_and_disable_interrupts();
    encoder->a_edges = 0;
    encoder->b_edges = 0;
    encoder->quadrature_count = 0;
    encoder->previous_state = read_encoder_state(encoder);
    restore_interrupts(interrupt_state);
}

static void print_encoder_result(unsigned int number,
                                 const encoder_state_t *encoder) {
    const uint32_t interrupt_state = save_and_disable_interrupts();
    const uint32_t a_edges = encoder->a_edges;
    const uint32_t b_edges = encoder->b_edges;
    const int32_t quadrature_count = encoder->quadrature_count;
    restore_interrupts(interrupt_state);

    printf("Motor %u one-wheel-revolution result:\n", number);
    printf("  phase A edges: %lu\n", (unsigned long)a_edges);
    printf("  phase B edges: %lu\n", (unsigned long)b_edges);
    printf("  signed quadrature count: %ld\n", (long)quadrature_count);
    printf("  counts/revolution: %lu\n",
           (unsigned long)(quadrature_count < 0
                               ? -quadrature_count
                               : quadrature_count));
}

static void initialise_motor_pwm(uint pin, uint *slice) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    *slice = pwm_gpio_to_slice_num(pin);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.0f);
    pwm_config_set_wrap(&config, PWM_WRAP);
    pwm_init(*slice, &config, true);
    pwm_set_gpio_level(pin, 0);
}

static void initialise_brake_pin(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, false);
}

static uint16_t motor_duty_level(void) {
    return (uint16_t)(((PWM_WRAP + 1u) * PWM_DUTY_PERCENT) / 100u);
}

static void stop_all_motors(void) {
    pwm_set_gpio_level(MOTOR_1_PWM_PIN, 0);
    pwm_set_gpio_level(MOTOR_1_BRAKE_PIN, 0);
    pwm_set_gpio_level(MOTOR_2_PWM_PIN, 0);
    pwm_set_gpio_level(MOTOR_2_BRAKE_PIN, 0);
}

static void drive_motor_1(bool reverse) {
    if (reverse) {
        pwm_set_gpio_level(MOTOR_1_PWM_PIN, 0);
        pwm_set_gpio_level(MOTOR_1_BRAKE_PIN, motor_duty_level());
    } else {
        pwm_set_gpio_level(MOTOR_1_BRAKE_PIN, 0);
        pwm_set_gpio_level(MOTOR_1_PWM_PIN, motor_duty_level());
    }
}

static void drive_motor_2(bool reverse) {
    if (reverse) {
        pwm_set_gpio_level(MOTOR_2_PWM_PIN, 0);
        pwm_set_gpio_level(MOTOR_2_BRAKE_PIN, motor_duty_level());
    } else {
        pwm_set_gpio_level(MOTOR_2_BRAKE_PIN, 0);
        pwm_set_gpio_level(MOTOR_2_PWM_PIN, motor_duty_level());
    }
}

static int32_t read_quadrature_count(const encoder_state_t *encoder) {
    const uint32_t interrupt_state = save_and_disable_interrupts();
    const int32_t count = encoder->quadrature_count;
    restore_interrupts(interrupt_state);
    return count;
}

static bool run_motor_1_leg(bool reverse, unsigned int cycle,
                            int32_t *cumulative_count) {
    reset_encoder(&encoder_1);
    printf("Cycle %u/%u, direction %s: running to %u counts.\n",
           cycle, MOTOR_1_TEST_CYCLES, reverse ? "B" : "A",
           MOTOR_1_TARGET_COUNTS);
    drive_motor_1(reverse);

    bool emergency_stop = false;
    bool timed_out = true;
    uint32_t elapsed_ms = 0;
    for (; elapsed_ms < MOTOR_1_TIMEOUT_MS;
         elapsed_ms += BUTTON_POLL_MS) {
        if (!gpio_get(MOTOR_2_BUTTON_PIN)) {
            emergency_stop = true;
            timed_out = false;
            break;
        }

        const int32_t signed_count = read_quadrature_count(&encoder_1);
        const uint32_t count = (uint32_t)(
            signed_count < 0 ? -signed_count : signed_count);
        if (count >= MOTOR_1_TARGET_COUNTS) {
            timed_out = false;
            break;
        }
        sleep_ms(BUTTON_POLL_MS);
    }

    stop_all_motors();
    sleep_ms(BRAKE_SETTLE_MS);
    const int32_t final_count = read_quadrature_count(&encoder_1);
    *cumulative_count += final_count;

    if (emergency_stop) {
        printf("Emergency stop activated.\n");
    } else if (timed_out) {
        printf("Safety timeout after %u ms.\n", MOTOR_1_TIMEOUT_MS);
    } else {
        printf("Count target reached after approximately %lu ms.\n",
               (unsigned long)elapsed_ms);
    }
    print_encoder_result(1, &encoder_1);
    return !emergency_stop && !timed_out;
}

static bool run_motor_2_leg(bool reverse, unsigned int cycle,
                            int32_t *cumulative_count) {
    reset_encoder(&encoder_2);
    printf("Cycle %u/%u, direction %s: running to %u counts.\n",
           cycle, MOTOR_2_TEST_CYCLES, reverse ? "B" : "A",
           MOTOR_2_TARGET_COUNTS);
    drive_motor_2(reverse);

    bool emergency_stop = false;
    bool timed_out = true;
    uint32_t elapsed_ms = 0;
    for (; elapsed_ms < MOTOR_2_TIMEOUT_MS;
         elapsed_ms += BUTTON_POLL_MS) {
        if (!gpio_get(MOTOR_1_BUTTON_PIN)) {
            emergency_stop = true;
            timed_out = false;
            break;
        }

        const int32_t signed_count = read_quadrature_count(&encoder_2);
        const uint32_t count = (uint32_t)(
            signed_count < 0 ? -signed_count : signed_count);
        if (count >= MOTOR_2_TARGET_COUNTS) {
            timed_out = false;
            break;
        }
        sleep_ms(BUTTON_POLL_MS);
    }

    stop_all_motors();
    sleep_ms(BRAKE_SETTLE_MS);
    const int32_t final_count = read_quadrature_count(&encoder_2);
    *cumulative_count += final_count;

    if (emergency_stop) {
        printf("Emergency stop activated.\n");
    } else if (timed_out) {
        printf("Safety timeout after %u ms.\n", MOTOR_2_TIMEOUT_MS);
    } else {
        printf("Count target reached after approximately %lu ms.\n",
               (unsigned long)elapsed_ms);
    }
    print_encoder_result(2, &encoder_2);
    return !emergency_stop && !timed_out;
}

static void initialise_button(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

int main(void) {
    stdio_init_all();

    initialise_motor_pwm(MOTOR_1_PWM_PIN, &motor_1_slice);
    gpio_set_function(MOTOR_1_BRAKE_PIN, GPIO_FUNC_PWM);
    pwm_set_gpio_level(MOTOR_1_BRAKE_PIN, 0);
    initialise_motor_pwm(MOTOR_2_PWM_PIN, &motor_2_slice);
    gpio_set_function(MOTOR_2_BRAKE_PIN, GPIO_FUNC_PWM);
    pwm_set_gpio_level(MOTOR_2_BRAKE_PIN, 0);
    stop_all_motors();

    initialise_encoder(&encoder_1, true);
    initialise_encoder(&encoder_2, false);
    initialise_button(MOTOR_1_BUTTON_PIN);
    initialise_button(MOTOR_2_BUTTON_PIN);

    sleep_ms(1000);
    printf("\nMotor 1 forward/backward drift test\n");
    printf("WARNING: power motors from VIN, not Raspberry Pi USB.\n");
    printf("Lift the wheel before starting the test.\n");
    printf("Tap GP20 to run %u forward/backward cycles.\n",
           MOTOR_1_TEST_CYCLES);
    printf("GP21 is the emergency stop. Motor 2 is disabled.\n");
    printf("Motor power: %u%%; stop threshold: %u counts.\n",
           PWM_DUTY_PERCENT, MOTOR_1_TARGET_COUNTS);

    bool previous_start_pressed = false;

    while (true) {
        const bool start_pressed = !gpio_get(MOTOR_1_BUTTON_PIN);

        if (start_pressed && !previous_start_pressed) {
            int32_t cumulative_count = 0;
            bool completed = true;

            for (unsigned int cycle = 1;
                 cycle <= MOTOR_1_TEST_CYCLES; ++cycle) {
                if (!run_motor_1_leg(false, cycle, &cumulative_count)) {
                    completed = false;
                    break;
                }
                sleep_ms(DIRECTION_PAUSE_MS);

                if (!run_motor_1_leg(true, cycle, &cumulative_count)) {
                    completed = false;
                    break;
                }

                const int32_t drift_millidegrees =
                    (cumulative_count * 360000) /
                    MOTOR_1_COUNTS_PER_REVOLUTION;
                printf("Cycle %u cumulative drift: %ld counts (%ld millidegrees)\n",
                       cycle, (long)cumulative_count,
                       (long)drift_millidegrees);
                sleep_ms(DIRECTION_PAUSE_MS);
            }

            stop_all_motors();
            if (completed) {
                printf("Ten-cycle test complete. Final drift: %ld counts.\n",
                       (long)cumulative_count);
                printf("Report the wheel marker's final position.\n");
            } else {
                printf("Test stopped before all cycles completed.\n");
            }

            while (!gpio_get(MOTOR_1_BUTTON_PIN)) {
                sleep_ms(BUTTON_POLL_MS);
            }
            previous_start_pressed = false;
            continue;
        }

        previous_start_pressed = start_pressed;
        sleep_ms(BUTTON_POLL_MS);
    }
}

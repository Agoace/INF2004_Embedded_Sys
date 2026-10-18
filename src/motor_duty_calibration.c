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

#define PWM_WRAP 6249
#define DUTY_STEP_PERCENT 1
#define DUTY_HOLD_MS 10000
#define DUTY_PAUSE_MS 5000
#define POLL_INTERVAL_MS 10
#define MOVEMENT_THRESHOLD_COUNTS 10

typedef struct {
    uint a_pin;
    uint b_pin;
    volatile int32_t count;
    volatile uint8_t previous_state;
} encoder_t;

typedef struct {
    bool valid;
    unsigned int motor_1_duty;
    unsigned int motor_2_duty;
    uint32_t motor_1_start_time_ms;
    uint32_t motor_2_start_time_ms;
} duty_result_t;

static encoder_t motor_1_encoder = {
    .a_pin = MOTOR_1_ENCODER_A_PIN,
    .b_pin = MOTOR_1_ENCODER_B_PIN,
};

static encoder_t motor_2_encoder = {
    .a_pin = MOTOR_2_ENCODER_A_PIN,
    .b_pin = MOTOR_2_ENCODER_B_PIN,
};

static duty_result_t last_result;

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

static void stop_motor(uint forward_pin, uint reverse_pin) {
    pwm_set_gpio_level(forward_pin, 0);
    pwm_set_gpio_level(reverse_pin, 0);
}

static void stop_all_motors(void) {
    stop_motor(MOTOR_1_FORWARD_PIN, MOTOR_1_REVERSE_PIN);
    stop_motor(MOTOR_2_FORWARD_PIN, MOTOR_2_REVERSE_PIN);
}

static void initialise_button(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

static bool emergency_stop_pressed(void) {
    return !gpio_get(EMERGENCY_STOP_BUTTON_PIN);
}

static void print_saved_result(void) {
    if (!last_result.valid) {
        return;
    }

    printf("\nSaved minimum-duty result:\n");
    printf("Motor 1: %u%% duty, approximately %lu ms to %u counts.\n",
           last_result.motor_1_duty,
           (unsigned long)last_result.motor_1_start_time_ms,
           MOVEMENT_THRESHOLD_COUNTS);
    printf("Motor 2: %u%% duty, approximately %lu ms to %u counts.\n",
           last_result.motor_2_duty,
           (unsigned long)last_result.motor_2_start_time_ms,
           MOVEMENT_THRESHOLD_COUNTS);
    printf("Result remains in RAM while VIN power stays on.\n");
}

static void run_duty_test(void) {
    last_result.valid = false;

    bool motor_1_found = false;
    bool motor_2_found = false;
    unsigned int motor_1_duty = 0;
    unsigned int motor_2_duty = 0;
    uint32_t motor_1_start_time_ms = 0;
    uint32_t motor_2_start_time_ms = 0;

    for (unsigned int duty = 0; duty <= 100; duty += DUTY_STEP_PERCENT) {
        if (duty > 0) {
            printf("Pause before %u%% duty.\n", duty);
            for (uint32_t elapsed = 0; elapsed < DUTY_PAUSE_MS;
                 elapsed += POLL_INTERVAL_MS) {
                if (emergency_stop_pressed()) {
                    stop_all_motors();
                    printf("Emergency stop activated during pause.\n");
                    return;
                }
                sleep_ms(POLL_INTERVAL_MS);
            }
        }

        reset_encoder(&motor_1_encoder);
        reset_encoder(&motor_2_encoder);

        if (!motor_1_found) {
            set_motor_forward(MOTOR_1_FORWARD_PIN, MOTOR_1_REVERSE_PIN,
                              duty);
        }
        if (!motor_2_found) {
            set_motor_forward(MOTOR_2_FORWARD_PIN, MOTOR_2_REVERSE_PIN,
                              duty);
        }

        printf("Testing %u%% duty for %u seconds.\n",
               duty, DUTY_HOLD_MS / 1000);

        for (uint32_t elapsed = 0; elapsed < DUTY_HOLD_MS;
             elapsed += POLL_INTERVAL_MS) {
            if (emergency_stop_pressed()) {
                stop_all_motors();
                printf("Emergency stop activated during duty test.\n");
                return;
            }

            if (!motor_1_found &&
                absolute_count(read_encoder_count(&motor_1_encoder)) >=
                    MOVEMENT_THRESHOLD_COUNTS) {
                motor_1_found = true;
                motor_1_duty = duty;
                motor_1_start_time_ms = elapsed;
                stop_motor(MOTOR_1_FORWARD_PIN, MOTOR_1_REVERSE_PIN);
                printf("Motor 1 started moving at %u%% after about %lu ms.\n",
                       duty, (unsigned long)elapsed);
            }

            if (!motor_2_found &&
                absolute_count(read_encoder_count(&motor_2_encoder)) >=
                    MOVEMENT_THRESHOLD_COUNTS) {
                motor_2_found = true;
                motor_2_duty = duty;
                motor_2_start_time_ms = elapsed;
                stop_motor(MOTOR_2_FORWARD_PIN, MOTOR_2_REVERSE_PIN);
                printf("Motor 2 started moving at %u%% after about %lu ms.\n",
                       duty, (unsigned long)elapsed);
            }

            if (motor_1_found && motor_2_found) {
                stop_all_motors();
                last_result.motor_1_duty = motor_1_duty;
                last_result.motor_2_duty = motor_2_duty;
                last_result.motor_1_start_time_ms = motor_1_start_time_ms;
                last_result.motor_2_start_time_ms = motor_2_start_time_ms;
                last_result.valid = true;
                printf("\nBoth movement thresholds found.\n");
                print_saved_result();
                return;
            }

            sleep_ms(POLL_INTERVAL_MS);
        }

        stop_all_motors();
        printf("End of %u%%: motor 1=%lu counts, motor 2=%lu counts.\n",
               duty,
               (unsigned long)absolute_count(
                   read_encoder_count(&motor_1_encoder)),
               (unsigned long)absolute_count(
                   read_encoder_count(&motor_2_encoder)));
    }

    stop_all_motors();
    printf("Test ended at 100%% without detecting both motors.\n");
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
    printf("\nMinimum free-running motor duty calibration\n");
    printf("WARNING: power motors from VIN, not Raspberry Pi USB.\n");
    printf("Tap GP20 to start both motors at 0%% duty.\n");
    printf("Each duty runs for %u seconds with a %u-second pause.\n",
           DUTY_HOLD_MS / 1000, DUTY_PAUSE_MS / 1000);
    printf("Duty increases by %u%%. Movement threshold: %u counts.\n",
           DUTY_STEP_PERCENT, MOVEMENT_THRESHOLD_COUNTS);
    printf("GP21 is the emergency stop. Keep both wheels lifted.\n");

    bool previous_start_pressed = false;
    absolute_time_t next_result_report = make_timeout_time_ms(2000);

    while (true) {
        const bool start_pressed = !gpio_get(START_BUTTON_PIN);
        if (start_pressed && !previous_start_pressed) {
            run_duty_test();
            printf("Tap GP20 to run the test again.\n");
            next_result_report = make_timeout_time_ms(2000);
            while (!gpio_get(START_BUTTON_PIN)) {
                sleep_ms(POLL_INTERVAL_MS);
            }
            previous_start_pressed = false;
            continue;
        }

        if (last_result.valid && time_reached(next_result_report)) {
            print_saved_result();
            next_result_report = make_timeout_time_ms(2000);
        }

        previous_start_pressed = start_pressed;
        sleep_ms(POLL_INTERVAL_MS);
    }
}

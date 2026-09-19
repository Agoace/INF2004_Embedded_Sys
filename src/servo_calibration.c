#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#define SERVO_PIN 15
#define LEFT_BUTTON_PIN 20
#define RIGHT_BUTTON_PIN 21

#define SERVO_PERIOD_US 20000
#define SERVO_MIN_US 500
#define SERVO_CENTER_US 1500
#define SERVO_MAX_US 2500
#define SERVO_STEP_US 10
#define REPEAT_INTERVAL_MS 100

static uint servo_slice;
static uint16_t pulse_us = SERVO_CENTER_US;

static void servo_write_microseconds(uint16_t value) {
    pulse_us = value;
    pwm_set_chan_level(
        servo_slice, pwm_gpio_to_channel(SERVO_PIN), pulse_us);
    printf("GP15 servo pulse: %u us\n", pulse_us);
}

static void initialise_button(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

int main(void) {
    stdio_init_all();

    initialise_button(LEFT_BUTTON_PIN);
    initialise_button(RIGHT_BUTTON_PIN);

    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    servo_slice = pwm_gpio_to_slice_num(SERVO_PIN);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, (float)clock_get_hz(clk_sys) / 1000000.0f);
    pwm_config_set_wrap(&config, SERVO_PERIOD_US - 1);
    pwm_init(servo_slice, &config, false);

    servo_write_microseconds(SERVO_CENTER_US);
    pwm_set_enabled(servo_slice, true);

    sleep_ms(1000);
    printf("\nPositional-servo calibration\n");
    printf("GP20: decrease by %u us\n", SERVO_STEP_US);
    printf("GP21: increase by %u us\n", SERVO_STEP_US);
    printf("GP20 + GP21: return to %u us center\n", SERVO_CENTER_US);
    printf("Stop before the servo buzzes, strains, or stops moving.\n");
    printf("Allowed software range: %u-%u us\n", SERVO_MIN_US, SERVO_MAX_US);

    bool previous_left = false;
    bool previous_right = false;

    while (true) {
        const bool left = !gpio_get(LEFT_BUTTON_PIN);
        const bool right = !gpio_get(RIGHT_BUTTON_PIN);

        if (left && right) {
            if (!(previous_left && previous_right)) {
                servo_write_microseconds(SERVO_CENTER_US);
            }
        } else if (left) {
            const uint16_t next = pulse_us > SERVO_MIN_US + SERVO_STEP_US
                                      ? pulse_us - SERVO_STEP_US
                                      : SERVO_MIN_US;
            servo_write_microseconds(next);
        } else if (right) {
            const uint16_t next = pulse_us < SERVO_MAX_US - SERVO_STEP_US
                                      ? pulse_us + SERVO_STEP_US
                                      : SERVO_MAX_US;
            servo_write_microseconds(next);
        }

        previous_left = left;
        previous_right = right;
        sleep_ms(REPEAT_INTERVAL_MS);
    }
}

#include <stdio.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#define SERVO_PIN 15
#define SERVO_PERIOD_US 20000
#define SERVO_MIN_US 1000
#define SERVO_NEUTRAL_US 1500
#define SERVO_MAX_US 2000
#define SERVO_START_DELAY_MS 2000
#define SERVO_HOLD_TIME_MS 3000
#define SERVO_TEST_CYCLES 1

static void servo_write_microseconds(uint slice, uint16_t pulse_us) {
    pwm_set_chan_level(slice, pwm_gpio_to_channel(SERVO_PIN), pulse_us);
}

int main(void) {
    stdio_init_all();

    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    const uint slice = pwm_gpio_to_slice_num(SERVO_PIN);

    // Run the PWM counter at 1 MHz so one counter step equals one microsecond.
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, (float)clock_get_hz(clk_sys) / 1000000.0f);
    pwm_config_set_wrap(&config, SERVO_PERIOD_US - 1);

    pwm_init(slice, &config, false);
    servo_write_microseconds(slice, SERVO_NEUTRAL_US);
    pwm_set_enabled(slice, true);
    sleep_ms(SERVO_START_DELAY_MS);

    for (unsigned int cycle = 0; cycle < SERVO_TEST_CYCLES; ++cycle) {
        printf("Servo cycle %u: %u us -> %u us.\n",
               cycle + 1, SERVO_MIN_US, SERVO_MAX_US);
        servo_write_microseconds(slice, SERVO_MIN_US);
        sleep_ms(SERVO_HOLD_TIME_MS);
        servo_write_microseconds(slice, SERVO_MAX_US);
        sleep_ms(SERVO_HOLD_TIME_MS);
    }

    servo_write_microseconds(slice, SERVO_NEUTRAL_US);
    printf("Servo test complete; GP15 returned to %u us.\n",
           SERVO_NEUTRAL_US);

    while (true) {
        sleep_ms(1000);
    }
}

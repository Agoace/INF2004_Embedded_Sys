#include "servo.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define SERVO_PERIOD_US 20000u

static unsigned int servo_gpio;
static unsigned int servo_slice;

void servo_init(unsigned int pin) {
    servo_gpio = pin;
    gpio_set_function(pin, GPIO_FUNC_PWM);
    servo_slice = pwm_gpio_to_slice_num(pin);

    /* 1 MHz counter: one count is 1 us. */
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, (float)clock_get_hz(clk_sys) / 1000000.0f);
    pwm_config_set_wrap(&config, SERVO_PERIOD_US - 1u);
    pwm_init(servo_slice, &config, false);

    (void)servo_set_pulse_us(SERVO_CENTRE_US);
    pwm_set_enabled(servo_slice, true);
}

uint16_t servo_set_pulse_us(uint16_t pulse_us) {
    if (pulse_us < SERVO_SAFE_MIN_US) {
        pulse_us = SERVO_SAFE_MIN_US;
    } else if (pulse_us > SERVO_SAFE_MAX_US) {
        pulse_us = SERVO_SAFE_MAX_US;
    }
    pwm_set_chan_level(servo_slice, pwm_gpio_to_channel(servo_gpio), pulse_us);
    return pulse_us;
}

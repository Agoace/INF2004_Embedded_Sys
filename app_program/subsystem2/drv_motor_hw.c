/* SDK-only adapter. Never include kernel headers in this file. */
#include "drv_motor_hw.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/resets.h"
#define MOTOR_WRAP 6249u
#define SERVO_WRAP 19999u
#define SERVO_PIN 15u
static const unsigned int FORWARD[2] = {8u, 10u};
static const unsigned int REVERSE[2] = {9u, 11u};
static const unsigned int ENC_A[2] = {16u, 4u};
static const unsigned int ENC_B[2] = {17u, 5u};
static uint16_t last_servo;
static void input(unsigned int pin, bool pull_up) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    if (pull_up) gpio_pull_up(pin);
    else gpio_disable_pulls(pin);
}
void drv_motor_hw_init(uint16_t servo_us) {
    /* USE_PTMR=0 leaves PWM reset asserted. This inline SDK reset helper
     * needs no SDK runtime. S2 owns PWM initialization before all clients. */
    reset_unreset_block_num_wait_blocking(RESET_PWM);
    for (unsigned int i = 0u; i < 2u; ++i) {
        gpio_set_function(FORWARD[i], GPIO_FUNC_PWM);
        gpio_set_function(REVERSE[i], GPIO_FUNC_PWM);
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&cfg, 1.0f);
        pwm_config_set_wrap(&cfg, MOTOR_WRAP);
        pwm_init(pwm_gpio_to_slice_num(FORWARD[i]), &cfg, true);
        pwm_set_gpio_level(FORWARD[i], 0u);
        pwm_set_gpio_level(REVERSE[i], 0u);
        input(ENC_A[i], false);
        input(ENC_B[i], false);
    }
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    pwm_config servo_cfg = pwm_get_default_config();
    /* Kernel clk_sys is fixed at 125 MHz. Do not use clock_get_hz(): its
     * SDK clocks runtime state is not initialized by this kernel. */
    pwm_config_set_clkdiv(&servo_cfg, 125.0f); /* 1 us per count */
    pwm_config_set_wrap(&servo_cfg, SERVO_WRAP);
    pwm_init(pwm_gpio_to_slice_num(SERVO_PIN), &servo_cfg, true);
    pwm_set_gpio_level(SERVO_PIN, servo_us);
    last_servo = servo_us;
}
uint8_t drv_motor_hw_ab(unsigned int wheel) {
    const uint32_t pins = gpio_get_all(); /* Both AB bits from one sample. */
    return (uint8_t)((((pins >> ENC_A[wheel]) & 1u) << 1u) |
                     ((pins >> ENC_B[wheel]) & 1u));
}
static void write_motor(unsigned int wheel, int16_t permille) {
    const uint32_t magnitude = (uint32_t)(permille < 0 ? -permille : permille);
    const uint16_t level = (uint16_t)(((MOTOR_WRAP + 1u) * magnitude) / 1000u);
    /* Clear the opposite pin first, as in src/motor_ctl_pico.c. */
    if (permille > 0) {
        pwm_set_gpio_level(REVERSE[wheel], 0u);
        pwm_set_gpio_level(FORWARD[wheel], level);
    } else if (permille < 0) {
        pwm_set_gpio_level(FORWARD[wheel], 0u);
        pwm_set_gpio_level(REVERSE[wheel], level);
    } else {
        pwm_set_gpio_level(FORWARD[wheel], 0u);
        pwm_set_gpio_level(REVERSE[wheel], 0u);
    }
}
void drv_motor_hw_write(const int16_t duty[2], uint16_t servo_us) {
    write_motor(0u, duty[0]);
    write_motor(1u, duty[1]);
    if (servo_us != last_servo) {
        pwm_set_gpio_level(SERVO_PIN, servo_us);
        last_servo = servo_us;
    }
}
void drv_motor_hw_stop(void) {
    /* Compare writes take effect at the next 20 kHz wrap, <= 50 us. */
    write_motor(0u, 0);
    write_motor(1u, 0);
}
void drv_motor_hw_buttons_init(void) { input(20u, true); input(21u, true); }
bool drv_motor_hw_button_down(unsigned int pin) { return !gpio_get(pin); }

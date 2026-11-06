/* Link-only test. Never called or flashed. */
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/resets.h"
uint32_t app_sdk_hw_link_probe(void) {
    reset_unreset_block_num_wait_blocking(RESET_PWM);
    gpio_set_function(8u, GPIO_FUNC_PWM);
    pwm_config p = pwm_get_default_config();
    pwm_config_set_clkdiv(&p, 125.0f);
    pwm_config_set_wrap(&p, 19999u);
    pwm_init(pwm_gpio_to_slice_num(8u), &p, true);
    pwm_set_gpio_level(8u, 1500u);
    adc_init();
    adc_gpio_init(26u);
    adc_select_input(0u);
    return (uint32_t)adc_read() + timer_time_us_32(timer_hw);
}

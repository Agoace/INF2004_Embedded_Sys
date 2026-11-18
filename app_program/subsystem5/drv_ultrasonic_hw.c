/* SDK-only; GPIO IRQ ownership stays with the kernel dispatcher. */
#include "drv_ultrasonic_hw.h"
#include "hardware/gpio.h"
#include "hardware/structs/timer.h"
void drv_ultrasonic_hw_init(void) {
    gpio_init(3u); gpio_put(3u, false); gpio_set_dir(3u, GPIO_OUT);
    gpio_init(2u); gpio_set_dir(2u, GPIO_IN); gpio_disable_pulls(2u);
}
bool drv_ultrasonic_hw_echo(void) { return gpio_get(2u); }
void drv_ultrasonic_hw_pulse(void) {
    gpio_put(3u, true);
    const uint32_t start = timer_hw->timerawl;
    while (timer_hw->timerawl - start < 10u) { }
    gpio_put(3u, false);
}

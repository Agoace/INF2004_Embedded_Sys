/* Selected instead of platform/drv_gpio_irq_hw.c in BENCH_KERNEL only. */
#if defined(BENCH_KERNEL) && BENCH_KERNEL
#include "bench_gpio.h"
#include "../platform/drv_gpio_irq_hw.h"
uint32_t drv_gpio_irq_hw_pending(unsigned int group)
{
    return bk_gpio_pending(group);
}
void drv_gpio_irq_hw_ack(unsigned int pin, uint32_t events)
{
    bk_gpio_ack(pin, events);
}
void drv_gpio_irq_hw_enable(uint32_t pins)
{
    bk_gpio_enable(pins);
}
#endif

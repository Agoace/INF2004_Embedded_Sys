/* Stage 0: no edge producer; native dispatcher remains linked. */
#include "drv_gpio_irq_hw.h"
#include "../sim/pil_stage0.h"

uint32_t drv_gpio_irq_hw_pending(unsigned int group)
{
    (void)group;
    return 0u;
}

void drv_gpio_irq_hw_ack(unsigned int pin, uint32_t events)
{
    (void)pin;
    (void)events;
}

void drv_gpio_irq_hw_enable(uint32_t pins)
{
    (void)pins;
}

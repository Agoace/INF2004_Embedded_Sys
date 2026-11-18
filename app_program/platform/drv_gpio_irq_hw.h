#ifndef DRV_GPIO_IRQ_HW_H
#define DRV_GPIO_IRQ_HW_H
#include <stdint.h>
uint32_t drv_gpio_irq_hw_pending(unsigned int group);
void drv_gpio_irq_hw_ack(unsigned int pin, uint32_t events);
void drv_gpio_irq_hw_enable(uint32_t pins);
#endif

#ifndef DRV_GPIO_IRQ_H
#define DRV_GPIO_IRQ_H
#include <stdint.h>
#define DRV_GPIO_IRQ_NUMBER 13u
#define DRV_GPIO_IRQ_PRIORITY 1
#define DRV_GPIO_EDGE_FALL 4u
#define DRV_GPIO_EDGE_RISE 8u
/* Callback runs on core 0 in IRQ context after edge acknowledgement. */
typedef void (*drv_gpio_irq_fn)(unsigned int pin, uint32_t events, void *ctx);
/* Init and add must run on kernel processor 1 (physical core 0). */
int drv_gpio_irq_init(void);
/* Edge-only subscriptions; overlapping pin masks are refused. No heap. */
int drv_gpio_irq_add(uint32_t pins, drv_gpio_irq_fn fn, void *ctx);
#endif

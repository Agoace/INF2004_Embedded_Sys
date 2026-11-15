#ifndef BENCH_KERNEL_GPIO_H
#define BENCH_KERNEL_GPIO_H
#include <stdbool.h>
#include <stdint.h>
/* Core0 only. Generator and provider access must be interrupt-serialized. */
void bk_gpio_reset(void);
void bk_gpio_enable(uint32_t pins);
bool bk_gpio_edge(unsigned int wheel, uint8_t next_ab);
uint8_t bk_gpio_ab(unsigned int wheel);
uint32_t bk_gpio_pending(unsigned int group);
void bk_gpio_ack(unsigned int pin, uint32_t events);
uint32_t bk_gpio_acked(void); /* Successfully consumed virtual edges. */
#endif

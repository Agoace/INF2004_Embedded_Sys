#ifndef SIM_HARDWARE_GPIO_H
#define SIM_HARDWARE_GPIO_H
#include <stdbool.h>
#define GPIO_IN false
void gpio_init(unsigned int pin);
void gpio_set_dir(unsigned int pin, bool out);
void gpio_pull_up(unsigned int pin);
bool gpio_get(unsigned int pin);
#endif

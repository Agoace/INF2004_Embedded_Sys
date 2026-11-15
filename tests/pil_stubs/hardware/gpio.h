#ifndef PIL_FAKE_GPIO_H
#define PIL_FAKE_GPIO_H
#define GPIO_FUNC_UART 2u
void gpio_set_function(unsigned int pin, unsigned int function);
void gpio_pull_up(unsigned int pin);
#endif

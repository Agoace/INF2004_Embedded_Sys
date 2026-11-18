/* SDK-only register access, no kernel headers. Core-0 ownership only. */
#include "drv_gpio_irq_hw.h"
#include "hardware/structs/io_bank0.h"
uint32_t drv_gpio_irq_hw_pending(unsigned int group) {
    return io_bank0_hw->proc0_irq_ctrl.ints[group];
}
void drv_gpio_irq_hw_ack(unsigned int pin, uint32_t events) {
    io_bank0_hw->intr[pin / 8u] = (events & 12u) << ((pin % 8u) * 4u);
}
void drv_gpio_irq_hw_enable(uint32_t pins) {
    for (unsigned int pin = 0u; pin < 30u; ++pin) {
        if ((pins & (1u << pin)) != 0u) {
            const unsigned int group = pin / 8u, shift = (pin % 8u) * 4u;
            drv_gpio_irq_hw_ack(pin, 12u);
            io_bank0_hw->proc0_irq_ctrl.inte[group] |= 12u << shift;
        }
    }
}

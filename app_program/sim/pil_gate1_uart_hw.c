#include "pil_gate1_uart_hw.h"
#include "pil_gate1_reset.h"
#include "hardware/structs/watchdog.h"
#include "hardware/structs/vreg_and_chip_reset.h"
#include "hardware/gpio.h"
#include "hardware/address_mapped.h"
#include "hardware/structs/uart.h"
#include "hardware/structs/resets.h"
#include "hardware/structs/timer.h"
#include "hardware/regs/resets.h"

void pil_gate1_reset_snapshot(pil_gate1_reset_t *out)
{
    out->reason = watchdog_hw->reason;
    out->chip_reset = vreg_and_chip_reset_hw->chip_reset;
    for (unsigned int i = 0u; i < 8u; ++i) {
        out->scratch[i] = watchdog_hw->scratch[i];
    }
}

bool pil_gate1_uart_init(uint32_t hz)
{
    if (hz != 125000000u) { return false; }
    hw_set_bits(&resets_hw->reset, RESETS_RESET_UART0_BITS);
    hw_clear_bits(&resets_hw->reset, RESETS_RESET_UART0_BITS);
    unsigned int retry;
    for (retry = 0u; retry < 100000u; ++retry) {
        if ((resets_hw->reset_done & RESETS_RESET_UART0_BITS) != 0u) { break; }
    }
    if (retry == 100000u) { return false; }
    uart0_hw->cr = 0u; uart0_hw->imsc = 0u; uart0_hw->dmacr = 0u;
    uart0_hw->rsr = 0u; uart0_hw->icr = 0x7ffu;
    const uint32_t divisor = (hz * 4u + 460800u) / 921600u;
    uart0_hw->ibrd = divisor / 64u;
    uart0_hw->fbrd = divisor % 64u;
    uart0_hw->lcr_h = 0x70u; /* 8N1, FIFO. Never loopback/flow control. */
    uart0_hw->ifls = 0x12u; /* RX/TX half FIFO. RX timeout handles tail. */
    gpio_set_function(0u, GPIO_FUNC_UART);
    gpio_set_function(1u, GPIO_FUNC_UART);
    gpio_pull_up(1u);
    uart0_hw->cr = 0x301u;
    uart0_hw->imsc = 0x7d0u;
    return true;
}
bool pil_gate1_uart_rx(uint8_t *byte, uint32_t *errors)
{
    if ((uart0_hw->fr & 16u) != 0u) { return false; }
    const uint32_t data = uart0_hw->dr;
    *byte = (uint8_t)data; *errors = (data >> 8u) & 7u;
    return true;
}
bool pil_gate1_uart_tx(uint8_t byte)
{
    if ((uart0_hw->fr & 32u) != 0u) { return false; }
    uart0_hw->dr = byte;
    return true;
}
void pil_gate1_uart_tx_enable(bool enabled)
{
    if (enabled) { uart0_hw->imsc |= 32u; }
    else { uart0_hw->imsc &= ~32u; }
}
uint32_t pil_gate1_uart_errors(void)
{
    const uint32_t errors = uart0_hw->rsr & 15u;
    uart0_hw->rsr = 0u;
    return errors;
}
void pil_gate1_uart_ack(void) { uart0_hw->icr = 0x7ffu; }
bool pil_gate1_uart_idle(void)
{
    return (uart0_hw->fr & (8u | 128u)) == 128u;
}
void pil_gate1_uart_disable(void)
{
    uart0_hw->imsc = 0u; uart0_hw->cr = 0u;
}
void pil_gate1_lease_set(bool enabled, uint32_t deadline)
{
    timer_hw->inte &= ~2u;
    timer_hw->intf &= ~2u; timer_hw->intr = 2u;
    if (!enabled) { return; }
    timer_hw->alarm[1] = deadline;
    timer_hw->inte |= 2u;
    if (timer_hw->timerawl - deadline < UINT32_C(0x80000000)) {
        timer_hw->intf |= 2u;
    }
}
void pil_gate1_lease_ack(void)
{
    timer_hw->intf &= ~2u; timer_hw->intr = 2u;
}

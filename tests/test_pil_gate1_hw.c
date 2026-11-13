#include "pil_gate1_uart_hw.h"
#include "hardware/structs/uart.h"
#include "hardware/structs/timer.h"
#include "hardware/structs/resets.h"
#include <assert.h>

uart_hw_t fake_uart;
resets_hw_t fake_resets;
pil_test_timer_t pil_test_timer;
#define fake_timer pil_test_timer
unsigned int pin_function[2];
static uint32_t real_time;
uint32_t drv_time_us(void) { return ++real_time; }
extern uint32_t pil_gate1_line_time_us(void);
extern uint32_t pil_gate1_native_entry(void);
void gpio_set_function(unsigned int pin, unsigned int function)
{
    assert(pin < 2u); pin_function[pin] = function;
}
void gpio_pull_up(unsigned int pin) { assert(pin == 1u); }
int main(void)
{
    assert(pil_gate1_line_time_us() == 1u);
    real_time = 100u;
    assert(pil_gate1_native_entry() == 1u);
    assert(pil_gate1_line_time_us() == 101u);
    assert(pil_gate1_native_entry() == 101u);
    assert(!pil_gate1_uart_init(125000000u));
    fake_resets.reset_done = 1u << 22u;
    assert(pil_gate1_uart_init(125000000u));
    assert(fake_uart.ibrd == 8u && fake_uart.fbrd == 31u);
    assert(fake_uart.lcr_h == 0x70u && fake_uart.cr == 0x301u);
    assert(pin_function[0] == 2u && pin_function[1] == 2u);
    assert((fake_uart.cr & (1u << 7u)) == 0u); /* No loopback. */
    pil_gate1_uart_tx_enable(true);
    assert((fake_uart.imsc & 32u) != 0u);
    pil_gate1_uart_tx_enable(false);
    assert((fake_uart.imsc & 32u) == 0u);
    uint8_t byte; uint32_t flags;
    fake_uart.fr = 16u;
    assert(!pil_gate1_uart_rx(&byte, &flags));
    fake_uart.fr = 0u; fake_uart.dr = 0x541u;
    assert(pil_gate1_uart_rx(&byte, &flags));
    assert(byte == 0x41u && flags == 5u);
    fake_uart.rsr = 8u;
    assert(pil_gate1_uart_errors() == 8u && fake_uart.rsr == 0u);
    fake_uart.fr = 32u;
    assert(!pil_gate1_uart_tx(42u));
    fake_uart.fr = 0u;
    assert(pil_gate1_uart_tx(42u) && fake_uart.dr == 42u);
    fake_timer.timerawl = 100u;
    pil_gate1_lease_set(true, 99u);
    assert(fake_timer.alarm[1] == 99u && (fake_timer.intf & 2u) != 0u);
    pil_gate1_lease_ack();
    assert((fake_timer.intf & 2u) == 0u && fake_timer.intr == 2u);
    pil_gate1_lease_set(false, 0u);
    assert((fake_timer.inte & 2u) == 0u);
    return 0;
}

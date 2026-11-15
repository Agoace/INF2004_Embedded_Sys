/* SDK-only MMIO boundary; kernel owns both IRQ registrations and clock. */
#ifndef PIL_GATE1_UART_HW_H
#define PIL_GATE1_UART_HW_H
#include <stdbool.h>
#include <stdint.h>
bool pil_gate1_uart_init(uint32_t peripheral_hz);
bool pil_gate1_uart_rx(uint8_t *byte, uint32_t *errors);
bool pil_gate1_uart_tx(uint8_t byte);
void pil_gate1_uart_tx_enable(bool enabled);
uint32_t pil_gate1_uart_errors(void);
void pil_gate1_uart_ack(void);
bool pil_gate1_uart_idle(void);
void pil_gate1_uart_disable(void);
void pil_gate1_lease_set(bool enabled, uint32_t deadline);
void pil_gate1_lease_ack(void);
#endif

#ifndef PIL_FAKE_UART_H
#define PIL_FAKE_UART_H
#include <stdint.h>
typedef struct {
    uint32_t dr, rsr, fr, ibrd, fbrd, lcr_h, cr, ifls, imsc, icr, dmacr;
} uart_hw_t;
extern uart_hw_t fake_uart;
#define uart0_hw (&fake_uart)
#endif

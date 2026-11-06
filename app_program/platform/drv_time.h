#ifndef DRV_TIME_H
#define DRV_TIME_H
#include <stdint.h>
/* Unsigned microsecond differences wrap safely for intervals below 2^32 us. */
uint32_t drv_time_us(void);
/* Milliseconds use the hardware high word too: no 71-minute discontinuity. */
uint32_t drv_time_ms(void);
#endif

#ifndef DRV_TIME_H
#define DRV_TIME_H
#include <stdint.h>
/* Task context, core 0 before subsystem startup; repeated calls are safe. */
int drv_time_init(void);
/* Compare TIMER against elapsed kernel time over about 100 ms, limit 2%. */
int drv_time_check(void);
/* Task context: window 1..1000 ms; optional measured outputs; kernel errors
 * propagate, clock mismatch returns E_SYS. Used by S0-U01 with 1000 ms/1%. */
int drv_time_check_window(uint32_t window_ms, unsigned int tolerance_pct,
                          uint32_t *timer_us, uint32_t *kernel_ms);
/* Unsigned microsecond differences wrap safely for intervals below 2^32 us. */
uint32_t drv_time_us(void);
/* Milliseconds use the hardware high word too: no 71-minute discontinuity. */
uint32_t drv_time_ms(void);
#endif

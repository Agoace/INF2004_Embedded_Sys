#ifndef DRV_TIME_HW_H
#define DRV_TIME_HW_H
#include <stdint.h>
/* 0 if configured and RUNNING, -1 on invalid cycles or bounded wait expiry. */
int drv_time_hw_init(uint32_t cycles);
uint32_t drv_time_hw_us(void);
uint32_t drv_time_hw_ms(void);
#endif

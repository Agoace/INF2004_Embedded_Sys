#ifndef DRV_LINE_HW_H
#define DRV_LINE_HW_H
#include <stdint.h>
int drv_line_hw_init(void);
int drv_line_hw_read(uint16_t raw[3]);
void drv_line_hw_alarm(uint32_t at_us);
void drv_line_hw_ack(void);
#endif

#ifndef DRV_ULTRASONIC_HW_H
#define DRV_ULTRASONIC_HW_H
#include <stdbool.h>
void drv_ultrasonic_hw_init(void);
bool drv_ultrasonic_hw_echo(void);
void drv_ultrasonic_hw_pulse(void);
#endif

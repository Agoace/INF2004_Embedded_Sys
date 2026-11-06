#include "drv_time.h"
#include "drv_time_hw.h"
uint32_t drv_time_us(void) { return drv_time_hw_us(); }
uint32_t drv_time_ms(void) { return drv_time_hw_ms(); }

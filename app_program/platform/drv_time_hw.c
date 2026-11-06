/* SDK-only translation unit. Never include kernel headers here. */
#include "drv_time_hw.h"
#include "hardware/structs/timer.h"
uint32_t drv_time_hw_us(void) { return timer_hw->timerawl; }
uint32_t drv_time_hw_ms(void) {
    uint32_t hi, lo;
    do {
        hi = timer_hw->timerawh;
        lo = timer_hw->timerawl;
    } while (hi != timer_hw->timerawh);
    return (uint32_t)((((uint64_t)hi << 32u) | lo) / 1000u);
}

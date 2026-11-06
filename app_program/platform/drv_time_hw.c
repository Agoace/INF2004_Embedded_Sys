/* SDK-only translation unit. Never include kernel headers here. */
#include "drv_time_hw.h"
#include "hardware/structs/timer.h"
#include "hardware/ticks.h"
int drv_time_hw_init(uint32_t cycles) {
    if (cycles == 0u || cycles > WATCHDOG_TICK_CYCLES_BITS) return -1;
    const uint32_t config = WATCHDOG_TICK_ENABLE_BITS | cycles;
    /* Do not restart an already configured generator, or reset TIMER itself. */
    if ((watchdog_hw->tick & (WATCHDOG_TICK_ENABLE_BITS | WATCHDOG_TICK_CYCLES_BITS)) != config)
        tick_start(TICK_WATCHDOG, cycles);
    /* SDK 2.3.0 tick_is_running() checks ENABLE, not RUNNING, on RP2040.
     * Bound the wait with CPU iterations: TIMER may not be running yet. */
    for (unsigned int retry = 0u; retry < 100000u; ++retry)
        if ((watchdog_hw->tick & WATCHDOG_TICK_RUNNING_BITS) != 0u) return 0;
    return -1;
}
uint32_t drv_time_hw_us(void) { return timer_hw->timerawl; }
uint32_t drv_time_hw_ms(void) {
    uint32_t hi, lo;
    do {
        hi = timer_hw->timerawh;
        lo = timer_hw->timerawl;
    } while (hi != timer_hw->timerawh);
    return (uint32_t)((((uint64_t)hi << 32u) | lo) / 1000u);
}

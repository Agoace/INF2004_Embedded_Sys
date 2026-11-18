#include <assert.h>
#include <hardware/watchdog.h>
#include "../app_program/platform/drv_watchdog_hw.h"
host_watchdog_t host_watchdog;
static unsigned int enables, feeds;
void watchdog_enable(uint32_t ms, bool pause) {
    assert(ms==100u && pause); ++enables;
    host_watchdog.load=ms*2000u; /* SDK RP2040-E1 contract. */
    host_watchdog.ctrl|=WATCHDOG_CTRL_ENABLE_BITS;
    host_watchdog.scratch[4]=HOST_WATCHDOG_MAGIC;
}
void watchdog_update(void) { ++feeds; }
bool watchdog_caused_reboot(void) { return host_watchdog.reason!=0u; }
bool watchdog_enable_caused_reboot(void) {
    return watchdog_caused_reboot() && host_watchdog.scratch[4]==HOST_WATCHDOG_MAGIC;
}
int main(void) {
    uint32_t previous;
    assert(drv_watchdog_hw_reset(&previous)==DRV_RESET_POWER_RUN && previous==UINT32_MAX);
    host_watchdog.reason=WATCHDOG_REASON_TIMER_BITS;
    host_watchdog.scratch[4]=HOST_WATCHDOG_MAGIC;
    assert(drv_watchdog_hw_reset(0)==DRV_RESET_WATCHDOG);
    host_watchdog.reason=WATCHDOG_REASON_FORCE_BITS;
    assert(drv_watchdog_hw_reset(0)==DRV_RESET_OTHER);
    host_watchdog.reason=WATCHDOG_REASON_TIMER_BITS;
    host_watchdog.scratch[4]=0xb007c0d3u; /* bootrom/picotool target reboot */
    assert(drv_watchdog_hw_reset(0)==DRV_RESET_OTHER);
    host_watchdog.scratch[4]=0u;
    assert(drv_watchdog_hw_reset(0)==DRV_RESET_OTHER);
    assert(drv_watchdog_hw_start(100u,12u)==-1 && enables==0u);
    host_watchdog.tick=WATCHDOG_TICK_ENABLE_BITS|WATCHDOG_TICK_RUNNING_BITS|12u;
    assert(drv_watchdog_hw_start(0u,12u)==-1);
    assert(drv_watchdog_hw_start(8389u,12u)==-1);
    assert(drv_watchdog_hw_start(100u,13u)==-1);
    assert(drv_watchdog_hw_start(100u,12u)==0 && enables==1u && host_watchdog.load==200000u);
    drv_watchdog_hw_feed(); assert(feeds==1u);
    drv_watchdog_hw_test_begin(); drv_watchdog_hw_test_sample(99u);
    host_watchdog.reason=WATCHDOG_REASON_TIMER_BITS;
    assert(drv_watchdog_hw_reset(&previous)==DRV_RESET_WATCHDOG && previous==99u);
    assert(drv_watchdog_hw_reset(&previous)==DRV_RESET_WATCHDOG && previous==UINT32_MAX);
    drv_watchdog_hw_test_begin(); drv_watchdog_hw_test_sample(88u);
    host_watchdog.reason=0u;
    assert(drv_watchdog_hw_reset(&previous)==DRV_RESET_POWER_RUN && previous==UINT32_MAX);
    return 0;
}

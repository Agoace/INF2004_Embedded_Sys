#include <assert.h>
#include <string.h>
#include <tk/tkernel.h>
#include "../app_program/platform/drv_watchdog.h"
#include "../app_program/platform/drv_watchdog_hw.h"
static bool time_ready;
static int hw_error;
static unsigned int starts, feeds, snapshots;
static uint32_t now;
bool drv_time_ready(void) { return time_ready; }
uint32_t drv_time_us(void) { return now; }
INT tk_get_prc(void) { return 1; }
int drv_watchdog_hw_start(uint32_t ms,uint32_t mhz) {
    assert(ms==100u && mhz==12u && snapshots==1u); ++starts; return hw_error;
}
void drv_watchdog_hw_feed(void) { ++feeds; }
drv_watchdog_reset_t drv_watchdog_hw_reset(uint32_t *previous) {
    ++snapshots; if (previous) *previous=99u; return DRV_RESET_WATCHDOG;
}
void drv_watchdog_hw_test_begin(void) { }
void drv_watchdog_hw_test_sample(uint32_t ms) { assert(ms==99u); }
int main(void) {
    drv_watchdog_feed(); assert(feeds==0u);
    assert(drv_watchdog_start(100u)==E_OBJ && starts==0u);
    assert(drv_watchdog_test_begin()==E_OBJ);
    time_ready=true;
    assert(drv_watchdog_start(0u)==E_PAR && drv_watchdog_start(8389u)==E_PAR);
    hw_error=-1; assert(drv_watchdog_start(100u)==E_SYS);
    drv_watchdog_feed(); assert(feeds==0u);
    hw_error=0; assert(drv_watchdog_start(100u)==E_OK && starts==2u);
    assert(drv_watchdog_start(100u)==E_OBJ && starts==2u);
    assert(drv_watchdog_last_reset()==DRV_RESET_WATCHDOG);
    assert(strcmp(drv_watchdog_reset_name(),"watchdog")==0 && snapshots==1u);
    drv_watchdog_feed(); drv_watchdog_feed(); assert(feeds==2u);
    uint32_t previous; assert(drv_watchdog_test_previous(&previous) && previous==99u);
    now=UINT32_MAX-50000u; assert(drv_watchdog_test_begin()==E_OK);
    now+=99000u; assert(drv_watchdog_test_sample()==99u);
    return 0;
}

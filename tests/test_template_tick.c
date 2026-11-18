#include <assert.h>
#include <stdint.h>
#ifndef TEST_XOSC_MHZ
#define TEST_XOSC_MHZ 12u
#endif
#define XOSC_MHz TEST_XOSC_MHZ
typedef uint32_t UW;
static UW tick_register,written;
static unsigned int reads,writes,running_after;
static UW in_w(unsigned long address) {
    assert(address==0x4005802cUL);++reads;
    if (running_after!=0u && reads>=running_after) tick_register|=0x400u;
    return tick_register;
}
static void out_w(unsigned long address,UW value) {
    assert(address==0x4005802cUL);++writes;written=value;tick_register=value;
}
#include "../kernel/sysdepend/pico_rp2040/timer_tick.h"
int main(void) {
#if TEST_XOSC_MHZ == 0 || TEST_XOSC_MHZ > 511
    assert(knl_startup_timer_tick()==0 && reads==0u && writes==0u);
#else
    assert(XOSC_MHz==12u);
    /* A cold register gets exactly ENABLE | 12: one microsecond at 12 MHz. */
    running_after=4u;
    assert(knl_startup_timer_tick()==1 && writes==1u && written==0x20cu && reads==4u);
    reads=0u;writes=0u;running_after=0u;
    assert(knl_startup_timer_tick()==1 && writes==0u && reads==2u);
    /* Incorrect cycles are repaired even if the old RUNNING bit was set. */
    tick_register=0x600u|24u;reads=0u;running_after=3u;
    assert(knl_startup_timer_tick()==1 && writes==1u && written==0x20cu);
    /* Correct configuration but no RUNNING: do not rewrite or loop forever. */
    tick_register=0x20cu;reads=0u;writes=0u;running_after=0u;
    assert(knl_startup_timer_tick()==0 && writes==0u && reads==100001u);
    /* Unconfigured and permanently stopped: one write, same bounded wait. */
    tick_register=0u;reads=0u;writes=0u;
    assert(knl_startup_timer_tick()==0 && writes==1u && reads==100001u);
#endif
    return 0;
}

/* SDK register headers only. No SDK IRQ, clock, timer or claim runtime. */
#if defined(BENCH_KERNEL) && BENCH_KERNEL
#include "bench_timer_hw.h"
#include "hardware/address_mapped.h"
#include "hardware/structs/timer.h"
uint32_t bk_timer_now(void) { return timer_hw->timerawl; }
void bk_line_timer_init(void) { hw_set_bits(&timer_hw->inte, 1u); }
void bk_line_timer_alarm(uint32_t at_us)
{
    timer_hw->alarm[0] = at_us;
    if ((int32_t)(at_us - timer_hw->timerawl) <= 0) {
        hw_set_bits(&timer_hw->intf, 1u); /* Preserve native alarm0 behavior. */
    }
}
void bk_line_timer_ack(void)
{
    hw_clear_bits(&timer_hw->intf, 1u);
    timer_hw->intr = 1u;
}
void bk_timer_ack(void)
{
    timer_hw->intr = 2u; /* Alarm1 W1C; never clear alarm0 or touch INTF. */
}
bool bk_timer_arm(uint32_t at_us)
{
    timer_hw->alarm[1] = at_us;
    if ((int32_t)(at_us - timer_hw->timerawl) <= 0) {
        timer_hw->armed = 2u;
        return false;
    }
    return true;
}
void bk_timer_enable(bool enable)
{
    if (enable) { hw_set_bits(&timer_hw->inte, 2u); }
    else {
        hw_clear_bits(&timer_hw->inte, 2u);
        timer_hw->armed = 2u;
        timer_hw->intr = 2u;
    }
}
void bk_gpio_pend(void)
{
    /* Core-local NVIC ISPR0; generator ownership is physical core0. */
    __asm__ volatile("dmb" ::: "memory");
    *(volatile uint32_t *)0xe000e200u = 1u << 13u;
}
#endif

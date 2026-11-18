/* Kernel/hardware doubles: arithmetic and error paths, not hardware timing. */
#include <assert.h>
#include <tk/tkernel.h>
#include "../app_program/platform/drv_time.h"
#include "../app_program/platform/drv_time_hw.h"
static uint32_t hw_us, os_ms, scale_num = 100u, scale_den = 100u;
static unsigned int init_calls, delay_calls;
static int init_error;
static ER delay_error, time_error;
int drv_time_hw_init(uint32_t cycles) {
    assert(cycles == 12u); ++init_calls; return init_error;
}
uint32_t drv_time_hw_us(void) { return hw_us; }
uint32_t drv_time_hw_ms(void) { return hw_us / 1000u; }
ER tk_get_otm(SYSTIM *time) {
    time->hi = 0; time->lo = os_ms; return time_error;
}
ER tk_dly_tsk(RELTIM timeout) {
    assert(timeout > 0); ++delay_calls;
    /* Like the kernel, add TIMER_PERIOD; the test must use measured time. */
    const uint32_t elapsed = (uint32_t)timeout + 10u;
    os_ms += elapsed;
    hw_us += (uint32_t)((uint64_t)elapsed * 1000u * scale_num / scale_den);
    return delay_error;
}
static void ratio(uint32_t num, uint32_t den, int expected) {
    scale_num = num; scale_den = den;
    assert(drv_time_check() == expected);
}
int main(void) {
    assert(!drv_time_ready());
    assert(drv_time_init() == E_OK && drv_time_init() == E_OK && init_calls == 2u && drv_time_ready());
    init_error = -1;
    assert(drv_time_init() == E_SYS && !drv_time_ready());
    init_error = 0;
    ratio(100u, 100u, E_OK);
    ratio(427u, 10u, E_SYS);
    ratio(10u, 427u, E_SYS); /* Cold-boot TIMER advances 42.7 times too slowly. */
    ratio(97u, 100u, E_SYS);
    ratio(103u, 100u, E_SYS);
    ratio(98u, 100u, E_OK);
    ratio(102u, 100u, E_OK);
    ratio(979u, 1000u, E_SYS);
    ratio(1021u, 1000u, E_SYS);
    scale_num = scale_den = 100u;
    uint32_t timer_us, kernel_ms;
    assert(drv_time_check_window(1000u, 1u, &timer_us, &kernel_ms) == E_OK);
    assert(timer_us == 1010000u && kernel_ms == 1010u);
    hw_us = UINT32_MAX - 100u; os_ms = UINT32_MAX - 10u;
    assert(drv_time_check() == E_OK); /* Both counter wraps are safe. */
    assert(drv_time_ms() == hw_us / 1000u);
    const unsigned int calls = delay_calls;
    assert(drv_time_check_window(0u, 2u, 0, 0) == E_PAR);
    assert(drv_time_check_window(1001u, 2u, 0, 0) == E_PAR);
    assert(drv_time_check_window(100u, 101u, 0, 0) == E_PAR);
    assert(delay_calls == calls);
    delay_error = E_CTX;
    assert(drv_time_check() == E_CTX);
    delay_error = E_OK; time_error = E_SYS;
    assert(drv_time_check() == E_SYS);
    return 0;
}

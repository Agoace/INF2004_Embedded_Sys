#include <assert.h>
#include <tk/tkernel.h>
#include "../app_program/subsystem5/drv_ultrasonic.h"
#include "../app_program/subsystem5/drv_ultrasonic_hw.h"
#include "../app_program/platform/drv_gpio_irq.h"
static drv_gpio_irq_fn edge;
static uint32_t now;
static bool high;
static int disabled;
int drv_time_init(void) { return E_OK; }
uint32_t drv_time_us(void) { return now; }
INT tk_get_prc(void) { return 1; }
void mock_di(UINT *irq) { assert(!disabled); *irq = 0u; disabled = 1; }
void mock_ei(UINT irq) { assert(irq == 0u && disabled); disabled = 0; }
int drv_gpio_irq_init(void) { return E_OK; }
int drv_gpio_irq_add(uint32_t pins, drv_gpio_irq_fn fn, void *ctx) { (void)ctx; assert(pins == 4u); edge = fn; return E_OK; }
void drv_ultrasonic_hw_init(void) { }
bool drv_ultrasonic_hw_echo(void) { return high; }
void drv_ultrasonic_hw_pulse(void) { now += 10u; }
static void echo(uint32_t at, uint32_t events) { now = at; edge(2u, events, 0); }
int main(void) {
    assert(drv_ultrasonic_init() == E_OK && drv_ultrasonic_init() == E_OK);
    assert(ultrasonic_result_is_none(drv_ultrasonic_latest()));
    assert(drv_ultrasonic_trigger() && drv_ultrasonic_busy());
    assert(!drv_ultrasonic_trigger());
    echo(215u, 8u); echo(1215u, 4u);
    assert(drv_ultrasonic_service() && !drv_ultrasonic_busy());
    assert(ultrasonic_result_distance(drv_ultrasonic_latest()) == 172u);
    assert(!drv_ultrasonic_trigger());
    now = 60000u; assert(drv_ultrasonic_trigger()); now += 2100u;
    assert(drv_ultrasonic_service());
    assert(ultrasonic_result_status(drv_ultrasonic_latest()) == ULTRASONIC_STATUS_NO_RISE);
    assert(ultrasonic_result_streak(drv_ultrasonic_latest()) == 1u);
    now = 120000u; assert(drv_ultrasonic_trigger()); echo(120215u, 8u); now += 3100u;
    assert(drv_ultrasonic_service());
    assert(ultrasonic_result_status(drv_ultrasonic_latest()) == ULTRASONIC_STATUS_NO_RETURN);
    assert(ultrasonic_result_streak(drv_ultrasonic_latest()) == 0u);
    high = true;
    for (unsigned int i = 0u; i < 7u; ++i) { now += 60000u; assert(drv_ultrasonic_trigger()); }
    assert(ultrasonic_result_fault(drv_ultrasonic_latest()));
    high = false; now += 60000u; assert(drv_ultrasonic_trigger());
    const uint32_t t = now; echo(t + 200u, 8u); echo(t + 300u, 4u);
    assert(drv_ultrasonic_service() && ultrasonic_result_too_near(drv_ultrasonic_latest()));
    assert(drv_ultrasonic_debug_latest().pulse_us == 100u);
    now += 60000u;
    const uint32_t late = now;
    assert(drv_ultrasonic_trigger());
    echo(late + 2100u, 8u); echo(late + 2500u, 4u);
    assert(drv_ultrasonic_service());
    assert(ultrasonic_result_status(drv_ultrasonic_latest()) == ULTRASONIC_STATUS_NO_RISE);
    now += 60000u;
    const uint32_t far = now;
    assert(drv_ultrasonic_trigger());
    echo(far + 200u, 8u); echo(far + 3200u, 4u);
    assert(drv_ultrasonic_service());
    assert(ultrasonic_result_status(drv_ultrasonic_latest()) == ULTRASONIC_STATUS_NO_RETURN);
    assert(drv_ultrasonic_debug_latest().distance_mm == 515u);
    return 0;
}

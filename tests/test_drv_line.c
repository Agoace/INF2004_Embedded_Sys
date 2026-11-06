#include <assert.h>
#include <tk/tkernel.h>
#include "../app_program/subsystem3/drv_line.h"
#include "../app_program/subsystem3/drv_line_hw.h"
static FP handler;
static uint32_t now, alarm;
static int disabled, hw_error;
static unsigned int wakes;
ER tk_wup_tsk(ID id) { assert(id==7);++wakes;return E_OK; }
int drv_time_init(void) { return E_OK; }
uint32_t drv_time_us(void) { return now; }
INT tk_get_prc(void) { return 1; }
ER tk_def_int(UINT n, const T_DINT *d) { assert(n == 0u && d->intatr == TA_HLNG); handler = d->inthdr; return E_OK; }
void DisableInt(UINT n) { assert(n == 0u); }
void ClearInt(UINT n) { assert(n == 0u); }
void EnableInt(UINT n, INT level) { assert(n == 0u && level == 2); }
void mock_di(UINT *irq) { assert(!disabled); *irq = 0u; disabled = 1; }
void mock_ei(UINT irq) { assert(irq == 0u && disabled); disabled = 0; }
int drv_line_hw_init(void) { return hw_error; }
int drv_line_hw_read(uint16_t raw[3]) { raw[0] = 1234u; raw[1] = 2345u; raw[2] = 3456u; now+=48u; return hw_error; }
void drv_line_hw_alarm(uint32_t at) { alarm = at; }
void drv_line_hw_ack(void) { }
static void fire(uint32_t at) { now = at; ((void (*)(UINT))handler)(0u); }
int main(void) {
    hw_error = -1; assert(drv_line_init() == E_SYS);
    hw_error = 0; assert(drv_line_init() == E_OK && alarm == 1000u);
    assert(drv_line_init() == E_OK);
    drv_line_listen(7);
    drv_line_sample_t s;
    assert(!drv_line_take(&s));
    fire(1000u); assert(alarm == 2000u && drv_line_take(&s));
    assert(s.us == 1000u && s.raw[0] == 1234u && s.raw[1] == 2345u && s.raw[2] == 3456u);
    fire(4500u); assert(alarm == 5000u);
    drv_line_stats_t st; drv_line_stats(&st);
    assert(st.samples == 2u && st.missed == 2u && st.isr_max_us==48u && wakes==2u && st.max[2]==3456u);
    drv_line_flush(); assert(!drv_line_take(&s));
    for (unsigned int i = 0u; i < 130u; ++i) fire(5000u + i * 1000u);
    drv_line_stats(&st); assert(st.dropped == 3u && st.min[0] == 1234u && st.max[1] == 2345u);
    drv_line_flush(); hw_error = -1; fire(135000u);
    assert(!drv_line_take(&s)); drv_line_stats(&st); assert(st.missed > 2u);
    return 0;
}

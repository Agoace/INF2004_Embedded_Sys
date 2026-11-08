/* Run the actual exclusive TEST observer with clock/button/board doubles. */
#include "../app_program/subsystem3/barcode_capture.h"
#include "../app_program/subsystem3/barcode_test.h"
#include "../app_program/subsystem3/line_service.h"
#include "../app_program/subsystem3/drv_line.h"
#include "../app_program/subsystem2/motion.h"
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static uint32_t now;
static unsigned int ranges;
static unsigned int results;
static unsigned int adc_rows;
static unsigned int stop_calls;
static unsigned int dly_calls;
static bool dump_started;
static bool button_ready;
static jmp_buf finished;

static void advance(uint32_t ms)
{
    for (uint32_t i = 0u; i < ms; ++i) {
        ++now;
        barcode_capture_observe(now * 1000u, (uint16_t)(300u + now % 200u));
    }
}

uint32_t drv_time_us(void)
{
    return now * 1000u;
}

uint32_t drv_time_ms(void)
{
    return now;
}

int motion_stop(void)
{
    ++stop_calls;
    return E_OK;
}

int motion_estop(void)
{
    assert(0);
    return E_SYS;
}

int motion_snapshot_try_read(motion_status_t *out, uint32_t *us)
{
    memset(out, 0, sizeof(*out));
    out->counts[0] = (int32_t)now;
    out->counts[1] = (int32_t)now;
    *us = drv_time_us();
    return E_OK;
}

int line_snapshot_try_read(line_snapshot_t *out)
{
    memset(out, 0, sizeof(*out));
    return E_OK;
}

void drv_line_stats(drv_line_stats_t *out)
{
    memset(out, 0, sizeof(*out));
}

void barcode_hw_button_init(void)
{
    button_ready = true;
}

bool barcode_hw_button_down(void)
{
    assert(button_ready);
    static const uint32_t presses[] = {100u, 1300u, 2500u, 3800u, 4200u};
    for (size_t i = 0u; i < sizeof(presses) / sizeof(presses[0]); ++i) {
        if (now >= presses[i] && now < presses[i] + 60u) {
            return true;
        }
    }
    return false;
}

ID tk_get_tid(void)
{
    return 2;
}

ID tk_cre_cyc(const T_CCYC *c)
{
    assert(c->cyctim == 10u && c->cycphs == 10u && c->cychdr != NULL);
    return 3;
}

ER tk_sta_cyc(ID id)
{
    assert(id == 3);
    return E_OK;
}

ER tk_stp_cyc(ID id)
{
    assert(id == 3);
    return E_OK;
}

ER tk_del_cyc(ID id)
{
    assert(id == 3);
    return E_OK;
}

ER tk_wup_tsk(ID id)
{
    assert(id == 2);
    return E_OK;
}

INT tk_can_wup(ID id)
{
    assert(id == TSK_SELF);
    return 0;
}

ER tk_slp_tsk(TMO timeout)
{
    assert(timeout == 100u && now < 10000u);
    advance(10u);
    return E_OK;
}

ER tk_dly_tsk(RELTIM ms)
{
    assert(!barcode_capture_active());
    ++dly_calls;
    advance(ms);
    return E_OK;
}

int tm_printf(const UB *format, ...)
{
    char text[512];
    va_list args;
    va_start(args, format);
    (void)vsnprintf(text, sizeof(text), (const char *)format, args);
    va_end(args);
    assert(strstr(text, "ABORT") == NULL);
    if (strstr(text, "[BC_RANGE]") != NULL) {
        ++ranges;
    }
    if (strstr(text, "[BC_RESULT]") != NULL) {
        assert(!barcode_capture_active() && ranges == 3u);
        ++results;
    }
    if (strstr(text, "[BC_DUMP] BEGIN") != NULL) {
        assert(!barcode_capture_active() && results == 2u);
        dump_started = true;
    }
    if (strncmp(text, "BC_ADC,", 7u) == 0 &&
        strncmp(text, "BC_ADC,index", 12u) != 0) {
        assert(dump_started && !barcode_capture_active());
        ++adc_rows;
    }
    if (strstr(text, "frozen; GP20 starts NEW pass") != NULL) {
        assert(adc_rows == 400u && dly_calls > 10u && stop_calls == 1u);
        assert(!barcode_capture_full() && barcode_capture_gaps() == 0u);
        longjmp(finished, 1);
    }
    return 0;
}

int main(void)
{
    if (setjmp(finished) == 0) {
        barcode_test();
        assert(0 && "observer must wait for explicit operator rearm");
    }
    return 0;
}

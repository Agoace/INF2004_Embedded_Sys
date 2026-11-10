#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <string.h>
#include <tk/tkernel.h>
#include "../app_program/subsystem3/line_service.h"
#if APP_BARCODE_CAPTURE
#include "../app_program/subsystem3/barcode_capture.h"
#endif
static FP entry;static jmp_buf end;static unsigned int sleeps;
static bool available;static uint32_t clock_us;static uint16_t value=500u;
static int held;static unsigned int listens;
static uint16_t barcode_raw = 4095u;
static unsigned int calibration_logs;
static unsigned long logged_minimum;
static unsigned long logged_maximum;
int tm_printf(const UB *format, ...)
{
    assert(format && !held);
    assert(strncmp((const char *)format, "[BC_CAL] ", 9u) == 0);
    if (strstr((const char *)format, "GP28 %lu..%lu")) {
        va_list args;
        va_start(args, format);
        (void)va_arg(args, unsigned long); /* generation */
        (void)va_arg(args, unsigned long); /* samples */
        logged_minimum = va_arg(args, unsigned long);
        logged_maximum = va_arg(args, unsigned long);
        va_end(args);
    }
    ++calibration_logs;
    return 0;
}
ID app_start_task(FP f,PRI p,UW core,const char *name) { (void)name;assert(p==11 && core==1u);entry=f;return 7; }
ID tk_cre_mtx(const T_CMTX *c) { assert(c->mtxatr==TA_INHERIT);return 1; }
ER tk_loc_mtx(ID id,TMO t) { assert(id==1 && t==TMO_FEVR && !held);held=1;return E_OK; }
ER tk_unl_mtx(ID id) { assert(id==1 && held);held=0;return E_OK; }
ER tk_slp_tsk(TMO t) { assert(t==TMO_FEVR);if (sleeps++) longjmp(end,1);return E_OK; }
INT tk_can_wup(ID id) { assert(id==TSK_SELF);return 0; }
int drv_line_init(void) { return E_OK; }
void drv_line_listen(int id) { assert(id==7);++listens; }
void drv_line_flush(void) { assert(held);available=false; }
void drv_line_stats(drv_line_stats_t *s) { *s=(drv_line_stats_t){.samples=42u}; }
bool drv_line_take(drv_line_sample_t *s) {
    if (!available) return false;
    available=false;
    *s = (drv_line_sample_t){.us = clock_us,
        .raw = {value, (uint16_t)(4000u - value), barcode_raw}};
    return true;
}
uint32_t drv_time_us(void) { return clock_us++; }
uint32_t drv_time_ms(void) { return clock_us/1000u; }
static void sample(uint16_t v) {
    value=v;clock_us+=1000u;available=true;sleeps=0u;
    if (setjmp(end)==0) ((void (*)(INT,void *))entry)(0,0);
    assert(!held);
}
int main(void) {
    barcode_cal_report_t calibration = {.samples = 123u};
    assert(line_barcode_calibration_try_read(0) == E_PAR);
    assert(line_barcode_calibration_try_read(&calibration) == E_OBJ);
    assert(calibration.samples == 123u);
    line_snapshot_t view;assert(line_snapshot_try_read(0)==E_PAR && line_snapshot_try_read(&view)==E_OBJ);
    line_snapshot_t s;assert(line_snapshot_read(&s,false)==E_OBJ && !line_service_running());
    assert(line_service_init()==E_OK && line_service_start()==E_OK && listens==1u && line_service_running());
    assert(line_service_start()==E_OBJ);
    assert(line_barcode_calibration_try_read(&calibration) == E_OK);
    assert(calibration.mode == BC_CAL_IDLE && calibration.samples == 0u);
    assert(line_snapshot_read(&s,false)==E_OK && !s.valid);
#if APP_BARCODE_CAPTURE
    barcode_capture_begin(1u);
#endif
    assert(line_calibration_begin()==E_OK);
    for (unsigned int i=0u;i<16u;++i) sample(i%2u ? 3500u : 500u);
    assert(line_calibration_end()==LP_CALIB_OK);
    assert(line_snapshot_read(&s,false)==E_OK && lp_calibrated(&s.line));
    assert(s.processed==16u && s.raw_bc==4095u && s.raw[2]==0u && s.line.cal_max[2]==0u);
    assert(line_barcode_calibration_try_read(&calibration) == E_OK);
    assert(calibration.mode == BC_CAL_INVALID && calibration.samples == 16u);
    assert(calibration.minimum == 4095u && calibration.maximum == 4095u);
    assert((calibration.faults & BC_CAL_FAULT_RAW) != 0u);
    assert(calibration_logs == 2u);
    sample(3500u);
    assert(line_snapshot_read(&s,false)==E_OK);
    const vc_event_t retained_event=s.event;
    assert(line_snapshot_try_read(&view)==E_OK && !held && view.sequence==17u && view.raw_bc==4095u);
    assert(view.event==retained_event && line_snapshot_read(&s,false)==E_OK && s.event==retained_event);
    assert(line_snapshot_read(&s,true)==E_OK && s.valid && s.sequence==17u && s.cpu_us>0u && s.sample_max_us>0u);
    assert(s.acquisition.samples==42u && s.line.pair);
    const uint16_t floor=lp_floor(&s.line,0u);
    assert(line_calibration_begin()==E_OK);sample(1234u);
    assert(line_calibration_abort()==E_OK);
    assert(line_snapshot_read(&s,false)==E_OK && lp_floor(&s.line,0u)==floor);
#if APP_BARCODE_CAPTURE
    barcode_capture_freeze();
    size_t count;
    const barcode_packed_t *rows = barcode_capture_rows(&count);
    assert(count == 18u && rows[0].raw_flags == 4095u);
    assert(rows[count - 1u].raw_flags == 4095u);
    assert(barcode_capture_max_us() > 0u && !held);
#endif
    assert(line_barcode_calibration_try_read(&calibration) == E_OK);
    assert(calibration.mode == BC_CAL_ABORTED && calibration.samples == 1u);
    assert(calibration.generation == 2u && calibration_logs == 4u);

    /* Evidence is observed independently of pure LINE's unused channel. */
    assert(line_calibration_begin() == E_OK);
    assert(line_barcode_calibration_try_read(&calibration) == E_OK);
    assert(calibration.mode == BC_CAL_COLLECTING);
    assert(calibration.generation == 3u && calibration.samples == 0u);
    barcode_raw = 160u;
    for (unsigned int i = 0u; i < 16u; ++i) {
        sample(i % 2u ? 3500u : 500u);
    }
    barcode_raw = 1200u;
    for (unsigned int i = 0u; i < 11u; ++i) {
        sample(i % 2u ? 3500u : 500u);
    }
    assert(line_calibration_end() == LP_CALIB_OK);
    assert(line_barcode_calibration_try_read(&calibration) == E_OK);
    assert(calibration.mode == BC_CAL_LEARNED);
    assert(calibration.samples == 27u && calibration_logs == 6u);
    assert(calibration.minimum == 160u && calibration.maximum == 1200u);
    assert(logged_minimum == 160u && logged_maximum == 1200u);
    assert(calibration.rise == 310u && calibration.fall == 235u);
    assert(line_snapshot_read(&s, false) == E_OK);
    assert(s.raw_bc == 1200u && s.raw[2] == 0u && s.line.cal_max[2] == 0u);
    sample(500u);
    barcode_cal_report_t after;
    assert(line_barcode_calibration_try_read(&after) == E_OK);
    assert(after.samples == 27u && after.generation == 3u);
    assert(calibration_logs == 6u);

    assert(line_calibration_begin() == E_OK);
    barcode_raw = 160u;
    for (unsigned int i = 0u; i < 16u; ++i) {
        sample(i % 2u ? 3500u : 500u);
    }
    assert(line_calibration_end() == LP_CALIB_OK);
    assert(line_barcode_calibration_try_read(&calibration) == E_OK);
    assert(calibration.mode == BC_CAL_LEARNED);
    assert(calibration.rise == 310u && calibration.fall == 235u);
    assert(calibration.black_samples == 0u && calibration_logs == 8u);

    assert(line_calibration_begin() == E_OK);
    barcode_raw = 1200u;
    for (unsigned int i = 0u; i < 16u; ++i) {
        sample(i % 2u ? 3500u : 500u);
    }
    assert(line_calibration_end() == LP_CALIB_OK);
    assert(line_barcode_calibration_try_read(&calibration) == E_OK);
    assert(calibration.mode == BC_CAL_FALLBACK && calibration_logs == 10u);
    assert(calibration.white_samples == 0u && calibration.white_mean == 0u);
    assert(calibration.rise == 320u && calibration.fall == 240u);
    assert((calibration.faults & BC_CAL_NO_WHITE) != 0u);

    /* A failed pair calibration still reports GP28 evidence, not readiness. */
    assert(line_calibration_begin() == E_OK);
    barcode_raw = 160u;
    for (unsigned int i = 0u; i < 16u; ++i) {
        sample(500u);
    }
    barcode_raw = 1200u;
    for (unsigned int i = 0u; i < 11u; ++i) {
        sample(500u);
    }
    assert(line_calibration_end() == LP_CALIB_LOW_CONTRAST);
    assert(line_barcode_calibration_try_read(&calibration) == E_OK);
    assert(calibration.mode == BC_CAL_ABORTED && calibration_logs == 12u);
    assert(calibration.minimum == 160u && calibration.maximum == 1200u);
    assert(line_tracking_reset()==E_OK);
    assert(line_snapshot_read(&s,false)==E_OK && s.event==VC_EVENT_NONE && s.norm_peak==0u);
    return 0;
}

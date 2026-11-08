#include <assert.h>
#include <setjmp.h>
#include <tk/tkernel.h>
#include "../app_program/subsystem3/line_service.h"
#if APP_BARCODE_CAPTURE
#include "../app_program/subsystem3/barcode_capture.h"
#endif
static FP entry;static jmp_buf end;static unsigned int sleeps;
static bool available;static uint32_t clock_us;static uint16_t value=500u;
static int held;static unsigned int listens;
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
    *s=(drv_line_sample_t){.us=clock_us,.raw={value,(uint16_t)(4000u-value),4095u}};return true;
}
uint32_t drv_time_us(void) { return clock_us++; }
uint32_t drv_time_ms(void) { return clock_us/1000u; }
static void sample(uint16_t v) {
    value=v;clock_us+=1000u;available=true;sleeps=0u;
    if (setjmp(end)==0) ((void (*)(INT,void *))entry)(0,0);
    assert(!held);
}
int main(void) {
    line_snapshot_t view;assert(line_snapshot_try_read(0)==E_PAR && line_snapshot_try_read(&view)==E_OBJ);
    line_snapshot_t s;assert(line_snapshot_read(&s,false)==E_OBJ && !line_service_running());
    assert(line_service_init()==E_OK && line_service_start()==E_OK && listens==1u && line_service_running());
    assert(line_service_start()==E_OBJ);
    assert(line_snapshot_read(&s,false)==E_OK && !s.valid);
#if APP_BARCODE_CAPTURE
    barcode_capture_begin();
#endif
    assert(line_calibration_begin()==E_OK);
    for (unsigned int i=0u;i<16u;++i) sample(i%2u ? 3500u : 500u);
    assert(line_calibration_end()==LP_CALIB_OK);
    assert(line_snapshot_read(&s,false)==E_OK && lp_calibrated(&s.line));
    assert(s.processed==16u && s.raw_bc==4095u && s.raw[2]==0u && s.line.cal_max[2]==0u);
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
    const barcode_sample_t *rows = barcode_capture_rows(&count);
    assert(count == 18u && rows[0].raw == 4095u);
    assert(rows[count - 1u].raw == 4095u && rows[count - 1u].flags == 0u);
    assert(barcode_capture_max_us() > 0u && !held);
#endif
    assert(line_tracking_reset()==E_OK);
    assert(line_snapshot_read(&s,false)==E_OK && s.event==VC_EVENT_NONE && s.norm_peak==0u);
    return 0;
}

#include "subsystem3.h"
#include "line_service.h"
#include "../platform/drv_time.h"
static ID mutex, task_id;
static bool ready, calibrating;
static line_snapshot_t snapshot;
static void line_task(INT code,void *ctx) {
    (void)code;(void)ctx;
    for (;;) {
        if (tk_slp_tsk(TMO_FEVR)<E_OK) continue;
        (void)tk_can_wup(TSK_SELF);
        drv_line_sample_t sample;
        while (drv_line_take(&sample)) {
            const uint32_t start=drv_time_us();
            if (tk_loc_mtx(mutex,TMO_FEVR)<E_OK) continue;
            snapshot.raw[0]=sample.raw[0];snapshot.raw[1]=sample.raw[1];snapshot.raw[2]=0u;
            snapshot.raw_bc=sample.raw[2];
            if (calibrating) lp_calib_add(&snapshot.line,snapshot.raw);
            else {
                const uint32_t ms=drv_time_ms()-(drv_time_us()-sample.us)/1000u;
                const vc_event_t event=lp_update(&snapshot.line,snapshot.raw,ms);
                if (vc_event_type(event)==VC_EVENT_LINE_LOST || vc_event_type(event)==VC_EVENT_LINE_FOUND)
                    snapshot.event=event;
                if (snapshot.line.norm[LP_CENTRE]>snapshot.norm_peak)
                    snapshot.norm_peak=snapshot.line.norm[LP_CENTRE];
            }
            snapshot.timestamp_us=sample.us;snapshot.valid=true;
            ++snapshot.sequence;++snapshot.processed;
            drv_line_stats(&snapshot.acquisition);
            const uint32_t cost=drv_time_us()-start;
            snapshot.cpu_us+=cost;
            if (cost>snapshot.sample_max_us) snapshot.sample_max_us=cost;
            (void)tk_unl_mtx(mutex);
        }
    }
}
int line_service_init(void) {
    if (ready) return E_OK;
    if (mutex<=0) { const T_CMTX c={.mtxatr=TA_INHERIT};mutex=tk_cre_mtx(&c); }
    if (mutex<0) return (int)mutex;
    lp_init(&snapshot.line);lp_set_pair(&snapshot.line,true,0);
    const int er=drv_line_init();
    if (er>=E_OK) ready=true;
    return er;
}
int line_service_start(void) {
    if (!ready) return E_OBJ;
    if (task_id>0) return E_OBJ;
    task_id=app_start_task((FP)line_task,11,TP_PRC1,"line_task");
    if (task_id<0) return (int)task_id;
    drv_line_listen((int)task_id);
    return E_OK;
}
int line_snapshot_read(line_snapshot_t *out,bool consume) {
    if (!out) return E_PAR;
    if (!ready) return E_OBJ;
    const ER er=tk_loc_mtx(mutex,TMO_FEVR);if (er<0) return (int)er;
    *out=snapshot;
    if (consume) snapshot.event=VC_EVENT_NONE;
    return (int)tk_unl_mtx(mutex);
}
int line_calibration_begin(void) {
    if (!ready) return E_OBJ;
    const ER er=tk_loc_mtx(mutex,TMO_FEVR);if (er<0) return (int)er;
    drv_line_flush();lp_calib_begin(&snapshot.line);calibrating=true;
    snapshot.event=VC_EVENT_NONE;snapshot.norm_peak=0u;snapshot.valid=false;
    return (int)tk_unl_mtx(mutex);
}
lp_calib_result_t line_calibration_end(void) {
    if (!ready || tk_loc_mtx(mutex,TMO_FEVR)<0) return LP_CALIB_NO_DATA;
    const lp_calib_result_t result=lp_calib_end(&snapshot.line,0);
    calibrating=false;snapshot.norm_peak=0u;
    (void)tk_unl_mtx(mutex);return result;
}
int line_tracking_reset(void) {
    if (!ready) return E_OBJ;
    const ER er=tk_loc_mtx(mutex,TMO_FEVR);if (er<0) return (int)er;
    lp_reset_tracking(&snapshot.line);snapshot.event=VC_EVENT_NONE;snapshot.norm_peak=0u;
    return (int)tk_unl_mtx(mutex);
}

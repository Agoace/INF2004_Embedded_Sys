#include "subsystem5.h"
#include "sonar_service.h"
#include "drv_ultrasonic.h"
#include "../platform/drv_time.h"
static ID mutex,requests,results,cyclic,task;
static bool ready,physical,await_ack,have_generation,once_reserved,admission_ready;
static uint32_t admitted_generation;
static sonar_request_t current,inflight;
static sonar_snapshot_t snapshot;
static uint32_t next_ms;
static void publish(uint8_t status,ultrasonic_result_t echo,uint32_t now) {
    if (tk_loc_mtx(mutex,TMO_FEVR)<E_OK) return;
    snapshot.generation=current.generation;snapshot.id=current.id;snapshot.angle=current.angle;
    snapshot.echo=echo;snapshot.status=status;snapshot.pending=false;
    snapshot.valid=status==SONAR_ECHO && ultrasonic_result_status(echo)==ULTRASONIC_STATUS_VALID;
    snapshot.timestamp_ms=now;
    snapshot.deadline_ms=current.mode==SONAR_WATCH ? now+(uint32_t)current.interval_ms*2u+20u : current.deadline_ms;
    ++snapshot.result_sequence;
    if (current.generation==admitted_generation && current.mode==SONAR_ONCE) once_reserved=false;
    (void)tk_unl_mtx(mutex);(void)tk_set_flg(results,1u);
}
static void accept(const sonar_request_t *r,uint32_t now) {
    if (have_generation && (int32_t)(r->generation-current.generation)<0) return;
    if (r->mode==SONAR_ACK) {
        if (r->generation==current.generation && r->id==current.id) await_ack=false;
        return;
    }
    const bool new_generation=!have_generation || r->generation!=current.generation;
    if (!new_generation && current.mode==SONAR_ONCE && r->mode!=SONAR_STOP) {
        /* One outstanding one-shot; do not silently replace it. */
        return;
    }
    current=*r;have_generation=true;await_ack=false;
    if (tk_loc_mtx(mutex,TMO_FEVR)>=E_OK) {
        snapshot.generation=r->generation;snapshot.id=r->id;snapshot.angle=r->angle;
        snapshot.valid=false;snapshot.pending=false;snapshot.status=SONAR_EMPTY;
        (void)tk_unl_mtx(mutex);
    }
    if (r->mode==SONAR_STOP) publish(SONAR_CANCELLED,ULTRASONIC_RESULT_NONE,now);
    next_ms=now+(r->mode==SONAR_WATCH ? r->interval_ms : 0u);
}
static void sonar_task(INT code,void *ctx) {
    (void)code;(void)ctx;
    for (;;) {
        if (tk_slp_tsk(TMO_FEVR)<E_OK) continue;
        (void)tk_can_wup(TSK_SELF);
        sonar_request_t r;
        while (tk_rcv_mbf(requests,&r,TMO_POL)==(INT)sizeof(r)) accept(&r,drv_time_ms());
        uint32_t now=drv_time_ms();
        if (drv_ultrasonic_service()) {
            if (physical && inflight.generation==current.generation && inflight.id==current.id && current.mode!=SONAR_STOP) {
                if ((int32_t)(now-inflight.deadline_ms)<=0 || current.mode==SONAR_WATCH)
                    publish(SONAR_ECHO,drv_ultrasonic_latest(),now);
                else publish(SONAR_DEADLINE,ULTRASONIC_RESULT_NONE,now);
                await_ack=current.mode==SONAR_WATCH;
                if (current.mode==SONAR_ONCE) current.mode=SONAR_STOP;
            }
            physical=false;
        }
        if (current.mode==SONAR_ONCE && (int32_t)(now-current.deadline_ms)>=0) {
            publish(SONAR_DEADLINE,ULTRASONIC_RESULT_NONE,now);current.mode=SONAR_STOP;
        }
        if ((current.mode!=SONAR_ONCE && current.mode!=SONAR_WATCH) || physical || await_ack ||
            (int32_t)(now-next_ms)<0) continue;
        if (!drv_ultrasonic_trigger()) continue;
        inflight=current;physical=true;next_ms=now+current.interval_ms;
        if (tk_loc_mtx(mutex,TMO_FEVR)>=E_OK) {
            snapshot.trigger_ms=now;snapshot.pending=true;++snapshot.ping_sequence;
            (void)tk_unl_mtx(mutex);
        }
        /* STUCK_HIGH is an immediate result, exactly as in the reference. */
        if (!drv_ultrasonic_busy()) {
            publish(SONAR_ECHO,drv_ultrasonic_latest(),now);physical=false;
            await_ack=current.mode==SONAR_WATCH;
            if (current.mode==SONAR_ONCE) current.mode=SONAR_STOP;
        }
    }
}
static void release(void *ctx) { (void)ctx;if (task>0) (void)tk_wup_tsk(task); }
int sonar_service_init(void) {
    if (ready) return E_OK;
    const T_CMTX m={.mtxatr=TA_INHERIT};mutex=tk_cre_mtx(&m);if (mutex<0) return (int)mutex;
    const T_CMBF b={.mbfatr=TA_TFIFO,.bufsz=256,.maxmsz=(INT)sizeof(sonar_request_t)};
    requests=tk_cre_mbf(&b);if (requests<0) return (int)requests;
    const T_CFLG f={.flgatr=TA_TFIFO,.iflgptn=0u};results=tk_cre_flg(&f);if (results<0) return (int)results;
    T_CCYC c={.cycatr=TA_HLNG,.cychdr=(FP)release,.cyctim=10u,.cycphs=10u};
#if TK_SUPPORT_SMP
    c.cycatr|=TA_ASSPRC;c.assprc=TP_PRC1;
#endif
    cyclic=tk_cre_cyc(&c);if (cyclic<0) return (int)cyclic;
    const int er=drv_ultrasonic_init();if (er>=E_OK) ready=true;return er;
}
int sonar_service_start(void) {
    if (!ready || task>0) return E_OBJ;
    task=app_start_task((FP)sonar_task,12,TP_PRC1,"sonar_task");if (task<0) return (int)task;
    drv_ultrasonic_listen((int)task);
    return (int)tk_sta_cyc(cyclic);
}
int sonar_request(const sonar_request_t *r) {
    if (!ready || task<=0) return E_OBJ;
    if (!r || r->mode>SONAR_ACK || r->angle < -30 || r->angle > 30 ||
        (r->mode==SONAR_WATCH && r->interval_ms<60u)) return E_PAR;
    const ER lock=tk_loc_mtx(mutex,TMO_POL);if (lock<0) return (int)lock;
    if ((admission_ready && (int32_t)(r->generation-admitted_generation)<0) ||
        (once_reserved && r->generation==admitted_generation && r->mode!=SONAR_STOP && r->mode!=SONAR_ACK)) {
        (void)tk_unl_mtx(mutex);return E_OBJ;
    }
    const ER er=tk_snd_mbf(requests,r,(INT)sizeof(*r),TMO_POL);
    if (er>=E_OK && r->mode!=SONAR_ACK) {
        admitted_generation=r->generation;admission_ready=true;once_reserved=r->mode==SONAR_ONCE;
    }
    (void)tk_unl_mtx(mutex);
    if (er>=E_OK) (void)tk_wup_tsk(task);
    return (int)er;
}
int sonar_result_read(sonar_snapshot_t *out,bool *is_new) {
    if (!ready) return E_OBJ;
    if (!out || !is_new) return E_PAR;
    const ER er=tk_loc_mtx(mutex,TMO_FEVR);if (er<0) return (int)er;
    *out=snapshot;
    UINT bits=0u;*is_new=tk_wai_flg(results,1u,TWF_ORW|TWF_BITCLR,&bits,TMO_POL)>=E_OK;
    return (int)tk_unl_mtx(mutex);
}

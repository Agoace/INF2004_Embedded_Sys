#include <assert.h>
#include <setjmp.h>
#include <string.h>
#include <tk/tkernel.h>
#include "../app_program/subsystem5/sonar_service.h"
static FP entry;static jmp_buf end;static unsigned int sleeps;
static sonar_request_t queue[8];static unsigned int head,tail;
static uint32_t now,last_trigger;static unsigned int triggers;
static bool busy,done,full,immediate,cyclic_failure;static int held;static UINT flags;
static ultrasonic_result_t result=100u;
ID tk_cre_mtx(const T_CMTX *c) { assert(c->mtxatr==TA_INHERIT);return 1; }
ID tk_cre_mbf(const T_CMBF *c) { assert(c->maxmsz==(INT)sizeof(sonar_request_t));return 2; }
ID tk_cre_flg(const T_CFLG *c) { (void)c;return 3; }
ID tk_cre_cyc(const T_CCYC *c) { assert(c->cyctim==10u && c->assprc==1u);return 4; }
ER tk_sta_cyc(ID id) { assert(id==4);return cyclic_failure ? E_SYS : E_OK; }
ER tk_loc_mtx(ID id,TMO t) { (void)t;assert(id==1 && !held);held=1;return E_OK; }
ER tk_unl_mtx(ID id) { assert(id==1 && held);held=0;return E_OK; }
ER tk_snd_mbf(ID id,const void *msg,INT size,TMO t) {
    assert(id==2 && size==(INT)sizeof(sonar_request_t) && t==TMO_POL);
    if (full) return E_TMOUT;
    queue[head++%8u]=*(const sonar_request_t *)msg;return E_OK;
}
INT tk_rcv_mbf(ID id,void *msg,TMO t) {
    assert(id==2 && t==TMO_POL);
    if (tail==head) return E_TMOUT;
    memcpy(msg,&queue[tail++%8u],sizeof(sonar_request_t));return (INT)sizeof(sonar_request_t);
}
ER tk_set_flg(ID id,UINT bits) { assert(id==3);flags|=bits;return E_OK; }
ER tk_clr_flg(ID id,UINT mask) { assert(id==3);flags&=mask;return E_OK; }
ER tk_wai_flg(ID id,UINT bits,UINT mode,UINT *out,TMO t) {
    assert(id==3 && mode==(TWF_ORW|TWF_BITCLR) && t==TMO_POL);
    if (!(flags&bits)) return E_TMOUT;
    *out=flags;flags&=~bits;return E_OK;
}
ID app_start_task(FP f,PRI p,UW core,const char *name) { (void)name;assert(p==12 && core==1u);entry=f;return 7; }
ER tk_wup_tsk(ID id) { assert(id==7 && !held);return E_OK; }
ER tk_slp_tsk(TMO t) { assert(t==TMO_FEVR);if (sleeps++) longjmp(end,1);return E_OK; }
INT tk_can_wup(ID id) { assert(id==TSK_SELF);return 0; }
uint32_t drv_time_ms(void) { return now; }
int drv_ultrasonic_init(void) { return E_OK; }
void drv_ultrasonic_listen(int id) { assert(id==7); }
bool drv_ultrasonic_service(void) { if (!done) return false;done=false;busy=false;return true; }
bool drv_ultrasonic_trigger(void) {
    if (busy || (triggers && now-last_trigger<60u)) return false;
    busy=!immediate;last_trigger=now;++triggers;return true;
}
bool drv_ultrasonic_busy(void) { return busy; }
ultrasonic_result_t drv_ultrasonic_latest(void) { return result; }
static void pump(uint32_t at) { now=at;sleeps=0u;if (setjmp(end)==0) ((void (*)(INT,void *))entry)(0,0);assert(!held); }
int main(int argc,char **argv) {
    (void)argv;cyclic_failure=argc>1;
    assert(!sonar_service_running() && sonar_service_init()==E_OK);
    if (cyclic_failure) {
        assert(sonar_service_start()==E_SYS && !sonar_service_running());
        const sonar_request_t rejected={.mode=SONAR_STOP};assert(sonar_request(&rejected)==E_OBJ);
        return 0;
    }
    assert(sonar_service_start()==E_OK && sonar_service_running());
    sonar_request_t r={.generation=1u,.id=1u,.mode=SONAR_ONCE,.deadline_ms=100u,.interval_ms=60u};
    assert(sonar_request(&r)==E_OK);pump(0u);assert(triggers==1u);
    r.id=2u;assert(sonar_request(&r)==E_OBJ); /* Explicit conflict, no silent overwrite. */
    sonar_snapshot_t s;bool fresh;
    assert(sonar_result_read(&s,&fresh)==E_OK && !fresh && s.pending && s.ping_sequence==1u && s.ping_generation==1u && s.ping_id==1u);
    done=true;pump(4u);assert(sonar_result_read(&s,&fresh)==E_OK && fresh && s.valid && s.id==1u);
    assert(sonar_result_read(&s,&fresh)==E_OK && !fresh);
    r.deadline_ms=200u;assert(sonar_request(&r)==E_OK);pump(4u);assert(triggers==1u);
    pump(60u);assert(triggers==2u);
    r.generation=2u;r.mode=SONAR_STOP;assert(sonar_request(&r)==E_OK);pump(61u);
    assert(sonar_result_read(&s,&fresh)==E_OK && fresh && s.status==SONAR_CANCELLED && !s.valid && s.ping_generation==1u && s.ping_id==2u);
    const uint32_t sequence=s.result_sequence;
    done=true;pump(62u);assert(sonar_result_read(&s,&fresh)==E_OK && !fresh && s.result_sequence==sequence);
    r.generation=1u;r.mode=SONAR_ONCE;assert(sonar_request(&r)==E_OBJ);
    r.generation=3u;r.id=4u;r.mode=SONAR_WATCH;assert(sonar_request(&r)==E_OK);pump(62u);
    full=true;assert(sonar_request(&r)==E_TMOUT);full=false;
    pump(122u);assert(triggers==3u);done=true;pump(124u);
    pump(200u);assert(triggers==3u); /* Don't ping over an unconsumed result. */
    r.mode=SONAR_ACK;assert(sonar_request(&r)==E_OK);pump(200u);assert(triggers==4u);
    r.generation=4u;r.id=5u;r.mode=SONAR_ONCE;r.deadline_ms=201u;
    assert(sonar_request(&r)==E_OK);pump(200u);pump(202u);
    assert(sonar_result_read(&s,&fresh)==E_OK && fresh && s.status==SONAR_DEADLINE && !s.valid);
    done=true;pump(203u);assert(sonar_result_read(&s,&fresh)==E_OK && !fresh);
    r.angle=31;assert(sonar_request(&r)==E_PAR);
    r.angle=0;r.generation=5u;r.id=6u;r.deadline_ms=400u;r.not_before_ms=300u;
    assert(sonar_request(&r)==E_OK);pump(240u);assert(triggers==4u);
    pump(300u);assert(triggers==5u);
    result=(ultrasonic_result_t)(ULTRASONIC_STATUS_NO_RETURN<<ULTRASONIC_STATUS_SHIFT);
    done=true;pump(304u);
    assert(sonar_result_read(&s,&fresh)==E_OK && fresh && !s.valid && s.status==SONAR_ECHO &&
        ultrasonic_result_status(s.echo)==ULTRASONIC_STATUS_NO_RETURN);
    immediate=true;result=(ultrasonic_result_t)(ULTRASONIC_STATUS_STUCK_HIGH<<ULTRASONIC_STATUS_SHIFT);
    r.generation=6u;r.id=7u;r.deadline_ms=500u;r.not_before_ms=0u;
    assert(sonar_request(&r)==E_OK);pump(400u);
    assert(sonar_result_read(&s,&fresh)==E_OK && fresh && !s.valid && !s.pending &&
        s.ping_generation==6u && s.ping_id==7u && s.ping_sequence==6u);
    immediate=false;result=100u;r.generation=7u;r.id=8u;r.mode=SONAR_WATCH;
    assert(sonar_request(&r)==E_OK);pump(400u);pump(460u);assert(triggers==7u);
    done=true;pump(800u); /* Delayed completion must keep its old observation time. */
    assert(sonar_result_read(&s,&fresh)==E_OK && fresh && !s.valid &&
        s.timestamp_ms==460u && s.deadline_ms==600u);
    r.generation=8u;r.id=9u;r.mode=SONAR_STOP;
    assert(sonar_request(&r)==E_OK);pump(801u); /* Leave cancellation notification unread. */
    r.generation=9u;r.id=10u;r.mode=SONAR_WATCH;
    assert(sonar_request(&r)==E_OK);pump(802u);
    assert(sonar_result_read(&s,&fresh)==E_OK && !fresh && s.status==SONAR_EMPTY && s.generation==9u && s.ping_generation==7u);
    return 0;
}

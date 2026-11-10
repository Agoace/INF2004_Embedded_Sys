/* Host scheduler: service-only inputs, calibration, stop, deadline and telemetry. */
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <string.h>
#include <tk/tkernel.h>
#include "../app_program/vehicle/vehicle_task.h"
#include "../app_program/subsystem2/motion.h"
#include "../app_program/subsystem3/line_service.h"
#include "../app_program/subsystem5/sonar_service.h"
#include "../app_program/subsystem1/telemetry.h"
#include "../app_program/subsystem4/imu_service.h"
#include "../app_program/vehicle/junction_source.h"
static bool junction_mode,junction_started;static int32_t junction_origin;
static unsigned int junction_results,spins;static ja_result_t junction_result;
static bool spin_started;static int32_t spin_origin[2];
static unsigned int junction_start_fail,snapshot_reads;
static FP entry;static jmp_buf done;static uint32_t now;
static unsigned int starts,notifications,estops,prints,line_drives,rows,requests,max_overruns;
static int line_error;static bool watchdog_ready,services_ready,calibrating,stale,overrun,sonar_stale;
bool line_service_running(void) { return services_ready; }
bool sonar_service_running(void) { return services_ready; }
bool telemetry_running(void) { return services_ready; }
static sonar_request_t sonar_command;
static sonar_snapshot_t echo_snapshot;
static bool echo_fresh,await_ack;
static uint32_t next_ping;
static unsigned int sonar_timeouts;
static int16_t asked[2];static int32_t counts[2];static ID listener;
static lp_t line;static vc_event_t line_event;static uint16_t raw[3],peak;
ID app_start_task(FP f,PRI p,UW core,const char *name) { (void)name;assert(p==13 && core==1u);entry=f;++starts;return 7; }
ID tk_get_tid(void) { return 7; }
void tk_ext_tsk(void) { assert(0); }
ER tk_slp_tsk(TMO t) {
    assert(t==20);now+=10u;counts[0]+=asked[0]/6;counts[1]+=asked[1]/6;
    for (unsigned int i=0u;i<10u;++i) {
        const uint32_t ms=now-9u+i;
        raw[0]=(ms/20u)%2u ? 500u : 3500u;raw[1]=(ms/30u)%2u ? 500u : 3500u;raw[2]=0u;
        if (junction_mode && (vehicle_run_epoch()&1u)) {
            const int32_t mean=(counts[0]+counts[1])/2;
            if (!junction_started) {junction_started=true;junction_origin=mean;}
            const int32_t travel=mean-junction_origin;
            raw[0]=raw[1]=travel>=300 && travel<524 ? 3500u : 1700u;
            if (spin_started && junction_results==0u) {
                const int32_t angle=((counts[1]-spin_origin[1])-(counts[0]-spin_origin[0]))*10000/2405;
                const int32_t target=JUNCTION_TEST_CMD==JA_UTURN ? 18000 :
                    (JUNCTION_TEST_CMD==JA_RIGHT ? -9000 : 9000);
                int32_t error=angle-target;if (error<0) error=-error;
                if (JUNCTION_TEST_CMD==JA_UTURN) {
                    int32_t old=angle-9000;if (old<0) old=-old;
                    if (old<error) error=old; /* Ignore intermediate '+' arm. */
                }
                if (error<200) raw[0]=raw[1]=1700u;
                else if (error<500) {raw[0]=2300u;raw[1]=1100u;}
                else raw[0]=raw[1]=500u; /* Real LOST during the encoder turn. */
            }
        }
        if (calibrating) lp_calib_add(&line,raw);
        else {
            const vc_event_t e=lp_update(&line,raw,ms);
            if (e!=VC_EVENT_NONE) line_event=e;
            if (line.norm[LP_CENTRE]>peak) peak=line.norm[LP_CENTRE];
        }
    }
    if (!sonar_stale && sonar_command.mode==SONAR_WATCH && !await_ack && (int32_t)(now-next_ping)>=0) {
        echo_snapshot.generation=sonar_command.generation;echo_snapshot.id=sonar_command.id;
        echo_snapshot.angle=sonar_command.angle;echo_snapshot.ping_generation=sonar_command.generation;
        echo_snapshot.ping_id=sonar_command.id;echo_snapshot.ping_angle=sonar_command.angle;
        ++echo_snapshot.ping_sequence;++echo_snapshot.result_sequence;
        echo_snapshot.trigger_ms=now;echo_snapshot.timestamp_ms=now;echo_snapshot.deadline_ms=now+(uint32_t)sonar_command.interval_ms*2u+20u;
        echo_snapshot.echo=(ultrasonic_result_t)(ULTRASONIC_STATUS_NO_RETURN<<ULTRASONIC_STATUS_SHIFT);
        echo_snapshot.status=SONAR_ECHO;echo_fresh=true;await_ack=true;
        next_ping=now+sonar_command.interval_ms;
    }
    if (now>=(junction_mode ? 16000u : 8000u)) longjmp(done,1);
    return E_OK;
}
INT tk_can_wup(ID id) { assert(id==TSK_SELF);return 0; }
/* Fresh IMU hump board of the current run: one closed 10.8 mm event. */
static unsigned int hump_reads,hump_lines,hump_none;static uint32_t run_seen;
int imu_hump_try_read(imu_hump_report_t *r) {
    const uint32_t e=vehicle_run_epoch();
    ++hump_reads;if (e&1u) run_seen=e;
    *r=(imu_hump_report_t){.timestamp_ms=stale ? 0u : now};
    r->hump=(imu_hump_output_t){.run_epoch=e,.running=(e&1u)!=0u,.valid_samples=1u,
        .count=1u,.max_peak_mm=10.8f,.last_peak_mm=10.8f,.flags=IMU_HUMP_FLAG_IMPLEMENTED};
    return E_OK;
}
static unsigned int diag_lines;
int imu_hump_diag_try_read(imu_hump_diag_report_t *r) {
    *r=(imu_hump_diag_report_t){.run_epoch=vehicle_run_epoch(),
        .timestamp_ms=stale ? 0u : now,.valid=true};
    r->peak=(imu_hump_peak_diag_t){.valid=true,.corrected_mm=10.8f,
        .filtered_mm=9.9f,.ms=100u};
    return E_OK;
}
int tm_printf(const UB *f,...) {
    va_list a;va_start(a,f);
    if (!strncmp((const char *)f,"[robot] hump highest_mm",23u)) {
        const unsigned int mm=va_arg(a,unsigned int),tenth=va_arg(a,unsigned int);
        assert(mm==10u && tenth==8u);++hump_lines;
        (void)va_arg(a,const char *);
        for (unsigned int i=0u;i<3u;++i) (void)va_arg(a,unsigned int);
        assert(va_arg(a,unsigned int)==IMU_HUMP_SOURCE);
        assert(!strcmp(va_arg(a,const char *),IMU_HUMP_SOURCE_NAME));
    } else if (!strncmp((const char *)f,"[robot] hump unavailable",24u)) ++hump_none;
    else if (!strncmp((const char *)f,"[S4_HUMP_DIAG]",14u)) ++diag_lines;
    else ++prints;
    va_end(a);return 0;
}
uint32_t drv_time_ms(void) { return now; }
uint32_t drv_time_us(void) { return now*1000u; }
int line_snapshot_read(line_snapshot_t *s,bool consume) {
    if (line_error) return line_error;
    *s=(line_snapshot_t){.line=line,.event=line_event,.valid=true,.timestamp_us=now*1000u,.norm_peak=peak,.raw_bc=4095u};
    for (unsigned int i=0u;i<3u;++i) s->raw[i]=raw[i];
    if (stale && now>=3000u) s->timestamp_us-=11000u;
    if (consume) line_event=VC_EVENT_NONE;
    return E_OK;
}
int line_calibration_begin(void)
{
    assert((vehicle_calibration_epoch() & 1u) != 0u);
    lp_calib_begin(&line);
    calibrating = true;
    return E_OK;
}
lp_calib_result_t line_calibration_end(void)
{
    unsigned int bad;

    assert((vehicle_calibration_epoch() & 1u) != 0u);
    calibrating = false;
    peak = 0u;
    return lp_calib_end(&line, &bad);
}
int line_calibration_abort(void)
{
    assert((vehicle_calibration_epoch() & 1u) != 0u);
    calibrating = false;
    return E_OK;
}
int sonar_result_read(sonar_snapshot_t *s,bool *fresh) { *s=echo_snapshot;*fresh=echo_fresh;echo_fresh=false;return E_OK; }
int sonar_request(const sonar_request_t *r) {
    assert(r->mode<=SONAR_ACK);++requests;
    if (r->mode==SONAR_ACK) await_ack=false;
    else { sonar_command=*r;await_ack=false;next_ping=r->not_before_ms; }
    return E_OK;
}
void telemetry_stats(telemetry_stats_t *s) { *s=(telemetry_stats_t){0}; }
int telemetry_send(const telemetry_message_t *m) {
    assert(m->kind<=TELEMETRY_JUNCTION);
    if (m->kind==TELEMETRY_JUNCTION) {
        assert(!junction_source_busy());++junction_results;junction_result=m->data.junction;
    }
    if (m->kind==TELEMETRY_ROW) ++rows;
    if (m->kind==TELEMETRY_INFO && m->data.info.overruns>max_overruns) max_overruns=m->data.info.overruns;
    if (m->kind==TELEMETRY_INFO && !strcmp(m->data.info.reason,"sonar watch stale")) ++sonar_timeouts;
    return E_OK;
}
void drv_motor_hw_buttons_init(void) { }
bool drv_motor_hw_button_down(unsigned int pin) {
    return pin==21u && ((now>=20u && now<50u) || (!sonar_stale && now>=(junction_mode ? 12000u : 5000u) && now<(junction_mode ? 12050u : 5050u)));
}
int motion_notify_after_tick(ID id) { listener=id;++notifications;return E_OK; }
int motion_status(motion_status_t *s) {
    *s=(motion_status_t){0};s->watchdog_armed=watchdog_ready;s->ticks=now/10u;
    s->counts[0]=counts[0];s->counts[1]=counts[1];return E_OK;
}
int motion_snapshot_try_read(motion_status_t *s,uint32_t *stamp) {
    ++snapshot_reads;if (junction_start_fail==1u) return E_TMOUT;
    *stamp=now*1000u-(junction_start_fail==2u ? 11000u : 0u);return motion_status(s);
}
int motion_clear_faults(void) { return E_OK; }
int motion_set_speed(int16_t l,int16_t r) {
    if (l>0 && r>0) ++line_drives;
    if (junction_mode && (vehicle_run_epoch()&1u) && l*r<0) {
        assert(vehicle_line_run_epoch()==0u && junction_source_busy());++spins;
        if (!spin_started) {
            spin_started=true;spin_origin[0]=counts[0];spin_origin[1]=counts[1];
        }
    }
    asked[0]=l;asked[1]=r;
    if (overrun && (l!=0 || r!=0)) now+=11u;
    return E_OK;
}
int motion_stop(void) { asked[0]=asked[1]=0;return E_OK; }
int motion_estop(void) { ++estops;return motion_stop(); }
int motion_set_servo(int deg,int *accepted) { (void)deg;(void)accepted;return E_OK; }
int main(int argc,char **argv) {
    junction_mode=argc>1 && !strcmp(argv[1],"junction");
    junction_start_fail=argc>1 && !strcmp(argv[1],"junctionbusy") ? 1u :
        (argc>1 && !strcmp(argv[1],"junctionstale") ? 2u : 0u);
    stale=argc>1 && !strcmp(argv[1],"stale");overrun=argc>1 && !strcmp(argv[1],"overrun");
    sonar_stale=argc>1 && !strcmp(argv[1],"sonarstale");
    lp_init(&line);lp_set_pair(&line,true,0);
    assert(vehicle_calibration_epoch() == 0u && vehicle_run_epoch() == 0u);
    imu_hump_summary_t last;assert(vehicle_hump_try_read(&last)==E_NOEXS);
    assert(vehicle_hump_try_read(NULL)==E_PAR);
    imu_hump_diag_report_t diag;
    assert(vehicle_hump_diag_try_read(NULL)==E_PAR);
    assert(vehicle_hump_diag_try_read(&diag)==E_NOEXS);
    assert(vehicle_run_start_ms()==0u && vehicle_line_run_epoch()==0u);
    assert(vehicle_task_start()==E_OBJ && starts==0u);watchdog_ready=true;
    assert(vehicle_task_start()==E_OBJ && starts==0u);services_ready=true;
    line_error=E_SYS;assert(vehicle_task_start()==E_SYS && starts==0u);
    line_error=E_OK;assert(vehicle_task_start()==E_OK && starts==1u);
    if (setjmp(done)==0) ((void (*)(INT,void *))entry)(0,0);
    assert(listener==7 && notifications==1u && prints==1u && asked[0]==0 && asked[1]==0);
    assert(vehicle_calibration_epoch() >= 2u);
    assert((vehicle_calibration_epoch() & 1u) == 0u);
    if (junction_start_fail) {
        assert(snapshot_reads>0u && vehicle_run_epoch()==0u && run_seen==0u);
        assert(junction_results==0u && !junction_source_busy());return 0;
    }
    /* The run epoch is odd only inside a run; each finished run prints one
     * hump line from a fresh same-run board, else "unavailable". */
    if (!overrun && !sonar_stale) {
        assert(run_seen!=0u && (vehicle_run_epoch()&1u)==0u && hump_reads>0u);
        assert(vehicle_hump_try_read(&last)==E_OK);
        assert(vehicle_hump_diag_try_read(&diag)==E_OK && diag_lines>=1u);
        assert(vehicle_line_run_epoch()==0u);
        assert(diag.valid==!stale && diag.peak.valid==!stale);
        if (!stale) assert(diag.peak.corrected_mm==10.8f && diag.peak.ms==100u);
        if (stale) assert(hump_none>=1u && last.peak_01mm==IMU_HUMP_PEAK_NONE);
        else assert(hump_lines==1u && hump_none==0u && last.peak_01mm==108u &&
                    last.count==1u && last.flags==IMU_HUMP_FLAG_IMPLEMENTED);
    }
    if (junction_mode) {
        assert(junction_started && junction_results==1u);
        assert(junction_result.command==JUNCTION_TEST_CMD && junction_result.source==1u);
        assert(junction_result.phase==JA_DONE && junction_result.claimed==1u);
        assert(junction_source_acknowledged()==junction_result.sequence);
        assert(JUNCTION_TEST_CMD==JA_STRAIGHT ? spins==0u : spins>0u);
    } else assert(junction_results==0u);
    if (overrun) assert(max_overruns>0u);
    else if (stale) assert(estops>0u);
    else if (sonar_stale) assert(sonar_timeouts>0u);
    else assert(estops==1u && line_drives>0u && rows>0u && requests>0u);
    return 0;
}

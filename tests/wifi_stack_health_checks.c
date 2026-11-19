/* M9: run the real Health refresh once against the real stack registry. */
#if TM_MQTT
#include <assert.h>
#include <setjmp.h>
#include <string.h>
#include "../app_program/app.h"
#include "../app_program/platform/app_task.h"
#include "../app_program/platform/drv_watchdog.h"
#include "../app_program/subsystem1/robot_health.h"
#include "../app_program/subsystem1/robot_network.h"
#include "../app_program/subsystem1/telemetry.h"
#include "../app_program/platform/run_timing.h"
#include "../app_program/platform/fault_mask.h"
#include "../app_program/subsystem2/motion.h"

static uint32_t owner_values[5];
static bool owners_readable = true;
int motion_snapshot_try_read(motion_status_t *out, uint32_t *us)
{
    if (!owners_readable) return E_TMOUT;
    memset(out,0,sizeof(*out));out->faults=(uint8_t)owner_values[0];*us=1234000u;
    return E_OK;
}
static bool owner_get(unsigned int n,uint32_t *out)
{
    if (!owners_readable) return false;
    *out=owner_values[n];return true;
}
bool vehicle_fault_mask_try_read(uint32_t *out) { return owner_get(1u,out); }
bool sonar_fault_mask_try_read(uint32_t *out) { return owner_get(2u,out); }
bool telemetry_fault_mask_try_read(uint32_t *out) { return owner_get(3u,out); }
bool robot_network_fault_mask_try_read(uint32_t *out) { return owner_get(4u,out); }

static FP health_entry;
static jmp_buf refreshed;
ID app_start_task(FP entry, PRI priority, UW core, const char *name)
{
    assert(priority == 31 && core == TP_PRC1);
    assert(strcmp(name, "health_task") == 0);
    health_entry = entry;
    return app_task_start(entry, priority, core, name);
}
uint32_t drv_time_us(void) { return 1234000u; }
uint32_t drv_time_ms(void) { return 1234u; }
drv_watchdog_reset_t drv_watchdog_last_reset(void)
{
    return DRV_RESET_POWER_RUN;
}
void idle_meter_sample_all(void) { }
void telemetry_stats(telemetry_stats_t *out) { memset(out, 0, sizeof(*out)); }
void robot_network_metrics(rm_heartbeat_t *out) { (void)out; }
ID tk_cre_cyc(const T_CCYC *config)
{
    assert(config->cyctim == 1000u);
    return 99;
}
ER tk_sta_cyc(ID id) { assert(id == 99); return E_OK; }
ER tk_wup_tsk(ID id) { (void)id; return E_OK; }
INT tk_can_wup(ID id) { (void)id; return 0; }
ER tk_slp_tsk(TMO timeout)
{
    assert(timeout == TMO_FEVR);
    longjmp(refreshed, 1);
}
void wifi_stack_health_check(uint32_t expected_free)
{
    assert(robot_health_init() == E_OK);
    assert(robot_health_start() == E_OK && health_entry);
    rt_begin(1u); /* P1: a run is active during this refresh */
    if (setjmp(refreshed) == 0) {
        ((void (*)(INT, void *))health_entry)(0, NULL);
        assert(0);
    }
    rm_heartbeat_t heartbeat;
    uint32_t at;
    assert(robot_health_try_read(&heartbeat, &at) == E_OK);
    assert(at == 1234000u && heartbeat.stack_min_free == expected_free);
    rt_snapshot_t rt;
    rt_snapshot(&rt);
    assert(rt.load_epoch == 1u && rt.load_samples == 1u);
    assert(rt.load_peak[0] == 100u);
#if TK_SUPPORT_SMP
    assert(rt.load_peak[1] == 200u);
#else
    assert(rt.load_peak[1] == 65535u);
#endif
    fault_mask_snapshot_t faults;
    assert(robot_health_faults_try_read(&faults,&at)==E_OK);
    assert(faults.now==0u && faults.run==0u && faults.valid==FAULT_MASK_ALL);
    const uint32_t stop_causes[]={FAULT_MOTION_STALL_LEFT,FAULT_IMU_SAFETY,FAULT_SONAR_SENSOR};
    for (unsigned int n=0u;n<3u;++n) {
        const uint32_t epoch=3u+2u*n;
        rt_begin(epoch);
        memset(owner_values,0,sizeof(owner_values));
        owner_values[n]=stop_causes[n];
        robot_health_faults_stop(epoch);
        robot_health_faults_stop(epoch+1u); /* Invalid even request cannot erase capture. */
        rt_begin(epoch+1u);
        fault_mask_snapshot_t unchanged=faults;
        const uint32_t unchanged_at=at;
        assert(robot_health_faults_try_read(&faults,&at)==E_TMOUT);
        assert(memcmp(&faults,&unchanged,sizeof(faults))==0 && at==unchanged_at);
        memset(owner_values,0,sizeof(owner_values)); /* cleared before Health runs */
        if (setjmp(refreshed)==0) ((void (*)(INT,void *))health_entry)(0,NULL);
        assert(robot_health_faults_try_read(&faults,&at)==E_OK);
        assert(faults.epoch==epoch && faults.run==stop_causes[n]);
        assert(faults.now==0u && faults.valid==FAULT_MASK_ALL);
        owner_values[4]=FAULT_WIFI_LINK_DOWN; /* after STOP: now only */
        if (setjmp(refreshed)==0) ((void (*)(INT,void *))health_entry)(0,NULL);
        assert(robot_health_faults_try_read(&faults,&at)==E_OK);
        assert(faults.run==stop_causes[n] && faults.now==FAULT_WIFI_LINK_DOWN);
    }
    rt_begin(9u); /* new START drops frozen old-run bits */
    assert(robot_health_faults_try_read(&faults,&at)==E_TMOUT);
    memset(owner_values,0,sizeof(owner_values));
    if (setjmp(refreshed)==0) ((void (*)(INT,void *))health_entry)(0,NULL);
    assert(robot_health_faults_try_read(&faults,&at)==E_OK && faults.run==0u && faults.epoch==9u);
    owners_readable=false;
    if (setjmp(refreshed)==0) ((void (*)(INT,void *))health_entry)(0,NULL);
    assert(robot_health_faults_try_read(&faults,&at)==E_OK && faults.valid==0u);
    rt_begin(10u);
    /* CR1 F pack: a STOP owner word with a bit outside its owner mask cannot
     * be stored in the packed capture. It is not lost silently: the owner is
     * stored as unknown (valid clear, not merged into run) and counted. The
     * other owners stay exact. */
    owners_readable=true;
    assert(robot_health_faults_stop_refused()==0u);
    rt_begin(11u);
    memset(owner_values,0,sizeof(owner_values));
    owner_values[0]=FAULT_MOTION_ESTOP;
    owner_values[1]=FAULT_IMU_SAFETY|FAULT_SONAR_SENSOR; /* wrong-owner bit 5 */
    owner_values[4]=FAULT_MQTT_DISCONNECTED;
    robot_health_faults_stop(11u);
    assert(robot_health_faults_stop_refused()==1u);
    rt_begin(12u);
    memset(owner_values,0,sizeof(owner_values));
    if (setjmp(refreshed)==0) ((void (*)(INT,void *))health_entry)(0,NULL);
    assert(robot_health_faults_try_read(&faults,&at)==E_OK && faults.epoch==11u);
    assert(faults.run==(FAULT_MOTION_ESTOP|FAULT_MQTT_DISCONNECTED));
    assert(faults.now==0u && faults.valid==FAULT_MASK_ALL);
    /* Motion bits above its 4-bit field are refused the same way. */
    rt_begin(13u);
    memset(owner_values,0,sizeof(owner_values));
    owner_values[0]=0x10u|FAULT_MOTION_STALL_RIGHT;
    owner_values[3]=FAULT_LOG_FULL;
    robot_health_faults_stop(13u);
    assert(robot_health_faults_stop_refused()==2u);
    rt_begin(14u);
    memset(owner_values,0,sizeof(owner_values));
    if (setjmp(refreshed)==0) ((void (*)(INT,void *))health_entry)(0,NULL);
    assert(robot_health_faults_try_read(&faults,&at)==E_OK && faults.epoch==13u);
    assert(faults.run==FAULT_LOG_FULL);
    rt_begin(15u);
}
#endif

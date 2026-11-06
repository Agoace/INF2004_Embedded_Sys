/* Host smoke scheduler: button ownership, calibration/start/estop, no run printing. */
#include <assert.h>
#include <setjmp.h>
#include <tk/tkernel.h>
#include "../app_program/vehicle/vehicle_task.h"
#include "../app_program/subsystem2/motion.h"
#include "../app_program/subsystem3/drv_line.h"
#include "../app_program/subsystem5/drv_ultrasonic.h"
static FP entry;
static jmp_buf done;
static uint32_t now;
static unsigned int samples, starts, notifications, estops, prints, line_drives;
static int line_error;
static int16_t asked[2];
static int32_t counts[2];
static ID listener;
ID app_start_task(FP fn,PRI priority,UW core,const char *name) {
    (void)name; assert(priority==12 && core==TP_PRC1); entry=fn; ++starts; return 7;
}
ID tk_get_tid(void) { return 7; }
void tk_ext_tsk(void) { assert(0); }
ER tk_slp_tsk(TMO timeout) {
    assert(timeout==20); now+=10u; samples=10u;
    counts[0]+=asked[0]/6; counts[1]+=asked[1]/6;
    if (now>=8000u) longjmp(done,1);
    return E_OK;
}
INT tk_can_wup(ID id) { assert(id==TSK_SELF); return 0; }
ER tk_dly_tsk(RELTIM delay) { (void)delay; assert(0); return E_OK; }
int tm_printf(const UB *format,...) { (void)format; ++prints; return 0; }
uint32_t drv_time_ms(void) { return now; }
uint32_t drv_time_us(void) { return now*1000u; }
int drv_line_init(void) { return line_error; }
void drv_line_stats(drv_line_stats_t *s) { *s=(drv_line_stats_t){0}; }
void drv_line_flush(void) { samples=0u; }
bool drv_line_take(drv_line_sample_t *s) {
    if (!samples) return false;
    --samples; s->us=(now- samples)*1000u;
    s->raw[0]=(now/20u)%2u ? 500u : 3500u;
    s->raw[1]=(now/30u)%2u ? 500u : 3500u; s->raw[2]=0u; return true;
}
int drv_ultrasonic_init(void) { return E_OK; }
bool drv_ultrasonic_service(void) { return false; }
bool drv_ultrasonic_busy(void) { return false; }
bool drv_ultrasonic_trigger(void) { return true; }
ultrasonic_result_t drv_ultrasonic_latest(void) { return (uint16_t)(ULTRASONIC_STATUS_NO_RETURN<<9u); }
void drv_motor_hw_buttons_init(void) { }
bool drv_motor_hw_button_down(unsigned int pin) {
    return pin==21u && ((now>=20u && now<50u) || (now>=5000u && now<5050u));
}
int motion_notify_after_tick(ID id) { listener=id; ++notifications; return E_OK; }
int motion_status(motion_status_t *s) {
    *s=(motion_status_t){0}; s->counts[0]=counts[0]; s->counts[1]=counts[1]; return E_OK;
}
int motion_clear_faults(void) { return E_OK; }
int motion_set_speed(int16_t l,int16_t r) {
    if (l>0 && r>0) ++line_drives;
    asked[0]=l; asked[1]=r; return E_OK;
}
int motion_stop(void) { asked[0]=asked[1]=0; return E_OK; }
int motion_estop(void) { ++estops; return motion_stop(); }
int motion_set_servo(int deg,int *accepted) { (void)deg; (void)accepted; return E_OK; }
int main(void) {
    line_error=E_SYS; assert(vehicle_task_start()==E_SYS && starts==0u);
    line_error=E_OK; assert(vehicle_task_start()==E_OK && starts==1u);
    if (setjmp(done)==0) ((void (*)(INT,void *))entry)(0,0);
    assert(listener==7 && notifications==1u && prints==1u && estops==1u);
    assert(asked[0]==0 && asked[1]==0 && line_drives>0u);
    return 0;
}

/* SIL-only ABI; include robot settings, do not retype/tune them. */
#include <stdint.h>
#include "app_program/subsystem3/line_position.h"
#include "app_program/subsystem3/line_steer.h"
#include "app_program/vehicle/vehicle_robot_config.h"
#include "app_program/subsystem2/motor_ctl_robot.h"
static lp_t lp;
static steer_mm_t steer;
static mc_t mc;
static mc_output_t output;
static int fault;
static int16_t desired[2];
void sil_reset(void) {
    const int32_t counts[2]={0,0};
    const mc_config_t config=MC_ROBOT_CONFIG;
    (void)SEARCH_CFG;(void)SCAN_CFG;(void)DETOUR_CFG;
    lp_init(&lp);lp_set_pair(&lp,true,0);steer_mm_init(&steer,&LINE_CFG.steer);
    mc_init(&mc,&config,counts,0);output=(mc_output_t){{0,0},1500};fault=0;desired[0]=0;desired[1]=0;
}
void sil_calib_begin(void) { lp_calib_begin(&lp); }
void sil_calib_add(uint16_t left,uint16_t centre) {
    const uint16_t raw[3]={left,centre,0};lp_calib_add(&lp,raw);
}
void sil_calib_end(int32_t *out) {
    unsigned int bad=0;out[0]=(int32_t)lp_calib_end(&lp,&bad);out[1]=(int32_t)bad;
    out[2]=lp_line_reads_high(&lp);out[3]=lp_floor(&lp,0);out[4]=lp_line(&lp,0);
    out[5]=lp_floor(&lp,1);out[6]=lp_line(&lp,1);
}
void sil_motor_meta(int32_t *v) {
    v[0]=LINE_CFG.steer.base_mm_s;v[1]=LINE_CFG.steer.gain_pct;v[2]=LINE_CFG.steer.kd_pct;
    v[3]=LINE_CFG.steer.min_mm_s;v[4]=LINE_CFG.steer.max_mm_s;v[5]=LINE_CFG.steer.far_from;
    v[6]=LINE_CFG.steer.far_gain_pct;v[7]=mc.cfg.accel_cpt_q8;v[8]=mc.cfg.max_speed_mm_s;
    v[9]=mc.cfg.tick_ms;
}
void sil_cal_spin_config(int32_t *out) {
    out[0]=CAL_SPEED_MM_S;out[1]=CAL_STEP_COUNTS;out[2]=CAL_STEP_TIMEOUT_MS;
    out[3]=CAL_PAUSE_MS;out[4]=CAL_CENTRE_STOP;out[5]=CAL_CENTRE_MIN;out[6]=CAL_PEAK_DROP;
}
void sil_cal_motor_stop(void) {
    mc_clear_faults(&mc);output.duty_permille[0]=0;output.duty_permille[1]=0;
}
void sil_cal_observe(uint16_t left,uint16_t centre,uint32_t ms,int32_t *out) {
    const uint16_t raw[3]={left,centre,0};
    (void)lp_update(&lp,raw,ms);out[0]=lp.norm[LP_CENTRE];
}
void sil_motor_tick(uint32_t ms,int32_t left_count,int32_t right_count,int16_t left_mm_s,int16_t right_mm_s,int32_t *out) {
    const int32_t counts[2]={left_count,right_count};
    /* motion notifies vehicle/bench after its tick: new command applies next tick. */
    mc_tick(&mc,counts,ms,&output);mc_set_speed(&mc,left_mm_s,right_mm_s,ms);
    out[0]=output.duty_permille[0];out[1]=output.duty_permille[1];out[2]=mc_faults(&mc);
}
void sil_motor_update(uint16_t left,uint16_t centre,uint32_t ms,int32_t left_count,int32_t right_count,int32_t *out) {
    const uint16_t raw[3]={left,centre,0};const int32_t counts[2]={left_count,right_count};
    const vc_event_t event=lp_update(&lp,raw,ms);
    if (!lp_line_seen(&lp)) fault=1;
    if (ms%10u==0u) {
        /* Robot ordering: motor tick, then vehicle wake/steering command. */
        if (fault) mc_stop(&mc);
        mc_tick(&mc,counts,ms,&output);
        const steer_mm_word_t word=steer_mm_update(&steer,lp.position);
        desired[0]=steer_mm_left(word);desired[1]=steer_mm_right(word);
        if (fault) mc_stop(&mc);
        else mc_set_speed(&mc,desired[0],desired[1],ms);
        if (mc_faults(&mc)!=0u) fault=1;
    }
    out[0]=lp.position;out[1]=(int32_t)event;out[2]=lp_line_seen(&lp);out[3]=lp_junction(&lp);
    out[4]=fault?0:output.duty_permille[0];out[5]=fault?0:output.duty_permille[1];
    out[6]=lp.norm[0];out[7]=lp.norm[1];out[8]=fault;
    out[9]=desired[0];out[10]=desired[1];out[11]=mc_faults(&mc);
}

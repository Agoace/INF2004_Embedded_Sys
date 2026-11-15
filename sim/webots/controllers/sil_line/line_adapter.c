/* Simulation-owned ABI; robot sources are compiled unchanged. */
#include <stdint.h>
#include "app_program/subsystem3/line_position.h"
#include "app_program/subsystem3/line_steer.h"
static lp_t lp;
static steer_t steer;
static uint32_t last_steer;
static steer_word_t duty;
static int have_steer;
static int fault;
void sil_reset(void) {
    const steer_config_t config = {20, 100, 100, 8, 40, 0, 50, 0};
    lp_init(&lp); lp_set_pair(&lp, true, 0);
    steer_init(&steer, &config); duty=0;have_steer=0;last_steer=0;fault=0;
}
void sil_calib_begin(void) { lp_calib_begin(&lp); }
void sil_calib_add(uint16_t left, uint16_t centre) {
    const uint16_t raw[3]={left,centre,0}; lp_calib_add(&lp,raw);
}
void sil_calib_end(int32_t *out) {
    unsigned int bad=0;
    out[0]=(int32_t)lp_calib_end(&lp,&bad);out[1]=(int32_t)bad;
    out[2]=lp_line_reads_high(&lp);out[3]=lp_floor(&lp,0);out[4]=lp_line(&lp,0);
    out[5]=lp_floor(&lp,1);out[6]=lp_line(&lp,1);
}
void sil_update(uint16_t left, uint16_t centre, uint32_t ms, int32_t *out) {
    const uint16_t raw[3]={left,centre,0};
    const vc_event_t event=lp_update(&lp,raw,ms);
    if (!lp_line_seen(&lp)) fault=1;
    if (!have_steer || ms-last_steer>=10u) {
        duty=steer_update(&steer,lp.position);last_steer=ms;have_steer=1;
    }
    out[0]=lp.position;out[1]=(int32_t)event;out[2]=lp_line_seen(&lp);
    out[3]=lp_junction(&lp);out[4]=fault?0:(duty&STEER_DUTY_MASK);
    out[5]=fault?0:((duty>>STEER_RIGHT_SHIFT)&STEER_DUTY_MASK);
    out[6]=lp.norm[0];out[7]=lp.norm[1];out[8]=fault;
}

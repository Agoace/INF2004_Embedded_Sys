/* Host-only analysis ABI; unchanged native controller and motor code. */
#include "motor_adapter.c"
void metrics_steer(int8_t previous,int8_t position,int32_t *out) {
    steer_mm_init(&steer,&LINE_CFG.steer);
    (void)steer_mm_update(&steer,previous);
    const steer_mm_word_t w=steer_mm_update(&steer,position);
    out[0]=steer_mm_left(w);out[1]=steer_mm_right(w);
}
void metrics_motor(uint32_t ms,int32_t left,int32_t right,int16_t ask_left,int16_t ask_right,int32_t *out) {
    sil_motor_tick(ms,left,right,ask_left,ask_right,out);
    out[3]=mc_speed_mm_s(&mc,MC_LEFT);out[4]=mc_speed_mm_s(&mc,MC_RIGHT);
}

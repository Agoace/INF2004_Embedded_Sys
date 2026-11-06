#include "vehicle_log.h"
static void put(vr_row_t *r, unsigned int bit, unsigned int width, uint32_t value) {
    for (unsigned int i = 0u; i < width; ++i)
        r->bytes[(bit+i)/8u] |= (uint8_t)(((value >> i) & 1u) << ((bit+i)%8u));
}
static uint32_t speed(int16_t value, uint8_t *flags) {
    if (value < -512) { value = -512; *flags |= 1u; }
    if (value > 511) { value = 511; *flags |= 1u; }
    return (uint32_t)(uint16_t)value & 1023u;
}
void vr_pack(vr_row_t *r, const vr_values_t *v) {
    for (unsigned int i = 0u; i < VR_BYTES; ++i) r->bytes[i] = 0u;
    uint8_t flags = 0u;
    put(r,VR_TIME,16u,v->elapsed_ms/20u);
    put(r,VR_STATE,2u,v->state); put(r,VR_MODULE,3u,v->module);
    put(r,VR_L,12u,v->raw[0]); put(r,VR_C,12u,v->raw[1]);
    put(r,VR_POS,8u,(uint8_t)v->position);
    put(r,VR_ASK_L,10u,speed(v->asked[0],&flags)); put(r,VR_ASK_R,10u,speed(v->asked[1],&flags));
    put(r,VR_MEAS_L,10u,speed(v->measured[0],&flags)); put(r,VR_MEAS_R,10u,speed(v->measured[1],&flags));
    put(r,VR_ECHO,16u,v->echo); put(r,VR_SERVO,7u,(uint8_t)v->servo & 127u);
    put(r,VR_EVENTS,8u,v->events); put(r,VR_FLAGS,4u,flags);
}
_Static_assert(sizeof(vr_row_t) == 16u, "fixed row size");

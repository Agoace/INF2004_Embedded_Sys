#include <assert.h>
#include <stdio.h>
#include "../app_program/vehicle/vehicle_log.h"
static uint32_t get(const vr_row_t *r,unsigned int bit,unsigned int width) {
    uint32_t value=0u;
    for (unsigned int i=0u;i<width;++i) value|=(uint32_t)((r->bytes[(bit+i)/8u]>>((bit+i)%8u))&1u)<<i;
    return value;
}
int main(void) {
    vr_row_t row;
    const vr_values_t v={.elapsed_ms=89980u,.raw={4095u,1234u},.echo=400u,
        .asked={-378,378},.measured={-511,511},.position=-100,.servo=-36,.state=2u,.module=4u,.events=0xa5u};
    vr_pack(&row,&v);
    assert(sizeof row==16u && get(&row,VR_TIME,16u)==4499u);
    assert(get(&row,VR_L,12u)==4095u && get(&row,VR_C,12u)==1234u);
    assert(get(&row,VR_ASK_L,10u)==646u && get(&row,VR_ASK_R,10u)==378u);
    assert(get(&row,VR_MEAS_L,10u)==513u && get(&row,VR_MEAS_R,10u)==511u);
    assert(get(&row,VR_ECHO,16u)==400u && get(&row,VR_FLAGS,4u)==0u);
    for (unsigned int i=0u;i<VR_BYTES;++i) printf("%02x",(unsigned int)row.bytes[i]);
    puts("");
    vr_values_t limit=v; limit.asked[0]=-1000; limit.measured[1]=1000;
    vr_pack(&row,&limit);
    assert(get(&row,VR_ASK_L,10u)==512u && get(&row,VR_MEAS_R,10u)==511u && get(&row,VR_FLAGS,4u)==1u);
    return 0;
}

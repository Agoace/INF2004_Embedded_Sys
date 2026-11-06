#include "subsystem5.h"
#include "sonar_service.h"
#include "../platform/drv_time.h"
#if APP_TEST
EXPORT void subsystem5_test(void) {
    uint16_t mm[20];unsigned int valid=0u,no_echo=0u;
    tm_printf((UB*)"[TEST] S5-U01: fixed object 60..511 mm, compare with a ruler.\n");
    for (unsigned int i=0u;i<20u;++i) {
        (void)tk_dly_tsk(100u);
        const uint32_t start=drv_time_ms();
        const sonar_request_t r={.generation=1000u+i,.id=i+1u,.mode=SONAR_ONCE,.deadline_ms=start+100u,.interval_ms=60u,.angle=0};
        if (sonar_request(&r)<E_OK) { ++no_echo;continue; }
        sonar_snapshot_t s={0};bool fresh=false;
        while (drv_time_ms()-start<150u) {
            (void)sonar_result_read(&s,&fresh);
            if (fresh && s.generation==r.generation && s.id==r.id) break;
            (void)tk_dly_tsk(1u); /* TEST deadline wait, not a periodic release. */
        }
        if (!fresh || s.generation!=r.generation || s.id!=r.id || !s.valid ||
            ultrasonic_result_too_near(s.echo)) ++no_echo;
        else mm[valid++]=ultrasonic_result_distance(s.echo);
    }
    for (unsigned int i=1u;i<valid;++i) {
        const uint16_t x=mm[i];unsigned int j=i;
        while (j>0u && mm[j-1u]>x) { mm[j]=mm[j-1u];--j; }
        mm[j]=x;
    }
    tm_printf((UB*)"[TEST] S5-U01 20 pings: valid %u unknown %u median %u min %u max %u mm; manual check required\n",
        valid,no_echo,valid ? (unsigned int)mm[valid/2u] : 0u,
        valid ? (unsigned int)mm[0] : 0u,valid ? (unsigned int)mm[valid-1u] : 0u);
    const sonar_request_t stale={.generation=1u,.id=1u,.mode=SONAR_ONCE,.deadline_ms=drv_time_ms()+100u};
    tm_printf((UB*)"[TEST] S5-U02 %s: reject old-generation request\n",sonar_request(&stale)==E_OBJ ? "PASS" : "FAIL");
}
#endif

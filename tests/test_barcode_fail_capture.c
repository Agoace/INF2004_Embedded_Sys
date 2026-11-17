#define PROHIBIT_DEF_SIZE_T
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <tk/tkernel.h>
#include "../app_program/subsystem3/barcode_fail_capture.h"
static char text[150000];
static size_t used;
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-format-attribute"
#pragma clang diagnostic ignored "-Wformat-nonliteral"
#endif
int tm_printf(const UB *fmt, ...) {
    va_list args; va_start(args,fmt);
    int n=vsnprintf(text+used,sizeof(text)-used,(const char *)fmt,args);
    va_end(args); assert(n>=0 && (size_t)n<sizeof(text)-used);used+=(size_t)n;return n;
}
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
#include "../app_program/subsystem3/barcode_fail_capture.inc"
static void attempt(uint32_t index, uint32_t reason, unsigned int samples) {
    barcode_window_t w={.phase=1u,.rise=320u,.fall=245u,.leading_mm=60.0f};
    barcode_window_motor_t motor={.us=10000u,.left=100u,.right=101u};
    barcode_window_adc_t a={.us=index*10000000u,.raw=170u};
    for (unsigned int i=0u;i<80u;++i) {
        a.us+=4000u;bcfc_observe(&w,a,(float)i,&motor);
    }
    a.us+=1000u;a.raw=600u;w.phase=2u;w.start_us=a.us;w.black=true;
    bcfc_start(&w,a,80.0f,&motor);
    for (unsigned int i=0u;i<samples;++i) {
        a.us+=4000u;a.raw=(i%2u) ? 600u:170u;
        bcfc_observe(&w,a,81.0f+(float)i,&motor);w.black=!w.black;
    }
    bcfc_reject(&w,reason);w.elements=0u;
    bcfc_close(&w);
    barcode_live_frame_t f={.record={.epoch=9u},.index=index,.reason=reason,
        .start_us=w.start_us,.terminal_us=a.us,.rise=320u,.fall=245u};
    bcfc_terminal(&f);
}
int main(void) {
    bcfc_epoch(9u);
    attempt(1u,BW_START,600u);attempt(2u,101u,10u);
    attempt(3u,BW_WIDTH,20u); /* First two failures survive later attempts. */
    bcfc_dump_begin();bcfc_dump(9u,1u);bcfc_dump(9u,2u);bcfc_dump(9u,3u);
    assert(strstr(text,"index 1 ") &&
        ((strstr(text,"index 2 ")!=NULL)==(BCFC_FAILED_SLOTS==2u)));
    assert(!strstr(text,"index 3 "));
    assert(strstr(text,"edge_lost 537") && strstr(text,"adc_lost 152"));
    assert(strstr(text,"[BC_FAIL_EDGE]") && strstr(text,"[BC_FAIL_ADC]"));
    used=0u;text[0]='\0';bcfc_epoch(0u);bcfc_dump_begin();bcfc_dump_remaining();
    assert(strstr(text,"index 1 ") &&
        ((strstr(text,"index 2 ")!=NULL)==(BCFC_FAILED_SLOTS==2u))); /* STOP retains. */
    used=0u;text[0]='\0';bcfc_epoch(10u);bcfc_dump_begin();bcfc_dump_remaining();
    assert(!strstr(text,"[BC_FAIL_CAPTURE]"));
    /* A reader-owned old slot cannot be reused or mutated at a new run. */
    bcfc_epoch(9u);attempt(4u,BW_START,2u);
    assert(__atomic_load_n(&bcfc_slots[0].state,__ATOMIC_SEQ_CST)==2u);
    __atomic_store_n(&bcfc_reading,1u,__ATOMIC_SEQ_CST);
    bcfc_epoch(11u);
    assert(bcfc_work()!=NULL && bcfc_active!=0);
    assert(bcfc_slots[0].frame.index==4u); /* Reader's payload untouched. */
    __atomic_store_n(&bcfc_reading,0u,__ATOMIC_SEQ_CST);
    /* Successful candidates consume no retained failure slot. */
    bcfc_epoch(12u);bcfc_epoch(9u);
    attempt(5u,BW_OK,10u);attempt(6u,BW_START,10u);attempt(7u,BW_WIDTH,10u);
    used=0u;text[0]='\0';bcfc_dump_begin();bcfc_dump_remaining();
    assert(!strstr(text,"index 5 ") && strstr(text,"index 6 ") &&
        ((strstr(text,"index 7 ")!=NULL)==(BCFC_FAILED_SLOTS==2u)));
    /* Fault-triggering raw samples bypass decimation and retain fault bits. */
    bcfc_epoch(13u);bcfc_epoch(9u);
    barcode_window_t w={.phase=2u,.start_us=UINT32_MAX-500u,.black=true};
    barcode_window_motor_t motor={.us=100u};
    bcfc_start(&w,(barcode_window_adc_t){w.start_us,600u,0u},0.0f,&motor);
    bcfc_bad_adc((barcode_window_adc_t){100u,0u,0u},2u);
    bcfc_reject(&w,BW_LOSS);
    barcode_live_frame_t f={.record={.epoch=9u},.index=8u,.reason=BW_LOSS,
        .start_us=w.start_us,.terminal_us=101u};
    bcfc_terminal(&f);used=0u;text[0]='\0';bcfc_dump_begin();bcfc_dump_remaining();
    assert(strstr(text,"raw 0 flags 32770"));
    return 0;
}

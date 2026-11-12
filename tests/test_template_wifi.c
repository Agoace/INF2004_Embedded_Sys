#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <tk/tkernel.h>
#include <tk/smp.h>
#ifndef APP_FORCE_CORE0
#define APP_FORCE_CORE0 0
#endif
#include "../lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.h"
#define locked test_smp_irq_mask
#define di_calls test_smp_saves
#define ei_calls test_smp_restores
static unsigned int clock_index,creates,starts,deletes;
static uint32_t stack_pointer = 0x20010000u;
uint32_t template_mock_stack_pointer(unsigned int level)
{
    assert(level == 0u);
    return stack_pointer;
}
extern void cyw43_utk_stack_top(uint32_t sp);
extern uint32_t cyw43_utk_stack_free(void);
static uint64_t clock_values[5];
static int create_result=7,start_result;
void template_mock_di(UINT *irq) { *irq = test_smp_irq_save(); }
void template_mock_ei(UINT irq) { test_smp_irq_restore(irq); }
uint64_t time_us_64(void) {
    assert(clock_index<5u);
    assert(locked==(clock_index==1u ? 1u : 0u));
    return clock_values[clock_index++];
}
void cyw43_utk_task(INT code,void *context) { (void)code;(void)context;assert(0); }
ID tk_cre_tsk(const T_CTSK *value) {
    assert(!locked && value && value->task==(FP)cyw43_utk_task);
    assert(value->itskpri==20 && value->stksz==(TM_NET_LWIP ? 8192 : 6144));
    assert(strcmp((const char *)value->dsname,"cyw43")==0);
#if TK_SUPPORT_SMP
    const UW expected = TP_PRC1 + (APP_FORCE_CORE0 ? 0u : 1u);
    assert(value->tskatr==(TA_HLNG|TA_RNG1|TA_ASSPRC));
    assert(value->assprc == expected);
#else
    assert(value->tskatr==(TA_HLNG|TA_RNG1) && value->assprc==0u);
#endif
    ++creates;return create_result;
}
ER tk_sta_tsk(ID id,INT code) { assert(id==7 && code==0 && !locked);++starts;return start_result; }
ER tk_del_tsk(ID id) { assert(id==7 && !locked);++deletes;return E_OK; }
static void begin_poll(uint64_t at,uint32_t work) {
    assert(!locked);clock_index=0u;
    clock_values[0]=at-2u;clock_values[1]=at;clock_values[2]=at+3u;
    clock_values[3]=at+work;clock_values[4]=at+work+4u;
    cyw43_utk_poll_begin();assert(clock_index==3u && !locked);
}
static void end_poll(void) { cyw43_utk_poll_end();assert(clock_index==5u && !locked); }
static void poll(uint64_t at,uint32_t work) { begin_poll(at,work);end_poll(); }
static T_CYW43_TIMING read_timing(void) {
    T_CYW43_TIMING value;assert(cyw43_utk_timing_read(&value)==E_OK && !locked);return value;
}
static void startup_tests(void) {
    assert(cyw43_utk_start_status()==E_OBJ);
    create_result=E_SYS;
    assert(cyw43_utk_start()==E_SYS && cyw43_utk_start_status()==E_SYS);
    assert(creates==1u && starts==0u && deletes==0u);
    create_result=7;start_result=E_OBJ;
    assert(cyw43_utk_start()==E_OBJ && cyw43_utk_start_status()==E_OBJ);
    assert(creates==2u && starts==1u && deletes==1u);
    start_result=E_OK;
    assert(cyw43_utk_start()==E_OK && cyw43_utk_start_status()==E_OK);
    assert(creates==3u && starts==2u && deletes==1u);
}
int main(int argc,char **argv) {
    (void)argv;startup_tests();
    T_CYW43_TIMING value=read_timing(),window;
    assert(value.polls==0u && value.gaps==0u && value.work_total_us==0u);
    assert(cyw43_utk_timing_read(NULL)==E_PAR && cyw43_utk_timing_window_end(NULL)==E_PAR);
    memset(&window,0xa5,sizeof(window));const T_CYW43_TIMING untouched=window;
    assert(cyw43_utk_timing_window_end(&window)==E_OBJ && memcmp(&window,&untouched,sizeof(window))==0);
    if (argc>1) {
        poll((uint64_t)UINT32_MAX-4u,10u);poll((uint64_t)UINT32_MAX+20u,15u);
        value=read_timing();
        assert(value.polls==2u && value.gaps==1u && value.gap_max_us==24u && value.gaps_over_10000==0u);
        assert(value.work_max_us==15u && value.work_total_us==25u && value.probe_max_us==9u);
    } else {
        poll(100u,30u);
        assert(cyw43_utk_timing_window_begin()==E_OK);
        assert(cyw43_utk_timing_window_begin()==E_OBJ); /* Must not reset live window. */
        poll(10100u,40u);poll(20101u,50u);
        begin_poll(30101u,60u); /* Window ends during this poll: exclude it. */
        assert(cyw43_utk_timing_window_end(NULL)==E_PAR); /* Does not end window. */
        assert(cyw43_utk_timing_window_end(&window)==E_OK);
        assert(window.polls==2u && window.gaps==1u && window.gap_max_us==10001u && window.gaps_over_10000==1u);
        assert(window.work_total_us==90u && window.work_max_us==50u && window.probe_max_us==9u);
        end_poll();
        value=read_timing();
        assert(value.polls==4u && value.gaps==3u && value.gaps_over_10000==1u);
        assert(value.work_total_us==180u && value.work_max_us==60u && value.probe_max_us==9u);
        begin_poll(40101u,70u); /* Window begins during this poll: exclude it. */
        assert(cyw43_utk_timing_window_begin()==E_OK);end_poll();
        assert(cyw43_utk_timing_window_end(&window)==E_OK && window.polls==0u && window.gaps==0u);
        assert(cyw43_utk_timing_window_end(&window)==E_OBJ);
        assert(cyw43_utk_timing_window_begin()==E_OK);
        poll(50101u,80u);begin_poll(60101u,90u);
        assert(cyw43_utk_timing_window_end(&window)==E_OK && window.polls==1u && window.gaps==0u);
        assert(cyw43_utk_timing_window_begin()==E_OK);end_poll(); /* New epoch excludes old poll. */
        poll(70101u,100u);
        assert(cyw43_utk_timing_window_end(&window)==E_OK && window.polls==1u && window.gaps==0u);
        assert(window.work_total_us==100u && window.work_max_us==100u);
        value=read_timing();
        assert(value.polls==8u && value.gaps==7u && value.gap_max_us==10001u && value.gaps_over_10000==1u);
        assert(value.work_total_us==520u && value.work_max_us==100u && value.probe_max_us==9u);
    }
    /* The SP adapter also exercises the production sampled low-water logic. */
    assert(cyw43_utk_stack_free() == 0u);
    cyw43_utk_stack_top(stack_pointer);
    const uint32_t capacity = TM_NET_LWIP ? 8192u : 6144u;
    assert(cyw43_utk_stack_free() == capacity);
    stack_pointer -= 512u;
    poll(1000000u, 20u);
    assert(cyw43_utk_stack_free() == capacity - 512u);
    stack_pointer += 256u;
    cyw43_utk_stack_top(stack_pointer); /* Cannot reset the original top. */
    poll(1010000u, 20u);
    assert(cyw43_utk_stack_free() == capacity - 512u);
    stack_pointer = 0x20010000u - capacity;
    poll(1020000u, 20u);
    assert(cyw43_utk_stack_free() == 0u);
    assert(!locked && di_calls==ei_calls);
    return 0;
}

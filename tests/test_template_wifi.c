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
#include <stdarg.h>
#include <stdio.h>
#include "../app_program/platform/app_task.h"
static unsigned char *wifi_stack;
static unsigned int wifi_size;
static int delete_result;
static unsigned int stack_rows;
static char report[256];
static INT creator_prc = TP_PRC1;
static void *expected_exinf;
INT tk_get_prc(void) { return creator_prc; }
void mock_di(UINT *irq) { *irq = test_smp_irq_save(); }
void mock_ei(UINT irq) { test_smp_irq_restore(irq); }
uint16_t idle_meter_load_permille(void) { return 100u; }
uint16_t idle_meter_load_permille_core1(void) { return 200u; }
#if defined(__clang__)
/* Clang accepts UB here; GCC requires char * for a printf attribute. */
int tm_printf(const UB *format, ...) __attribute__((format(printf, 1, 2)));
#endif
int tm_printf(const UB *format, ...)
{
    if (strncmp((const char *)format, "[stack]", 7u) == 0) {
        va_list args;
        va_start(args, format);
        (void)vsnprintf(report, sizeof(report), (const char *)format, args);
        va_end(args);
        ++stack_rows;
    }
    return 0;
}
static unsigned int read_wifi(uint32_t free_bytes)
{
    app_stack_t rows[APP_TASK_MAX_SLOTS];
    assert(app_task_stacks(rows, 0u) == 0u);
    const unsigned int n = app_task_stacks(rows, APP_TASK_MAX_SLOTS);
    assert(n == 1u && strcmp(rows[0].name, "cyw43") == 0);
    assert(rows[0].size == wifi_size && rows[0].free == free_bytes);
    assert(rows[0].used == wifi_size - free_bytes);
    assert(rows[0].core == (TK_SUPPORT_SMP && !APP_FORCE_CORE0 ? 1u : 0u));
    return n;
}
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
    assert(!locked && value && value->task);
    if (value->task != (FP)cyw43_utk_task) {
        assert((value->tskatr & (TA_RNG3 | TA_USERBUF)) ==
               (TA_RNG3 | TA_USERBUF));
        assert(value->bufptr && ((uintptr_t)value->bufptr & 7u) == 0u);
        const unsigned char *bytes = value->bufptr;
        for (int i = 0; i < value->stksz; ++i) {
            assert(bytes[i] == 0xa5u);
        }
        ++creates;
        return (ID)(100u + creates);
    }
    assert(value->itskpri == 20 && value->exinf == expected_exinf);
    /* Non-USERBUF creation used to add the port's 128 B default stack. */
    assert(value->stksz == (TM_NET_LWIP ? 8320 : 6272));
    assert(value->bufptr && ((uintptr_t)value->bufptr & 7u) == 0u);
    wifi_stack = value->bufptr;
    wifi_size = (unsigned int)value->stksz;
    for (unsigned int i = 0u; i < wifi_size; ++i) {
        assert(wifi_stack[i] == 0xa5u);
    }
    if (create_result > 0) {
        memset(wifi_stack + wifi_size - 32u, 0, 32u); /* Initial context. */
    }
    assert(strcmp((const char *)value->dsname,"cyw43")==0);
#if TK_SUPPORT_SMP
    const UW expected = TP_PRC1 + (APP_FORCE_CORE0 ? 0u : 1u);
    assert(value->tskatr==(TA_HLNG|TA_RNG1|TA_ASSPRC|TA_USERBUF));
    assert(value->assprc == expected);
#else
    assert(value->tskatr==(TA_HLNG|TA_RNG1|TA_USERBUF));
    assert(value->assprc == 0u);
#endif
    ++creates;return create_result;
}
ER tk_sta_tsk(ID id,INT code)
{
    assert((id == 7 || id >= 100) && code == 0 && !locked);
    ++starts;
    return start_result;
}
ER tk_del_tsk(ID id)
{
    assert(id == 7 && !locked);
    ++deletes;
    return delete_result;
}
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
static void unused_task(void) { }
#if TM_MQTT
void wifi_stack_health_check(uint32_t expected_free);
#endif
static void full_registry_tests(void)
{
    const char *names[] = {"motion_task", "line_task", "sonar_task",
        "imu_task", "vehicle_task", "telemetry_task"};
    const UW core1 = TK_SUPPORT_SMP && !APP_FORCE_CORE0 ? TP_PRC1 + 1u :
                     TP_PRC1;
    for (unsigned int i = 0u; i < 6u; ++i) {
        const UW core = i == 3u || i == 5u ? core1 : TP_PRC1;
        assert(app_task_start((FP)unused_task, 10, core, names[i]) > 0);
    }
    unsigned int expected = 7u;
#if TM_MQTT
    assert(app_task_start((FP)unused_task, 32, TP_PRC1, "idle_meter") > 0);
#if TK_SUPPORT_SMP
    assert(app_task_start((FP)unused_task, 32, core1, "idle_meter_1") > 0);
    ++expected;
#endif
    wifi_stack_health_check(0u); /* Wi-Fi is the unique minimum. */
    expected += 2u;
#endif
    app_stack_t rows[APP_TASK_MAX_SLOTS + 1u];
    memset(rows, 0, sizeof(rows));
    rows[APP_TASK_MAX_SLOTS].size = 0x12345678u;
    assert(app_task_stacks(rows, APP_TASK_MAX_SLOTS) == expected);
    assert(rows[expected - 1u].free == 0u);
    assert(strcmp(rows[expected - 1u].name, "cyw43") == 0);
    assert(rows[APP_TASK_MAX_SLOTS].size == 0x12345678u);
    assert(app_task_stacks(rows, 1u) == 1u);
}
static void config_tests(void)
{
    T_CTSK request = {0};
    request.task = (FP)cyw43_utk_task;
    request.itskpri = 20;
    request.tskatr = TA_HLNG | TA_RNG1;
    request.dsname = (UB *)"cyw43";
    request.stksz = TM_NET_LWIP ? 8192 : 6144;
    int context = 123;
    request.exinf = &context;
    expected_exinf = &context;
#if TK_SUPPORT_SMP
    request.tskatr |= TA_ASSPRC;
    request.assprc = TP_PRC1 + (APP_FORCE_CORE0 ? 0u : 1u);
    creator_prc = 2;
    assert(app_task_start_config(&request, "cyw43") == E_CTX);
    creator_prc = TP_PRC1;
#endif
    assert(app_task_start_config(NULL, "cyw43") == E_PAR);
    assert(app_task_start_config(&request, NULL) == E_PAR);
    assert(app_task_start_config(&request, "unknown") == E_PAR);
    --request.stksz;
    assert(app_task_start_config(&request, "cyw43") == E_PAR);
    ++request.stksz;
    request.tskatr |= TA_USERBUF;
    assert(app_task_start_config(&request, "cyw43") == E_PAR);
    request.tskatr &= ~TA_USERBUF;
#if TK_SUPPORT_SMP
    request.tskatr &= ~TA_ASSPRC;
    assert(app_task_start_config(&request, "cyw43") == E_PAR);
    request.tskatr |= TA_ASSPRC;
    request.assprc = APP_FORCE_CORE0 ? TP_PRC1 + 1u : TP_PRC1;
    assert(app_task_start_config(&request, "cyw43") == E_CTX);
    request.assprc = TP_PRC1 + (APP_FORCE_CORE0 ? 0u : 1u);
#endif
    assert(creates == 0u && starts == 0u);
    unsigned char before[sizeof(request)];
    memcpy(before, &request, sizeof(request));
    assert(app_task_start_config(&request, "cyw43") == 7);
    assert(memcmp(&request, before, sizeof(request)) == 0);
    assert(read_wifi(wifi_size - 32u) == 1u);
}
static void startup_tests(void) {
    assert(cyw43_utk_start_status()==E_OBJ);
    create_result=E_SYS;
    assert(cyw43_utk_start()==E_SYS && cyw43_utk_start_status()==E_SYS);
    assert(creates==1u && starts==0u && deletes==0u);
    assert(app_task_stacks(NULL, APP_TASK_MAX_SLOTS) == 0u);
    create_result=7;start_result=E_OBJ;
    assert(cyw43_utk_start()==E_OBJ && cyw43_utk_start_status()==E_OBJ);
    assert(creates==2u && starts==1u && deletes==1u);
    assert(app_task_stacks(NULL, APP_TASK_MAX_SLOTS) == 0u);
    start_result=E_OK;
    assert(cyw43_utk_start()==E_OK && cyw43_utk_start_status()==E_OK);
    assert(creates==3u && starts==2u && deletes==1u);
}
int main(int argc,char **argv) {
    if (argc > 1 && strcmp(argv[1], "config") == 0) {
        config_tests();
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "cleanup_failure") == 0) {
        start_result = E_SYS;
        delete_result = E_OBJ;
        assert(cyw43_utk_start() == E_SYS);
        assert(cyw43_utk_start_status() == E_SYS);
        assert(creates == 1u && starts == 1u && deletes == 1u);
        assert(app_task_stacks(NULL, APP_TASK_MAX_SLOTS) == 0u);
        wifi_stack[0] = 0x17u;
        assert(cyw43_utk_start() == E_OBJ);
        assert(creates == 1u && starts == 1u && deletes == 1u);
        assert(wifi_stack[0] == 0x17u); /* Still owned: never repaint. */
        return 0;
    }
    startup_tests();
    assert(read_wifi(wifi_size - 32u) == 1u);
    wifi_stack[wifi_size - 512u] = 0; /* Deep call has already returned. */
    assert(cyw43_utk_start() == E_OBJ && creates == 3u);
    assert(wifi_stack[wifi_size - 512u] == 0u); /* No duplicate repaint. */
    assert(read_wifi(wifi_size - 512u) == 1u);
    wifi_stack[wifi_size - 512u] = 0xa5u;
    assert(read_wifi(wifi_size - 512u) == 1u); /* Monotonic mark. */
    const uint32_t margins[] = {257u, 256u, 255u, 0u};
    for (unsigned int i = 0u; i < 4u; ++i) {
        wifi_stack[margins[i]] = 0u;
        assert(read_wifi(margins[i]) == 1u);
        stack_rows = 0u;
        app_task_report();
        assert(stack_rows == 1u && strstr(report, "[stack] cyw43 core"));
        assert(strstr(report, margins[i] < 256u ? "LOW MARGIN" : " OK"));
        assert(strstr(report, "<=") == NULL);
    }
    T_CYW43_TIMING value=read_timing(),window;
    assert(value.polls==0u && value.gaps==0u && value.work_total_us==0u);
    assert(cyw43_utk_timing_read(NULL)==E_PAR && cyw43_utk_timing_window_end(NULL)==E_PAR);
    memset(&window,0xa5,sizeof(window));const T_CYW43_TIMING untouched=window;
    assert(cyw43_utk_timing_window_end(&window)==E_OBJ && memcmp(&window,&untouched,sizeof(window))==0);
    if (argc > 1 && strcmp(argv[1], "wrap") == 0) {
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
    full_registry_tests();
    assert(!locked && di_calls==ei_calls);
    return 0;
}

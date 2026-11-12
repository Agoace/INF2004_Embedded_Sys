/* Startup ordering and TEST priority policy, not a scheduler timing model. */
#include <assert.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdarg.h>
#include "../app_program/subsystem4/imu_settings.h"
#include <string.h>
#include <tk/tkernel.h>
#include "../app_program/platform/app_task.h"
#include "../app_program/platform/app_startup.h"
static jmp_buf finished;
static unsigned int inits,starts,tests,reports,watchdog_tests,priority_calls,aborts;
static unsigned int source_lines, migration_tests;
static PRI priority=1;
static bool fail_priority, fail_startup, provider_ready, vehicle_returned;
static unsigned int startup_publications;

/* The real usermain must publish only after every startup call returns. */
bool app_startup_complete(void)
{
    return startup_publications != 0u;
}

void app_startup_complete_publish(void)
{
    assert(inits == 5u && starts == 5u && vehicle_returned);
    assert(startup_publications == 0u);
    assert(provider_ready == !fail_startup);
    ++startup_publications;
}
int drv_time_init(void) { return E_OK; }
int idle_meter_calibrate(void)
{
    assert(!app_startup_complete() && inits == 0u);
    return E_OK;
}
int boot_id_init(void)
{
    assert(!app_startup_complete() && inits == 0u);
    return E_OK;
}
ER tk_ref_ver(T_RVER *v) { *v=(T_RVER){0x300u,1u};return E_OK; }
INT tk_get_prc(void) { return 1; }
ID app_task_start(FP entry,PRI pri,UW core,const char *name) {
    (void)entry;(void)pri;(void)core;(void)name;assert(0);return E_SYS;
}
#if APP_BARCODE_CAPTURE
#define EXPECT_SUBSYSTEM(n) assert(0 && "barcode must skip all other tests")
#elif APP_IMU_STREAM
#define EXPECT_SUBSYSTEM(n) assert(priority == 20 && n == 4 && tests == 0u)
#else
#define EXPECT_SUBSYSTEM(n) assert(priority == 20 && tests == n - 1u)
#endif
#define SUBSYSTEM(n) \
ER subsystem##n##_init(void) { \
    assert(!app_startup_complete()); \
    assert(priority == 1 && inits == n - 1u && starts == 0u); \
    ++inits; \
    return fail_startup && n == 3 ? E_SYS : E_OK; \
} \
ER subsystem##n##_start(void) { \
    assert(!app_startup_complete()); \
    assert(priority == 1 && inits == 5u && starts == n - 1u); \
    ++starts; \
    if (n == 3) { provider_ready = !fail_startup; } \
    return fail_startup && n == 3 ? E_OBJ : E_OK; \
} \
void subsystem##n##_test(void) { EXPECT_SUBSYSTEM(n); ++tests; }
SUBSYSTEM(1)
SUBSYSTEM(2)
SUBSYSTEM(3)
SUBSYSTEM(4)
SUBSYSTEM(5)
void barcode_test(void)
{
    assert(priority == 20 && tests == 0u);
    ++tests;
}
int vehicle_task_start(void)
{
    assert(priority == 1 && starts == 5u);
    assert(!app_startup_complete());
    vehicle_returned = true;
    return E_OK;
}
ER tk_chg_pri(ID id,PRI pri) {
    assert(app_startup_complete());
    assert(APP_TEST && id==TSK_SELF && pri==20 && starts==5u && tests==0u);
    ++priority_calls;if (fail_priority) return E_SYS;priority=pri;return E_OK;
}
void app_lock_stress_run(void)
{
    assert(priority == 20 && tests == 5u && migration_tests == 0u);
    ++migration_tests;
}
void log_stream_admission_selftest(void)
{
    assert(priority == 20 && tests == 5u && migration_tests == 1u);
    ++migration_tests;
}
void telemetry_contention_test(void)
{
    assert(priority == 20 && tests == 5u && migration_tests == 2u);
    ++migration_tests;
}
void app_task_report(void)
{
    assert(priority == 20 && tests == 5u && migration_tests == 3u);
    ++reports;
}
void subsystem2_watchdog_test(void) { assert(priority==20 && tests==5u && reports==1u);++watchdog_tests; }
int tm_printf(const UB *format,...) {
    if (!strcmp((const char *)format, "all subsystems started\n\n")) {
        assert(app_startup_complete());
    }
    if (strstr((const char *)format,"ABORT")) ++aborts;
    if (!strcmp((const char *)format,"[S4] hump_source %u %s\n")) {
        va_list a;va_start(a,format);
        assert(va_arg(a,unsigned int)==IMU_HUMP_SOURCE);
        assert(!strcmp(va_arg(a,const char *),IMU_HUMP_SOURCE_NAME));
        va_end(a);++source_lines;
    }
    return 0;
}
ER tk_slp_tsk(TMO timeout) { assert(timeout==TMO_FEVR);longjmp(finished,1); }
extern INT usermain(void);
int main(int argc,char **argv) {
    fail_priority = argc > 1 && !strcmp(argv[1], "priority_failure");
    fail_startup = argc > 1 && !strcmp(argv[1], "startup_failure");
    if (setjmp(finished)==0) { (void)usermain();assert(0); }
    assert(inits==5u && starts==5u && source_lines==1u);
    assert(startup_publications == 1u);
    assert(provider_ready == !fail_startup);
#if APP_TEST
    assert(priority_calls==1u);
    if (fail_priority) {
        assert(priority == 1 && tests == 0u && reports == 0u);
        assert(watchdog_tests == 0u && aborts == 1u && migration_tests == 0u);
    }
#if APP_IMU_STREAM || APP_BARCODE_CAPTURE
    else {
        assert(priority == 20 && tests == 1u && reports == 0u);
        assert(watchdog_tests == 0u && aborts == 0u && migration_tests == 0u);
    }
#else
    else {
        assert(priority == 20 && tests == 5u && reports == 1u);
        assert(watchdog_tests == 1u && aborts == 0u && migration_tests == 3u);
    }
#endif
#else
    assert(priority_calls == 0u && priority == 1 && tests == 0u);
    assert(reports == 0u && watchdog_tests == 0u && aborts == 0u);
    assert(migration_tests == 0u);
#endif
    return 0;
}

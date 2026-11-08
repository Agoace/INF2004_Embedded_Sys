#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <string.h>
#include "../app_program/app.h"
#include "../app_program/subsystem4/imu_service.h"

static jmp_buf stop;
static unsigned int wakes;
static unsigned int reports;
static unsigned int diagnostics;
static unsigned int stack_reports;
static unsigned int mark_lines;
static unsigned int requests;
static unsigned int stop_calls;
static unsigned int marker_inits;
static unsigned int deleted;
static bool create_error;
static bool start_error;
static bool stop_error;
static uint32_t acknowledged;

uint32_t drv_time_ms(void) { return wakes * 20u; }
ID tk_get_tid(void) { return 8; }
ID tk_cre_cyc(const T_CCYC *c)
{
    assert(c->cyctim == 20u && c->cycphs == 20u);
    assert(c->assprc == TP_PRC1);
    return create_error ? E_SYS : 9;
}
ER tk_sta_cyc(ID id) { assert(id == 9); return start_error ? E_SYS : E_OK; }
ER tk_stp_cyc(ID id) { assert(id == 9); return E_OK; }
ER tk_del_cyc(ID id) { assert(id == 9); ++deleted; return E_OK; }
ER tk_wup_tsk(ID id) { assert(id == 8); return E_OK; }
ER tk_slp_tsk(TMO timeout)
{
    assert(timeout == 100);
    if (++wakes > 50u) {
        longjmp(stop, 1);
    }
    return E_OK;
}
INT tk_can_wup(ID id) { assert(id == TSK_SELF); return 0; }
int motion_stop(void) { ++stop_calls; return stop_error ? E_SYS : E_OK; }
void imu_hw_mark_init(void) { ++marker_inits; }
bool imu_hw_mark_down(void) { return wakes >= 3u && wakes <= 8u; }
uint32_t imu_request_mark(void) { return ++requests; }
int imu_snapshot_try_read(imu_snapshot_t *out)
{
    memset(out, 0, sizeof(*out));
    out->timestamp_ms = drv_time_ms();
    out->marks = acknowledged;
    acknowledged = requests; /* Acknowledge on the following observer poll. */
    out->value.valid = true;
    out->value.motion_valid = true;
    return E_OK;
}
void app_task_report(void) { ++stack_reports; }
int tm_printf(const UB *format, ...)
{
    const char *text = (const char *)format;

    if (strncmp(text, "[S4] t_ms", 9u) == 0) {
        ++reports;
    } else if (strncmp(text, "[S4_DIAG]", 9u) == 0) {
        ++diagnostics;
    } else if (strcmp(text, "[S4] MARK %lu\n") == 0) {
        va_list args;

        va_start(args, format);
        assert(va_arg(args, unsigned long) == 1u);
        va_end(args);
        ++mark_lines;
    }
    return 0;
}

int main(int argc, char **argv)
{
    if (argc > 1) {
        create_error = strcmp(argv[1], "create_failure") == 0;
        start_error = strcmp(argv[1], "start_failure") == 0;
        stop_error = strcmp(argv[1], "stop_failure") == 0;
        subsystem4_test();
        assert(stop_calls == 1u && reports == 0u);
        assert(deleted == (start_error ? 1u : 0u));
        return 0;
    }
    if (setjmp(stop) == 0) {
        subsystem4_test();
        assert(false); /* IMU_STREAM does not end by itself. */
    }
    assert(reports == 50u); /* Every 20 ms, not the old 10 Hz print gate. */
    assert(diagnostics == 1u && stack_reports == 1u);
    assert(stop_calls == 1u && marker_inits == 1u);
    assert(requests == 1u && mark_lines == 1u);
    return 0;
}

#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <string.h>
#include "../app_program/app.h"
#include "../app_program/subsystem4/imu_service.h"
#include "../app_program/subsystem4/imu_drive.h"
#include "../app_program/subsystem2/motion.h"

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
static const char *mode = "";
static unsigned int limit = 50u;
static unsigned int speed_calls;
static unsigned int first_speed_wake;
static unsigned int estop_calls;
static unsigned int drive_stops;
static unsigned int last_reason;
static unsigned int failed_stops;
static uint32_t test_epoch;
static unsigned int run_starts;
static unsigned int hump_lines;
static unsigned int hump_peak;
static unsigned int diag_lines;

int imu_hump_diag_try_read(imu_hump_diag_report_t *out)
{
    *out = (imu_hump_diag_report_t){.run_epoch = test_epoch,
        .timestamp_ms = wakes * 20u, .valid = true};
    return E_OK;
}

uint32_t drv_time_us(void) { return wakes * 20000u; }
int motion_snapshot_try_read(motion_status_t *out, uint32_t *stamp)
{
    memset(out, 0, sizeof(*out));
    out->watchdog_armed = true;
    if (wakes >= 15u && strcmp(mode, "drive_motor_fault") == 0) {
        out->faults = 1u;
    }
    *stamp = drv_time_us();
    return E_OK;
}
int motion_set_speed(int16_t left, int16_t right)
{
    assert(left == 100 && right == 100);
    if (speed_calls == 0u) {
        first_speed_wake = wakes;
    }
    ++speed_calls;
    return wakes >= 15u && strcmp(mode, "drive_command_fault") == 0
               ? E_SYS : E_OK;
}
int motion_estop(void)
{
    ++estop_calls;
    return strcmp(mode, "drive_double_stop") == 0 ||
           strcmp(mode, "drive_double_cleanup") == 0 ? E_SYS : E_OK;
}
void imu_hw_drive_init(void) { }
bool imu_hw_drive_down(void)
{
    if (strncmp(mode, "drive_", 6u) != 0) {
        return false;
    }
    if (strcmp(mode, "drive_held_boot") == 0) {
        return wakes <= 10u || (wakes >= 21u && wakes <= 26u);
    }
    return (wakes >= 3u && wakes <= 8u) ||
        (strcmp(mode, "drive_button") == 0 && wakes >= 20u && wakes <= 25u);
}

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
    ++wakes;
    if (wakes == 15u && strcmp(mode, "drive_double_cleanup") == 0) {
        return E_TMOUT;
    }
    if (wakes > limit) {
        longjmp(stop, 1);
    }
    return E_OK;
}
INT tk_can_wup(ID id)
{
    assert(id == TSK_SELF);
    return wakes == 15u && strcmp(mode, "drive_wake_fault") == 0 ? 2 : 0;
}
int motion_stop(void)
{
    ++stop_calls;
    if (wakes >= 15u && failed_stops < 2u &&
        (strcmp(mode, "drive_double_stop") == 0 ||
         strcmp(mode, "drive_double_cleanup") == 0)) {
        ++failed_stops;
        return E_SYS;
    }
    return stop_error ? E_SYS : E_OK;
}
void imu_hw_mark_init(void) { ++marker_inits; }
bool imu_hw_mark_down(void) { return wakes >= 3u && wakes <= 8u; }
uint32_t imu_request_mark(void) { return ++requests; }
uint32_t imu_test_run(bool active)
{
    if (((test_epoch & 1u) != 0u) != active) {
        ++test_epoch;
        run_starts += active ? 1u : 0u;
    }
    return test_epoch;
}
/* Fake capture handoff: armed at drive start, frozen at the stop request,
 * then the observer must dump ONE batch per wake and release once. */
static unsigned int capture_state; /* 0 idle, 1 armed, 2 frozen */
static unsigned int capture_requests;
static unsigned int capture_stops;
static unsigned int capture_reads;
static unsigned int capture_releases;
static unsigned int capture_stop_wake;
static unsigned int last_read_wake;
static unsigned int raw_lines;
static unsigned int raw_end_lines;
static bool capture_mode(void)
{
    return strcmp(mode, "drive_distance") == 0 ||
           strcmp(mode, "drive_button") == 0;
}
bool imu_capture_request(void)
{
    if (!capture_mode() || capture_state != 0u) {
        return false;
    }
    ++capture_requests;
    capture_state = 1u;
    return true;
}
void imu_capture_stop_request(void)
{
    if (capture_state == 1u) {
        ++capture_stops;
        capture_state = 2u;
        capture_stop_wake = wakes;
    }
}
bool imu_capture_frozen(uint32_t *samples, uint32_t *batches,
                         imu_capture_reason_t *reason)
{
    if (capture_state != 2u) {
        return false;
    }
    *samples = 10u;
    *batches = 3u;
    *reason = IMU_CAPTURE_STOP;
    return true;
}
bool imu_capture_batch_read(uint32_t index, imu_capture_batch_t *batch,
                             int16_t raw[IMU_FIFO_MAX_BATCH][3])
{
    assert(capture_state == 2u && index < 3u && drive_stops == 1u);
    assert(wakes > last_read_wake); /* Throttle: one batch per wake. */
    last_read_wake = wakes;
    ++capture_reads;
    batch->first = index * 4u;
    batch->count = index == 2u ? 2u : 4u;
    batch->timestamp_us = index * 10000u;
    memset(raw, 0, sizeof(int16_t) * 3u * IMU_FIFO_MAX_BATCH);
    return true;
}
bool imu_capture_release(void)
{
    assert(capture_state == 2u && capture_reads == 3u);
    ++capture_releases;
    capture_state = 0u;
    return true;
}
int imu_snapshot_try_read(imu_snapshot_t *out)
{
    memset(out, 0, sizeof(*out));
    out->timestamp_ms = drv_time_ms();
    out->timestamp_us = drv_time_us();
    out->value.calibrated = true;
    out->raw.mag_valid = true;
    out->value.travel_mm = strcmp(mode, "drive_timeout") == 0
                               ? 0.0f : (float)wakes * 2.0f;
    if (wakes >= 15u) {
        if (strcmp(mode, "drive_unavailable") == 0) {
            return E_TMOUT;
        }
        if (strcmp(mode, "drive_fault") == 0 ||
            strcmp(mode, "drive_double_stop") == 0) {
            out->probe.faults = 1u;
        }
        if (strcmp(mode, "drive_mag_fault") == 0) {
            out->raw.mag_valid = false;
        }
        if (strcmp(mode, "drive_stale") == 0) {
            out->timestamp_us -= 40000u;
        }
    }
    out->marks = acknowledged;
    acknowledged = requests; /* Acknowledge on the following observer poll. */
    out->value.valid = true;
    out->value.motion_valid = true;
    /* One closed 10.5 mm RIGHT event in the current GP21 run. */
    out->value.hump = (imu_hump_output_t){
        .run_epoch = test_epoch, .running = (test_epoch & 1u) != 0u,
        .valid_samples = 5u, .max_peak_mm = 10.5f, .count = 1u,
        .flags = IMU_HUMP_FLAG_IMPLEMENTED};
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
    } else if (strcmp(text, "[S4_DRIVE] active %u reason %u") == 0) {
        va_list args;

        va_start(args, format);
        const unsigned int active = va_arg(args, unsigned int);

        last_reason = va_arg(args, unsigned int);
        if (active == 0u) {
            ++drive_stops;
        }
        va_end(args);
    } else if (strncmp(text, "[S4_HUMP]", 9u) == 0) {
        va_list args;

        va_start(args, format);
        hump_peak = va_arg(args, unsigned int);
        va_end(args);
        ++hump_lines;
    } else if (strncmp(text, "[S4_HUMP_DIAG]", 14u) == 0) {
        ++diag_lines;
    } else if (strncmp(text, "[S4_RAW] %lu", 12u) == 0) {
        ++raw_lines;
    } else if (strncmp(text, "[S4_RAW_END]", 12u) == 0) {
        ++raw_end_lines;
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
    if (argc > 1 && strncmp(argv[1], "drive_", 6u) == 0) {
        mode = argv[1];
        limit = strcmp(mode, "drive_timeout") == 0 ? 510u : 350u;
    }
    if (argc > 1 && strncmp(argv[1], "drive_", 6u) != 0) {
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
        /* Only observer wake failure exits after stop is confirmed. */
        assert(strcmp(mode, "drive_double_cleanup") == 0);
    }
    if (strncmp(mode, "drive_", 6u) == 0) {
        assert(speed_calls > 0u && drive_stops == 1u);
        if (strcmp(mode, "drive_double_stop") == 0 ||
            strcmp(mode, "drive_double_cleanup") == 0) {
            assert(stop_calls == 4u && estop_calls == 2u);
            assert(speed_calls == 10u); /* Never reissue after failed stop. */
        } else {
            assert(stop_calls == 2u && estop_calls == 0u);
        }
        if (strcmp(mode, "drive_held_boot") == 0) {
            assert(first_speed_wake == 23u);
        }
        unsigned int expected = IMU_DRIVE_FAULT;

        if (strcmp(mode, "drive_distance") == 0 ||
            strcmp(mode, "drive_held_boot") == 0) {
            expected = IMU_DRIVE_DISTANCE;
        } else if (strcmp(mode, "drive_timeout") == 0) {
            expected = IMU_DRIVE_TIMEOUT;
        } else if (strcmp(mode, "drive_button") == 0) {
            expected = IMU_DRIVE_BUTTON;
        }

        assert(last_reason == expected);
        /* One run, one result line; the IMU run ends with the drive. */
        assert(run_starts == 1u && hump_lines == 1u && diag_lines == 1u);
        assert((test_epoch & 1u) == 0u);
        /* The last good copy is at most one 20 ms wake old: fresh. */
        assert(hump_peak == 105u);
        if (capture_mode()) {
            /* Dump starts only after the confirmed stop, never in a drive. */
            assert(capture_requests == 1u && capture_stops == 1u);
            assert(capture_reads == 3u && capture_releases == 1u);
            assert(raw_lines == 10u && raw_end_lines == 1u);
            assert(last_read_wake == capture_stop_wake + 2u);
        } else {
            assert(capture_reads == 0u && capture_releases == 0u);
        }
        return 0;
    }
    assert(reports == 50u); /* Every 20 ms, not the old 10 Hz print gate. */
    assert(diagnostics == 1u && stack_reports == 1u);
    assert(stop_calls == 1u && marker_inits == 1u);
    assert(requests == 1u && mark_lines == 1u);
    return 0;
}

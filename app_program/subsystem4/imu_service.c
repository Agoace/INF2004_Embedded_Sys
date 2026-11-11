/* Kernel owner of the bounded IMU probe and pure observe-only core.
 * Polling has its own cyclic; never replace motion's vehicle wake client. */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "../app.h"
#include "imu_service.h"
#include "../platform/app_board.h"
#include "../platform/drv_time.h"
#include "../subsystem2/motion.h"
#include "../subsystem2/motor_ctl_robot.h"
#include "../vehicle/vehicle_task.h"

#if !APP_BENCH
#define IMU_PERIOD_MS 10u
#define IMU_TASK_PRIORITY 14
#define MOTION_MAX_AGE_US 30000u

static ID task;
static ID cyclic;
static bool running;
static imu_core_t core;
static imu_probe_t probe;
static imu_snapshot_t value;
static APP_BOARD(imu_snapshot_t) board;
static imu_hump_report_t hump_value;
static APP_BOARD(imu_hump_report_t) hump_board;
/* Keep diagnostic windows OUT of core's rollback copy and snapshot. */
static imu_hump_diag_t hump_diag;
static imu_hump_diag_report_t diag_value;
static APP_BOARD(imu_hump_diag_report_t) diag_board;
static uint32_t mark_requests;
static uint32_t wake_lost;
static uint32_t previous_us;
static int32_t previous_counts[2];
static uint32_t previous_motion_us;
static uint32_t previous_motion_ms;
static uint32_t motion_fraction_us;
static bool have_counts;
static bool wheels_still;
static float travel_mm;
static float yaw_rad;
static const mc_config_t motion_config = MC_ROBOT_CONFIG;
#if APP_TEST
/* TEST GP21 drive boundary; single TEST writer, odd while driving. */
static uint32_t test_run_epoch;
static uint32_t test_run_start_ms;
#endif

#if APP_TEST && APP_IMU_STREAM && IMU_CAPTURE_ENABLE
#define CAP_IDLE 0u
#define CAP_ARMED 1u
#define CAP_ACTIVE 2u
#define CAP_FROZEN 3u
static imu_capture_t capture;
static uint32_t capture_state;
static uint32_t capture_stop;
#endif

static void capture_tick(bool valid, uint32_t stamp)
{
#if APP_TEST && APP_IMU_STREAM && IMU_CAPTURE_ENABLE
    uint32_t state = __atomic_load_n(&capture_state, __ATOMIC_ACQUIRE);

    if (state == CAP_ARMED) {
        imu_capture_begin(&capture, stamp);
        __atomic_store_n(&capture_state, CAP_ACTIVE, __ATOMIC_RELEASE);
        state = CAP_ACTIVE;
    }
    if (state == CAP_ACTIVE) {
        if (__atomic_load_n(&capture_stop, __ATOMIC_ACQUIRE) != 0u) {
            imu_capture_finish(&capture, IMU_CAPTURE_STOP);
        } else {
            imu_capture_append(&capture, probe.fifo_raw,
                valid ? probe.fifo_last_samples : 0u, stamp,
                !valid && !probe.no_sample);
        }
        if (!capture.active) {
            __atomic_store_n(&capture_state, CAP_FROZEN, __ATOMIC_RELEASE);
        }
    }
#else
    (void)valid;
    (void)stamp;
#endif
}

/* Modular count difference without signed overflow or implementation-
 * defined conversion of a large unsigned value to int32_t. */
static int32_t count_delta(int32_t current, int32_t previous)
{
    const uint32_t bits = (uint32_t)current - (uint32_t)previous;

    return bits <= INT32_MAX ? (int32_t)bits :
           -(int32_t)(UINT32_MAX - bits) - 1;
}

static void encoder_input(imu_core_input_t *input)
{
    motion_status_t status;
    uint32_t stamp;
    input->motion_valid = motion_snapshot_try_read(&status, &stamp) == E_OK;
    const uint32_t now = drv_time_us();

    input->now_ms = drv_time_ms();
    if (!input->motion_valid || (uint32_t)(now - stamp) > MOTION_MAX_AGE_US) {
        input->motion_valid = false;
        return;
    }
    if (!have_counts || stamp != previous_motion_us) {
        const int32_t left = have_counts
            ? count_delta(status.counts[0], previous_counts[0]) : 0;
        const int32_t right = have_counts
            ? count_delta(status.counts[1], previous_counts[1]) : 0;

        if (have_counts) {
            const uint32_t elapsed = stamp - previous_motion_us;

            previous_motion_ms += elapsed / 1000u;
            motion_fraction_us += elapsed % 1000u;
            previous_motion_ms += motion_fraction_us / 1000u;
            motion_fraction_us %= 1000u;
        }
        wheels_still = have_counts && left == 0 && right == 0;
        travel_mm += ((float)left + (float)right) * 500.0f /
                     (float)motion_config.counts_per_m;
        yaw_rad += ((float)right - (float)left) * 1000.0f /
                   ((float)motion_config.counts_per_m * IMU_TRACK_MM);
        previous_counts[0] = status.counts[0];
        previous_counts[1] = status.counts[1];
        previous_motion_us = stamp;
        have_counts = true;
    }
    input->travel_mm = travel_mm;
    input->yaw_rad = yaw_rad;
    /* A relative encoder clock from distinct source stamps. Preserve us
     * fractions and modular wrap; duplicate stamps keep the SAME time. */
    input->travel_ms = previous_motion_ms;
    input->wheels_still = wheels_still && status.duty_permille[0] == 0 &&
                          status.duty_permille[1] == 0;
}

static uint32_t run_epoch(void)
{
#if APP_TEST
    return __atomic_load_n(&test_run_epoch, __ATOMIC_SEQ_CST);
#else
    return vehicle_run_epoch();
#endif
}

/* Once after the final computation/replay. No speculative latch. */
static void diagnostic_tick(const imu_core_input_t *input)
{
#if APP_TEST
    const uint32_t started = __atomic_load_n(&test_run_start_ms,
                                             __ATOMIC_SEQ_CST);
    const bool line = false;
#else
    const uint32_t started = vehicle_run_start_ms();
    const bool line = vehicle_line_run_epoch() == input->run_epoch;
#endif
    const imu_hump_diag_input_t in = {
        .hump = &core.output.hump,
        .corrected_mm = core.hump_corrected_mm,
        .filtered_mm = core.hump_filtered_mm,
        .travel_mm = core.hump_travel_mm,
        .speed_mm_s = core.output.speed_mm_s,
        .yaw_rad_s = core.output.yaw_rate_rad_s,
        .yaw_accel_rad_s2 = core.output.yaw_accel_rad_s2,
        .lateral_mg = core.output.lateral_fake_mm_s2 / 9.80665f,
        .now_ms = input->now_ms, .run_start_ms = started,
        .valid = core.hump_valid, .motion_valid = core.output.motion_valid,
        .line_active = line};

    imu_hump_diag_step(&hump_diag, &in, &diag_value);
    if (run_epoch() != input->run_epoch) {
        diag_value.valid = false;
    }
    app_board_publish(&diag_board.sequence, diag_board.words,
                      &diag_value, sizeof(diag_value));
}

static void sample(void)
{
    const uint32_t started = drv_time_us();
    const uint32_t epoch = vehicle_calibration_epoch();
    imu_core_input_t input = {0};
    const bool valid = imu_probe_read(&probe, drv_time_ms(), &value.raw);
    const uint32_t read_done = drv_time_us(); /* TEST capture batch time. */

    input.now_ms = drv_time_ms();
    input.accel_valid = valid;
    for (unsigned int axis = 0u; axis < 3u; ++axis) {
        input.accel_mg[axis] = probe.mean_accel_mg[axis];
    }
    encoder_input(&input);
    input.calibration_epoch = vehicle_calibration_epoch();
    input.calibration_allowed = epoch == input.calibration_epoch &&
                                (epoch & 1u) == 0u;
    input.run_epoch = run_epoch();
    const uint32_t compute_start = drv_time_us();
    const uint32_t requested = __atomic_load_n(&mark_requests,
                                               __ATOMIC_SEQ_CST);

    const imu_core_t before = core;

    const bool empty = !valid && probe.no_sample;

    if (!empty) {
        imu_core_step(&core, &input);
    }
    capture_tick(valid, read_done);
    const uint32_t after = vehicle_calibration_epoch();

    if (!empty && after != input.calibration_epoch) {
        /* Vehicle priority 13 may preempt this priority-14 computation.
         * Discard a candidate calibration across ANY phase transition;
         * replay once with calibration forbidden, never wait/retry. */
        core = before;
        input.calibration_epoch = after;
        input.calibration_allowed = false;
        imu_core_step(&core, &input);
    }
    if (!empty && requested != value.marks) {
        /* MARK acknowledges a zero-height reference at THIS observation,
         * not an interval of travel collected before the press. */
        imu_core_mark(&core);
        value.marks = requested;
    }
    if (!empty) {
        diagnostic_tick(&input);
    }
    const uint32_t ended = drv_time_us();
    const uint32_t compute = ended - compute_start;
    const uint32_t gap = started - previous_us;

    if (core.output.samples > 1u && gap > value.gap_max_us) {
        value.gap_max_us = gap;
    }
    previous_us = started;
    value.value = core.output;
    value.probe = probe;
    /* EMPTY is not a new observation. Publish counters but retain the
     * actual data timestamp so consumers can detect age/duplicates. */
    if (!empty) {
        value.timestamp_us = ended;
        value.timestamp_ms = input.now_ms;
        /* Only a real observation refreshes the hump board time, so a
         * stalled sensor makes the copy stale for every reader. */
        hump_value.hump = core.output.hump;
        hump_value.timestamp_ms = input.now_ms;
        app_board_publish(&hump_board.sequence, hump_board.words,
                          &hump_value, sizeof(hump_value));
    }
    value.compute_last_us = compute;
    value.compute_total_us += compute;
    if (compute > value.compute_max_us) {
        value.compute_max_us = compute;
    }
    value.wake_lost = __atomic_load_n(&wake_lost, __ATOMIC_SEQ_CST);
    app_board_publish(&board.sequence, board.words, &value, sizeof(value));
    /* Full sample wall time, including publication. Published next cycle;
     * preemption is included, so this is not exclusive CPU timing. */
    const uint32_t work = drv_time_us() - started;

    value.work_last_us = work;
    value.work_total_us += work;
    if (work > value.work_max_us) {
        value.work_max_us = work;
    }
}

static void entry(INT code, void *context)
{
    (void)code;
    (void)context;
    for (;;) {
        sample();
        (void)tk_slp_tsk(TMO_FEVR);
        const INT missed = tk_can_wup(TSK_SELF);

        if (missed > 0) {
            value.missed_wakes += (uint32_t)missed;
        }
    }
}

static void release(void *context)
{
    (void)context;
    if (task > 0 && tk_wup_tsk(task) < E_OK) {
        const uint32_t lost = __atomic_load_n(&wake_lost, __ATOMIC_SEQ_CST);

        __atomic_store_n(&wake_lost, lost + 1u, __ATOMIC_SEQ_CST);
    }
}

int imu_service_init(void)
{
    if (cyclic > 0) {
        return E_OK;
    }
    T_CCYC config = {.cycatr = TA_HLNG, .cychdr = (FP)release,
                     .cyctim = IMU_PERIOD_MS, .cycphs = IMU_PERIOD_MS};

#if TK_SUPPORT_SMP
    config.cycatr |= TA_ASSPRC;
    /* The cyclic handler must run on the same core as the task it feeds. */
    config.assprc = app_prc_core1();
#endif
    imu_core_init(&core);
    imu_probe_init(&probe);
    cyclic = tk_cre_cyc(&config);
    return cyclic < 0 ? (int)cyclic : E_OK;
}

int imu_service_start(void)
{
    if (cyclic <= 0 || task > 0) {
        return E_OBJ;
    }
    task = app_start_task((FP)entry, IMU_TASK_PRIORITY, app_prc_core1(), "imu_task");
    if (task < 0) {
        return (int)task;
    }
    const ER er = tk_sta_cyc(cyclic);

    running = er >= E_OK;
    return (int)er;
}

int imu_snapshot_try_read(imu_snapshot_t *out)
{
    imu_snapshot_t scratch;

    if (out == NULL) {
        return E_PAR;
    }
    if (!running) {
        return E_OBJ;
    }
    if (!app_board_try_read(&board.sequence, board.words, &scratch,
                             sizeof(scratch))) {
        return E_TMOUT;
    }
    *out = scratch;
    return E_OK;
}

int imu_hump_try_read(imu_hump_report_t *out)
{
    imu_hump_report_t scratch;

    if (out == NULL) {
        return E_PAR;
    }
    if (!running) {
        return E_OBJ;
    }
    if (!app_board_try_read(&hump_board.sequence, hump_board.words,
                             &scratch, sizeof(scratch))) {
        return E_TMOUT;
    }
    *out = scratch;
    return E_OK;
}

int imu_hump_diag_try_read(imu_hump_diag_report_t *out)
{
    imu_hump_diag_report_t scratch;

    if (out == NULL) {
        return E_PAR;
    }
    if (!running) {
        return E_OBJ;
    }
    if (!app_board_try_read(&diag_board.sequence, diag_board.words,
                            &scratch, sizeof(scratch))) {
        return E_TMOUT;
    }
    *out = scratch;
    return E_OK;
}

uint32_t imu_test_run(bool active)
{
#if APP_TEST
    const uint32_t epoch = __atomic_load_n(&test_run_epoch,
                                           __ATOMIC_SEQ_CST);
    const uint32_t next = ((epoch & 1u) != 0u) == active ? epoch
                                                         : epoch + 1u;

    if (active && next != epoch) {
        __atomic_store_n(&test_run_start_ms, drv_time_ms(),
                         __ATOMIC_SEQ_CST);
    }
    __atomic_store_n(&test_run_epoch, next, __ATOMIC_SEQ_CST);
    return next;
#else
    (void)active;
    return 0u;
#endif
}

uint32_t imu_request_mark(void)
{
    const uint32_t next = __atomic_load_n(&mark_requests, __ATOMIC_SEQ_CST)
                          + 1u;

    __atomic_store_n(&mark_requests, next, __ATOMIC_SEQ_CST);
    return next;
}
#else
int imu_service_init(void)
{
    return E_OK;
}

int imu_service_start(void)
{
    return E_OK;
}
int imu_snapshot_try_read(imu_snapshot_t *out)
{
    return out == NULL ? E_PAR : E_OBJ;
}
int imu_hump_try_read(imu_hump_report_t *out)
{
    return out == NULL ? E_PAR : E_OBJ;
}
int imu_hump_diag_try_read(imu_hump_diag_report_t *out)
{
    return out == NULL ? E_PAR : E_OBJ;
}
uint32_t imu_test_run(bool active)
{
    (void)active;
    return 0u;
}
uint32_t imu_request_mark(void)
{
    return 0u;
}
#endif

bool imu_capture_request(void)
{
#if !APP_BENCH && APP_TEST && APP_IMU_STREAM && IMU_CAPTURE_ENABLE
    /* Single TEST requester. The worker never transitions out of IDLE;
     * no compare-exchange/runtime helper or multi-requester claim needed. */
    if (__atomic_load_n(&capture_state, __ATOMIC_ACQUIRE) != CAP_IDLE) {
        return false;
    }
    __atomic_store_n(&capture_state, CAP_ARMED, __ATOMIC_RELEASE);
    return true;
#else
    return false;
#endif
}

void imu_capture_stop_request(void)
{
#if !APP_BENCH && APP_TEST && APP_IMU_STREAM && IMU_CAPTURE_ENABLE
    const uint32_t state = __atomic_load_n(&capture_state, __ATOMIC_ACQUIRE);

    if (state == CAP_ARMED || state == CAP_ACTIVE) {
        __atomic_store_n(&capture_stop, 1u, __ATOMIC_RELEASE);
    }
#endif
}

bool imu_capture_frozen(uint32_t *samples, uint32_t *batches,
                         imu_capture_reason_t *reason)
{
#if !APP_BENCH && APP_TEST && APP_IMU_STREAM && IMU_CAPTURE_ENABLE
    if (__atomic_load_n(&capture_state, __ATOMIC_ACQUIRE) != CAP_FROZEN ||
        samples == NULL || batches == NULL || reason == NULL) {
        return false;
    }
    *samples = capture.samples;
    *batches = capture.batches;
    *reason = capture.reason;
    return true;
#else
    (void)samples;
    (void)batches;
    (void)reason;
    return false;
#endif
}

bool imu_capture_batch_read(uint32_t index, imu_capture_batch_t *batch,
                             int16_t raw[IMU_FIFO_MAX_BATCH][3])
{
#if !APP_BENCH && APP_TEST && APP_IMU_STREAM && IMU_CAPTURE_ENABLE
    if (__atomic_load_n(&capture_state, __ATOMIC_ACQUIRE) != CAP_FROZEN ||
        index >= capture.batches || batch == NULL || raw == NULL) {
        return false;
    }
    *batch = capture.batch[index];
    for (uint32_t i = 0u; i < batch->count; ++i) {
        for (unsigned int axis = 0u; axis < 3u; ++axis) {
            raw[i][axis] = capture.raw[batch->first + i][axis];
        }
    }
    return true;
#else
    (void)index;
    (void)batch;
    (void)raw;
    return false;
#endif
}

bool imu_capture_release(void)
{
#if !APP_BENCH && APP_TEST && APP_IMU_STREAM && IMU_CAPTURE_ENABLE
    if (__atomic_load_n(&capture_state, __ATOMIC_ACQUIRE) != CAP_FROZEN) {
        return false;
    }
    /* Sole TEST consumer releases after its last copy. Clear stop BEFORE
     * IDLE permits a new arm; writer never touches frozen storage. */
    __atomic_store_n(&capture_stop, 0u, __ATOMIC_RELEASE);
    __atomic_store_n(&capture_state, CAP_IDLE, __ATOMIC_RELEASE);
    return true;
#else
    return false;
#endif
}

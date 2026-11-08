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
static const mc_config_t motion_config = MC_ROBOT_CONFIG;

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
        previous_counts[0] = status.counts[0];
        previous_counts[1] = status.counts[1];
        previous_motion_us = stamp;
        have_counts = true;
    }
    input->travel_mm = travel_mm;
    /* A relative encoder clock from distinct source stamps. Preserve us
     * fractions and modular wrap; duplicate stamps keep the SAME time. */
    input->travel_ms = previous_motion_ms;
    input->wheels_still = wheels_still && status.duty_permille[0] == 0 &&
                          status.duty_permille[1] == 0;
}

static void sample(void)
{
    const uint32_t started = drv_time_us();
    const uint32_t epoch = vehicle_calibration_epoch();
    imu_core_input_t input = {0};
    const bool valid = imu_probe_read(&probe, drv_time_ms(), &value.raw);

    input.now_ms = drv_time_ms();
    input.accel_valid = valid;
    for (unsigned int axis = 0u; axis < 3u; ++axis) {
        input.accel_mg[axis] = (float)value.raw.accel_mg[axis];
    }
    encoder_input(&input);
    input.calibration_epoch = vehicle_calibration_epoch();
    input.calibration_allowed = epoch == input.calibration_epoch &&
                                (epoch & 1u) == 0u;
    const uint32_t compute_start = drv_time_us();
    const uint32_t requested = __atomic_load_n(&mark_requests,
                                               __ATOMIC_SEQ_CST);

    const imu_core_t before = core;

    imu_core_step(&core, &input);
    const uint32_t after = vehicle_calibration_epoch();

    if (after != input.calibration_epoch) {
        /* Vehicle priority 13 may preempt this priority-14 computation.
         * Discard a candidate calibration across ANY phase transition;
         * replay once with calibration forbidden, never wait/retry. */
        core = before;
        input.calibration_epoch = after;
        input.calibration_allowed = false;
        imu_core_step(&core, &input);
    }
    if (requested != value.marks) {
        /* MARK acknowledges a zero-height reference at THIS observation,
         * not an interval of travel collected before the press. */
        imu_core_mark(&core);
        value.marks = requested;
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
    value.timestamp_us = ended;
    value.timestamp_ms = input.now_ms;
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
    config.assprc = TP_PRC1;
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
    task = app_start_task((FP)entry, IMU_TASK_PRIORITY, TP_PRC1, "imu_task");
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
uint32_t imu_request_mark(void)
{
    return 0u;
}
#endif

#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdint.h>
#include <string.h>
#include "../app_program/app.h"
#include "../app_program/subsystem4/imu_service.h"
#include "../app_program/subsystem2/motion.h"

static FP entry_function;
static FP cyclic_function;
static jmp_buf stop;
static unsigned int frame_goal;
static unsigned int frames;
static uint64_t clock_us;
static uint64_t tick_us;
static uint32_t epoch;
static unsigned int epoch_reads;
static unsigned int flip_epoch_read;
static int create_error;
static int start_error;
static int cyclic_error;
static int motion_error;
static int missed;
static int wake_error;
static bool sensor_ok = true;
static bool sensor_empty;
static bool stale_motion;
static bool corrupt_read;
static unsigned int create_calls;
static int32_t counts[2];
static int32_t count_step[2];
static int16_t duty;

uint32_t drv_time_us(void) { return (uint32_t)clock_us++; }
uint32_t drv_time_ms(void) { return (uint32_t)(clock_us / 1000u); }
uint32_t vehicle_calibration_epoch(void)
{
    ++epoch_reads;
    if (epoch_reads == flip_epoch_read) {
        epoch += 2u;
    }
    return epoch;
}

void imu_probe_init(imu_probe_t *probe)
{
    memset(probe, 0, sizeof(*probe));
}

bool imu_probe_read(imu_probe_t *probe, uint32_t now, imu_units_t *out)
{
    (void)now;
    clock_us += 530u;
    probe->last_us = 530u;
    probe->max_us = 530u;
    probe->no_sample = sensor_empty;
    probe->fifo_last_samples = 0u;
    if (sensor_empty) {
        return false;
    }
    if (!sensor_ok) {
        ++probe->faults;
        ++probe->nacks;
        return false;
    }
    ++probe->reads;
    probe->total_us += 530u;
    out->accel_mg[0] = -20;
    out->accel_mg[1] = 80;
    out->accel_mg[2] = 1375;
    out->mag_valid = true;
    probe->fifo_last_samples = 4u;
    for (unsigned int i = 0u; i < 4u; ++i) {
        probe->fifo_raw[i][0] = -320;
        probe->fifo_raw[i][1] = 1280;
        probe->fifo_raw[i][2] = 22000;
    }
    for (unsigned int axis = 0u; axis < 3u; ++axis) {
        probe->mean_accel_mg[axis] = (float)out->accel_mg[axis];
    }
    return true;
}

int motion_snapshot_try_read(motion_status_t *out, uint32_t *stamp)
{
    if (motion_error != 0) {
        return motion_error;
    }
    memset(out, 0, sizeof(*out));
    out->counts[0] = counts[0];
    out->counts[1] = counts[1];
    out->duty_permille[0] = duty;
    out->duty_permille[1] = duty;
    *stamp = (uint32_t)tick_us - (stale_motion ? 40000u : 0u);
    return E_OK;
}

ID tk_cre_cyc(const T_CCYC *config)
{
    ++create_calls;
    assert(config->cyctim == 10u && config->cycphs == 10u);
    assert(config->assprc == TP_PRC1);
    cyclic_function = config->cychdr;
    return create_error != 0 ? E_SYS : 6;
}

ER tk_sta_cyc(ID id)
{
    assert(id == 6);
    return cyclic_error != 0 ? E_SYS : E_OK;
}

ID app_start_task(FP entry, PRI priority, UW core, const char *name)
{
    assert(priority == 14 && core == TP_PRC1);
    assert(strcmp(name, "imu_task") == 0);
    entry_function = entry;
    return start_error != 0 ? E_SYS : 7;
}

static void add_count(unsigned int wheel)
{
    const uint32_t bits = (uint32_t)counts[wheel] +
                          (uint32_t)count_step[wheel];

    counts[wheel] = bits <= INT32_MAX ? (int32_t)bits :
                    -(int32_t)(UINT32_MAX - bits) - 1;
}

ER tk_slp_tsk(TMO timeout)
{
    assert(timeout == TMO_FEVR);
    ++frames;
    tick_us += 10000u;
    clock_us = tick_us;
    add_count(0u);
    add_count(1u);
    if (frames >= frame_goal) {
        longjmp(stop, 1);
    }
    return E_OK;
}

INT tk_can_wup(ID id)
{
    assert(id == TSK_SELF);
    return missed;
}

ER tk_wup_tsk(ID id)
{
    assert(id == 7);
    return wake_error != 0 ? E_SYS : E_OK;
}

static void run(unsigned int count)
{
    frame_goal = count;
    frames = 0u;
    if (setjmp(stop) == 0) {
        ((void (*)(INT, void *))entry_function)(0, NULL);
    }
    assert(frames == count);
}

void test_imu_board_hook(unsigned int index)
{
    if (index == 0u && corrupt_read) {
        corrupt_read = false;
        run(1u); /* Publish a new board after the reader's first word. */
    }
}

static imu_snapshot_t read_snapshot(void)
{
    imu_snapshot_t out;

    memset(&out, 0xa5, sizeof(out));
    assert(imu_snapshot_try_read(&out) == E_OK);
    return out;
}

int main(int argc, char **argv)
{
    imu_snapshot_t out;
    imu_snapshot_t saved;

    memset(&out, 0xa5, sizeof(out));
    saved = out;
    assert(imu_snapshot_try_read(NULL) == E_PAR);
    assert(imu_snapshot_try_read(&out) == E_OBJ);
    assert(memcmp(&out, &saved, sizeof(out)) == 0);
    assert(imu_service_start() == E_OBJ);
    if (argc > 1 && strcmp(argv[1], "create_failure") == 0) {
        create_error = 1;
        assert(imu_service_init() == E_SYS);
        assert(imu_service_start() == E_OBJ);
        return 0;
    }
    assert(imu_service_init() == E_OK);
    assert(imu_service_init() == E_OK && create_calls == 1u);
    if (argc > 1 && strcmp(argv[1], "start_failure") == 0) {
        start_error = 1;
        assert(imu_service_start() == E_SYS);
        assert(imu_snapshot_try_read(&out) == E_OBJ);
        return 0;
    }
    if (argc > 1 && strcmp(argv[1], "cyclic_failure") == 0) {
        cyclic_error = 1;
        assert(imu_service_start() == E_SYS);
        assert(imu_snapshot_try_read(&out) == E_OBJ);
        return 0;
    }
    assert(imu_service_start() == E_OK);
    assert(imu_service_start() == E_OBJ);
    assert(imu_snapshot_try_read(&out) == E_TMOUT);
    assert(memcmp(&out, &saved, sizeof(out)) == 0);
    if (argc > 1 && strcmp(argv[1], "fifo_empty") == 0) {
        run(103u);
        saved = read_snapshot();
        sensor_empty = true;
        const uint32_t mark = imu_request_mark();

        run(3u);
        out = read_snapshot();
        assert(out.timestamp_us == saved.timestamp_us);
        assert(out.value.samples == saved.value.samples);
        assert(out.marks != mark);
        sensor_empty = false;
        run(1u);
        assert(read_snapshot().marks == mark);
        return 0;
    }
#if APP_IMU_STREAM
    if (argc > 1 && strcmp(argv[1], "capture") == 0) {
        uint32_t samples, batches;
        imu_capture_reason_t reason;
        imu_capture_batch_t b;
        int16_t raw[IMU_FIFO_MAX_BATCH][3];

        imu_capture_stop_request(); /* Idle stop cannot poison next arm. */
        assert(imu_capture_request() && !imu_capture_request());
        assert(!imu_capture_release());
        run(200u);
        assert(imu_capture_frozen(&samples, &batches, &reason));
        assert(samples == 800u && batches == 200u);
        assert(reason == IMU_CAPTURE_FULL && !imu_capture_request());
        assert(imu_capture_batch_read(199u, &b, raw));
        assert(b.first == 796u && b.count == 4u && raw[3][2] == 22000);
        assert(!imu_capture_batch_read(200u, &b, raw));
        assert(!imu_capture_frozen(NULL, &batches, &reason));
        run(2u);
        assert(imu_capture_release() && !imu_capture_release());
        assert(imu_capture_request());
        run(1u);
        (void)imu_request_mark();
        run(1u);
        imu_capture_stop_request();
        run(1u);
        assert(imu_capture_frozen(&samples, &batches, &reason));
        assert(samples == 8u && reason == IMU_CAPTURE_STOP);
        assert(imu_capture_release() && imu_capture_request());
        sensor_ok = false;
        run(1u);
        assert(imu_capture_frozen(&samples, &batches, &reason));
        assert(samples == 0u && reason == IMU_CAPTURE_GAP);
        assert(imu_capture_release());
        return 0;
    }
#endif
    if (argc > 1 && strcmp(argv[1], "count_wrap") == 0) {
        counts[0] = INT32_MAX - 1;
        counts[1] = INT32_MAX - 1;
        count_step[0] = 3;
        count_step[1] = 3;
        run(2u);
        out = read_snapshot();
        assert(fabsf(out.value.travel_mm - 3000.0f / 12414.0f) < 0.0001f);
        assert(!out.value.calibrated);
        return 0;
    }
    /* Start just before timer-us wrap, while the ms clock stays monotonic. */
    tick_us = UINT32_MAX - 15000u;
    clock_us = tick_us;
    run(103u);
    out = read_snapshot();
    assert(out.value.calibrated && out.value.calibrations == 1u);
    assert(out.value.bias_mg[2] == 375.0f);
    assert(out.probe.reads == 103u && out.probe.faults == 0u);
    assert(out.work_last_us >= 530u && out.work_total_us > 0u);
    assert(out.timestamp_ms > UINT32_MAX / 1000u);
    saved = out;
    corrupt_read = true;
    assert(imu_snapshot_try_read(&out) == E_TMOUT);
    assert(memcmp(&out, &saved, sizeof(out)) == 0);
    out = read_snapshot();
    assert(out.value.samples > saved.value.samples);
    /* Opposite wheels with zero mean travel cannot refresh calibration. */
    count_step[0] = 3;
    count_step[1] = -3;
    run(150u);
    out = read_snapshot();
    assert(out.value.calibrations == 1u);
    count_step[0] = 0;
    count_step[1] = 0;
    duty = 100;
    run(150u);
    assert(read_snapshot().value.calibrations == 1u);
    duty = 0;
    epoch = 1u;
    run(150u);
    assert(read_snapshot().value.calibrations == 1u);
    epoch = 2u;
    run(100u);
    /* Three epoch reads per frame. Flip after computation on the tick
     * which would otherwise accept its completed stationary window. */
    flip_epoch_read = epoch_reads + 3u;
    run(1u);
    assert(read_snapshot().value.calibrations == 1u);
    flip_epoch_read = 0u;
    run(102u);
    assert(read_snapshot().value.calibrations == 2u);
    sensor_ok = false;
    run(3u);
    out = read_snapshot();
    assert(!out.value.valid && out.probe.faults == 3u);
    assert(out.value.calibrated); /* Fault is NOT a bias/height reset. */
    sensor_ok = true;
    motion_error = E_TMOUT;
    run(2u);
    assert(!read_snapshot().value.motion_valid);
    motion_error = E_OBJ;
    run(2u);
    assert(!read_snapshot().value.motion_valid);
    motion_error = 0;
    stale_motion = true;
    run(2u);
    assert(!read_snapshot().value.motion_valid);
    stale_motion = false;
    const uint32_t mark = imu_request_mark();

    count_step[0] = 3;
    count_step[1] = 3;
    run(1u);
    out = read_snapshot();
    assert(out.marks == mark && mark == 1u);
    assert(out.value.height_mm == 0.0f && out.value.peak_mm == 0.0f);
    assert(out.value.distance_mm == 0.0f);
    missed = 2;
    wake_error = 1;
    ((void (*)(void *))cyclic_function)(NULL);
    run(2u);
    out = read_snapshot();
    assert(out.marks == mark && mark == 1u);
    assert(out.value.height_mm == 0.0f && out.value.peak_mm == 0.0f);
    assert(out.wake_lost == 1u && out.missed_wakes >= 2u);
    return 0;
}

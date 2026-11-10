/* Public register/FIFO doubles; no bus, network or private configuration. */
#include "../app_program/subsystem4/imu_probe.h"
#include "../app_program/subsystem4/imu_hw.h"
#include <assert.h>
#include <string.h>

static uint8_t registers[2][256];
static int16_t fifo[32][3];
static unsigned int queued, consumed, calls, fail_call;
static int fail_result, status_override;
static uint32_t now_us, cycle_start;
static bool wrong_readback, arrive, force_deadline;

static int step(void)
{
    ++calls;
    now_us += 40u;
    return calls == fail_call ? fail_result : IMU_IO_OK;
}
uint32_t imu_hw_time_us(void) { return now_us; }
int imu_hw_init(void) { return step(); }
void imu_hw_cycle_begin(void) { cycle_start = now_us; }
int imu_hw_cycle_end(void)
{
    return force_deadline || now_us - cycle_start >= 2000u ?
           IMU_IO_TIMEOUT : IMU_IO_OK;
}

int imu_hw_read(uint8_t address, uint8_t reg, uint8_t *out, size_t length)
{
    const unsigned int device = address == IMU_ACCEL_ADDR ? 0u : 1u;
    const int result = step();

    if (result != IMU_IO_OK) { return result; }
    if (device == 0u && reg == 0x2fu) {
        assert(length == 1u);
        *out = status_override >= 0 ? (uint8_t)status_override :
            queued == consumed ? IMU_FIFO_EMPTY : (uint8_t)(queued-consumed);
    } else if (device == 0u && reg == 0xa8u) {
        const unsigned int n = (unsigned int)(length / 6u);

        assert(length % 6u == 0u && n <= queued - consumed);
        assert(length <= IMU_HW_MAX_READ);
        for (unsigned int i = 0u; i < n; ++i) {
            for (unsigned int axis = 0u; axis < 3u; ++axis) {
                const uint16_t w = (uint16_t)fifo[consumed + i][axis];
                const unsigned int j = i * 6u + axis * 2u;

                out[j] = (uint8_t)(w & 0xffu);
                out[j + 1u] = (uint8_t)(w >> 8);
            }
        }
        consumed += n;
        if (arrive) {
            arrive = false;
            fifo[queued][0] = 16080; /* 1005 mg, arrives during the burst. */
            ++queued;
        }
    } else if (device == 1u && reg == 3u) {
        assert(length == 6u);
        memset(out, 0, length);
    } else {
        memcpy(out, &registers[device][reg], length);
        if (wrong_readback && reg == 0x23u) { out[0] ^= 1u; }
    }
    return IMU_IO_OK;
}
int imu_hw_write(uint8_t address, uint8_t reg, uint8_t value)
{
    const int result = step();
    const unsigned int device = address == IMU_ACCEL_ADDR ? 0u : 1u;

    if (result == IMU_IO_OK) {
        registers[device][reg] = value;
        if (device == 0u && reg == 0x2eu && value == 0u) {
            queued = consumed = 0u;
            status_override = -1;
        }
    }
    return result;
}
static void feed(unsigned int n)
{
    queued = n;
    consumed = 0u;
    for (unsigned int i = 0u; i < n; ++i) {
        fifo[i][0] = (int16_t)(16000u + 16u * i);
        fifo[i][1] = -16;
        fifo[i][2] = 22000;
    }
}
static void reset(imu_probe_t *p)
{
    memset(registers, 0, sizeof(registers));
    memset(fifo, 0, sizeof(fifo));
    registers[0][0x0f] = 0x33u;
    registers[1][0x0a] = 0x48u;
    registers[1][0x0b] = 0x34u;
    registers[1][0x0c] = 0x33u;
    calls = fail_call = now_us = queued = consumed = 0u;
    status_override = -1;
    wrong_readback = arrive = force_deadline = false;
    imu_probe_init(p);
}
int main(void)
{
    imu_probe_t p;
    imu_units_t value;

    reset(&p);
    assert(!imu_probe_read(&p, 0u, &value));
    const unsigned int setup_calls = calls;

    assert(p.ready && !p.faults);
    assert(p.ctrl1 == 0x77u && p.ctrl3 == 0u && p.ctrl4 == 0x88u);
    assert(p.ctrl5 == 0x40u && p.fifo_ctrl == 0x80u);
    assert(registers[1][0] == 0x18u && registers[1][1] == 0x20u);
    feed(4u);
    assert(imu_probe_read(&p, 20u, &value));
    assert(p.reads == 1u && p.last_us == 120u); /* status, burst, mag */
    assert(p.fifo_last_samples == 4u && p.mean_accel_mg[0] == 1001.5f);
    assert(value.accel_mg[0] == 1001 && value.accel_mg[2] == 1375);
    const imu_units_t saved = value;

    /* Count 0: EMPTY is reported, not a duplicate of the previous sample. */
    assert(!imu_probe_read(&p, 30u, &value));
    assert(p.no_sample && p.fifo_empty == 1u && p.ready);
    assert(memcmp(&saved, &value, sizeof(value)) == 0);
    /* A sample that arrives during the burst is NOT read now. It is the
     * first sample of the next average: no gap and no duplicate. */
    feed(4u);
    arrive = true;
    assert(imu_probe_read(&p, 40u, &value));
    assert(p.fifo_last_samples == 4u && p.mean_accel_mg[0] == 1001.5f);
    assert(queued - consumed == 1u);
    assert(imu_probe_read(&p, 45u, &value));
    assert(p.fifo_last_samples == 1u && p.mean_accel_mg[0] == 1005.0f);
    assert(p.fifo_raw[0][0] == 16080 && queued == consumed);
    /* Count 8: the largest accepted batch is one 48-byte burst. */
    feed(8u);
    assert(imu_probe_read(&p, 47u, &value));
    assert(p.fifo_last_samples == 8u && p.mean_accel_mg[0] == 1003.5f);
    assert(p.fifo_min == 1u && p.fifo_max == 8u);
    assert(p.fifo_batches == 4u && p.fifo_samples == 17u);
    /* Count 9: backlog. Reset FIFO, discard, invalidate, NOT a bus fault. */
    feed(9u);
    assert(!imu_probe_read(&p, 50u, &value));
    assert(p.ready && p.fifo_backlogs == 1u && p.fifo_recoveries == 1u);
    assert(!p.no_sample && p.fifo_last_samples == 0u && p.faults == 0u);
    assert(registers[0][0x2e] == 0x80u && queued == 0u);
    assert(p.fifo_batches == 4u && p.reads == 4u);
    /* 32 unread (OVRN, FSS=31): overrun, reset and invalidate. */
    feed(4u);
    status_override = 0x5f;
    assert(!imu_probe_read(&p, 60u, &value));
    assert(p.fifo_overruns == 1u && p.fifo_recoveries == 2u);
    assert(status_override < 0 && p.faults == 0u);
    /* EMPTY with a nonzero count is contradictory: reset, not trusted. */
    feed(4u);
    status_override = 0x23;
    assert(!imu_probe_read(&p, 65u, &value));
    assert(p.fifo_bad_status == 1u && p.fifo_recoveries == 3u);
    feed(4u);
    const imu_units_t before_failure = value;

    force_deadline = true;
    assert(!imu_probe_read(&p, 70u, &value));
    assert(p.timeouts == 1u && !p.ready && p.fifo_last_samples == 0u);
    assert(memcmp(&before_failure, &value, sizeof(value)) == 0);
    const unsigned int before = calls;

    force_deadline = false;
    assert(!imu_probe_read(&p, 1069u, &value) && calls == before);
    assert(!imu_probe_read(&p, 1070u, &value) && p.ready);
    for (unsigned int i = 1u; i <= setup_calls; ++i) {
        reset(&p);
        fail_call = i;
        fail_result = IMU_IO_NACK;
        assert(!imu_probe_read(&p, 0u, &value));
        assert(p.nacks == 1u && !p.ready);
    }
    reset(&p);
    wrong_readback = true;
    assert(!imu_probe_read(&p, 0u, &value) && p.config_errors == 1u);
    reset(&p);
    fail_call = 1u;
    fail_result = IMU_IO_NACK;
    assert(!imu_probe_read(&p, UINT32_MAX - 500u, &value));
    assert(!imu_probe_read(&p, 498u, &value));
    assert(!imu_probe_read(&p, 499u, &value) && p.ready);
    return 0;
}

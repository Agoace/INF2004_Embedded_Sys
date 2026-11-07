/* No bus/network: public register fixtures only. */
#include "../app_program/subsystem4/imu_probe.h"
#include "../app_program/subsystem4/imu_hw.h"
#include <assert.h>
#include <string.h>

static uint8_t registers[2][256];
static uint32_t now_us;
static unsigned int calls;
static unsigned int fail_call;
static int fail_result;
static bool wrong_readback;

static int step(void)
{
    ++calls;
    now_us += 40u;
    return (calls == fail_call) ? fail_result : IMU_IO_OK;
}

uint32_t imu_hw_time_us(void) { return now_us; }
int imu_hw_init(void) { return step(); }

int imu_hw_read(uint8_t address, uint8_t reg, uint8_t *out, size_t length)
{
    const unsigned int device = (address == IMU_ACCEL_ADDR) ? 0u : 1u;
    const int result = step();

    assert(address == IMU_ACCEL_ADDR || address == IMU_MAG_ADDR);
    if (result != IMU_IO_OK) {
        return result;
    }
    if ((reg == 0xa8u) || ((device == 1u) && (reg == 3u))) {
        assert(length == 6u);
        memset(out, 0, length);
        return IMU_IO_OK;
    }
    memcpy(out, &registers[device][reg], length);
    if (wrong_readback && reg == 0x23u) {
        out[0] ^= 1u;
    }
    return IMU_IO_OK;
}

int imu_hw_write(uint8_t address, uint8_t reg, uint8_t value)
{
    const int result = step();
    const unsigned int device = (address == IMU_ACCEL_ADDR) ? 0u : 1u;

    if (result == IMU_IO_OK) {
        registers[device][reg] = value;
    }
    return result;
}

static void reset(imu_probe_t *probe)
{
    memset(registers, 0, sizeof(registers));
    registers[0][0x0f] = 0x33u;
    registers[1][0x0a] = 0x48u;
    registers[1][0x0b] = 0x34u;
    registers[1][0x0c] = 0x33u;
    calls = 0u;
    fail_call = 0u;
    now_us = 0u;
    wrong_readback = false;
    imu_probe_init(probe);
}

int main(void)
{
    imu_probe_t probe;
    imu_units_t value;
    unsigned int setup_calls;
    unsigned int before;

    reset(&probe);
    assert(!imu_probe_read(&probe, 0u, &value)); /* setup, no sample yet */
    setup_calls = calls;
    assert(probe.ready && !probe.faults);
    assert(probe.ctrl1 == 0x57u && probe.ctrl4 == 0x88u);
    assert(registers[0][0x20] == 0x57u && registers[0][0x23] == 0x88u);
    assert(registers[1][0] == 0x18u && registers[1][1] == 0x20u);
    assert(registers[1][2] == 0u);
    assert(imu_probe_read(&probe, 20u, &value));
    assert(probe.reads == 1u && probe.last_us == 80u && probe.max_us == 80u);
    fail_call = calls + 2u;
    fail_result = IMU_IO_TIMEOUT;
    memset(&value, 0x55, sizeof(value));
    {
        const imu_units_t saved = value;
        assert(!imu_probe_read(&probe, 30u, &value));
        assert(memcmp(&saved, &value, sizeof(value)) == 0);
    }
    assert(probe.timeouts == 1u && probe.faults == 1u && !probe.ready);
    before = calls;
    assert(!imu_probe_read(&probe, 1029u, &value) && calls == before);
    assert(!imu_probe_read(&probe, 1030u, &value) && probe.ready);
    /* Fault at each setup stage: init, identities, writes and readbacks. */
    for (unsigned int i = 1u; i <= setup_calls; ++i) {
        reset(&probe);
        fail_call = i;
        fail_result = IMU_IO_NACK;
        assert(!imu_probe_read(&probe, 0u, &value));
        assert(!probe.ready && probe.nacks == 1u && probe.faults == 1u);
    }
    reset(&probe);
    registers[0][0x0f] = 0u;
    assert(!imu_probe_read(&probe, 0u, &value));
    assert(probe.identity_errors == 1u);
    reset(&probe);
    registers[1][0x0b] = 0u;
    assert(!imu_probe_read(&probe, 0u, &value));
    assert(probe.identity_errors == 1u);
    reset(&probe);
    wrong_readback = true;
    assert(!imu_probe_read(&probe, 0u, &value));
    assert(probe.config_errors == 1u);
    /* Retry deadline across a 32-bit millisecond wrap. */
    reset(&probe);
    fail_call = 1u;
    fail_result = IMU_IO_NACK;
    assert(!imu_probe_read(&probe, UINT32_MAX - 500u, &value));
    before = calls;
    assert(!imu_probe_read(&probe, 498u, &value) && calls == before);
    assert(!imu_probe_read(&probe, 499u, &value) && probe.ready);
    return 0;
}

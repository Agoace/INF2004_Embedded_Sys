/* Register protocol adapted from Jaison, IMU-Telemetry b2a6bb6.
 * The bus calls are bounded; no kernel or SDK headers here. */
#include "imu_probe.h"
#include "imu_hw.h"
#include <string.h>

#define PROBE_ID_ERROR -4
#define PROBE_CONFIG_ERROR -5

typedef struct {
    uint8_t address;
    uint8_t reg;
    uint8_t value;
} reg_setting_t;

static const reg_setting_t settings[] = {
    {IMU_ACCEL_ADDR, 0x20u, 0x00u}, /* power down during configuration */
    {IMU_ACCEL_ADDR, 0x21u, 0x00u},
    {IMU_ACCEL_ADDR, 0x22u, 0x10u}, /* I1_DRDY1: accel DRDY on INT1/GP7 */
    {IMU_ACCEL_ADDR, 0x23u, 0x88u}, /* BDU, HR, +/-2 g, little endian */
    {IMU_ACCEL_ADDR, 0x24u, 0x00u},
    {IMU_ACCEL_ADDR, 0x25u, 0x00u}, /* high-active, default output mode */
    {IMU_MAG_ADDR, 0x00u, 0x18u},   /* 75 Hz */
    {IMU_MAG_ADDR, 0x01u, 0x20u},   /* +/-1.3 gauss */
    {IMU_MAG_ADDR, 0x02u, 0x00u},   /* continuous */
    {IMU_ACCEL_ADDR, 0x20u, 0x57u} /* 100 Hz, XYZ on */
};

void imu_probe_init(imu_probe_t *probe)
{
    memset(probe, 0, sizeof(*probe));
}

static int configure(imu_probe_t *probe)
{
    uint8_t id[3];
    int result = imu_hw_init();

    if (result != IMU_IO_OK) {
        return result;
    }
    /* LSM303DLHC accel WHO_AM_I=0x33, mag ID bytes spell H43. */
    result = imu_hw_read(IMU_ACCEL_ADDR, 0x0fu, id, 1u);
    if (result != IMU_IO_OK) {
        return result;
    }
    if (id[0] != 0x33u) {
        return PROBE_ID_ERROR;
    }
    result = imu_hw_read(IMU_MAG_ADDR, 0x0au, id, 3u);
    if (result != IMU_IO_OK) {
        return result;
    }
    if ((id[0] != 0x48u) || (id[1] != 0x34u) || (id[2] != 0x33u)) {
        return PROBE_ID_ERROR;
    }
    for (size_t i = 0u; i < sizeof(settings) / sizeof(settings[0]); ++i) {
        const reg_setting_t *s = &settings[i];
        result = imu_hw_write(s->address, s->reg, s->value);
        if (result != IMU_IO_OK) {
            return result;
        }
    }
    /* Read back the final configuration, including continuous mode. */
    for (size_t i = 1u; i < sizeof(settings) / sizeof(settings[0]); ++i) {
        const reg_setting_t *s = &settings[i];
        result = imu_hw_read(s->address, s->reg, id, 1u);
        if (result != IMU_IO_OK) {
            return result;
        }
        if (id[0] != s->value) {
            return PROBE_CONFIG_ERROR;
        }
        if ((s->address == IMU_ACCEL_ADDR) && (s->reg == 0x20u)) {
            probe->ctrl1 = id[0];
        }
        if ((s->address == IMU_ACCEL_ADDR) && (s->reg == 0x22u)) {
            probe->ctrl3 = id[0];
        }
        if ((s->address == IMU_ACCEL_ADDR) && (s->reg == 0x23u)) {
            probe->ctrl4 = id[0];
        }
    }
    return IMU_IO_OK;
}

static void fault(imu_probe_t *probe, int result, uint32_t now_ms)
{
    ++probe->faults;
    probe->nacks += (result == IMU_IO_NACK) ? 1u : 0u;
    probe->timeouts += (result == IMU_IO_TIMEOUT) ? 1u : 0u;
    probe->identity_errors += (result == PROBE_ID_ERROR) ? 1u : 0u;
    probe->config_errors += (result == PROBE_CONFIG_ERROR) ? 1u : 0u;
    probe->ready = false;
    probe->retry_at = now_ms + 1000u;
}

bool imu_probe_read(imu_probe_t *probe, uint32_t now_ms, imu_units_t *out)
{
    uint8_t accel[6];
    uint8_t mag[6];
    const uint32_t started = imu_hw_time_us();
    int result = IMU_IO_OK;

    if (!probe->ready) {
        if ((probe->faults != 0u) &&
            ((int32_t)(now_ms - probe->retry_at) < 0)) {
            probe->last_us = 0u;
            return false;
        }
        result = configure(probe);
        probe->ready = result == IMU_IO_OK;
        /* Wait for actual conversions on the next cycle. */
        if (probe->ready) {
            probe->last_us = imu_hw_time_us() - started;
            return false;
        }
    } else {
        probe->int1_before = imu_hw_int1_high();
        result = imu_hw_read(IMU_ACCEL_ADDR, 0xa8u, accel, sizeof(accel));
        probe->int1_after = imu_hw_int1_high();
        if (result == IMU_IO_OK) {
            ++probe->int1_checks;
            probe->int1_before_high += probe->int1_before ? 1u : 0u;
            probe->int1_after_low += probe->int1_after ? 0u : 1u;
            const bool match = probe->int1_before && !probe->int1_after;

            probe->int1_matches += match ? 1u : 0u;
            probe->int1_mismatches += match ? 0u : 1u;
            /* Mag auto-increments without setting the accel-specific bit 7. */
            result = imu_hw_read(IMU_MAG_ADDR, 0x03u, mag, sizeof(mag));
        }
        if (result == IMU_IO_OK) {
            (void)imu_decode(accel, mag, &imu_robot_axes, out);
            ++probe->reads;
        }
    }
    probe->last_us = imu_hw_time_us() - started;
    if (probe->last_us > probe->max_us) {
        probe->max_us = probe->last_us;
    }
    probe->total_us += probe->last_us;
    if (result != IMU_IO_OK) {
        fault(probe, result, now_ms);
    }
    return result == IMU_IO_OK;
}

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
    {IMU_ACCEL_ADDR, 0x2eu, 0x00u}, /* transient bypass clears FIFO */
    {IMU_ACCEL_ADDR, 0x21u, 0x00u},
    {IMU_ACCEL_ADDR, 0x22u, 0x00u}, /* FIFO count replaces DRDY polling */
    {IMU_ACCEL_ADDR, 0x23u, 0x88u}, /* BDU, HR, +/-2 g, little endian */
    {IMU_ACCEL_ADDR, 0x24u, 0x40u}, /* FIFO_EN */
    {IMU_ACCEL_ADDR, 0x25u, 0x00u}, /* high-active, default output mode */
    {IMU_MAG_ADDR, 0x00u, 0x18u},   /* 75 Hz */
    {IMU_MAG_ADDR, 0x01u, 0x20u},   /* +/-1.3 gauss */
    {IMU_MAG_ADDR, 0x02u, 0x00u},   /* continuous */
    {IMU_ACCEL_ADDR, 0x2eu, 0x80u}, /* stream mode, no IRQ/watermark */
    {IMU_ACCEL_ADDR, 0x20u, 0x77u} /* 400 Hz, XYZ on */
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
    for (size_t i = 2u; i < sizeof(settings) / sizeof(settings[0]); ++i) {
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
        if ((s->address == IMU_ACCEL_ADDR) && (s->reg == 0x24u)) {
            probe->ctrl5 = id[0];
        }
        if ((s->address == IMU_ACCEL_ADDR) && (s->reg == 0x2eu)) {
            probe->fifo_ctrl = id[0];
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

static int recover_fifo(imu_probe_t *probe)
{
    int result = imu_hw_write(IMU_ACCEL_ADDR, 0x2eu, 0x00u);

    if (result == IMU_IO_OK) {
        result = imu_hw_write(IMU_ACCEL_ADDR, 0x2eu, 0x80u);
    }
    if (result == IMU_IO_OK) {
        ++probe->fifo_recoveries;
    }
    probe->fifo_last_samples = 0u;
    probe->no_sample = false;
    return result;
}

/* ONE status read and ONE burst per cycle: worst case is 4 + 51 bytes plus
 * the 9-byte mag read, inside the shared 2 ms cycle deadline. A sample that
 * arrives during the burst stays queued for the NEXT average, so there is no
 * gap and no duplicate. AN3308 9.2.3/9.5 (LIS3DH): FSS is the unread count,
 * OVRN means 32 unread, and the burst address rolls back from 0x2D to 0x28.
 * The LSM303DLHC datasheet does not state this; verify on hardware. */
static int collect_fifo(imu_probe_t *probe, uint8_t last[6])
{
    uint8_t burst[IMU_HW_MAX_READ];
    unsigned int count = 0u;
    int result = imu_hw_read(IMU_ACCEL_ADDR, 0x2fu, &probe->fifo_status, 1u);

    if (result != IMU_IO_OK) {
        return result;
    }
    const bool status_ok = imu_fifo_count(probe->fifo_status, &count);

    if (!status_ok || (count > IMU_FIFO_MAX_BATCH)) {
        if ((probe->fifo_status & IMU_FIFO_OVERRUN) != 0u) {
            ++probe->fifo_overruns;
        } else if (!status_ok) {
            ++probe->fifo_bad_status;
        } else {
            ++probe->fifo_backlogs;
        }
        return recover_fifo(probe);
    }
    if (count == 0u) {
        probe->no_sample = true;
        return IMU_IO_OK;
    }
    result = imu_hw_read(IMU_ACCEL_ADDR, 0xa8u, burst, 6u * count);
    if (result != IMU_IO_OK) {
        return result;
    }
    for (unsigned int i = 0u; i < count; ++i) {
        imu_fifo_words(&burst[6u * i], probe->fifo_raw[i]);
    }
    (void)memcpy(last, &burst[6u * (count - 1u)], 6u);
    probe->fifo_last_samples = count;
    return IMU_IO_OK;
}

bool imu_probe_read(imu_probe_t *probe, uint32_t now_ms, imu_units_t *out)
{
    uint8_t accel[6];
    uint8_t mag[6];
    imu_units_t decoded;
    const uint32_t started = imu_hw_time_us();
    int result = IMU_IO_OK;
    bool valid = false;
    const bool reading = probe->ready;

    probe->no_sample = false;
    probe->fifo_last_samples = 0u;
    if (!probe->ready) {
        if ((probe->faults != 0u) &&
            ((int32_t)(now_ms - probe->retry_at) < 0)) {
            probe->last_us = 0u;
            return false;
        }
        result = configure(probe);
        probe->ready = result == IMU_IO_OK;
        /* Wait for actual conversions on the next cycle. Setup time has
         * its own last_us and stays out of the read maximum/total. */
        if (probe->ready) {
            probe->last_us = imu_hw_time_us() - started;
            return false;
        }
    } else {
        ++probe->fifo_cycles;
        imu_hw_cycle_begin();
        result = collect_fifo(probe, accel);
        if (result == IMU_IO_OK) {
            /* Keep the 75 Hz continuous magnetometer read every 10 ms,
             * also in empty/recovered accel cycles. No GPIO IRQ. */
            result = imu_hw_read(IMU_MAG_ADDR, 0x03u, mag, sizeof(mag));
        }
        if (result == IMU_IO_OK && probe->fifo_last_samples > 0u) {
            float mean[3];

            (void)imu_fifo_mean(probe->fifo_raw, probe->fifo_last_samples,
                                mean);
            (void)imu_decode(accel, mag, &imu_robot_axes, &decoded);
            for (unsigned int axis = 0u; axis < 3u; ++axis) {
                const imu_axis_map_t *map = &imu_robot_axes.accel;

                probe->mean_accel_mg[axis] = mean[map->source[axis]] *
                                           (float)map->sign[axis];
                decoded.accel_mg[axis] =
                    (int32_t)probe->mean_accel_mg[axis]; /* Display only. */
            }
            valid = true;
        }
        const int deadline = imu_hw_cycle_end();

        if (result == IMU_IO_OK) {
            result = deadline;
        }
        if (result == IMU_IO_OK && probe->no_sample) {
            ++probe->fifo_empty;
        }
    }
    probe->last_us = imu_hw_time_us() - started;
    if (probe->last_us > probe->max_us) {
        probe->max_us = probe->last_us;
    }
    probe->total_us += probe->last_us;
    if (reading && probe->last_us >= 2000u && result == IMU_IO_OK) {
        result = IMU_IO_TIMEOUT;
    }
    if (result != IMU_IO_OK) {
        probe->fifo_last_samples = 0u;
        probe->no_sample = false;
        fault(probe, result, now_ms);
        return false;
    }
    if (valid) {
        const unsigned int count = probe->fifo_last_samples;

        *out = decoded;
        ++probe->reads;
        ++probe->fifo_batches;
        probe->fifo_samples += count;
        if (probe->fifo_batches == 1u || count < probe->fifo_min) {
            probe->fifo_min = count;
        }
        if (count > probe->fifo_max) {
            probe->fifo_max = count;
        }
    }
    return valid;
}

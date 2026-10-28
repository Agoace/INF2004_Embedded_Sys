/**
 * @file imu_lsm303.h
 * @brief GY-511 / LSM303DLHC register access, configuration, and readback check.
 *
 * The device has no self-test register, so a "sensor self-test" is a
 * functional check: identity, configuration readback, and one plausible live
 * sample. Identity and config happen here; the plausible-sample stage is
 * staged across 13 ms cycles by the core, so nothing blocks.
 */
#ifndef IMU_LSM303_H
#define IMU_LSM303_H

#include "imu_types.h"

/** Expected identity values. */
#define IMU_LSM303_WHO_AM_I_ACCEL   0x33u
#define IMU_LSM303_WHO_AM_I_MAG     0x48u

/** Self-test detail codes carried in IMU_EVT_SELFTEST. */
#define IMU_LSM303_ST_ACCEL_ID      1u
#define IMU_LSM303_ST_MAG_ID        2u
#define IMU_LSM303_ST_ACCEL_CFG     3u
#define IMU_LSM303_ST_MAG_CFG       4u
#define IMU_LSM303_ST_DATA          5u

typedef enum
{
    IMU_LSM303_OK = 0,
    IMU_LSM303_ERR_BUS,
    IMU_LSM303_ERR_IDENTITY,
    IMU_LSM303_ERR_CONFIG
} imu_lsm303_status_t;

/**
 * @brief Verify both identities, apply the configuration, verify the readback.
 *
 * On failure the detail argument, when not NULL, receives the failing stage.
 */
imu_lsm303_status_t imu_lsm303_configure(uint8_t * detail);

/** Read one raw sample from both devices. */
imu_lsm303_status_t imu_lsm303_read(imu_sample_t * out);

/** Convert accelerometer counts to g (+/-2 g high resolution, 1 mg per LSB). */
void imu_lsm303_accel_to_g(const int16_t raw[IMU_AXIS_COUNT],
                           float out_g[IMU_AXIS_COUNT]);

/** Convert magnetometer counts to gauss at the +/-1.3 gauss gain. */
void imu_lsm303_mag_to_gauss(const int16_t raw[IMU_AXIS_COUNT],
                             float out_gauss[IMU_AXIS_COUNT]);

#endif /* IMU_LSM303_H */

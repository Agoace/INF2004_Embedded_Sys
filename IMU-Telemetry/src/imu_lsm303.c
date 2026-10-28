/**
 * @file imu_lsm303.c
 * @brief LSM303DLHC register definitions and access.
 */
#include "imu_lsm303.h"

#include "imu_config.h"
#include "imu_hal.h"

/* ---- Register map ------------------------------------------------------ */

/* Accelerometer. */
#define ACCEL_WHO_AM_I_A    0x0Fu
#define ACCEL_CTRL_REG1_A   0x20u
#define ACCEL_CTRL_REG2_A   0x21u
#define ACCEL_CTRL_REG3_A   0x22u
#define ACCEL_CTRL_REG4_A   0x23u
#define ACCEL_CTRL_REG5_A   0x24u
#define ACCEL_OUT_X_L_A     0x28u
#define ACCEL_AUTO_INC      0x80u

/* Magnetometer. */
#define MAG_CRA_REG_M       0x00u
#define MAG_CRB_REG_M       0x01u
#define MAG_MR_REG_M        0x02u
#define MAG_OUT_X_H_M       0x03u
#define MAG_IRA_REG_M       0x0Au
#define MAG_AUTO_INC        0x80u

/* Configuration values.
 *
 * CTRL_REG1_A 0x57: ODR 100 Hz (0101), low-power off, X/Y/Z enabled.
 * CTRL_REG4_A: high resolution (bit 3) plus the full-scale bits from
 *              imu_config.h, so the range and the scale factor cannot drift
 *              apart.
 * CRA_REG_M   0x18: data output rate 75 Hz (bits 4:2 = 110), temperature off.
 * CRB_REG_M   0x20: gain +/-1.3 gauss (bits 7:5 = 001).
 */
#define ACCEL_CTRL1_VALUE   0x57u
#define ACCEL_CTRL4_VALUE   (0x08u | IMU_CFG_ACCEL_FS_BITS)
#define MAG_CRA_VALUE       0x18u
#define MAG_CRB_VALUE       0x20u

/* Readback masks: only the fields this driver sets are compared. */
#define ACCEL_CTRL1_MASK    0xF8u
#define ACCEL_CTRL4_MASK    0x38u
#define MAG_CRA_MASK        0x1Cu
#define MAG_CRB_MASK        0xE0u

/* ---- Helpers ----------------------------------------------------------- */

static bool
write_reg (uint8_t addr, uint8_t reg, uint8_t value)
{
    return imu_hal_i2c_write_reg(addr, reg, value);
}

static bool
read_reg (uint8_t addr, uint8_t reg, uint8_t * dst, size_t len)
{
    return imu_hal_i2c_read_reg(addr, reg, dst, len);
}

/** Compare a readback register against the value and mask that matter. */
static bool
readback_matches (uint8_t addr, uint8_t reg, uint8_t expect, uint8_t mask)
{
    uint8_t value = 0u;

    if (!read_reg(addr, reg, &value, 1u))
    {
        return false;
    }

    return ((value & mask) == (expect & mask));
}

/* ---- Public API -------------------------------------------------------- */

/** Record the failing stage, so the self-test event names the real cause. */
static imu_lsm303_status_t
fail (uint8_t * detail, uint8_t stage, imu_lsm303_status_t status)
{
    if (NULL != detail)
    {
        *detail = stage;
    }

    return status;
}

imu_lsm303_status_t
imu_lsm303_configure (uint8_t * detail)
{
    uint8_t id = 0u;

    if (!read_reg(IMU_CFG_ACCEL_ADDR, ACCEL_WHO_AM_I_A, &id, 1u))
    {
        return fail(detail, IMU_LSM303_ST_ACCEL_ID, IMU_LSM303_ERR_BUS);
    }

    if (IMU_LSM303_WHO_AM_I_ACCEL != id)
    {
        return fail(detail, IMU_LSM303_ST_ACCEL_ID, IMU_LSM303_ERR_IDENTITY);
    }

    if (!read_reg(IMU_CFG_MAG_ADDR, MAG_IRA_REG_M, &id, 1u))
    {
        return fail(detail, IMU_LSM303_ST_MAG_ID, IMU_LSM303_ERR_BUS);
    }

    if (IMU_LSM303_WHO_AM_I_MAG != id)
    {
        return fail(detail, IMU_LSM303_ST_MAG_ID, IMU_LSM303_ERR_IDENTITY);
    }

    /* Accept the magnetometer configuration before switching it to
     * continuous conversion, as the datasheet requires. */
    if (!write_reg(IMU_CFG_MAG_ADDR, MAG_CRA_REG_M, MAG_CRA_VALUE) ||
        !write_reg(IMU_CFG_MAG_ADDR, MAG_CRB_REG_M, MAG_CRB_VALUE) ||
        !write_reg(IMU_CFG_MAG_ADDR, MAG_MR_REG_M, 0x00u))
    {
        return fail(detail, IMU_LSM303_ST_MAG_CFG, IMU_LSM303_ERR_BUS);
    }

    if (!write_reg(IMU_CFG_ACCEL_ADDR, ACCEL_CTRL_REG2_A, 0x00u) ||
        !write_reg(IMU_CFG_ACCEL_ADDR, ACCEL_CTRL_REG3_A, 0x00u) ||
        !write_reg(IMU_CFG_ACCEL_ADDR, ACCEL_CTRL_REG4_A, ACCEL_CTRL4_VALUE) ||
        !write_reg(IMU_CFG_ACCEL_ADDR, ACCEL_CTRL_REG5_A, 0x00u))
    {
        return fail(detail, IMU_LSM303_ST_ACCEL_CFG, IMU_LSM303_ERR_BUS);
    }

    /* Enable the accelerometer last, so the data registers are settled by the
     * time the first sample is read. */
    if (!write_reg(IMU_CFG_ACCEL_ADDR, ACCEL_CTRL_REG1_A, ACCEL_CTRL1_VALUE))
    {
        return fail(detail, IMU_LSM303_ST_ACCEL_CFG, IMU_LSM303_ERR_BUS);
    }

    if (!readback_matches(IMU_CFG_ACCEL_ADDR, ACCEL_CTRL_REG1_A,
                          ACCEL_CTRL1_VALUE, ACCEL_CTRL1_MASK) ||
        !readback_matches(IMU_CFG_ACCEL_ADDR, ACCEL_CTRL_REG4_A,
                          ACCEL_CTRL4_VALUE, ACCEL_CTRL4_MASK))
    {
        return fail(detail, IMU_LSM303_ST_ACCEL_CFG, IMU_LSM303_ERR_CONFIG);
    }

    if (!readback_matches(IMU_CFG_MAG_ADDR, MAG_CRA_REG_M, MAG_CRA_VALUE,
                          MAG_CRA_MASK) ||
        !readback_matches(IMU_CFG_MAG_ADDR, MAG_CRB_REG_M, MAG_CRB_VALUE,
                          MAG_CRB_MASK))
    {
        return fail(detail, IMU_LSM303_ST_MAG_CFG, IMU_LSM303_ERR_CONFIG);
    }

    return IMU_LSM303_OK;
}

imu_lsm303_status_t
imu_lsm303_read (imu_sample_t * out)
{
    uint8_t buf[6];

    if (!read_reg(IMU_CFG_ACCEL_ADDR, (uint8_t) (ACCEL_OUT_X_L_A | ACCEL_AUTO_INC),
                  buf, sizeof(buf)))
    {
        return IMU_LSM303_ERR_BUS;
    }

    /* Accelerometer data is little-endian and 12-bit left-justified. */
    for (int a = 0; a < IMU_AXIS_COUNT; a++)
    {
        int16_t const raw = (int16_t) ((uint16_t) buf[2 * a] |
                                       ((uint16_t) buf[(2 * a) + 1] << 8));

        out->accel_raw[a] = (int16_t) (raw / 16);
    }

    if (!read_reg(IMU_CFG_MAG_ADDR, (uint8_t) (MAG_OUT_X_H_M | MAG_AUTO_INC),
                  buf, sizeof(buf)))
    {
        return IMU_LSM303_ERR_BUS;
    }

    /* Magnetometer data is big-endian and ordered X, Z, Y. */
    int16_t const mag_x = (int16_t) (((uint16_t) buf[0] << 8) | buf[1]);
    int16_t const mag_z = (int16_t) (((uint16_t) buf[2] << 8) | buf[3]);
    int16_t const mag_y = (int16_t) (((uint16_t) buf[4] << 8) | buf[5]);

    out->mag_raw[0] = mag_x;
    out->mag_raw[1] = mag_y;
    out->mag_raw[2] = mag_z;

    return IMU_LSM303_OK;
}

void
imu_lsm303_accel_to_g (const int16_t raw[IMU_AXIS_COUNT],
                       float out_g[IMU_AXIS_COUNT])
{
    for (int a = 0; a < IMU_AXIS_COUNT; a++)
    {
        out_g[a] = ((float) raw[a] * IMU_CFG_ACCEL_MG_PER_LSB) /
                   IMU_CFG_MG_PER_G;
    }
}

void
imu_lsm303_mag_to_gauss (const int16_t raw[IMU_AXIS_COUNT],
                         float out_gauss[IMU_AXIS_COUNT])
{
    out_gauss[0] = (float) raw[0] / IMU_CFG_MAG_LSB_PER_GAUSS_XY;
    out_gauss[1] = (float) raw[1] / IMU_CFG_MAG_LSB_PER_GAUSS_XY;
    out_gauss[2] = (float) raw[2] / IMU_CFG_MAG_LSB_PER_GAUSS_Z;
}

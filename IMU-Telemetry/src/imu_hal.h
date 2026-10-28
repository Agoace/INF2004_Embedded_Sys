/**
 * @file imu_hal.h
 * @brief Hardware seam between the platform-independent IMU code and the board.
 *
 * The firmware implements these in imu_hal_pico.c. The host tests implement
 * them over a fake LSM303DLHC register file, so the production state machine
 * runs unchanged on both.
 */
#ifndef IMU_HAL_H
#define IMU_HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Bring up the I2C pins and the data-ready input. */
bool imu_hal_bus_init(void);

/** Write one device register. */
bool imu_hal_i2c_write_reg(uint8_t addr7, uint8_t reg, uint8_t value);

/** Read `len` bytes starting at one device register. */
bool imu_hal_i2c_read_reg(uint8_t addr7, uint8_t reg, uint8_t * dst, size_t len);

/** Monotonic milliseconds. */
uint32_t imu_hal_now_ms(void);

/** True when the data-ready line is asserted. A sampling hint only. */
bool imu_hal_drdy_asserted(void);

#endif /* IMU_HAL_H */

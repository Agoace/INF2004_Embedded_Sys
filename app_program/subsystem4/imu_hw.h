/* TEST-only I2C0 adapter. Register ideas: Jaison, b2a6bb6. */
#ifndef IMU_HW_H
#define IMU_HW_H
#include <stddef.h>
#include <stdint.h>
#define IMU_ACCEL_ADDR 0x19u
#define IMU_MAG_ADDR   0x1eu
#define IMU_IO_OK       0
#define IMU_IO_NACK    -1
#define IMU_IO_TIMEOUT -2
#define IMU_IO_PARAM   -3
/* Each transfer has a 2 ms timeout and a finite spin limit. */
int imu_hw_init(void);
int imu_hw_read(uint8_t address, uint8_t reg, uint8_t *out, size_t length);
int imu_hw_write(uint8_t address, uint8_t reg, uint8_t value);
uint32_t imu_hw_time_us(void);
#endif

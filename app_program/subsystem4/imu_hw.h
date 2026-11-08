/* Robot/TEST I2C0 adapter. Register ideas: Jaison, b2a6bb6. */
#ifndef IMU_HW_H
#define IMU_HW_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define IMU_ACCEL_ADDR 0x19u
#define IMU_MAG_ADDR   0x1eu
#define IMU_IO_OK       0
#define IMU_IO_NACK    -1
#define IMU_IO_TIMEOUT -2
#define IMU_IO_PARAM   -3
#define IMU_HW_MAX_READ 48u
/* Each transfer has a 2 ms timeout and a finite spin limit. Between
 * cycle_begin and cycle_end, ONE shared 2 ms deadline also covers ALL
 * FIFO status, FIFO burst and mag transfers of that cycle. Configuration
 * uses only the per-transfer bound. */
void imu_hw_cycle_begin(void);
int imu_hw_cycle_end(void);
int imu_hw_init(void);
int imu_hw_read(uint8_t address, uint8_t reg, uint8_t *out, size_t length);
int imu_hw_write(uint8_t address, uint8_t reg, uint8_t value);
uint32_t imu_hw_time_us(void);
/* STREAM-only GP20 input, active low, internal pull-up. */
void imu_hw_mark_init(void);
bool imu_hw_mark_down(void);
/* IMU_STREAM-only GP21 bounded drive job; active low, no IRQ. */
void imu_hw_drive_init(void);
bool imu_hw_drive_down(void);
#endif

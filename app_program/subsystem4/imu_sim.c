/* Stage 0: no I2C access or register emulator. Fail closed as absent IMU. */
#include "../sim/pil_stage0.h"
#if !APP_BENCH
#include "imu_hw.h"
#include "../platform/drv_time_hw.h"

void imu_hw_cycle_begin(void)
{
}

int imu_hw_cycle_end(void)
{
    return IMU_IO_NACK;
}

uint32_t imu_hw_time_us(void)
{
    return drv_time_hw_us(); /* Real TIMER; never virtualize kernel time. */
}

int imu_hw_init(void)
{
    return IMU_IO_NACK;
}

int imu_hw_write(uint8_t address, uint8_t reg, uint8_t value)
{
    (void)address;
    (void)reg;
    (void)value;
    return IMU_IO_NACK;
}

int imu_hw_read(uint8_t address, uint8_t reg, uint8_t *out, size_t length)
{
    (void)address;
    (void)reg;
    if (out == NULL || length == 0u || length > IMU_HW_MAX_READ) {
        return IMU_IO_PARAM;
    }
    return IMU_IO_NACK; /* Leave caller data untouched. */
}

void imu_hw_mark_init(void)
{
}

bool imu_hw_mark_down(void)
{
    return false;
}

void imu_hw_drive_init(void)
{
}

bool imu_hw_drive_down(void)
{
    return false;
}
#endif

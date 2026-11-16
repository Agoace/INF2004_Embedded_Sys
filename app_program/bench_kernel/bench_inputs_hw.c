/* Step0 only: constant line; unavailable IMU/sonar; no physical bus/pulse. */
#if defined(BENCH_KERNEL) && BENCH_KERNEL
#include "../subsystem3/drv_line_hw.h"
#include "../subsystem4/imu_hw.h"
#include "../subsystem5/drv_ultrasonic_hw.h"
#include "bench_timer_hw.h"
int drv_line_hw_init(void) { bk_line_timer_init(); return 0; }
void drv_line_hw_alarm(uint32_t at_us) { bk_line_timer_alarm(at_us); }
void drv_line_hw_ack(void) { bk_line_timer_ack(); }
int drv_line_hw_read(uint16_t raw[3])
{
    if (raw == 0) { return -1; }
    raw[0] = 1800u; raw[1] = 1800u; raw[2] = 1800u;
    return 0;
}
void drv_ultrasonic_hw_init(void) { }
bool drv_ultrasonic_hw_echo(void) { return false; }
void drv_ultrasonic_hw_pulse(void) { }
void imu_hw_mark_init(void) { }
bool imu_hw_mark_down(void) { return false; }
void imu_hw_drive_init(void) { }
bool imu_hw_drive_down(void) { return false; }
void imu_hw_cycle_begin(void) { }
int imu_hw_cycle_end(void) { return IMU_IO_NACK; }
int imu_hw_init(void) { return IMU_IO_NACK; }
uint32_t imu_hw_time_us(void) { return bk_timer_now(); }
int imu_hw_read(uint8_t address, uint8_t reg, uint8_t *out, size_t length)
{
    (void)address; (void)reg; (void)out; (void)length;
    return IMU_IO_NACK;
}
int imu_hw_write(uint8_t address, uint8_t reg, uint8_t value)
{
    (void)address; (void)reg; (void)value;
    return IMU_IO_NACK;
}
#endif

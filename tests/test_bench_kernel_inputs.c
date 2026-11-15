#include <assert.h>
#include <stdint.h>
#include "../app_program/bench_kernel/bench_motor.h"
#include "../app_program/bench_kernel/bench_gpio.h"
#include "../app_program/subsystem2/drv_motor_hw.h"
#include "../app_program/subsystem3/drv_line_hw.h"
#include "../app_program/subsystem4/imu_hw.h"
#include "../app_program/subsystem5/drv_ultrasonic_hw.h"
static unsigned int init, alarms, acks;
uint32_t bk_timer_now(void) { return 123u; }
void bk_line_timer_init(void) { ++init; }
void bk_line_timer_alarm(uint32_t at) { assert(at == 1000u); ++alarms; }
void bk_line_timer_ack(void) { ++acks; }
bool bk_runtime_button(unsigned int pin) { return pin == 20u; }
int main(void)
{
    drv_motor_hw_init(1500u);
    assert(bk_motor_compares() == 0u && bk_motor_servo() == 1500u);
    const int16_t duty[2] = {200, -200};
    drv_motor_hw_write(duty, 1600u);
    const uint32_t levels = bk_motor_compares();
    assert((int16_t)(levels & 0xffffu) == 1250);
    assert((int16_t)(levels >> 16u) == -1250);
    assert(bk_motor_servo() == 1600u);
    drv_motor_hw_stop();
    assert(bk_motor_compares() == 0u);
    drv_motor_hw_buttons_init();
    assert(drv_motor_hw_button_down(20u));
    assert(!drv_motor_hw_button_down(21u));
    assert(drv_line_hw_init() == 0 && init == 1u);
    drv_line_hw_alarm(1000u); drv_line_hw_ack();
    assert(alarms == 1u && acks == 1u);
    uint16_t raw[3] = {0};
    assert(drv_line_hw_read(raw) == 0);
    assert(raw[0] == 1800u && raw[1] == 1800u && raw[2] == 1800u);
    assert(drv_line_hw_read(0) == -1);
    uint8_t byte = 99u;
    assert(imu_hw_init() == IMU_IO_NACK);
    assert(imu_hw_read(IMU_ACCEL_ADDR, 0u, &byte, 1u) == IMU_IO_NACK);
    assert(byte == 99u); /* Never invent valid unavailable register data. */
    assert(imu_hw_write(IMU_MAG_ADDR, 0u, 1u) == IMU_IO_NACK);
    assert(imu_hw_time_us() == 123u);
    imu_hw_cycle_begin();
    assert(imu_hw_cycle_end() == IMU_IO_NACK);
    assert(!imu_hw_drive_down() && !imu_hw_mark_down());
    drv_ultrasonic_hw_init(); drv_ultrasonic_hw_pulse();
    assert(!drv_ultrasonic_hw_echo());
    return 0;
}

#include <assert.h>
#include <string.h>
#include "hardware/structs/timer.h"
#include "subsystem2/drv_motor_hw.h"
#include "subsystem3/drv_line_hw.h"
#include "subsystem3/barcode_hw.h"
#include "subsystem4/imu_hw.h"
#include "subsystem5/drv_ultrasonic_hw.h"
#include "platform/drv_gpio_irq_hw.h"

pil_test_timer_t pil_test_timer;
static unsigned long console_state;

unsigned long tm_usb_state(void)
{
    return console_state;
}

uint32_t drv_time_hw_us(void)
{
    return pil_test_timer.timerawl;
}

int main(void)
{
    const int16_t duties[2] = {1000, -1000};
    drv_motor_hw_init(1500u);
    drv_motor_hw_write(duties, 1900u);
    drv_motor_hw_stop();
    drv_motor_hw_buttons_init();
    assert(drv_motor_hw_ab(0u) == 0u && drv_motor_hw_ab(1u) == 0u);
    assert(!drv_motor_hw_button_down(20u));
    assert(!drv_motor_hw_button_down(21u));
    barcode_hw_button_init();
    assert(!barcode_hw_button_down() && !barcode_hw_drive_down());
    assert(!barcode_hw_usb_ready());
    console_state = 3UL;
    assert(barcode_hw_usb_ready());
    drv_ultrasonic_hw_init();
    drv_ultrasonic_hw_pulse();
    assert(!drv_ultrasonic_hw_echo());
    drv_gpio_irq_hw_enable(UINT32_C(0x3fffffff));
    for (unsigned int group = 0u; group < 4u; ++group) {
        drv_gpio_irq_hw_ack(group * 8u, 12u);
        assert(drv_gpio_irq_hw_pending(group) == 0u);
    }
    uint16_t raw[3] = {101u, 202u, 303u};
    pil_test_timer.inte = 2u;
    assert(drv_line_hw_init() == 0 && pil_test_timer.inte == 3u);
    assert(drv_line_hw_read(raw) < 0);
    assert(raw[0] == 101u && raw[1] == 202u && raw[2] == 303u);
    pil_test_timer.timerawl = 100u;
    drv_line_hw_alarm(200u);
    assert(pil_test_timer.alarm[0] == 200u && pil_test_timer.intf == 0u);
    drv_line_hw_alarm(100u);
    assert(pil_test_timer.intf == 1u);
    drv_line_hw_ack();
    assert(pil_test_timer.intf == 0u && pil_test_timer.intr == 1u);
    pil_test_timer.timerawl = UINT32_C(0xfffffff0);
    drv_line_hw_alarm(0x20u);
    assert(pil_test_timer.intf == 0u);
    pil_test_timer.timerawl = 0x20u;
    drv_line_hw_alarm(UINT32_C(0xfffffff0));
    assert(pil_test_timer.intf == 1u);
    assert(imu_hw_time_us() == 0x20u);
    assert(imu_hw_init() == IMU_IO_NACK);
    imu_hw_cycle_begin();
    assert(imu_hw_cycle_end() == IMU_IO_NACK);
    assert(imu_hw_write(IMU_ACCEL_ADDR, 0x20u, 0x77u) == IMU_IO_NACK);
    uint8_t bytes[48], expected[48];
    memset(bytes, 0xa5, sizeof(bytes));
    memcpy(expected, bytes, sizeof(bytes));
    assert(imu_hw_read(IMU_ACCEL_ADDR, 0xa8u, bytes, 48u) == IMU_IO_NACK);
    assert(memcmp(expected, bytes, sizeof(bytes)) == 0);
    assert(imu_hw_read(IMU_ACCEL_ADDR, 0u, NULL, 1u) == IMU_IO_PARAM);
    assert(imu_hw_read(IMU_ACCEL_ADDR, 0u, bytes, 0u) == IMU_IO_PARAM);
    assert(imu_hw_read(IMU_ACCEL_ADDR, 0u, bytes, 49u) == IMU_IO_PARAM);
    imu_hw_mark_init();
    imu_hw_drive_init();
    assert(!imu_hw_mark_down() && !imu_hw_drive_down());
    return 0;
}

/* Register doubles exercise success, NACK and stuck-controller exits.
 * Timer deliberately never advances: the finite spin bound must still work.
 */
#include "imu_registers.h"
#include "../app_program/subsystem4/imu_hw.h"
#include <assert.h>
#include <stddef.h>
#include <string.h>

fake_i2c_t fake_i2c;
fake_reset_t fake_reset;
fake_timer_t fake_timer;
static unsigned int pins;
static bool int1;
static bool pulled_down;

void gpio_set_function(unsigned int pin, unsigned int function)
{
    assert((pin < 2u) && (function == GPIO_FUNC_I2C));
    pins |= 1u << pin;
}
void gpio_pull_up(unsigned int pin)
{
    assert(pin < 2u || pin == 20u || pin == 21u);
}
void gpio_pull_down(unsigned int pin)
{
    assert(pin == 7u);
    pulled_down = true;
}
void gpio_init(unsigned int pin)
{
    assert(pin == 7u || pin == 20u || pin == 21u);
}
void gpio_set_dir(unsigned int pin, unsigned int direction)
{
    assert((pin == 7u || pin == 20u || pin == 21u) &&
           direction == GPIO_IN);
}
bool gpio_get(unsigned int pin)
{
    assert(pin == 7u || pin == 20u || pin == 21u);
    return pin == 7u ? int1 : false;
}

int main(void)
{
    uint8_t bytes[6];

    imu_hw_mark_init();
    assert(imu_hw_mark_down());
    imu_hw_drive_init();
    assert(imu_hw_drive_down());

    assert(imu_hw_init() == IMU_IO_TIMEOUT); /* reset never completes */
    fake_reset.reset_done = RESETS_RESET_I2C0_BITS;
    assert(imu_hw_init() == IMU_IO_OK && pins == 3u);
    assert(fake_i2c.fs_scl_hcnt == 126u && fake_i2c.fs_scl_lcnt == 187u);
    assert(fake_i2c.sda_hold == 38u && fake_i2c.intr_mask == 0u);
    assert(pulled_down && !imu_hw_int1_high());
    int1 = true;
    assert(imu_hw_int1_high());
    fake_i2c.enable_status = 1u;
    assert(imu_hw_read(0x19u, 0x0fu, bytes, 1u) == IMU_IO_TIMEOUT);
    fake_i2c.enable_status = 0u;
    fake_i2c.tx_abrt_source = 1u;
    assert(imu_hw_read(0x19u, 0x0fu, bytes, 1u) == IMU_IO_NACK);
    fake_i2c.tx_abrt_source = 0u;
    assert(imu_hw_read(0x19u, 0x0fu, bytes, 1u) == IMU_IO_TIMEOUT);
    fake_i2c.rxflr = 1u;
    assert(imu_hw_read(0x19u, 0x0fu, bytes, 1u) == IMU_IO_TIMEOUT);
    fake_i2c.raw_intr_stat = I2C_IC_RAW_INTR_STAT_STOP_DET_BITS;
    assert(imu_hw_read(0x19u, 0xa8u, bytes, 6u) == IMU_IO_OK);
    assert(imu_hw_write(0x19u, 0x20u, 0x57u) == IMU_IO_OK);
    assert(fake_i2c.data_cmd == (0x57u | I2C_IC_DATA_CMD_STOP_BITS));
    fake_i2c.tx_abrt_source = 1u;
    assert(imu_hw_write(0x1eu, 0x02u, 0u) == IMU_IO_NACK);
    assert(imu_hw_read(0x19u, 0u, NULL, 1u) == IMU_IO_PARAM);
    assert(imu_hw_read(0x19u, 0u, bytes, 0u) == IMU_IO_PARAM);
    assert(imu_hw_read(0x19u, 0u, bytes, 7u) == IMU_IO_PARAM);
    return 0;
}

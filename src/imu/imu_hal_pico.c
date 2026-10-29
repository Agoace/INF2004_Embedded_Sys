/**
 * @file imu_hal_pico.c
 * @brief Pico SDK implementation of the hardware seam.
 *
 * I2C0 on GP0/GP1 with the GY-511 board pull-ups. The data-ready line on GP6
 * is an input hint only; it never gates the 13 ms step.
 */
#include "imu_hal.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#include "imu_config.h"

static i2c_inst_t *
imu_i2c (void)
{
#if (IMU_CFG_I2C_INSTANCE == 0)
    return i2c0;
#else
    return i2c1;
#endif
}

bool
imu_hal_bus_init (void)
{
    i2c_inst_t * const bus = imu_i2c();

    i2c_init(bus, IMU_CFG_I2C_BAUD_HZ);

    /* No internal pull-ups: the module carries its own. */
    gpio_set_function(IMU_CFG_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(IMU_CFG_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_CFG_SDA_GPIO);
    gpio_pull_up(IMU_CFG_SCL_GPIO);

    /* Data-ready: driven by the module, active high. */
    gpio_init(IMU_CFG_DRDY_GPIO);
    gpio_set_dir(IMU_CFG_DRDY_GPIO, GPIO_IN);

    return true;
}

bool
imu_hal_i2c_write_reg (uint8_t addr7, uint8_t reg, uint8_t value)
{
    uint8_t const buf[2] = { reg, value };

    int const written = i2c_write_blocking(imu_i2c(), addr7, buf, sizeof(buf),
                                           false);

    return (written == (int) sizeof(buf));
}

bool
imu_hal_i2c_read_reg (uint8_t addr7, uint8_t reg, uint8_t * dst, size_t len)
{
    int const written = i2c_write_blocking(imu_i2c(), addr7, &reg, 1u,
                                           true /* keep the bus */);

    if (written != 1)
    {
        return false;
    }

    int const read = i2c_read_blocking(imu_i2c(), addr7, dst, len, false);

    return (read == (int) len);
}

uint32_t
imu_hal_now_ms (void)
{
    return to_ms_since_boot(get_absolute_time());
}

bool
imu_hal_drdy_asserted (void)
{
    return (0 != gpio_get(IMU_CFG_DRDY_GPIO));
}

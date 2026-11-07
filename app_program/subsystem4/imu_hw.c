/* SDK-only LSM303DLHC bus adapter. Port ideas: Jaison, b2a6bb6.
 * Register-level I2C: no SDK time, IRQ, clock or runtime ownership.
 * Only the TEST runner calls this adapter; normal robot builds do not. */
#if APP_TEST
#include "imu_hw.h"
#include "hardware/gpio.h"
#include "hardware/regs/i2c.h"
#include "hardware/regs/resets.h"
#include "hardware/structs/i2c.h"
#include "hardware/structs/resets.h"
#include "hardware/structs/timer.h"

#if !BOARD_UART0_FREE || TM_CONSOLE_UART
#error "IMU on GP0/GP1 requires CRASH_REPORT=0 and CONSOLE=usb_cdc"
#endif

#define BUS_TIMEOUT_US 2000u
#define BUS_SPIN_LIMIT 100000u
#define SDA_PIN 0u
#define SCL_PIN 1u

void imu_hw_mark_init(void)
{
    gpio_init(20u);
    gpio_set_dir(20u, GPIO_IN);
    gpio_pull_up(20u);
}

bool imu_hw_mark_down(void)
{
    return !gpio_get(20u);
}

static uint32_t started;
static uint32_t spins;

uint32_t imu_hw_time_us(void)
{
    return timer_hw->timerawl;
}

static void begin_wait(void)
{
    started = imu_hw_time_us();
    spins = 0u;
}

static bool expired(void)
{
    ++spins;
    return ((uint32_t)(imu_hw_time_us() - started) >= BUS_TIMEOUT_US) ||
           (spins >= BUS_SPIN_LIMIT);
}

static int poll_error(void)
{
    if (i2c0_hw->tx_abrt_source != 0u) {
        (void)i2c0_hw->clr_tx_abrt;
        return IMU_IO_NACK;
    }
    return expired() ? IMU_IO_TIMEOUT : IMU_IO_OK;
}

int imu_hw_init(void)
{
    /* No unbounded SDK reset/clock helpers. clk_sys is fixed at 125 MHz. */
    hw_set_bits(&resets_hw->reset, RESETS_RESET_I2C0_BITS);
    hw_clear_bits(&resets_hw->reset, RESETS_RESET_I2C0_BITS);
    begin_wait();
    while ((resets_hw->reset_done & RESETS_RESET_I2C0_BITS) == 0u) {
        if (expired()) {
            return IMU_IO_TIMEOUT;
        }
    }
    i2c0_hw->enable = 0u;
    i2c0_hw->con = I2C_IC_CON_MASTER_MODE_BITS |
                  I2C_IC_CON_IC_SLAVE_DISABLE_BITS |
                  I2C_IC_CON_IC_RESTART_EN_BITS |
                  (I2C_IC_CON_SPEED_VALUE_FAST << I2C_IC_CON_SPEED_LSB);
    /* SDK baud formula: round(125 MHz / 400 kHz) = 313 cycles.
     * Low 187, high 126; hold 38 cycles >= 300 ns. */
    i2c0_hw->fs_scl_lcnt = 187u;
    i2c0_hw->fs_scl_hcnt = 126u;
    i2c0_hw->fs_spklen = 11u;
    i2c0_hw->sda_hold = 38u;
    i2c0_hw->rx_tl = 0u;
    i2c0_hw->tx_tl = 0u;
    i2c0_hw->intr_mask = 0u;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    return IMU_IO_OK;
}

static int select_device(uint8_t address)
{
    begin_wait();
    i2c0_hw->enable = 0u;
    while (i2c0_hw->enable_status != 0u) {
        if (expired()) {
            return IMU_IO_TIMEOUT;
        }
    }
    (void)i2c0_hw->clr_intr;
    i2c0_hw->tar = address;
    i2c0_hw->enable = 1u;
    return IMU_IO_OK;
}

static int finish(void)
{
    int result;

    while ((i2c0_hw->raw_intr_stat &
            I2C_IC_RAW_INTR_STAT_STOP_DET_BITS) == 0u) {
        result = poll_error();
        if (result != IMU_IO_OK) {
            i2c0_hw->enable = 0u;
            return result;
        }
    }
    result = poll_error();
    (void)i2c0_hw->clr_stop_det;
    return result;
}

int imu_hw_write(uint8_t address, uint8_t reg, uint8_t value)
{
    int result = select_device(address);

    if (result != IMU_IO_OK) {
        return result;
    }
    i2c0_hw->data_cmd = reg;
    i2c0_hw->data_cmd = (uint32_t)value | I2C_IC_DATA_CMD_STOP_BITS;
    return finish();
}

int imu_hw_read(uint8_t address, uint8_t reg, uint8_t *out, size_t length)
{
    int result;

    if ((out == NULL) || (length == 0u) || (length > 6u)) {
        return IMU_IO_PARAM;
    }
    result = select_device(address);
    if (result != IMU_IO_OK) {
        return result;
    }
    /* At most seven FIFO words (depth 16). Register pointer then repeated
     * start to read; the last byte sends STOP. No IRQ or DMA is used. */
    i2c0_hw->data_cmd = reg;
    for (size_t i = 0u; i < length; ++i) {
        const uint32_t restart = (i == 0u) ? I2C_IC_DATA_CMD_RESTART_BITS : 0u;
        const uint32_t stop = (i + 1u == length)
                                  ? I2C_IC_DATA_CMD_STOP_BITS : 0u;
        i2c0_hw->data_cmd = I2C_IC_DATA_CMD_CMD_BITS | restart | stop;
    }
    for (size_t i = 0u; i < length; ++i) {
        while (i2c0_hw->rxflr == 0u) {
            result = poll_error();
            if (result != IMU_IO_OK) {
                i2c0_hw->enable = 0u;
                return result;
            }
        }
        out[i] = (uint8_t)i2c0_hw->data_cmd;
    }
    return finish();
}
#endif

/* Only the register/API surface used by imu_hw.c; no real hardware. */
#ifndef IMU_TEST_REGISTERS_H
#define IMU_TEST_REGISTERS_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    volatile uint32_t enable, con, fs_scl_lcnt, fs_scl_hcnt, fs_spklen;
    volatile uint32_t sda_hold, rx_tl, tx_tl, intr_mask, tx_abrt_source;
    volatile uint32_t clr_tx_abrt, enable_status, clr_intr, tar, data_cmd;
    volatile uint32_t raw_intr_stat, clr_stop_det, rxflr;
} fake_i2c_t;
typedef struct { volatile uint32_t reset, reset_done; } fake_reset_t;
typedef struct { volatile uint32_t timerawl; } fake_timer_t;
extern fake_i2c_t fake_i2c;
extern fake_reset_t fake_reset;
extern fake_timer_t fake_timer;
#define i2c0_hw (&fake_i2c)
#define resets_hw (&fake_reset)
#define timer_hw (&fake_timer)
#define RESETS_RESET_I2C0_BITS (1u << 3)
#define I2C_IC_CON_MASTER_MODE_BITS 1u
#define I2C_IC_CON_IC_SLAVE_DISABLE_BITS (1u << 6)
#define I2C_IC_CON_IC_RESTART_EN_BITS (1u << 5)
#define I2C_IC_CON_SPEED_VALUE_FAST 2u
#define I2C_IC_CON_SPEED_LSB 1u
#define I2C_IC_RAW_INTR_STAT_STOP_DET_BITS (1u << 9)
#define I2C_IC_DATA_CMD_STOP_BITS (1u << 9)
#define I2C_IC_DATA_CMD_RESTART_BITS (1u << 10)
#define I2C_IC_DATA_CMD_CMD_BITS (1u << 8)
#define GPIO_FUNC_I2C 3u
#define GPIO_IN 0u
void gpio_init(unsigned int pin);
void gpio_set_dir(unsigned int pin, unsigned int direction);
bool gpio_get(unsigned int pin);
static inline void hw_set_bits(volatile uint32_t *p, uint32_t value)
{ *p |= value; }
static inline void hw_clear_bits(volatile uint32_t *p, uint32_t value)
{ *p &= ~value; }
void gpio_set_function(unsigned int pin, unsigned int function);
void gpio_pull_up(unsigned int pin);
void gpio_pull_down(unsigned int pin);
#endif

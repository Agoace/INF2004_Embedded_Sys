/* SDK-only register access. Never include kernel headers in this file. */
#include "drv_motor_hw.h"
#include "hardware/structs/io_bank0.h"
#include "hardware/structs/pads_bank0.h"
#include "hardware/structs/pwm.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/resets.h"
#include "hardware/regs/resets.h"
#include "hardware/regs/pads_bank0.h"
#include "hardware/regs/pwm.h"
#define MOTOR_WRAP 6249u
#define SERVO_WRAP 19999u
static uint16_t last_servo;
static void pin(unsigned int gpio, uint32_t function, bool pull_up) {
    sio_hw->gpio_oe_clr = 1u << gpio;
    uint32_t pad = pads_bank0_hw->io[gpio];
    pad &= ~(PADS_BANK0_GPIO0_OD_BITS | PADS_BANK0_GPIO0_PUE_BITS | PADS_BANK0_GPIO0_PDE_BITS);
    pad |= PADS_BANK0_GPIO0_IE_BITS;
    if (pull_up) pad |= PADS_BANK0_GPIO0_PUE_BITS;
    pads_bank0_hw->io[gpio] = pad;
    io_bank0_hw->io[gpio].ctrl = function;
}
static void slice_init(unsigned int slice, uint32_t div, uint32_t top, uint32_t cc) {
    pwm_hw->slice[slice].csr = 0u;
    pwm_hw->slice[slice].ctr = 0u;
    pwm_hw->slice[slice].div = div << PWM_CH0_DIV_INT_LSB;
    pwm_hw->slice[slice].top = top;
    pwm_hw->slice[slice].cc = cc;
    pwm_hw->slice[slice].csr = PWM_CH0_CSR_EN_BITS;
}
void drv_motor_hw_init(uint16_t servo_us) {
    /* Kernel cpu_clock.c sets clk_sys to 125 MHz. With USE_PTMR=0 the
     * kernel leaves PWM reset asserted; this hardware owner releases it. */
    hw_clear_bits(&resets_hw->reset, RESETS_RESET_PWM_BITS);
    while ((resets_hw->reset_done & RESETS_RESET_PWM_BITS) == 0u) { }
    slice_init(4u, 1u, MOTOR_WRAP, 0u);
    slice_init(5u, 1u, MOTOR_WRAP, 0u);
    for (unsigned int gpio = 8u; gpio <= 11u; ++gpio) pin(gpio, 4u, false);
    slice_init(7u, 125u, SERVO_WRAP, (uint32_t)servo_us << 16u);
    pin(15u, 4u, false);
    last_servo = servo_us;
    pin(16u, 5u, false); pin(17u, 5u, false);
    pin(4u, 5u, false); pin(5u, 5u, false);
}
uint8_t drv_motor_hw_ab(unsigned int wheel) {
    const uint32_t pins = sio_hw->gpio_in;
    const unsigned int a = wheel == 0u ? 16u : 4u;
    const unsigned int b = wheel == 0u ? 17u : 5u;
    return (uint8_t)((((pins >> a) & 1u) << 1u) | ((pins >> b) & 1u));
}
static uint32_t motor_cc(int16_t permille) {
    const uint32_t mag = (uint32_t)(permille < 0 ? -permille : permille);
    const uint32_t level = ((MOTOR_WRAP + 1u) * mag) / 1000u;
    /* GP8/10 = channel A forward; GP9/11 = channel B reverse. */
    return permille > 0 ? level : level << 16u;
}
void drv_motor_hw_write(const int16_t duty[2], uint16_t servo_us) {
    pwm_hw->slice[4].cc = motor_cc(duty[0]);
    pwm_hw->slice[5].cc = motor_cc(duty[1]);
    if (servo_us != last_servo) {
        pwm_hw->slice[7].cc = (uint32_t)servo_us << 16u;
        last_servo = servo_us;
    }
}
void drv_motor_hw_stop(void) {
    /* Update both channels together; the next 20 kHz wrap applies zero. */
    pwm_hw->slice[4].cc = 0u;
    pwm_hw->slice[5].cc = 0u;
}
void drv_motor_hw_buttons_init(void) { pin(20u, 5u, true); pin(21u, 5u, true); }
bool drv_motor_hw_button_down(unsigned int gpio) {
    return (sio_hw->gpio_in & (1u << gpio)) == 0u;
}

/* Stage 0 only: no physical PWM, encoders or buttons. Never flash. */
#include "drv_motor_hw.h"
#include "../sim/pil_stage0.h"

void drv_motor_hw_init(uint16_t servo_us)
{
    (void)servo_us;
}

uint8_t drv_motor_hw_ab(unsigned int wheel)
{
    (void)wheel;
    return 0u;
}

void drv_motor_hw_write(const int16_t duty[2], uint16_t servo_us)
{
    (void)duty;
    (void)servo_us;
}

void drv_motor_hw_stop(void)
{
}

void drv_motor_hw_buttons_init(void)
{
}

bool drv_motor_hw_button_down(unsigned int pin)
{
    (void)pin;
    return false;
}

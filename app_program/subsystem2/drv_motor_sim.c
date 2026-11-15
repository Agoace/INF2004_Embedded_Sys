/* Stage 0 only: no physical PWM, encoders or buttons. Never flash. */
#include "drv_motor_hw.h"
#include "../sim/pil_stage0.h"
#if APP_SIM_GATE1
#include "../sim/pil_gate1_runtime.h"
#endif

void drv_motor_hw_init(uint16_t servo_us)
{
    (void)servo_us;
#if APP_SIM_GATE1
    pil_gate1_motor_seen(0, 0, servo_us);
#endif
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
#if APP_SIM_GATE1
    pil_gate1_motor_seen(duty[0], duty[1], servo_us);
#endif
}

void drv_motor_hw_stop(void)
{
#if APP_SIM_GATE1
    pil_gate1_motor_stopped();
#endif
}

void drv_motor_hw_buttons_init(void)
{
}

bool drv_motor_hw_button_down(unsigned int pin)
{
    (void)pin;
    return false;
}

/* Replaces only drv_motor_hw.c. No GPIO/PWM peripheral output. */
#if defined(BENCH_KERNEL) && BENCH_KERNEL
#include "../subsystem2/drv_motor_hw.h"
#include "bench_gpio.h"
#include "bench_motor.h"
#include "bench_runtime.h"
static uint32_t compares;
static uint32_t servo;
void drv_motor_hw_init(uint16_t servo_us)
{
    __atomic_store_n(&compares, 0u, __ATOMIC_RELAXED);
    __atomic_store_n(&servo, servo_us, __ATOMIC_RELAXED);
}
uint8_t drv_motor_hw_ab(unsigned int wheel) { return bk_gpio_ab(wheel); }
static uint16_t compare(int16_t duty)
{
    const int32_t d = duty;
    const uint32_t mag = (uint32_t)(d < 0 ? -d : d);
    const int32_t level = (int32_t)(6250u * mag / 1000u);
    return (uint16_t)(d < 0 ? -level : level);
}
void drv_motor_hw_write(const int16_t duty[2], uint16_t servo_us)
{
    const uint32_t packed = (uint32_t)compare(duty[0]) |
                            ((uint32_t)compare(duty[1]) << 16u);
    __atomic_store_n(&compares, packed, __ATOMIC_RELEASE);
    __atomic_store_n(&servo, servo_us, __ATOMIC_RELEASE);
}
void drv_motor_hw_stop(void)
{
    __atomic_store_n(&compares, 0u, __ATOMIC_RELEASE);
}
void drv_motor_hw_buttons_init(void) { }
bool drv_motor_hw_button_down(unsigned int pin)
{
    return bk_runtime_button(pin);
}
uint32_t bk_motor_compares(void)
{
    return __atomic_load_n(&compares, __ATOMIC_ACQUIRE);
}
uint16_t bk_motor_servo(void)
{
    return (uint16_t)__atomic_load_n(&servo, __ATOMIC_ACQUIRE);
}
#endif

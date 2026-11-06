#ifndef DRV_MOTOR_HW_H
#define DRV_MOTOR_HW_H
#include <stdbool.h>
#include <stdint.h>
#define DRV_MOTOR_ENCODER_MASK ((1u << 16u) | (1u << 17u) | (1u << 4u) | (1u << 5u))
void drv_motor_hw_init(uint16_t servo_us);
uint8_t drv_motor_hw_ab(unsigned int wheel);
void drv_motor_hw_write(const int16_t duty[2], uint16_t servo_us);
void drv_motor_hw_stop(void);
void drv_motor_hw_buttons_init(void);
bool drv_motor_hw_button_down(unsigned int pin);
#endif

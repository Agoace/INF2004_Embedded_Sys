#ifndef BENCH_KERNEL_MOTOR_H
#define BENCH_KERNEL_MOTOR_H
#include <stdint.h>
/* Observed signed20kHz PWM compares, packed L low16/R high16. No pin write. */
uint32_t bk_motor_compares(void);
uint16_t bk_motor_servo(void);
#endif

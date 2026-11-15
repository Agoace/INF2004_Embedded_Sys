#ifndef BENCH_KERNEL_TIMER_HW_H
#define BENCH_KERNEL_TIMER_HW_H
#include <stdbool.h>
#include <stdint.h>
uint32_t bk_timer_now(void);
void bk_line_timer_init(void);
void bk_line_timer_alarm(uint32_t at_us);
void bk_line_timer_ack(void);
void bk_timer_ack(void);
bool bk_timer_arm(uint32_t at_us);
void bk_timer_enable(bool enable);
void bk_gpio_pend(void);
#endif

#ifndef BENCH_KERNEL_GENERATOR_H
#define BENCH_KERNEL_GENERATOR_H
#include <stdbool.h>
#include <stdint.h>
/* Core0-only; caller serializes IRQ access. No native counter assignment. */
bool bk_gen_start(uint32_t start_us, uint32_t duration_us);
bool bk_gen_active(void);
bool bk_gen_failed(void);
uint32_t bk_gen_next(void);
unsigned int bk_gen_tick(uint32_t now_us);
void bk_gen_account(uint32_t elapsed_us);
void bk_gen_abort(uint32_t now_us);
void bk_gen_publish(void);
#endif

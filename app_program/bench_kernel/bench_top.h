/* Bench top-20 values per P1 counter (--wrap=rt_call/rt_max, report only). */
#ifndef BENCH_KERNEL_TOP_H
#define BENCH_KERNEL_TOP_H
#include <tk/tkernel.h>
#include "../platform/run_timing.h"

#define BK_TOP_N 20u

void __wrap_rt_call(rt_call_t which, uint32_t us);
void __wrap_rt_max(rt_max_t which, uint32_t us);
void __real_rt_call(rt_call_t which, uint32_t us);
void __real_rt_max(rt_max_t which, uint32_t us);

/* Print one [bench_top] line per counter after the window closes; pause
 * after each line so the console ring can drain. */
void bk_top_report(void (*pause)(void));

#endif

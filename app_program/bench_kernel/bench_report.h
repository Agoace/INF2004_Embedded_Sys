/* Bench step 0: [bench] line after the GP20 timeline (BENCH_KERNEL=1). */
#ifndef BENCH_KERNEL_REPORT_H
#define BENCH_KERNEL_REPORT_H
#include "bench_stats.h"

/* Generator cost limit for step 0: 3 % of core 0, in basis points. */
#define BENCH_COST_LIMIT_BP 300u
/* Longest valid window; a longer one may hide a 32-bit us wrap. */
#define BENCH_WINDOW_MAX_US 3600000000u

/* Link with -Wl,--wrap=rt_timeline_report (BENCH_KERNEL=1 only). */
void __real_rt_timeline_report(void (*pause)(void));
void __wrap_rt_timeline_report(void (*pause)(void));
#endif

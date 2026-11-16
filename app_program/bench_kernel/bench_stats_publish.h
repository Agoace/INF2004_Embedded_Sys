#ifndef BENCH_KERNEL_STATS_PUBLISH_H
#define BENCH_KERNEL_STATS_PUBLISH_H
#include "bench_stats.h"
/* Sole core0 writer, non-reentrant. False invalidates the old snapshot. */
bool bench_stats_publish(const bench_stats_t *in);
#endif

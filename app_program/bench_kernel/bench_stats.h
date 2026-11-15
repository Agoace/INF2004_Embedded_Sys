#ifndef BENCH_KERNEL_STATS_H
#define BENCH_KERNEL_STATS_H
#include <stdbool.h>
#include <stdint.h>
/* 10-minute step0 only; maximum accepted measurement window is 60 minutes. */
typedef struct {
    uint32_t edges_sched;
    uint32_t edges_delivered;
    uint32_t sched_left, sched_right;
    uint32_t ack_left, ack_right;
    uint32_t coalesced;
    uint32_t overflow;
    uint32_t gen_us_sum;
    uint32_t gen_us_max;
    uint32_t gen_calls;
    uint32_t window_start_us;
    uint32_t window_end_us;
    uint32_t stopped; /* 0 running, 1 stopped/drained; end_us may wrap to0. */
    uint32_t stop_late_us;
    uint32_t stop_position_us; /* From window start;0 if no generator fault. */
} bench_stats_t;
/* Called on physical core0 only, after stopping/draining or under IRQ mask.
 * Cross-core reporting must obtain its snapshot on core0, then publish it.
 * No core1 local IRQ mask can make this core0-owned state consistent. */
void bench_stats_get(bench_stats_t *out);
/* Any-core, bounded single attempt. False leaves out unchanged if there is
 * no published snapshot, or if publication overlaps the read. No busy wait. */
bool bench_stats_read(bench_stats_t *out);
#endif

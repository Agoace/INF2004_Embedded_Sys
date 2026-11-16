#include <assert.h>
#include <stdint.h>
#include "../app_program/bench_kernel/bench_stats.h"
#include "../app_program/bench_kernel/bench_stats_publish.h"
#ifdef BENCH_STATS_THREADS
#include <pthread.h>
static uint32_t done;
static void *publish_many(void *unused)
{
    (void)unused;
    for (uint32_t i = 1u; i <= 100000u; ++i) {
        const bench_stats_t row = {.edges_sched = i,
            .edges_delivered = ~i, .gen_calls = i ^ 0x12345678u,
            .window_end_us = i, .stopped = i & 1u};
        assert(bench_stats_publish(&row));
    }
    __atomic_store_n(&done, 1u, __ATOMIC_RELEASE);
    return 0;
}
static void concurrent_snapshots(void)
{
    pthread_t writer;
    assert(pthread_create(&writer, 0, publish_many, 0) == 0);
    do {
        bench_stats_t row;
        if (bench_stats_read(&row)) {
            assert(row.edges_delivered == ~row.edges_sched);
            assert(row.gen_calls == (row.edges_sched ^ 0x12345678u));
            assert(row.window_end_us == row.edges_sched);
            assert(row.stopped == (row.edges_sched & 1u));
        }
    } while (__atomic_load_n(&done, __ATOMIC_ACQUIRE) == 0u);
    assert(pthread_join(writer, 0) == 0);
}
#endif
int main(void)
{
    bench_stats_t out = {.edges_sched = 99u};
    assert(!bench_stats_read(&out) && out.edges_sched == 99u);
    assert(!bench_stats_read(0));
    bench_stats_t in = {.edges_sched = 123u, .edges_delivered = 120u,
        .overflow = 3u, .gen_us_sum = 42u, .gen_us_max = 3u,
        .gen_calls = 100u, .window_start_us = UINT32_MAX - 99u,
        .window_end_us = 100u};
    assert(bench_stats_publish(&in));
    assert(bench_stats_read(&out));
    assert(out.edges_sched == 123u && out.edges_delivered == 120u);
    assert(out.overflow == 3u && out.gen_us_sum == 42u);
    assert(out.window_start_us == UINT32_MAX - 99u);
    assert(out.window_end_us == 100u);
    assert(out.stopped == 0u);
    in.stopped = 1u;
    in.window_start_us = UINT32_MAX;
    in.window_end_us = 0u; /* Zero is a valid stopped end timestamp. */
    assert(bench_stats_publish(&in));
    assert(bench_stats_read(&out) && out.stopped == 1u);
    assert(out.window_end_us == 0u);
    in.window_start_us = 0u;
    in.window_end_us = 3600000000u; /* Exactly60min remains accepted. */
    assert(bench_stats_publish(&in));
    assert(bench_stats_read(&out));
    in.window_end_us = 3600000001u;
    assert(!bench_stats_publish(&in));
    const bench_stats_t saved = out;
    assert(!bench_stats_read(&out)); /* Never report stale valid window. */
    assert(out.edges_sched == saved.edges_sched);
    assert(out.window_end_us == saved.window_end_us);
    in.window_end_us = 10u;
    in.stopped = 2u;
    assert(!bench_stats_publish(&in));
    assert(!bench_stats_publish(0));
    assert(!bench_stats_read(&out));
#ifdef BENCH_STATS_THREADS
    concurrent_snapshots();
#endif
    return 0;
}

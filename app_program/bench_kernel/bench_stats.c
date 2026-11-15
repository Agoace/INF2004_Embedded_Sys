/* Bounded cross-core publication. Atomic32 operations need no M0+ locks. */
#if defined(BENCH_KERNEL) && BENCH_KERNEL
#include "bench_stats_publish.h"
#include <stddef.h>
#define FIELD_LIST(X) \
    X(edges_sched) X(edges_delivered) X(coalesced) X(overflow) \
    X(gen_us_sum) X(gen_us_max) X(gen_calls) \
    X(window_start_us) X(window_end_us) X(stopped) \
    X(stop_late_us) X(stop_position_us)
static uint32_t sequence;
static uint32_t valid;
static bench_stats_t published;

bool bench_stats_publish(const bench_stats_t *in)
{
    const uint32_t old = __atomic_load_n(&sequence, __ATOMIC_RELAXED);
    __atomic_store_n(&sequence, old + 1u, __ATOMIC_RELAXED);
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    const bool ok = in != NULL && in->stopped <= 1u &&
        in->window_end_us - in->window_start_us <= 3600000000u;
    if (ok) {
#define STORE(f) __atomic_store_n(&published.f, in->f, __ATOMIC_RELAXED);
        FIELD_LIST(STORE)
#undef STORE
    }
    __atomic_store_n(&valid, ok ? 1u : 0u, __ATOMIC_RELAXED);
    __atomic_store_n(&sequence, old + 2u, __ATOMIC_RELEASE);
    return ok;
}
bool bench_stats_read(bench_stats_t *out)
{
    if (out == NULL) { return false; }
    const uint32_t before = __atomic_load_n(&sequence, __ATOMIC_ACQUIRE);
    if ((before & 1u) != 0u) { return false; }
    const uint32_t seen = __atomic_load_n(&valid, __ATOMIC_RELAXED);
    bench_stats_t copy;
#define LOAD(f) copy.f = __atomic_load_n(&published.f, __ATOMIC_RELAXED);
    FIELD_LIST(LOAD)
#undef LOAD
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    const uint32_t after = __atomic_load_n(&sequence, __ATOMIC_RELAXED);
    if (seen == 0u || before != after) { return false; }
    *out = copy;
    return true;
}
#endif

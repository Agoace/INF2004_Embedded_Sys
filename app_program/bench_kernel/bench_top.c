/*
 * Bench top-20 values per P1 counter. --wrap=rt_call and --wrap=rt_max
 * forward every value to run_timing.c, then latch it here while the bench
 * window (P1 epoch 1, odd) is live. Each counter has one writer (one ISR or
 * one task), so no lock is needed; the report reads only after the window
 * closed (epoch even). Insert cost is one compare unless the value enters
 * the top 20.
 */
#if defined(BENCH_KERNEL) && BENCH_KERNEL
#include <tm/tmonitor.h>
#include "bench_top.h"

typedef struct {
    uint32_t top[BK_TOP_N]; /* descending; 0 = empty */
    uint32_t ge100;         /* all values >= 100 us in the window */
} bk_top_t;

static bk_top_t calls[RT_CALL_COUNT];
static bk_top_t maxes[RT_MAX_COUNT];

/* Same order as rt_call_t / rt_max_t (run_timing.h). */
static const char *const call_name[] = {
    "isr_wup", "line_wake", "line_can", "line_loc", "line_unl",
    "mot_wup", "veh_wake", "veh_can", "veh_status", "veh_send"};
static const char *const max_name[] = {
    "isr_late", "isr_adc", "isr_wup", "line_sample", "lp_update",
    "barcode", "wake", "read_line", "command", "sonar", "vsm", "log",
    "publish", "status"};
_Static_assert(sizeof(call_name) / sizeof(call_name[0]) ==
               (unsigned int)RT_CALL_COUNT, "call names");
_Static_assert(sizeof(max_name) / sizeof(max_name[0]) ==
               (unsigned int)RT_MAX_COUNT, "max names");

static int live(void)
{
    return (rt_epoch() & 1u) != 0u;
}

static void latch(bk_top_t *t, uint32_t us)
{
    if (us >= 100u) {
        ++t->ge100;
    }
    if (us <= t->top[BK_TOP_N - 1u]) {
        return;
    }
    unsigned int i = BK_TOP_N - 1u;
    while (i > 0u && t->top[i - 1u] < us) {
        t->top[i] = t->top[i - 1u];
        --i;
    }
    t->top[i] = us;
}

void __wrap_rt_call(rt_call_t which, uint32_t us)
{
    __real_rt_call(which, us);
    if (live() && (unsigned int)which < (unsigned int)RT_CALL_COUNT) {
        latch(&calls[which], us);
    }
}

void __wrap_rt_max(rt_max_t which, uint32_t us)
{
    __real_rt_max(which, us);
    if (live() && (unsigned int)which < (unsigned int)RT_MAX_COUNT) {
        latch(&maxes[which], us);
    }
}

static void print_one(const char *kind, const char *name, const bk_top_t *t,
                      void (*pause)(void))
{
    (void)tm_printf((UB *)"[bench_top] %s %s n_ge100 %lu top", kind, name,
                    (unsigned long)t->ge100);
    for (unsigned int i = 0u; i < BK_TOP_N && t->top[i] != 0u; ++i) {
        (void)tm_printf((UB *)" %lu", (unsigned long)t->top[i]);
    }
    (void)tm_printf((UB *)"\n");
    if (pause != 0) {
        pause();
    }
}

void bk_top_report(void (*pause)(void))
{
    const uint32_t e = rt_epoch();
    if (e == 0u || (e & 1u) != 0u) {
        (void)tm_printf((UB *)"[bench_top] window open\n");
        return;
    }
    for (unsigned int i = 0u; i < (unsigned int)RT_CALL_COUNT; ++i) {
        print_one("call", call_name[i], &calls[i], pause);
    }
    for (unsigned int i = 0u; i < (unsigned int)RT_MAX_COUNT; ++i) {
        print_one("max", max_name[i], &maxes[i], pause);
    }
}
#endif

/* On-chip runtime source with host timer/NVIC doubles, not an IRQ emulator. */
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <tk/tkernel.h>
#include "../app_program/bench_kernel/bench_gpio.h"
#include "../app_program/bench_kernel/bench_generator.h"
#include "../app_program/bench_kernel/bench_runtime.h"
#include "../app_program/bench_kernel/bench_stats.h"
#include "../app_program/platform/run_timing.h"
static FP handler;
static uint32_t now_us, target;
static unsigned int pends, acks, enabled, startup, reg_calls;
static int irq_context, past_arm;
void __wrap_app_startup_complete_publish(void);
void __real_app_startup_complete_publish(void) { ++startup; }
uint32_t drv_time_us(void) { return now_us++; }
INT tk_get_prc(void) { assert(!irq_context); return 1; }
void DisableInt(UINT irq) { assert(!irq_context && irq == 1u); }
void ClearInt(UINT irq) { assert(!irq_context && irq == 1u); }
void EnableInt(UINT irq, INT level)
{
    assert(!irq_context && irq == 1u && level == 1); ++enabled;
}
ER tk_def_int(UINT irq, const T_DINT *d)
{
    assert(!irq_context && irq == 1u && d->intatr == TA_HLNG);
    assert(startup == 1u);
    ++reg_calls; handler = d->inthdr; return E_OK;
}
void bk_timer_ack(void) { assert(irq_context); ++acks; }
bool bk_timer_arm(uint32_t at)
{
    if (past_arm) { now_us = at + 7u; return false; }
    assert((int32_t)(at - now_us) > 0);
    target = at; return true;
}
void bk_timer_enable(bool on) { (void)on; }
void bk_gpio_pend(void) { assert(irq_context); ++pends; }
/* P1 run epoch for the bench window: task context, open 1 then close 2.
 * The idle meter is started once and sampled just before each mark, so
 * [load] covers the same window as [kcall]. */
static uint32_t rt_epochs[4];
static unsigned int rt_begins, load_starts, samples, samples_at_mark[4];
void bk_load_start(void)
{
    assert(!irq_context && rt_begins == 0u); ++load_starts;
}
void idle_meter_sample_all(void)
{
    assert(!irq_context && load_starts == 1u); ++samples;
}
void rt_begin(uint32_t epoch)
{
    assert(!irq_context && rt_begins < 4u);
    samples_at_mark[rt_begins] = samples;
    rt_epochs[rt_begins++] = epoch;
}
void rt_tl(rt_tl_site_t site, uint32_t us, uint32_t arg)
{
    (void)us; (void)arg;
    assert(irq_context && (site == (rt_tl_site_t)10 ||
                          site == (rt_tl_site_t)11));
}
int main(int argc, char **argv)
{
    const int miss = argc > 1 && strcmp(argv[1], "past") == 0;
    const int late_poll = argc > 1 && strcmp(argv[1], "late_poll") == 0;
    bk_gpio_reset();
    bk_gpio_enable((1u << 16) | (1u << 17) | (1u << 4) | (1u << 5));
    assert(!bk_runtime_button(20u));
    __wrap_app_startup_complete_publish();
    assert(enabled == 1u && reg_calls == 1u && handler);
    assert(rt_begins == 1u && rt_epochs[0] == 1u);
    assert(load_starts == 1u && samples_at_mark[0] == 1u);
    past_arm = miss;
    bench_stats_t s;
    assert(bench_stats_read(&s) && !s.stopped);
    do {
        now_us = target;
        irq_context = 1;
        ((void (*)(UINT))handler)(1u);
        irq_context = 0;
        for (unsigned int pin = 0u; pin < 30u; ++pin) {
            bk_gpio_ack(pin, 12u);
        }
        assert(acks <= 2280001u);
        if (!bk_gen_active()) {
            assert(bench_stats_read(&s) && !s.stopped);
            now_us += late_poll ? 500000u : 20000u;
            const bool down = bk_runtime_button(20u);
            assert(down == (late_poll != 0));
            /* Even a first poll after500ms publishes BEFORE visible press. */
            assert(bench_stats_read(&s) && s.stopped);
            /* The P1 epoch closes with the stopped publication. */
            assert(rt_begins == 2u && rt_epochs[1] == 2u);
            assert(samples_at_mark[1] == 2u);
        }
        assert(bench_stats_read(&s));
    } while (!s.stopped);
    if (miss) {
        assert(s.overflow == 1u && s.stop_late_us >= 7u);
        assert(s.stop_position_us > 0u);
    } else {
        assert(s.edges_sched == 2280000u && s.edges_delivered == 2280000u);
        assert(s.sched_left == 1140000u && s.sched_right == 1140000u);
        assert(s.ack_left == 1140000u && s.ack_right == 1140000u);
        assert(pends == 2280000u && acks == 2280001u);
        assert(s.gen_calls == acks && !s.overflow && !s.coalesced);
        assert(s.window_end_us - s.window_start_us == 600000000u);
    }
    if (late_poll) {
        assert(bk_runtime_button(20u));
        now_us += 200001u;
        assert(!bk_runtime_button(20u));
        return 0;
    }
    assert(!bk_runtime_button(20u) && !bk_runtime_button(21u));
    assert(rt_begins == 2u && samples == 2u); /* closed once only */
    now_us += 500000u;
    assert(bk_runtime_button(20u));
    assert(!bk_runtime_button(21u));
    now_us += 200001u;
    assert(!bk_runtime_button(20u));
    return 0;
}

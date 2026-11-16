#include <assert.h>
#include <stdint.h>
#include "../app_program/bench_kernel/bench_generator.h"
#include "../app_program/bench_kernel/bench_gpio.h"
#include "../app_program/bench_kernel/bench_stats.h"
static void drain(void)
{
    for (unsigned int pin = 0u; pin < 30u; ++pin) {
        bk_gpio_ack(pin, 12u);
    }
}
int main(void)
{
    bk_gpio_reset();
    bk_gpio_enable((1u << 16) | (1u << 17) | (1u << 4) | (1u << 5));
    const uint32_t start = UINT32_MAX - 500u;
    assert(bk_gen_start(start, 10000u));
    bench_stats_t s;
    assert(bench_stats_read(&s) && !s.stopped);
    unsigned int ticks = 0u;
    while (bk_gen_active()) {
        const uint32_t at = bk_gen_next();
        const unsigned int edges = bk_gen_tick(at);
        assert(edges <= 1u);
        if (edges != 0u) { ++ticks; }
        drain();
    }
    bk_gen_publish();
    assert(ticks == 38u); /* Alternate wheels:3800IRQs/s,1900edges/wheel. */
    assert(bench_stats_read(&s) && s.stopped);
    assert(s.edges_sched == 38u && s.edges_delivered == 38u);
    assert(s.overflow == 0u && s.coalesced == 0u);
    assert(s.window_start_us == start && s.window_end_us == start + 10000u);
    assert(!bk_gen_start(0u, 3600000001u));
    bk_gpio_reset();
    bk_gpio_enable((1u << 16) | (1u << 17) | (1u << 4) | (1u << 5));
    assert(bk_gen_start(100u, 10000u));
    assert(bk_gen_tick(bk_gen_next()) == 1u);
    assert(bk_gen_tick(bk_gen_next()) == 1u); /* Other wheel. */
    /* Fail closed instead of coalescing a second edge into a pending wheel. */
    assert(bk_gen_tick(bk_gen_next()) == 0u);
    assert(!bk_gen_active());
    bk_gen_publish();
    assert(bench_stats_read(&s) && s.stopped && s.overflow != 0u);
    assert(s.stop_position_us != 0u);
    bk_gpio_reset();
    bk_gpio_enable((1u << 16) | (1u << 17) | (1u << 4) | (1u << 5));
    assert(bk_gen_start(100u, 10000u));
    const uint32_t late = bk_gen_next() + 263u;
    assert(bk_gen_tick(late) == 0u && !bk_gen_active());
    bk_gen_publish();
    assert(bench_stats_read(&s) && s.stop_late_us == 263u);
    assert(s.stop_position_us == late - 100u);
    assert(s.window_end_us == late);
    return 0;
}

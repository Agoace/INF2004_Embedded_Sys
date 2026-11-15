/* Step0: independent encoder source, alternating3800 events/s. */
#if defined(BENCH_KERNEL) && BENCH_KERNEL
#include "bench_generator.h"
#include "bench_gpio.h"
#include "bench_stats_publish.h"
static struct {
    bench_stats_t stats;
    uint32_t next_us, end_us;
    unsigned int fraction, wheel, phase[2];
    bool active, draining;
} g;
static const uint8_t states[2][4] = {{2u, 3u, 1u, 0u}, {1u, 3u, 2u, 0u}};

static void advance(void)
{
    g.next_us += 263u;
    g.fraction += 3u;
    if (g.fraction >= 19u) { g.fraction -= 19u; ++g.next_us; }
}
bool bk_gen_start(uint32_t start_us, uint32_t duration_us)
{
    if (g.active || duration_us < 1000u || duration_us > 3600000000u) {
        return false;
    }
    g.stats = (bench_stats_t){.window_start_us = start_us,
        .window_end_us = start_us + duration_us};
    g.next_us = start_us;
    g.end_us = start_us + duration_us;
    g.fraction = 0u; g.wheel = 0u;
    g.phase[0] = 0u; g.phase[1] = 0u;
    g.draining = false; g.active = true;
    advance();
    return bench_stats_publish(&g.stats);
}
bool bk_gen_active(void) { return g.active; }
bool bk_gen_failed(void) { return g.stats.overflow != 0u; }
uint32_t bk_gen_next(void) { return g.next_us; }
__attribute__((noinline)) void bk_gen_abort(uint32_t now_us)
{
    g.stats.stop_late_us = (int32_t)(now_us - g.next_us) > 0 ?
                           now_us - g.next_us : 0u;
    g.stats.stop_position_us = now_us - g.stats.window_start_us;
    g.stats.window_end_us = now_us;
    ++g.stats.overflow;
    g.active = false;
    g.stats.stopped = 1u;
}
unsigned int bk_gen_tick(uint32_t now_us)
{
    if (!g.active) { return 0u; }
    if ((int32_t)(now_us - g.next_us) < 0) { return 0u; }
    if (g.draining) {
        g.stats.edges_delivered = bk_gpio_acked();
        if (g.stats.edges_delivered != g.stats.edges_sched) {
            bk_gen_abort(now_us);
            return 0u;
        }
        g.active = false;
        g.stats.stopped = 1u;
        return 0u;
    }
    ++g.stats.edges_sched;
    /* Missing a source opportunity is fatal; never replay/catch up. */
    if (now_us - g.next_us >= 263u) { bk_gen_abort(now_us); return 0u; }
    if (!bk_gpio_edge(g.wheel, states[g.wheel][g.phase[g.wheel]])) {
        ++g.stats.coalesced;
        bk_gen_abort(now_us);
        return 0u;
    }
    g.phase[g.wheel] = (g.phase[g.wheel] + 1u) & 3u;
    g.wheel ^= 1u;
    advance();
    /* Unsigned elapsed time supports windows beyond signed half-range. */
    if (g.next_us - g.stats.window_start_us >
        g.end_us - g.stats.window_start_us) {
        g.draining = true;
        g.next_us = g.end_us + 1000u;
    }
    return 1u;
}
void bk_gen_account(uint32_t elapsed_us)
{
    ++g.stats.gen_calls;
    g.stats.gen_us_sum += elapsed_us;
    if (elapsed_us > g.stats.gen_us_max) { g.stats.gen_us_max = elapsed_us; }
}
void bench_stats_get(bench_stats_t *out)
{
    if (out != 0) {
        g.stats.edges_delivered = bk_gpio_acked();
        *out = g.stats;
    }
}
void bk_gen_publish(void)
{
    g.stats.edges_delivered = bk_gpio_acked();
    (void)bench_stats_publish(&g.stats);
}
#endif

/* IMU INT1 wake policy and host SIL timeline (no hardware, no kernel).
 * The SIL models the accel FIFO, the INT1 watermark level, the core-0
 * GPIO edge interrupt, the 10 ms cyclic fallback, task wake latency and
 * the unchanged probe rules (read all n<=8; n>8 = backlog reset). */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define IMU_INT1_WAKE 1
#include "../app_program/subsystem4/imu_wake.h"

static void test_policy(void)
{
    imu_wake_t w;
    memset(&w, 0, sizeof(w));
    assert(IMU_WAKE_CTRL3 == 0x04u && IMU_WAKE_FIFO_CTRL == 0x83u);
    /* No edge yet: every tick is a fallback poll (today's cadence). */
    assert(imu_wake_tick(&w, 1000u) && imu_wake_tick(&w, 11000u));
    assert(!imu_wake_edge(&w, IMU_WAKE_EDGE_FALL, 12000u));
    assert(w.falls == 1u && w.rises == 0u && w.have_edge == 0u);
    assert(imu_wake_edge(&w, IMU_WAKE_EDGE_RISE, 20000u));
    assert(imu_wake_tick(&w, 20000u + IMU_WAKE_FALLBACK_US + 1u));
    assert(!imu_wake_edge(&w, 0u, 1u) && w.rises == 1u);
    assert(imu_wake_edge(&w, IMU_WAKE_EDGE_RISE | IMU_WAKE_EDGE_FALL, 40000u));
    assert(w.rises == 2u && w.falls == 2u);
    assert(!imu_wake_tick(&w, 40000u + IMU_WAKE_FALLBACK_US));
    assert(imu_wake_tick(&w, 40000u + IMU_WAKE_FALLBACK_US + 1u));
    assert(w.ticks == 5u && w.fallbacks == 4u);
    /* Microsecond wrap. */
    assert(imu_wake_edge(&w, IMU_WAKE_EDGE_RISE, UINT32_MAX - 3000u));
    assert(!imu_wake_tick(&w, 5000u));
    assert(imu_wake_tick(&w, 9001u));
    /* After-sample: histogram, invalid skipped, one bounded extra pass. */
    assert(!imu_wake_after(&w, 4u, true, false, false));
    assert(imu_wake_after(&w, 4u, true, true, false));
    assert(!imu_wake_after(&w, 0u, true, true, true));
    assert(!imu_wake_after(&w, IMU_WAKE_HISTOGRAM, false, true, false));
    assert(w.batches[4] == 2u && w.batches[0] == 1u && w.extra == 1u);
    imu_wake_t copy;
    imu_wake_copy(&copy, &w);
    assert(memcmp(&copy, &w, sizeof(w)) == 0);
}

typedef struct {
    unsigned int period_us, fth, max_latency_us, kill_at_ms, end_ms;
    bool at_least; /* WTM when count >= FTH (else count > FTH). */
} sil_t;
typedef struct {
    unsigned int max_n, backlogs, empties, fallbacks, batches, extra;
    unsigned int max_gap_us, max_age_us, first_fallback_after_kill_us;
    unsigned int fallbacks_before_kill, backlogs_before_kill, max_n_before_kill;
    unsigned int histogram[IMU_WAKE_HISTOGRAM];
    uint64_t samples;
} sil_result_t;

static uint32_t seed = 12345u;
static unsigned int rnd(unsigned int limit)
{
    seed = seed * 1103515245u + 12345u;
    return limit == 0u ? 0u : (seed >> 8) % (limit + 1u);
}

static sil_result_t simulate(const sil_t *s)
{
    sil_result_t r;
    memset(&r, 0, sizeof(r));
    imu_wake_t w;
    memset(&w, 0, sizeof(w));
    unsigned int count = 0u;
    bool level = false, pending = false, busy = false, killed = false;
    uint64_t next_sample = 1000u, next_tick = 10000u, start_at = 0u;
    uint64_t last_obs = 0u, last_edge = 0u, oldest_unread = 0u;
    const uint64_t end = (uint64_t)s->end_ms * 1000u;
    const unsigned int read_us = 1300u;

    for (uint64_t t = 0u; t < end; ++t) {
        if (t == next_sample) {
            next_sample += s->period_us;
            if (count == 0u) { oldest_unread = t; }
            if (count < 32u) { ++count; }
            ++r.samples;
        }
        const bool high = s->at_least ? count >= s->fth : count > s->fth;
        if (!killed && s->kill_at_ms != 0u && t >= (uint64_t)s->kill_at_ms * 1000u) {
            killed = true;
        }
        if (high && !level && !killed) {
            if (imu_wake_edge(&w, IMU_WAKE_EDGE_RISE, (uint32_t)t)) { pending = true; }
            last_edge = t;
        } else if (!high && level && !killed) {
            (void)imu_wake_edge(&w, IMU_WAKE_EDGE_FALL, (uint32_t)t);
        }
        level = high;
        if (t == next_tick) {
            next_tick += 10000u;
            if (imu_wake_tick(&w, (uint32_t)t)) {
                pending = true;
                if (killed && r.first_fallback_after_kill_us == 0u) {
                    r.first_fallback_after_kill_us = (unsigned int)(t - last_edge);
                }
            }
        }
        if (!busy && pending) {
            pending = false; /* tk_can_wup: queued extras are cancelled. */
            busy = true;
            start_at = t + rnd(s->max_latency_us);
        }
        if (busy && t == start_at) {
            bool extra_done = false;
            for (;;) {
                const unsigned int n = count;
                unsigned int hist = n;
                if (n > 8u) {
                    ++r.backlogs; count = 0u; hist = IMU_WAKE_HISTOGRAM;
                } else {
                    if (n == 0u) { ++r.empties; } else { ++r.batches; }
                    if (n > r.max_n) { r.max_n = n; }
                    if (n > 0u) {
                        const unsigned int age = (unsigned int)(t - oldest_unread);
                        if (age > r.max_age_us) { r.max_age_us = age; }
                    }
                    count -= n;
                    if (n > 0u && last_obs != 0u &&
                        t - last_obs > r.max_gap_us) {
                        r.max_gap_us = (unsigned int)(t - last_obs);
                    }
                    if (n > 0u) { last_obs = t; }
                }
                /* Samples arriving during the 1.3 ms burst stay in FIFO. */
                for (uint64_t u = t; u < t + read_us; ++u) {
                    if (u == next_sample) {
                        next_sample += s->period_us;
                        if (count == 0u) { oldest_unread = u; }
                        ++count; ++r.samples;
                    }
                }
                t += read_us;
                const bool now_high = s->at_least ? count >= s->fth : count > s->fth;
                if (now_high && !level && !killed) {
                    pending = true;
                    (void)imu_wake_edge(&w, IMU_WAKE_EDGE_RISE, (uint32_t)t);
                    last_edge = t;
                }
                level = now_high;
                if (!imu_wake_after(&w, hist, true, now_high, extra_done)) {
                    break;
                }
                extra_done = true;
            }
            busy = false;
            while (next_tick <= t) {
                next_tick += 10000u; /* Ticks during the read: handler ran. */
                if (imu_wake_tick(&w, (uint32_t)t)) { pending = true; }
            }
        }
        if (!killed) {
            r.fallbacks_before_kill = w.fallbacks;
            r.backlogs_before_kill = r.backlogs;
            r.max_n_before_kill = r.max_n;
        }
    }
    r.fallbacks = w.fallbacks;
    r.extra = w.extra;
    memcpy(r.histogram, w.batches, sizeof(r.histogram));
    return r;
}

static void report(const char *name, const sil_result_t *r)
{
    printf("%s: batches %u maxn %u backlog %u empty %u fallback %u "
           "(before kill %u) extra %u maxgap %u us maxage %u us "
           "kill->fallback %u us hist", name, r->batches, r->max_n,
           r->backlogs, r->empties, r->fallbacks, r->fallbacks_before_kill,
           r->extra, r->max_gap_us, r->max_age_us,
           r->first_fallback_after_kill_us);
    for (unsigned int i = 0u; i < IMU_WAKE_HISTOGRAM; ++i) {
        printf(" %u", r->histogram[i]);
    }
    printf("\n");
}

int main(void)
{
    test_policy();
    const unsigned int periods[] = {2557u, 2500u, 2445u}; /* 391/400/409 Hz */
    for (unsigned int p = 0u; p < 3u; ++p) {
        for (unsigned int ge = 0u; ge < 2u; ++ge) {
            /* Normal INT1, FTH 3, both WTM semantics, 0..6 ms latency. */
            sil_t s = {periods[p], 3u, 6000u, 0u, 6000u, ge != 0u};
            sil_result_t r = simulate(&s);
            report(ge ? "fth3_ge" : "fth3_gt", &r);
            assert(r.backlogs == 0u && r.fallbacks_before_kill <= 1u);
            assert(r.max_n <= 8u && r.max_gap_us <= 20000u);
            assert(r.max_age_us < 30000u);
            assert(r.histogram[ge ? 3u : 4u] * 10u >= r.batches * 7u);
            /* INT1 dies at 3 s: today's polling continues, no freeze. */
            sil_t k = {periods[p], 3u, 6000u, 3000u, 6000u, ge != 0u};
            sil_result_t d = simulate(&k);
            report(ge ? "fth3_ge_dead" : "fth3_gt_dead", &d);
            assert(d.backlogs_before_kill == 0u);
            assert(d.first_fallback_after_kill_us > IMU_WAKE_FALLBACK_US &&
                   d.first_fallback_after_kill_us <= IMU_WAKE_FALLBACK_US + 10000u);
            assert(d.backlogs <= 1u && d.max_gap_us <= 30000u);
            assert(d.fallbacks >= 250u); /* ~3 s of 10 ms fallback polls. */
        }
    }
    /* Why not FTH 4: with ">" it would give n=5 (~12.8 ms): false
     * fallbacks every few cycles. FTH 3 never reaches the 12 ms rule. */
    sil_t bad = {2557u, 4u, 0u, 0u, 5000u, false};
    sil_result_t b = simulate(&bad);
    report("fth4_gt", &b);
    assert(b.fallbacks > 100u);
    puts("imu INT1 wake policy and SIL PASS");
    return 0;
}

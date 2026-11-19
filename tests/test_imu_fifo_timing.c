/* Option D timing fixtures (bench B1 model): the gap fixtures of
 * test_imu_fifo_gap.c with a timed I2C fake. Every bus byte costs 25 us
 * (400 kHz, 9 bits, plus gaps); each transfer adds 3 bytes of address and
 * register; compute (mean + decode) costs 330 + 10*(n-4) us, charged before
 * the probe's 2 ms deadline check; the line task preempts once per 1 ms
 * boundary for 132 us (UP line sample max, MEASUREMENTS). Model: read time
 * = 25*(3+1) + 25*(3+6n) + 25*(3+6) + 138 + 15n + 132 per 1 ms boundary.
 * Calibrated to the robot: n=4 gives 1330 us (robot mean 1315-1346); n=8
 * gives 1990-2122 us (robot B1 max 1996 and 2007). Requirement: 0 TIMEOUT
 * in every catch-up cycle.
 * Base: REAL imu_probe + imu_safety + ig_step(active=true)
 * over a time-based accel FIFO fake (32 deep, OVRN at 32 unread). The IMU
 * task reads every 10 ms, then one forced gap of G ms, then 10 ms again.
 * Safety input follows imu_service.c safety_tick: read_failed = !valid; an
 * EMPTY cycle is not an observation. The gate runs at every 10 ms tick.
 * Expect: G = 22/30/40/49 ms -> 0 READ_FAILED, 0 latch, no FIFO reset.
 *         G = 60 ms -> STALE stop (>50 ms age rule), as before option D.
 *         G = 80 ms -> OVRN reset (READ_FAILED) and a stop, as before. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../app_program/subsystem4/imu_probe.h"
#include "../app_program/subsystem4/imu_hw.h"
#include "../app_program/subsystem4/imu_safety.h"
#include "../app_program/vehicle/imu_gate.h"

static uint8_t regs[2][256];
static uint64_t now_us, cycle_start, sample_period_ns, produced, consumed;
static uint64_t next_sample_ns, next_preempt;
static bool ovrn;
static unsigned int burst_n;
static uint64_t phase_us; /* Line alarm phase offset, swept 0..950 us. */

#define BYTE_US 25u
#define PREEMPT_US 132u
static void advance(uint64_t us)
{
    now_us += us;
    while (now_us >= next_preempt) {
        next_preempt += 1000u;
        now_us += PREEMPT_US;
    }
}

static void produce(void)
{
    while (next_sample_ns <= now_us * 1000u) {
        next_sample_ns += sample_period_ns;
        if (produced - consumed < 32u) {
            ++produced;
        } else {
            ovrn = true;
            ++produced;
            ++consumed; /* Stream mode drops the oldest. */
        }
    }
}
uint32_t imu_hw_time_us(void) { return (uint32_t)now_us; }
int imu_hw_init(void) { return IMU_IO_OK; }
static bool cycle_reading;
void imu_hw_cycle_begin(void) { cycle_start = now_us; burst_n = 0u; cycle_reading = true; }
int imu_hw_cycle_end(void)
{
    /* Mean + decode ran before this call in imu_probe_read. */
    if (cycle_reading) {
        advance(138u + 15u * burst_n);
    }
    cycle_reading = false;
    return now_us - cycle_start >= 2000u ? IMU_IO_TIMEOUT : IMU_IO_OK;
}
int imu_hw_read(uint8_t address, uint8_t reg, uint8_t *out, size_t length)
{
    const unsigned int dev = address == IMU_ACCEL_ADDR ? 0u : 1u;

    advance(BYTE_US * (3u + (uint64_t)length));
    produce();
    if (dev == 0u && reg == 0x2fu) {
        const uint64_t n = produced - consumed;
        /* FSS = unread count; OVRN reports FSS=31 with bit 6. */
        *out = ovrn ? 0x5fu : n == 0u ? 0x20u : (uint8_t)n;
    } else if (dev == 0u && reg == 0xa8u) {
        const size_t n = length / 6u;

        assert(length % 6u == 0u && n <= produced - consumed && n <= 8u);
        burst_n = (unsigned int)n;
        for (size_t i = 0u; i < n; ++i) {
            const uint8_t s[6] = {0, 0, 0, 0, 0x80u, 0x3eu}; /* Z=16000 */
            memcpy(out + 6u * i, s, 6u);
            ++consumed; /* One sample per 6 bytes: FSS drops by n. */
        }
    } else if (dev == 1u && reg == 3u) {
        memset(out, 0, length);
    } else {
        memcpy(out, &regs[dev][reg], length);
    }
    return IMU_IO_OK;
}
int imu_hw_write(uint8_t address, uint8_t reg, uint8_t value)
{
    const unsigned int dev = address == IMU_ACCEL_ADDR ? 0u : 1u;

    advance(BYTE_US * 3u);
    regs[dev][reg] = value;
    if (dev == 0u && reg == 0x2eu && value == 0u) {
        consumed = produced; /* Bypass empties the FIFO. */
        ovrn = false;
    }
    return IMU_IO_OK;
}

typedef struct {
    unsigned int read_failed, latches, observations, partial;
    unsigned int resets, overruns, timeouts;
    uint32_t read_max_us, n6_us, n8_us;
    uint8_t first_cause;
} gap_result_t;

static gap_result_t run(unsigned int hz, unsigned int gap_ms)
{
    gap_result_t r;
    memset(&r, 0, sizeof(r));
    memset(regs, 0, sizeof(regs));
    regs[0][0x0f] = 0x33u;
    regs[1][0x0a] = 0x48u;
    regs[1][0x0b] = 0x34u;
    regs[1][0x0c] = 0x33u;
    now_us = 0u;
    produced = consumed = 0u;
    ovrn = false;
    sample_period_ns = 1000000000ull / hz;
    next_sample_ns = 1000000u;
    next_preempt = 1000u;
    imu_probe_t probe;
    imu_probe_init(&probe);
    imu_safety_t safety;
    imu_safety_init(&safety);
    ig_t gate;
    ig_init(&gate);
    imu_safety_report_t report;
    bool have = false;
    const uint64_t gap_start = 2000000u, gap_end = gap_start + gap_ms * 1000u;
    uint64_t next_read = 10000u;

    for (uint64_t tick = 10000u; tick < 4000000u; tick += 10000u) {
        /* The IMU task reads at its tick (+1 ms), except in the gap. */
        while (next_read <= tick) {
            if (next_read >= gap_start && next_read < gap_end) {
                next_read = gap_end;
                continue;
            }
            now_us = next_read;
            next_preempt = (now_us / 1000u + 1u) * 1000u - phase_us;
            imu_units_t out;
            const bool valid = imu_probe_read(&probe, (uint32_t)(now_us / 1000u), &out);
            const bool empty = !valid && probe.no_sample;
            if (probe.ready && !empty && (valid || probe.reads > 0u)) {
                imu_safety_input_t in = {
                    .now_ms = (uint32_t)(now_us / 1000u), .accel_valid = valid,
                    .calibrated = true, .wheels_still = true,
                    .read_failed = !valid};
                for (unsigned int a = 0u; a < 3u; ++a) {
                    in.accel_mg[a] = probe.mean_accel_mg[a];
                }
                imu_safety_step(&safety, &in);
                report.value = safety.output;
                report.applied_request = 0u;
                have = true;
                ++r.observations;
                r.read_failed += valid ? 0u : 1u;
            }
            next_read += 10000u;
        }
        /* Vehicle tick: the gate, as in a run (active = true). */
        /* The vehicle runs after the IMU read in this tick (age >= 0). */
        const uint64_t vt = now_us > tick ? now_us : tick;
        now_us = vt;
        if (have && tick >= 1000000u) {
            const ig_action_t a = ig_step(&gate, &report,
                                          (uint32_t)(vt / 1000u), true);
            if (a.stop_edge) {
                if (r.latches == 0u) { r.first_cause = gate.cause; }
                ++r.latches;
                ig_init(&gate); /* Count every stop edge. */
            }
        }
    }
    r.partial = probe.fifo_partial;
    r.resets = probe.fifo_recoveries;
    r.overruns = probe.fifo_overruns;
    r.timeouts = probe.timeouts;
    r.read_max_us = probe.max_us;
    r.n6_us = probe.max_us_n6;
    r.n8_us = probe.max_us_n8;
    printf("%u Hz gap %u ms: observations %u READ_FAILED %u latches %u "
           "cause %u partial %u resets %u overruns %u depth_max %lu "
           "timeouts %u read_max_us %lu\n",
           hz, gap_ms, r.observations, r.read_failed, r.latches,
           (unsigned int)r.first_cause, r.partial, r.resets, r.overruns,
           (unsigned long)probe.fifo_depth_max, r.timeouts,
           (unsigned long)r.read_max_us);
    return r;
}

int main(void)
{
    const unsigned int rates[] = {391u, 400u, 409u};
    const unsigned int ok[] = {22u, 30u, 40u, 49u};
    unsigned int runs = 0u;
    uint32_t worst = 0u;

    /* Every line-alarm phase in 50 us steps: a catch-up read must never
     * meet the 2 ms deadline, whatever the preemption timing. */
    for (phase_us = 0u; phase_us < 1000u; phase_us += 50u) {
        for (unsigned int i = 0u; i < 3u; ++i) {
            gap_result_t base = run(rates[i], 0u);
            assert(base.n6_us == 0u && base.n8_us == 0u); /* n is 3..5. */
            assert(base.timeouts == 0u && base.read_failed == 0u);
            assert(base.latches == 0u && base.partial == 0u);
            for (unsigned int g = 0u; g < 4u; ++g) {
                gap_result_t r = run(rates[i], ok[g]);
                fflush(stdout);
                assert(r.timeouts == 0u && r.read_max_us < 2000u);
                assert(r.read_failed == 0u && r.latches == 0u);
                assert(r.resets == 0u && r.partial >= 1u);
                /* TEST counters: a catch-up read is a 6-sample read. */
                assert(r.n6_us > 0u && r.n6_us == r.read_max_us);
                worst = r.read_max_us > worst ? r.read_max_us : worst;
                ++runs;
            }
        }
    }
    printf("imu FIFO option D timing fixtures PASS: %u gap runs, worst "
           "catch-up read %lu us\n", runs, (unsigned long)worst);
    return 0;
}

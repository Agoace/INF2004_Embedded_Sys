/* Option D gap fixtures: REAL imu_probe + imu_safety + ig_step(active=true)
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
static uint64_t next_sample_ns;
static bool ovrn;

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
void imu_hw_cycle_begin(void) { cycle_start = now_us; }
int imu_hw_cycle_end(void)
{
    return now_us - cycle_start >= 2000u ? IMU_IO_TIMEOUT : IMU_IO_OK;
}
int imu_hw_read(uint8_t address, uint8_t reg, uint8_t *out, size_t length)
{
    const unsigned int dev = address == IMU_ACCEL_ADDR ? 0u : 1u;

    now_us += 40u;
    produce();
    if (dev == 0u && reg == 0x2fu) {
        const uint64_t n = produced - consumed;
        /* FSS = unread count; OVRN reports FSS=31 with bit 6. */
        *out = ovrn ? 0x5fu : n == 0u ? 0x20u : (uint8_t)n;
    } else if (dev == 0u && reg == 0xa8u) {
        const size_t n = length / 6u;

        assert(length % 6u == 0u && n <= produced - consumed && n <= 8u);
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

    now_us += 40u;
    regs[dev][reg] = value;
    if (dev == 0u && reg == 0x2eu && value == 0u) {
        consumed = produced; /* Bypass empties the FIFO. */
        ovrn = false;
    }
    return IMU_IO_OK;
}

typedef struct {
    unsigned int read_failed, latches, observations, partial;
    unsigned int resets, overruns;
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
        now_us = tick;
        if (have && tick >= 1000000u) {
            const ig_action_t a = ig_step(&gate, &report,
                                          (uint32_t)(tick / 1000u), true);
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
    printf("%u Hz gap %u ms: observations %u READ_FAILED %u latches %u "
           "cause %u partial %u resets %u overruns %u depth_max %lu\n",
           hz, gap_ms, r.observations, r.read_failed, r.latches,
           (unsigned int)r.first_cause, r.partial, r.resets, r.overruns,
           (unsigned long)probe.fifo_depth_max);
    return r;
}

int main(void)
{
    const unsigned int rates[] = {391u, 400u, 409u};
    for (unsigned int i = 0u; i < 3u; ++i) {
        /* No gap: today's behaviour, never a partial read. */
        gap_result_t base = run(rates[i], 0u);
        assert(base.read_failed == 0u && base.latches == 0u);
        assert(base.partial == 0u && base.resets == 0u);
        const unsigned int ok[] = {22u, 30u, 40u, 49u};
        for (unsigned int g = 0u; g < 4u; ++g) {
            gap_result_t r = run(rates[i], ok[g]);
            assert(r.read_failed == 0u && r.latches == 0u);
            assert(r.resets == 0u && r.partial >= 1u);
        }
        gap_result_t stale = run(rates[i], 60u);
        assert(stale.latches >= 1u);
        assert(stale.first_cause == IMU_SAFETY_CAUSE_STALE);
        gap_result_t over = run(rates[i], 80u);
        assert(over.latches >= 1u && over.overruns == 1u);
        assert(over.read_failed == 1u && over.resets == 1u);
    }
    puts("imu FIFO option D gap fixtures PASS");
    return 0;
}

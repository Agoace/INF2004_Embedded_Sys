/*
 * cpu_load.c - pure CPU load arithmetic. See cpu_load.h.
 */
#include "cpu_load.h"

#define CPU_LOAD_US_PER_S 1000000u

/* Iterations per second for a count over an interval, rounded down. */
static uint64_t rate_of(uint32_t iterations, uint32_t elapsed_us)
{
    return (uint64_t)iterations * CPU_LOAD_US_PER_S / elapsed_us;
}

void cpu_load_init(cpu_load_t *meter, uint32_t iterations, uint32_t elapsed_us)
{
    meter->have_last = false;
    meter->last_count = 0u;
    meter->last_us = 0u;
    meter->peak_rate = 0u;
    if (iterations == 0u || elapsed_us == 0u) {
        return;
    }
    const uint64_t peak = rate_of(iterations, elapsed_us);

    meter->peak_rate = (peak > UINT32_MAX) ? UINT32_MAX : (uint32_t)peak;
}

uint16_t cpu_load_sample(cpu_load_t *meter, uint32_t count, uint32_t now_us)
{
    const bool had_last = meter->have_last;
    const uint32_t step_count = count - meter->last_count;
    const uint32_t step_us = now_us - meter->last_us;

    meter->have_last = true;
    meter->last_count = count;
    meter->last_us = now_us;
    if (!had_last || step_us == 0u || meter->peak_rate == 0u) {
        return CPU_LOAD_UNKNOWN;
    }
    const uint64_t rate = rate_of(step_count, step_us);
    const uint64_t peak = meter->peak_rate;

    if (rate >= peak) {
        return 0u;
    }
    const uint64_t idle =
        (rate * CPU_LOAD_FULL + peak / 2u) / peak; /* rounded permille idle */

    return (uint16_t)(CPU_LOAD_FULL - idle);
}

/* Host checks for the idle-meter arithmetic, including timer/count wrap. */
#include <assert.h>
#include <stdint.h>
#include "../app_program/platform/cpu_load.h"

static void check_load(unsigned int busy)
{
    cpu_load_t meter;
    /* Fixed loop costs 1 us. Calibrate with no stolen time. */
    cpu_load_init(&meter, 4000u, 4000u);
    assert(meter.peak_rate == 1000000u);
    assert(cpu_load_sample(&meter, 0u, 0u) == CPU_LOAD_UNKNOWN);
    const uint32_t idle_count = (1000u - busy) * 1000u;
    assert(cpu_load_sample(&meter, idle_count, 1000000u) == busy);
}

int main(void)
{
    for (unsigned int busy = 0u; busy <= 1000u; ++busy) {
        check_load(busy);
    }
    cpu_load_t meter;
    cpu_load_init(&meter, 0u, 4000u);
    assert(cpu_load_sample(&meter, 0u, 0u) == CPU_LOAD_UNKNOWN);
    assert(cpu_load_sample(&meter, 100u, 1000u) == CPU_LOAD_UNKNOWN);
    cpu_load_init(&meter, 100u, 0u);
    assert(meter.peak_rate == 0u);
    cpu_load_init(&meter, 4000u, 4000u);
    assert(cpu_load_sample(&meter, UINT32_MAX - 99u, UINT32_MAX - 999u) ==
           CPU_LOAD_UNKNOWN);
    assert(cpu_load_sample(&meter, 400u, 0u) == 500u);
    assert(cpu_load_sample(&meter, 400u, 0u) == CPU_LOAD_UNKNOWN);
    assert(cpu_load_sample(&meter, 1400u, 1000u) == 0u);
    /* Faster than calibration clamps to idle, not unsigned overflow. */
    assert(cpu_load_sample(&meter, 3400u, 2000u) == 0u);
    assert(cpu_load_sample(&meter, 3400u, 3000u) == 1000u);
    /* Non-integral reference rate and long elapsed interval. */
    cpu_load_init(&meter, 12345u, 4003u);
    assert(meter.peak_rate == 3083937u);
    (void)cpu_load_sample(&meter, 0u, 0u);
    assert(cpu_load_sample(&meter, 1541968u, 1000000u) == 500u);
    /* Quantisation error in a 1-second sample is <=1 permille. */
    for (unsigned int busy = 0u; busy <= 1000u; ++busy) {
        cpu_load_init(&meter, 12345u, 4003u);
        (void)cpu_load_sample(&meter, 0u, 0u);
        const uint32_t count =
            (uint32_t)((uint64_t)meter.peak_rate * (1000u - busy) / 1000u);
        const unsigned int measured = cpu_load_sample(&meter, count, 1000000u);
        assert(measured <= busy + 1u && measured + 1u >= busy);
    }
    return 0;
}

/* Host doubles for bounded ROSC extraction and one-boot identity lifetime. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include "../app_program/platform/boot_entropy.h"
#include "../app_program/platform/boot_id.h"
#include "../app_program/platform/boot_id_hw.h"

static bool timer_ready = true;
static bool enabled = true;
static bool stuck;
static uint32_t now;
static uint32_t source = 0x11223344u;
static unsigned int samples;

bool drv_time_ready(void)
{
    return timer_ready;
}

uint32_t drv_time_us(void)
{
    return now++;
}

/* Each pair is 01 for output zero or 10 for output one. */
uint32_t boot_register(uintptr_t address)
{
    if (address == (uintptr_t)0x40060018u) {
        return enabled ? (1u << 12u) : 0u;
    }
    assert(address == (uintptr_t)0x4006001cu);
    const unsigned int sample = samples++;
    if (stuck) {
        return 1u;
    }
    const unsigned int bit = (source >> (31u - ((sample / 2u) % 32u))) & 1u;
    return (sample & 1u) ? (bit ^ 1u) : bit;
}

static void test_extractor(void)
{
    boot_entropy_t entropy = {0};
    for (unsigned int i = 0u; i < 500u; ++i) {
        assert(!boot_entropy_pair(&entropy, 0u, 0u));
        assert(!boot_entropy_pair(&entropy, 1u, 1u));
        assert(entropy.bits == 0u && entropy.value == 0u);
    }
    for (unsigned int bit = 0u; bit < 32u; ++bit) {
        const unsigned int value = (0xa5a55a5au >> (31u - bit)) & 1u;
        const bool complete = boot_entropy_pair(&entropy, value, value ^ 1u);
        assert(complete == (bit == 31u));
    }
    assert(entropy.value == 0xa5a55a5au && entropy.bits == 32u);
    assert(boot_entropy_pair(&entropy, 0u, 1u));
    assert(entropy.value == 0xa5a55a5au); /* completed state is immutable */
}

int main(void)
{
    uint32_t output = 99u;
    test_extractor();
    assert(!boot_id_hw_generate(0));
    timer_ready = false;
    assert(!boot_id_hw_generate(&output) && output == 99u);
    timer_ready = true;
    enabled = false;
    assert(!boot_id_hw_generate(&output) && output == 99u);
    enabled = true;
    stuck = true;
    samples = 0u;
    assert(!boot_id_hw_generate(&output) && output == 99u);
    assert(samples == 32768u); /* bounded rejection, not an endless wait */
    stuck = false;
    source = 0u;
    samples = 0u;
    assert(!boot_id_hw_generate(&output) &&
           output == 99u); /* reserve ID zero */
    assert(samples == 32768u);
    source = 0x11223344u;
    samples = 0u;
    now = UINT32_MAX - 10u;
    assert(boot_id_hw_generate(&output) && output == source && samples == 64u);
    source = 0xaabbccddu;
    samples = 0u;
    assert(boot_id_hw_generate(&output) && output == source);
    assert(boot_id_get() == 0u);
    samples = 0u;
    assert(boot_id_init() == 0 && boot_id_get() == source);
    source = 0xdeadbeefu;
    assert(boot_id_init() == 0 && boot_id_get() == 0xaabbccddu);
    return 0;
}

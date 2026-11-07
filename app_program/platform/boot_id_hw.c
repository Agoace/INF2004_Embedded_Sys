/*
 * boot_id_hw.c - RP2040 ROSC random-bit boot label.
 *
 * RANDOMBIT reads the asynchronous oscillator output. The kernel uses XOSC/
 * PLL for the system clock and leaves ROSC enabled. Samples are separated by
 * a TIMER microsecond; equal bit pairs are rejected. This is an identity
 * label, not a cryptographic random generator or a guaranteed unique ID.
 * The bounded loop never enables, retunes or changes any clock.
 */
#include "boot_id_hw.h"
#include "boot_entropy.h"
#include "drv_time.h"

#define BOOT_ROSC_BASE          ((uintptr_t)0x40060000u)
#define BOOT_ROSC_STATUS_OFFSET 0x18u
#define BOOT_ROSC_RANDOM_OFFSET 0x1cu
#define BOOT_ROSC_ENABLED_MASK  (1u << 12u)
#define BOOT_SAMPLE_GAP_US      1u
#define BOOT_MAX_PAIRS          16384u

#ifndef BOOT_ID_HW_TEST
/* Read one RP2040 register; no SDK or kernel header is needed. */
static uint32_t boot_register(uintptr_t address)
{
    return *(const volatile uint32_t *)address;
}
#else
/* Host register double; no production indirection is added. */
extern uint32_t boot_register(uintptr_t address);
#endif

/* Sample one bit after a bounded-rate TIMER separation from the last sample. */
static unsigned int next_bit(void)
{
    const uint32_t start = drv_time_us();

    while ((uint32_t)(drv_time_us() - start) < BOOT_SAMPLE_GAP_US) {
        /* TIMER was checked before this module is called. */
    }
    return boot_register(BOOT_ROSC_BASE + BOOT_ROSC_RANDOM_OFFSET) & 1u;
}

/* Collect one full nonzero word before the application/watchdog starts. */
bool boot_id_hw_generate(uint32_t *out)
{
    boot_entropy_t entropy = {0};

    if (!out || !drv_time_ready() ||
        (boot_register(BOOT_ROSC_BASE + BOOT_ROSC_STATUS_OFFSET) &
         BOOT_ROSC_ENABLED_MASK) == 0u) {
        return false;
    }
    for (unsigned int pair = 0u; pair < BOOT_MAX_PAIRS; ++pair) {
        const unsigned int first = next_bit();
        const unsigned int second = next_bit();

        if (boot_entropy_pair(&entropy, first, second)) {
            if (entropy.value != 0u) {
                *out = entropy.value;
                return true;
            }
            entropy = (boot_entropy_t){0}; /* zero is reserved for unknown */
        }
    }
    return false; /* stuck/disabled source cannot create a false identity */
}

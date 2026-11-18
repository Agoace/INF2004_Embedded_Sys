/*
 * boot_entropy.h - pure bounded-state Von Neumann bit-pair extraction.
 * Equal samples are discarded; differing pairs contribute the first bit.
 * This removes stationary independent-bit bias, not arbitrary correlation.
 */
#ifndef BOOT_ENTROPY_H
#define BOOT_ENTROPY_H

#include <stdbool.h>
#include <stdint.h>

#define BOOT_ENTROPY_BITS 32u

typedef struct {
    uint32_t value;
    unsigned int bits;
} boot_entropy_t;

/* Feed two single-bit samples. True means a complete 32-bit word is ready. */
bool boot_entropy_pair(boot_entropy_t *state, unsigned int first,
                       unsigned int second);

#endif

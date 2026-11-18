/* boot_entropy.c - pure bit-pair extraction; see boot_entropy.h. */
#include "boot_entropy.h"

/* Consume at most one output bit; a complete word is immutable. */
bool boot_entropy_pair(boot_entropy_t *state, unsigned int first,
                       unsigned int second)
{
    if (state->bits >= BOOT_ENTROPY_BITS) {
        return true;
    }
    if ((first & 1u) != (second & 1u)) {
        state->value = (state->value << 1u) | (first & 1u);
        ++state->bits;
    }
    return state->bits == BOOT_ENTROPY_BITS;
}

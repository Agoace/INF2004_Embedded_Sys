#include "buddy1_motion.h"
#include "robot_shared.h" /* Actual external Buddy 2 contract, never a stand-in. */

_Static_assert(MSNAP_W1_SPEED_BITS == 9 && MSNAP_W1_L_SHIFT == 0 &&
               MSNAP_W1_R_SHIFT == 9 && MSNAP_W1_STATE_SHIFT == 18 &&
               MSNAP_W1_STATE_MASK == 15 && MSNAP_W1_FAULT_BIT == (1u << 22) &&
               MSNAP_W1_SLOW_BIT == (1u << 23) && MSNAP_W1_STALE_BIT == (1u << 24),
               "Motion contract changed: review Buddy 1 component encoding");

static bool valid(uint32_t w1)
{
    const uint32_t used = (MSNAP_W1_SPEED_MASK << MSNAP_W1_L_SHIFT) |
                          (MSNAP_W1_SPEED_MASK << MSNAP_W1_R_SHIFT) |
                          (MSNAP_W1_STATE_MASK << MSNAP_W1_STATE_SHIFT) |
                          MSNAP_W1_FAULT_BIT | MSNAP_W1_SLOW_BIT | MSNAP_W1_STALE_BIT;
    return (w1 & ~used) == 0 &&
           ((w1 >> MSNAP_W1_STATE_SHIFT) & MSNAP_W1_STATE_MASK) < MOTION_STATE_COUNT &&
           msnap_unpack_speed(w1, MSNAP_W1_L_SHIFT) != -256 &&
           msnap_unpack_speed(w1, MSNAP_W1_R_SHIFT) != -256;
}

bool b1_motion_encode(uint32_t w1, uint32_t w2, uint8_t *out, size_t length)
{
    if (out == NULL || length != B1_MOTION_COMPONENT_BYTES || !valid(w1)) return false;
    for (unsigned int i = 0; i < 4; ++i) {
        out[i] = (uint8_t)(w1 >> (24u - 8u * i));
        out[i + 4u] = (uint8_t)(w2 >> (24u - 8u * i));
    }
    return true;
}

bool b1_motion_decode(const uint8_t *bytes, size_t length, uint32_t *w1, uint32_t *w2)
{
    if (w1 == NULL || w2 == NULL || w1 == w2) return false;
    *w1 = *w2 = 0;
    if (bytes == NULL || length != B1_MOTION_COMPONENT_BYTES) return false;
    uint32_t a = 0, b = 0;
    for (unsigned int i = 0; i < 4; ++i) {
        a = (a << 8) | bytes[i];
        b = (b << 8) | bytes[i + 4u];
    }
    if (!valid(a)) return false;
    *w1 = a; *w2 = b;
    return true;
}

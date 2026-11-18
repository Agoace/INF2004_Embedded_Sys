/* Fixed robot observation storage. Not the generic controller format. */
#ifndef MOTION_PACK_H
#define MOTION_PACK_H
#include "motion.h"

#define MOTION_PACK_WORDS 15u

typedef struct {
    uint32_t words[MOTION_PACK_WORDS];
} motion_packed_t;

/* Words 0..11: counts L/R, travel L/R, heading, ticks, tick maximum,
 * gap minimum/maximum, late/missed wakes, publication time (all 32 bits).
 * Word 12: signed speeds L/R, 16 bits each.
 * Word 13: duty L/R biased by 800 (11 each), angle biased by 36 (7).
 * Word 14: pulse code (7), settled (1), independent faults (4), armed (1).
 * Pulse code 0..72 maps to the 73 pulses for angles -36..36. It is kept
 * separate from angle: removing this duplicate is a different RAM change.
 * Unused bits are zero. Invalid input leaves output unchanged.
 * Only motion_task publishes this value; all unpacking is on local copies.
 */
bool motion_pack(motion_packed_t *out, const motion_status_t *value,
                 uint32_t us);
bool motion_unpack(motion_status_t *out, uint32_t *us,
                   const motion_packed_t *value);
#endif

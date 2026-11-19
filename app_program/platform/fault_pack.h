/*
 * CR1 F pack: minimum-bit storage for the Health fault snapshot and the
 * STOP capture. RAM only. The API values (fault_mask_snapshot_t), the USB
 * FAULT_MASK line and MQTT do not change.
 *
 * Snapshot (8 B):  word0 bits 0..9 now, 10..19 run, 20..29 valid
 *                  (30..31 zero); word1 epoch (exact u32).
 * STOP capture (12 B): word0 bits 0..3 Motion (fault bits 0..3),
 *                  4..5 Vehicle (fault bits 4, 6), 6 Sonar (bit 5),
 *                  7 Telemetry (bit 7), 8..9 Wi-Fi (bits 8, 9),
 *                  10..19 valid (fault bits 0..9), 20..31 zero;
 *                  word1 epoch; word2 us (exact u32).
 *
 * Pack refuses any bit outside its field and writes nothing. Unpack has no
 * failure: every stored pattern decodes. Masks and shifts only; no C
 * bit-fields. Each packed value has one writer (see robot_health.c).
 */
#ifndef APP_FAULT_PACK_H
#define APP_FAULT_PACK_H

#include <stdbool.h>
#include <stdint.h>
#include "fault_mask.h"

typedef struct {
    uint32_t masks;
    uint32_t epoch;
} fault_snapshot_packed_t;
_Static_assert(sizeof(fault_snapshot_packed_t) == 8u, "Packed fault snapshot");

/* Native STOP capture values: one word per owner, as the owners publish. */
typedef struct {
    uint32_t owner[5]; /* Motion, Vehicle, Sonar, Telemetry, Wi-Fi */
    uint32_t valid;
    uint32_t epoch;
    uint32_t us;
} fault_stop_values_t;

typedef struct {
    uint32_t masks;
    uint32_t epoch;
    uint32_t us;
} fault_stop_packed_t;
_Static_assert(sizeof(fault_stop_packed_t) == 12u, "Packed STOP capture");

#define FAULT_PACK_OWNERS 5u

static const uint32_t fault_pack_owner_masks[FAULT_PACK_OWNERS] = {
    FAULT_OWNER_MOTION, FAULT_OWNER_VEHICLE, FAULT_OWNER_SONAR,
    FAULT_OWNER_TELEMETRY, FAULT_OWNER_NETWORK};

static inline bool fault_snapshot_pack(fault_snapshot_packed_t *out,
                                       const fault_mask_snapshot_t *v)
{
    if (out == 0 || v == 0 ||
        ((v->now | v->run | v->valid) & ~FAULT_MASK_ALL) != 0u) {
        return false;
    }
    out->masks = v->now | (v->run << 10) | (v->valid << 20);
    out->epoch = v->epoch;
    return true;
}

static inline void fault_snapshot_unpack(fault_mask_snapshot_t *out,
                                         const fault_snapshot_packed_t *p)
{
    out->now = p->masks & FAULT_MASK_ALL;
    out->run = (p->masks >> 10) & FAULT_MASK_ALL;
    out->valid = (p->masks >> 20) & FAULT_MASK_ALL;
    out->epoch = p->epoch;
}

/* Fault bits 0..9 <-> STOP field bits 0..9. Bits 0..3, 7..9 keep their
 * place; Vehicle bit 6 and Sonar bit 5 swap places so that each owner
 * field is contiguous (Vehicle 4..5, Sonar 6). The map is its own inverse. */
static inline uint32_t fault_stop_swap(uint32_t bits)
{
    return (bits & ~(FAULT_SONAR_SENSOR | FAULT_SHELL_OTHER)) |
           ((bits & FAULT_SONAR_SENSOR) << 1) |
           ((bits & FAULT_SHELL_OTHER) >> 1);
}

static inline bool fault_stop_pack(fault_stop_packed_t *out,
                                   const fault_stop_values_t *v)
{
    if (out == 0 || v == 0 || (v->valid & ~FAULT_MASK_ALL) != 0u) {
        return false;
    }
    uint32_t faults = 0u;
    for (unsigned int i = 0u; i < FAULT_PACK_OWNERS; ++i) {
        if ((v->owner[i] & ~fault_pack_owner_masks[i]) != 0u) {
            return false;
        }
        faults |= v->owner[i]; /* owner masks are disjoint */
    }
    out->masks = fault_stop_swap(faults) | (v->valid << 10);
    out->epoch = v->epoch;
    out->us = v->us;
    return true;
}

static inline void fault_stop_unpack(fault_stop_values_t *out,
                                     const fault_stop_packed_t *p)
{
    const uint32_t faults = fault_stop_swap(p->masks & FAULT_MASK_ALL);
    for (unsigned int i = 0u; i < FAULT_PACK_OWNERS; ++i) {
        out->owner[i] = faults & fault_pack_owner_masks[i];
    }
    out->valid = (p->masks >> 10) & FAULT_MASK_ALL;
    out->epoch = p->epoch;
    out->us = p->us;
}

#endif

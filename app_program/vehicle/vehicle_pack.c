/* Pure codec; no kernel or shared storage access. */
#include "vehicle_pack.h"
#include <limits.h>

#define META_MASK UINT32_C(0x003fffff)
_Static_assert(sizeof(vehicle_packed_t) == 36u, "vehicle payload size");

static int16_t signed_half(uint32_t word)
{
    const uint32_t half = word & UINT32_C(0xffff);
    return (int16_t)(half <= INT16_MAX ? (int32_t)half
                                      : (int32_t)half - 65536);
}

bool vehicle_pack(vehicle_packed_t *out, const vehicle_snapshot_t *v)
{
    if (!out || !v || v->servo_deg < -36 || v->servo_deg > 36 ||
        v->state > 2u || v->module > 4u || v->phase > 3u ||
        v->command_opcode > 4u || v->command_status > 5u) {
        return false;
    }
    vehicle_packed_t p = {{0}};
    p.words[0] = v->timestamp_us;
    p.words[1] = v->sequence;
    p.words[2] = v->wake_lost;
    p.words[3] = v->overruns;
    p.words[4] = v->response_max_us;
    p.words[5] = v->handoffs;
    p.words[6] = v->command_id;
    p.words[7] = (uint32_t)(uint16_t)v->left_mm_s |
                 ((uint32_t)(uint16_t)v->right_mm_s << 16u);
    p.words[8] = (uint32_t)(v->servo_deg + 36) |
        ((uint32_t)v->state << 7u) | ((uint32_t)v->module << 9u) |
        ((uint32_t)v->phase << 12u) | ((uint32_t)v->command_opcode << 14u) |
        ((uint32_t)v->command_status << 17u) |
        ((uint32_t)v->active << 20u) | ((uint32_t)v->calibrated << 21u);
    *out = p;
    return true;
}

bool vehicle_unpack(vehicle_snapshot_t *out, const vehicle_packed_t *p)
{
    if (!out || !p || (p->words[8] & ~META_MASK) != 0u) {
        return false;
    }
    const uint32_t meta = p->words[8];
    const uint32_t angle = meta & 127u;
    const uint32_t state = (meta >> 7u) & 3u;
    const uint32_t module = (meta >> 9u) & 7u;
    const uint32_t opcode = (meta >> 14u) & 7u;
    const uint32_t status = (meta >> 17u) & 7u;
    if (angle > 72u || state > 2u || module > 4u || opcode > 4u || status > 5u) {
        return false;
    }
    const vehicle_snapshot_t v = {
        .timestamp_us = p->words[0], .sequence = p->words[1],
        .wake_lost = p->words[2], .overruns = p->words[3],
        .response_max_us = p->words[4], .handoffs = p->words[5],
        .command_id = p->words[6],
        .left_mm_s = signed_half(p->words[7]),
        .right_mm_s = signed_half(p->words[7] >> 16u),
        .servo_deg = (int8_t)((int32_t)angle - 36),
        .state = (uint8_t)state, .module = (uint8_t)module,
        .phase = (uint8_t)((meta >> 12u) & 3u),
        .command_opcode = (uint8_t)opcode, .command_status = (uint8_t)status,
        .active = ((meta >> 20u) & 1u) != 0u,
        .calibrated = ((meta >> 21u) & 1u) != 0u
    };
    *out = v;
    return true;
}

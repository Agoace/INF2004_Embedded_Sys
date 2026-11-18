/* Explicit masks and shifts; never read-modify-write shared storage. */
#include "motion_pack.h"
#include <limits.h>

#define DUTY_BIAS 800
#define ANGLE_BIAS 36
#define DUTY_MASK UINT32_C(0x7ff)
#define CODE_MASK UINT32_C(0x7f)
#define FAULT_MASK UINT32_C(0xf)
#define WORD13_MASK UINT32_C(0x1fffffff)
#define WORD14_MASK UINT32_C(0x1fff)

_Static_assert(sizeof(motion_packed_t) == 60u, "motion payload size");
_Static_assert((MC_FAULT_LATCHED | MC_FAULT_STALE) == FAULT_MASK,
               "review compact fault allocation when conditions change");

/* Preserve the controller's exact whole-degree pulse rounding. */
static uint16_t pulse_for_code(uint32_t code)
{
    const int32_t angle = (int32_t)code - ANGLE_BIAS;
    const int32_t offset = angle >= 0 ? (angle * 111 + 5) / 10
                                     : -((-angle * 111 + 5) / 10);
    return (uint16_t)(1500 + offset);
}

/* Conversion without implementation-defined unsigned-to-signed overflow. */
static int32_t signed_word(uint32_t word)
{
    return word <= INT32_MAX ? (int32_t)word
                            : -1 - (int32_t)(UINT32_MAX - word);
}

static int16_t signed_half(uint32_t word)
{
    const uint32_t half = word & UINT32_C(0xffff);
    const int32_t value = half <= INT16_MAX ? (int32_t)half
                                          : (int32_t)half - 65536;
    return (int16_t)value;
}

/* Validate the full source before any destination modification. */
bool motion_pack(motion_packed_t *out, const motion_status_t *v, uint32_t us)
{
    if (!out || !v || v->servo_deg < -ANGLE_BIAS ||
        v->servo_deg > ANGLE_BIAS || (v->faults & ~FAULT_MASK) != 0u) {
        return false;
    }
    for (unsigned int i = 0u; i < MC_WHEELS; ++i) {
        if (v->duty_permille[i] < -DUTY_BIAS ||
            v->duty_permille[i] > DUTY_BIAS) {
            return false;
        }
    }
    const int32_t delta = (int32_t)v->servo_us - 1500;
    const int32_t angle = delta >= 0 ? (delta * 10 + 55) / 111
                                   : -((-delta * 10 + 55) / 111);
    if (angle < -ANGLE_BIAS || angle > ANGLE_BIAS) {
        return false;
    }
    const uint32_t pulse_code = (uint32_t)(angle + ANGLE_BIAS);
    if (pulse_for_code(pulse_code) != v->servo_us) {
        return false;
    }
    motion_packed_t p = {{0}};
    p.words[0] = (uint32_t)v->counts[0];
    p.words[1] = (uint32_t)v->counts[1];
    p.words[2] = (uint32_t)v->travel_mm[0];
    p.words[3] = (uint32_t)v->travel_mm[1];
    p.words[4] = (uint32_t)v->heading_cdeg;
    p.words[5] = v->ticks;
    p.words[6] = v->tick_max_us;
    p.words[7] = v->gap_min_us;
    p.words[8] = v->gap_max_us;
    p.words[9] = v->late_wakes;
    p.words[10] = v->missed_wakes;
    p.words[11] = us;
    p.words[12] = (uint32_t)(uint16_t)v->speed_mm_s[0] |
                   ((uint32_t)(uint16_t)v->speed_mm_s[1] << 16u);
    p.words[13] = ((uint32_t)(v->duty_permille[0] + DUTY_BIAS) & DUTY_MASK) |
        (((uint32_t)(v->duty_permille[1] + DUTY_BIAS) & DUTY_MASK) << 11u) |
        (((uint32_t)(v->servo_deg + ANGLE_BIAS) & CODE_MASK) << 22u);
    p.words[14] = pulse_code | ((uint32_t)v->servo_settled << 7u) |
        (((uint32_t)v->faults & FAULT_MASK) << 8u) |
        ((uint32_t)v->watchdog_armed << 12u);
    *out = p;
    return true;
}

/* Unpack a stable board copy, never the shared words themselves. */
bool motion_unpack(motion_status_t *out, uint32_t *us,
                   const motion_packed_t *p)
{
    if (!out || !us || !p || (p->words[13] & ~WORD13_MASK) != 0u ||
        (p->words[14] & ~WORD14_MASK) != 0u) {
        return false;
    }
    const uint32_t left = p->words[13] & DUTY_MASK;
    const uint32_t right = (p->words[13] >> 11u) & DUTY_MASK;
    const uint32_t angle = (p->words[13] >> 22u) & CODE_MASK;
    const uint32_t pulse = p->words[14] & CODE_MASK;
    if (left > 1600u || right > 1600u || angle > 72u || pulse > 72u) {
        return false;
    }
    motion_status_t v = {0};
    v.counts[0] = signed_word(p->words[0]);
    v.counts[1] = signed_word(p->words[1]);
    v.travel_mm[0] = signed_word(p->words[2]);
    v.travel_mm[1] = signed_word(p->words[3]);
    v.heading_cdeg = signed_word(p->words[4]);
    v.ticks = p->words[5];
    v.tick_max_us = p->words[6];
    v.gap_min_us = p->words[7];
    v.gap_max_us = p->words[8];
    v.late_wakes = p->words[9];
    v.missed_wakes = p->words[10];
    v.speed_mm_s[0] = signed_half(p->words[12]);
    v.speed_mm_s[1] = signed_half(p->words[12] >> 16u);
    v.duty_permille[0] = (int16_t)((int32_t)left - DUTY_BIAS);
    v.duty_permille[1] = (int16_t)((int32_t)right - DUTY_BIAS);
    v.servo_deg = (int8_t)((int32_t)angle - ANGLE_BIAS);
    v.servo_us = pulse_for_code(pulse);
    v.servo_settled = ((p->words[14] >> 7u) & 1u) != 0u;
    v.faults = (uint8_t)((p->words[14] >> 8u) & FAULT_MASK);
    v.watchdog_armed = ((p->words[14] >> 12u) & 1u) != 0u;
    *out = v;
    *us = p->words[11];
    return true;
}

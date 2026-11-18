/* Pure IMU safety latch capture ring. See capture.h. */
#include "capture.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static int16_t to_i16(float v)
{
    if (!isfinite(v)) { return 0; }
    if (v >= (float)INT16_MAX) { return INT16_MAX; }
    if (v <= (float)INT16_MIN) { return INT16_MIN; }
    return (int16_t)lrintf(v);
}

static uint16_t to_u16(float v)
{
    if (!isfinite(v) || v <= 0.0f) { return 0u; }
    if (v >= (float)UINT16_MAX) { return UINT16_MAX; }
    return (uint16_t)lrintf(v);
}

static uint8_t to_u8(uint32_t v)
{
    return v > UINT8_MAX ? (uint8_t)UINT8_MAX : (uint8_t)v;
}

static void inc(uint32_t *v)
{
    if (*v != UINT32_MAX) { ++*v; }
}

static void set_state(isc_t *c, uint8_t state)
{
    __atomic_store_n(&c->state, state, __ATOMIC_SEQ_CST);
}

void isc_init(isc_t *c)
{
    if (c != NULL) {
        memset(c, 0, sizeof(*c));
    }
}

static void put(isc_t *c, const imu_safety_input_t *in,
                const imu_safety_t *after)
{
    const imu_safety_output_t *o = &after->output;
    isc_sample_t *s = &c->ring[c->head];

    s->ms16 = (uint16_t)in->now_ms;
    for (unsigned int i = 0u; i < 3u; ++i) {
        s->a[i] = to_i16(in->accel_mg[i]);
    }
    s->g = to_u16(o->g_mg);
    s->tilt = (int16_t)(o->tilt_ddeg > INT16_MAX ? INT16_MAX :
                        o->tilt_ddeg < INT16_MIN ? INT16_MIN : o->tilt_ddeg);
    s->vert = to_i16(o->vertical_mg);
    s->flags = o->flags;
    s->ff = to_u8(after->freefall_samples);
    s->lift = to_u8(o->lift_samples);
    s->ot = to_u8(o->overturn_samples);
    s->trig = (uint8_t)(((o->current_trigger & 0x0fu) << 4) |
                        (o->latched_trigger & 0x0fu));
    s->in = (uint8_t)((in->accel_valid ? ISC_IN_VALID : 0u) |
                      (in->calibrated ? ISC_IN_CALIBRATED : 0u) |
                      (in->wheels_still ? ISC_IN_WHEELS_STILL : 0u) |
                      (in->read_failed ? ISC_IN_READ_FAILED : 0u));
    c->head = (uint16_t)((c->head + 1u) % ISC_SAMPLES);
}

void isc_observe(isc_t *c, const imu_safety_input_t *in,
                 const imu_safety_t *after, uint8_t latched_before)
{
    if (c == NULL || in == NULL || after == NULL) {
        return;
    }
    const bool edge = latched_before == (uint8_t)IMU_SAFETY_NONE &&
                      after->output.latched_trigger !=
                          (uint8_t)IMU_SAFETY_NONE;
    const uint32_t requests = __atomic_load_n(&c->requests, __ATOMIC_SEQ_CST);
    const bool request = requests != c->requests_seen;
    uint8_t state = isc_state(c);

    c->requests_seen = requests; /* consumed or dropped in every state */
    if (edge) {
        inc(&c->latches);
    }
    if (state == ISC_REARM) {
        c->head = 0u;
        c->count = 0u;
        state = ISC_ARMED;
        set_state(c, state);
    }
    if (state == ISC_FROZEN) {
        if (edge) { inc(&c->missed); }
        return;
    }
    if (state == ISC_ARMED && (edge || request)) {
        const uint16_t pre_max = edge ? ISC_PRE : ISC_MANUAL_PRE;
        if (c->count > pre_max) { c->count = pre_max; }
        c->pre = c->count;
        c->source = edge ? ISC_SOURCE_LATCH : ISC_SOURCE_ESTOP;
        c->trigger = edge ? after->output.latched_trigger : 0u;
        c->latch_ms = in->now_ms;
        c->post_left = edge ? ISC_POST : ISC_MANUAL_POST;
        state = ISC_COLLECTING;
        set_state(c, state);
    } else if (edge) {
        inc(&c->missed);
    }
    put(c, in, after);
    if (state == ISC_ARMED) {
        if (c->count < ISC_SAMPLES) { ++c->count; }
        return;
    }
    ++c->count;
    if (--c->post_left == 0u) {
        inc(&c->captures);
        set_state(c, ISC_FROZEN);
    }
}

void isc_request(isc_t *c)
{
    if (c == NULL) {
        return;
    }
    /* One requester task: a plain load and store is a safe increment
     * (Cortex-M0+ has no atomic read-modify-write). */
    const uint32_t next =
        __atomic_load_n(&c->requests, __ATOMIC_SEQ_CST) + 1u;
    __atomic_store_n(&c->requests, next, __ATOMIC_SEQ_CST);
}

uint8_t isc_state(const isc_t *c)
{
    return __atomic_load_n(&c->state, __ATOMIC_SEQ_CST);
}

bool isc_frozen(const isc_t *c)
{
    return isc_state(c) == ISC_FROZEN;
}

unsigned int isc_count(const isc_t *c)
{
    return c->count;
}

const isc_sample_t *isc_at(const isc_t *c, unsigned int index)
{
    if (index >= c->count) {
        return NULL;
    }
    const unsigned int oldest =
        (c->head + ISC_SAMPLES - c->count) % ISC_SAMPLES;
    return &c->ring[(oldest + index) % ISC_SAMPLES];
}

int32_t isc_rel_ms(const isc_t *c, const isc_sample_t *s)
{
    return (int32_t)(int16_t)(uint16_t)(s->ms16 - (uint16_t)c->latch_ms);
}

void isc_rearm(isc_t *c)
{
    /* No compare-and-swap on Cortex-M0+. Safe without it: in FROZEN only
     * the reader changes the state; the writer just reads it. */
    if (isc_state(c) == ISC_FROZEN) {
        set_state(c, ISC_REARM);
    }
}

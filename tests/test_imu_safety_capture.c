/* Pure IMU safety latch capture ring: 64 pre + 192 post samples, frozen at
 * the first latch edge, one writer, reader re-arms after printing. */
#include "../app_program/imu_safety_capture/capture.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static isc_t cap;
static imu_safety_t det;
static imu_safety_input_t in;

/* One observation at ms with a forced latched value after the step. */
static void step(uint32_t ms, uint8_t latched_after)
{
    const uint8_t before = det.output.latched_trigger;
    in.now_ms = ms;
    in.accel_mg[0] = (float)ms;
    in.accel_mg[1] = -40000.0f; /* clamps to INT16_MIN */
    in.accel_mg[2] = 40000.0f;  /* clamps to INT16_MAX */
    det.output.latched_trigger = latched_after;
    det.output.current_trigger = latched_after;
    det.output.g_mg = 70000.0f; /* clamps to UINT16_MAX */
    det.output.tilt_ddeg = 900;
    det.output.vertical_mg = -650.4f;
    det.output.flags = 0x15u;
    det.output.lift_samples = 300u; /* saturates at 255 */
    det.output.overturn_samples = 4u;
    det.freefall_samples = 2u;
    isc_observe(&cap, &in, &det, before);
}

static void test_short_pre_and_freeze(void)
{
    isc_init(&cap);
    memset(&det, 0, sizeof(det));
    memset(&in, 0, sizeof(in));
    in.accel_valid = true;
    in.wheels_still = true;
    assert(isc_state(&cap) == ISC_ARMED && !isc_frozen(&cap));
    for (uint32_t ms = 10u; ms <= 100u; ms += 10u) { step(ms, 0u); }
    step(110u, IMU_SAFETY_LIFT); /* edge: post sample 0 */
    assert(isc_state(&cap) == ISC_COLLECTING);
    assert(cap.pre == 10u && cap.trigger == IMU_SAFETY_LIFT &&
           cap.latch_ms == 110u && cap.latches == 1u);
    for (unsigned int i = 1u; i < ISC_POST; ++i) {
        assert(!isc_frozen(&cap));
        step(110u + 10u * i, IMU_SAFETY_LIFT);
    }
    assert(isc_frozen(&cap) && isc_count(&cap) == 10u + ISC_POST);
    const isc_sample_t *s = isc_at(&cap, 0u);
    assert(s->ms16 == 10u && s->a[0] == 10 && s->a[1] == INT16_MIN &&
           s->a[2] == INT16_MAX && s->g == UINT16_MAX && s->tilt == 900 &&
           s->vert == -650 && s->flags == 0x15u && s->ff == 2u &&
           s->lift == 255u && s->ot == 4u && s->trig == 0u &&
           s->in == (ISC_IN_VALID | ISC_IN_WHEELS_STILL));
    s = isc_at(&cap, 10u);
    assert(s->ms16 == 110u &&
           s->trig == (uint8_t)((IMU_SAFETY_LIFT << 4) | IMU_SAFETY_LIFT));
    assert(isc_at(&cap, isc_count(&cap) - 1u)->ms16 ==
           (uint16_t)(110u + 10u * (ISC_POST - 1u)));
    assert(isc_at(&cap, isc_count(&cap)) == NULL);
    /* Frozen: later samples and a new edge do not overwrite. */
    step(5000u, 0u);
    step(5010u, IMU_SAFETY_OVERTURN);
    assert(cap.missed == 1u && cap.latches == 2u);
    assert(isc_at(&cap, 0u)->ms16 == 10u);
    /* Reader re-arms; the writer resets on its next sample. */
    isc_rearm(&cap);
    assert(isc_state(&cap) == ISC_REARM);
    step(6000u, IMU_SAFETY_OVERTURN); /* still latched: no edge */
    assert(isc_state(&cap) == ISC_ARMED && isc_count(&cap) == 1u);
    assert(cap.captures == 1u);
}

static void test_full_pre_window_and_wrap(void)
{
    isc_init(&cap);
    memset(&det, 0, sizeof(det));
    uint32_t ms = 65000u; /* 16-bit ms wraps inside the capture */
    for (unsigned int i = 0u; i < 1000u; ++i) { step(ms, 0u); ms += 10u; }
    assert(isc_count(&cap) == ISC_SAMPLES); /* ARMED keeps the full ring */
    const uint32_t edge = ms;
    for (unsigned int i = 0u; i < ISC_POST; ++i) {
        step(ms, IMU_SAFETY_OVERTURN);
        ms += 10u;
    }
    assert(isc_frozen(&cap) && cap.pre == ISC_PRE &&
           isc_count(&cap) == ISC_SAMPLES && cap.source == ISC_SOURCE_LATCH);
    assert(isc_at(&cap, 0u)->ms16 == (uint16_t)(edge - 10u * ISC_PRE));
    assert(isc_at(&cap, ISC_PRE)->ms16 == (uint16_t)edge);
    assert(isc_rel_ms(&cap, isc_at(&cap, 0u)) == -(int32_t)(10u * ISC_PRE));
    assert(isc_rel_ms(&cap, isc_at(&cap, ISC_SAMPLES - 1u)) ==
           (int32_t)(10u * (ISC_POST - 1u)));
    /* An edge seen while collecting is counted, never restarts. */
    isc_rearm(&cap);
    step(ms, 0u); ms += 10u;
    step(ms, IMU_SAFETY_LIFT); ms += 10u;
    det.output.latched_trigger = 0u;
    step(ms, IMU_SAFETY_LIFT); ms += 10u;
    assert(cap.missed == 1u && cap.trigger == IMU_SAFETY_LIFT &&
           cap.pre == 1u);
    assert(isc_state(&cap) == ISC_COLLECTING);
}

/* A Vehicle ESTOP request freezes the ring without a latch: up to
 * ISC_MANUAL_PRE samples before it and ISC_MANUAL_POST from it. */
static void test_manual_request(void)
{
    isc_init(&cap);
    memset(&det, 0, sizeof(det));
    uint32_t ms = 1000u;
    for (unsigned int i = 0u; i < 300u; ++i) { step(ms, 0u); ms += 10u; }
    isc_request(&cap);
    const uint32_t at = ms;
    for (unsigned int i = 0u; i < ISC_MANUAL_POST; ++i) {
        assert(!isc_frozen(&cap));
        step(ms, 0u);
        ms += 10u;
    }
    assert(isc_frozen(&cap) && cap.source == ISC_SOURCE_ESTOP &&
           cap.trigger == 0u && cap.latch_ms == at &&
           cap.pre == ISC_MANUAL_PRE && isc_count(&cap) == ISC_SAMPLES &&
           cap.latches == 0u && cap.missed == 0u);
    assert(isc_rel_ms(&cap, isc_at(&cap, 0u)) ==
           -(int32_t)(10u * ISC_MANUAL_PRE));
    /* While frozen a request is ignored, and not a missed latch. */
    isc_request(&cap);
    step(ms, 0u); ms += 10u;
    assert(cap.missed == 0u && isc_at(&cap, ISC_MANUAL_PRE)->ms16 ==
           (uint16_t)at);
    /* The ignored request does not survive the re-arm. */
    isc_rearm(&cap);
    for (unsigned int i = 0u; i < 300u; ++i) { step(ms, 0u); ms += 10u; }
    assert(isc_state(&cap) == ISC_ARMED);
    /* The IMU stop calls ESTOP after the latch: the latch wins, and the
     * request during COLLECTING is dropped silently. */
    step(ms, IMU_SAFETY_LIFT); ms += 10u;
    isc_request(&cap);
    for (unsigned int i = 1u; i < ISC_POST; ++i) {
        step(ms, IMU_SAFETY_LIFT); ms += 10u;
    }
    assert(isc_frozen(&cap) && cap.source == ISC_SOURCE_LATCH &&
           cap.pre == ISC_PRE && cap.missed == 0u);
    isc_rearm(&cap);
    step(ms, IMU_SAFETY_LIFT); ms += 10u;
    assert(isc_state(&cap) == ISC_ARMED);
    isc_request(NULL);
}

static void test_input_bits_and_nan(void)
{
    isc_init(&cap);
    memset(&det, 0, sizeof(det));
    memset(&in, 0, sizeof(in));
    in.calibrated = true;
    in.read_failed = true;
    in.accel_mg[0] = 0.0f / 0.0f;
    det.output.g_mg = -5.0f;
    det.output.vertical_mg = 0.0f / 0.0f;
    isc_observe(&cap, &in, &det, 0u);
    const isc_sample_t *s = isc_at(&cap, 0u);
    assert(s->in == (ISC_IN_CALIBRATED | ISC_IN_READ_FAILED));
    assert(s->a[0] == 0 && s->g == 0u && s->vert == 0);
    isc_observe(NULL, &in, &det, 0u);
    isc_observe(&cap, NULL, &det, 0u);
    isc_observe(&cap, &in, NULL, 0u);
    assert(isc_count(&cap) == 1u);
    isc_rearm(&cap); /* not frozen: ignored */
    assert(isc_state(&cap) == ISC_ARMED);
    assert(sizeof(isc_sample_t) == 20u);
}

int main(void)
{
    test_short_pre_and_freeze();
    test_full_pre_window_and_wrap();
    test_manual_request();
    test_input_bits_and_nan();
    puts("IMU safety capture PASS");
    return 0;
}

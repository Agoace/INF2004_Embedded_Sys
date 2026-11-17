/* Pure IMU safety detector. Never drives; decides a candidate trigger only. */
#include "imu_safety.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define RAD_TO_DDEG 572.9577951f

bool imu_safety_upright_ok(const float accel_mg[3])
{
    if (accel_mg == NULL) {
        return false;
    }
    /* A NaN comparison is false, so a corrupt sample is rejected. */
    return accel_mg[2] >= IMU_SAFETY_UPRIGHT_Z_MG &&
           fabsf(accel_mg[0]) <= IMU_SAFETY_UPRIGHT_XY_MG &&
           fabsf(accel_mg[1]) <= IMU_SAFETY_UPRIGHT_XY_MG;
}

int32_t imu_safety_tilt_ddeg(const float accel_mg[3])
{
    if (accel_mg == NULL) {
        return 0;
    }
    const float x = accel_mg[0];
    const float y = accel_mg[1];
    const float z = accel_mg[2];

    if (!isfinite(x) || !isfinite(y) || !isfinite(z)) {
        return 0;
    }
    const float horizontal = sqrtf((x * x) + (y * y));
    const float magnitude = sqrtf((horizontal * horizontal) + (z * z));

    if (magnitude <= IMU_SAFETY_G_UNIT_MG) {
        return 0;
    }
    return (int32_t)(atan2f(horizontal, z) * RAD_TO_DDEG);
}

void imu_safety_init(imu_safety_t *s)
{
    if (s != NULL) {
        memset(s, 0, sizeof(*s));
    }
}

static float magnitude_mg(const float accel_mg[3])
{
    return sqrtf((accel_mg[0] * accel_mg[0]) + (accel_mg[1] * accel_mg[1]) +
                 (accel_mg[2] * accel_mg[2]));
}

/* Signed vertical change: (current - prior gravity) . prior gravity unit.
 * Zero until a quiet gravity history exists. */
static float vertical_change_mg(const imu_safety_t *s,
                                const float accel_mg[3])
{
    if (!s->gravity_ready) {
        return 0.0f;
    }
    const float gravity =
        magnitude_mg(s->gravity_mg);
    if (!(gravity > IMU_SAFETY_G_UNIT_MG)) {
        return 0.0f;
    }
    const float dx = accel_mg[0] - s->gravity_mg[0];
    const float dy = accel_mg[1] - s->gravity_mg[1];
    const float dz = accel_mg[2] - s->gravity_mg[2];

    return ((dx * s->gravity_mg[0]) + (dy * s->gravity_mg[1]) +
            (dz * s->gravity_mg[2])) / gravity;
}

/* Update the gravity history from a quiet observation. Frozen otherwise,
 * so a transient cannot become the new reference. */
static void update_gravity(imu_safety_t *s, const float accel_mg[3], bool quiet)
{
    if (!quiet) {
        return;
    }
    if (!s->gravity_ready) {
        s->gravity_mg[0] = accel_mg[0];
        s->gravity_mg[1] = accel_mg[1];
        s->gravity_mg[2] = accel_mg[2];
        s->gravity_ready = true;
        return;
    }
    for (unsigned int axis = 0u; axis < 3u; ++axis) {
        s->gravity_mg[axis] += IMU_SAFETY_GRAVITY_ALPHA *
                               (accel_mg[axis] - s->gravity_mg[axis]);
    }
}

/* Elapsed observation time, clamped, plus whether the raw gap was too long
 * to count as a continuous observation. */
static uint32_t elapsed_ms(imu_safety_t *s, uint32_t now_ms, bool *gap)
{
    if (gap != NULL) {
        *gap = false;
    }
    if (!s->have_previous) {
        s->have_previous = true;
        s->previous_ms = now_ms;
        return 0u;
    }
    const int32_t delta = (int32_t)(now_ms - s->previous_ms);

    s->previous_ms = now_ms;
    if (delta <= 0) {
        return 0u;
    }
    if (gap != NULL) {
        *gap = (uint32_t)delta > IMU_SAFETY_CLEAR_GAP_MS;
    }
    return (uint32_t)delta > IMU_SAFETY_MAX_DELTA_MS
               ? IMU_SAFETY_MAX_DELTA_MS
               : (uint32_t)delta;
}

void imu_safety_step(imu_safety_t *s, const imu_safety_input_t *in)
{
    if ((s == NULL) || (in == NULL)) {
        return;
    }
    imu_safety_output_t *o = &s->output;
    bool gap = false;
    const uint32_t delta_ms = elapsed_ms(s, in->now_ms, &gap);

    o->observation_ms = in->now_ms;
    o->current_trigger = (uint8_t)IMU_SAFETY_NONE;

    const bool usable = in->accel_valid && in->calibrated;
    const float g_mg = magnitude_mg(in->accel_mg);
    const bool finite = isfinite(g_mg);

    o->g_mg = finite ? g_mg : 0.0f;
    if (!usable) {
        o->flags = 0u;
        o->tilt_ddeg = 0;
        o->vertical_mg = 0.0f;
        o->overturn_samples = 0u;
        o->lift_samples = 0u;
        o->clear_ms = 0u;
        s->freefall_samples = 0u;
        return;
    }

    const bool near_one_g =
        finite && (fabsf(g_mg - IMU_SAFETY_GRAVITY_MG) <=
                   IMU_SAFETY_NEAR_ONE_G_MG);
    const bool freefall = finite && g_mg < IMU_SAFETY_FREEFALL_MG;
    const bool upright = imu_safety_upright_ok(in->accel_mg);
    const int32_t tilt = imu_safety_tilt_ddeg(in->accel_mg);
    /* Measured against the PRIOR reference, before any update below. */
    const float vertical = vertical_change_mg(s, in->accel_mg);

    o->tilt_ddeg = tilt;
    o->vertical_mg = vertical;
    o->flags = IMU_SAFETY_FLAG_VALID;
    if (upright) {
        o->flags |= IMU_SAFETY_FLAG_UPRIGHT;
    }
    if (s->gravity_ready) {
        o->flags |= IMU_SAFETY_FLAG_GRAVITY;
    }
    if (near_one_g) {
        o->flags |= IMU_SAFETY_FLAG_NEAR_ONE_G;
    }
    if (freefall) {
        o->flags |= IMU_SAFETY_FLAG_FREEFALL;
    }

    /* The all-clear window measures the pose, so it must run even while a
     * trigger is latched; the safe restart needs it. A gap longer than the
     * observation cadence restarts the window: the pose was not observed in
     * that time, so it cannot count as continuously clear (RT-0094/F1). */
    if (!gap && upright && in->wheels_still && near_one_g) {
        const uint32_t limit = IMU_SAFETY_CLEAR_MS;

        o->clear_ms = (o->clear_ms >= limit - delta_ms) ? limit
                                                        : o->clear_ms + delta_ms;
    } else {
        o->clear_ms = 0u;
    }

    if (o->latched_trigger != (uint8_t)IMU_SAFETY_NONE) {
        /* Detection counters freeze at the latch, so the reported reason
         * cannot drift after the stop. */
        return;
    }

    s->freefall_samples =
        freefall ? (uint16_t)(s->freefall_samples + 1u) : 0u;

    /* Overturn needs valid near-1 g data: a low-g transient belongs to the
     * lift path, and a strong transient must not refresh the reference. */
    if (near_one_g &&
        (tilt > (int32_t)(IMU_SAFETY_OVERTURN_DEG * 10.0f))) {
        if (o->overturn_samples != UINT16_MAX) {
            ++o->overturn_samples;
        }
    } else {
        o->overturn_samples = 0u;
    }

    /* A pure tilt rotates gravity while |a| stays near 1 g, which would
     * otherwise read as a huge vertical change against the prior vector.
     * Separation: a magnitude deviation is a lift/fall transient; a
     * direction change at near 1 g belongs to the overturn path. A combined
     * tilt-and-lift that keeps |a| near 1 g is the documented blind spot of
     * an IMU-only detector. */
    const bool vertical_transient =
        s->gravity_ready && !near_one_g &&
        fabsf(vertical) > IMU_SAFETY_VERTICAL_MG;

    if (vertical_transient) {
        if (o->lift_samples != UINT16_MAX) {
            ++o->lift_samples;
        }
    } else {
        o->lift_samples = 0u;
    }

    const bool lift_confirmed =
        s->freefall_samples >= IMU_SAFETY_FREEFALL_SAMPLES ||
        o->lift_samples >= IMU_SAFETY_VERTICAL_SAMPLES;
    const bool overturn_confirmed =
        o->overturn_samples >= IMU_SAFETY_OVERTURN_SAMPLES;

    /* Precedence is stated, not inferred (RT-0091/F4): if both ever confirm,
     * LIFT is reported. With the separation gate above they are mutually
     * exclusive in one observation - overturn needs near 1 g, while free
     * fall and the vertical path both require the opposite - so this branch
     * is a deliberate ordering rule that a later change to either gate
     * cannot silently reverse, not a case the fixtures can reach. */
    if (lift_confirmed) {
        o->latched_trigger = (uint8_t)IMU_SAFETY_LIFT;
        o->current_trigger = (uint8_t)IMU_SAFETY_LIFT;
    } else if (overturn_confirmed) {
        o->latched_trigger = (uint8_t)IMU_SAFETY_OVERTURN;
        o->current_trigger = (uint8_t)IMU_SAFETY_OVERTURN;
    }
    if (o->current_trigger != (uint8_t)IMU_SAFETY_NONE &&
        o->stops != UINT8_MAX) {
        ++o->stops;
    }

    /* Quiet enough to become the reference: no fall, no strong vertical
     * change. This runs after the vertical comparison above, so the change
     * is always measured against the prior reference. */
    update_gravity(s, in->accel_mg, near_one_g && !freefall &&
                                            !vertical_transient);
}

bool imu_safety_clear(imu_safety_t *s)
{
    if (s == NULL) {
        return false;
    }
    if (s->output.clear_ms < IMU_SAFETY_CLEAR_MS) {
        return false;
    }
    s->output.latched_trigger = (uint8_t)IMU_SAFETY_NONE;
    s->output.current_trigger = (uint8_t)IMU_SAFETY_NONE;
    s->output.overturn_samples = 0u;
    s->output.lift_samples = 0u;
    s->output.clear_ms = 0u;
    s->freefall_samples = 0u;
    return true;
}

uint8_t imu_safety_evaluate(const imu_safety_output_t *out, bool have,
                            uint32_t now_ms, uint32_t max_age_ms)
{
    if (!have || (out == NULL)) {
        return (uint8_t)IMU_SAFETY_IMU_UNAVAILABLE;
    }
    const int32_t age = (int32_t)(now_ms - out->observation_ms);

    if ((age > (int32_t)max_age_ms) ||
        (out->flags & IMU_SAFETY_FLAG_VALID) == 0u) {
        return (uint8_t)IMU_SAFETY_IMU_UNAVAILABLE;
    }
    return out->latched_trigger;
}

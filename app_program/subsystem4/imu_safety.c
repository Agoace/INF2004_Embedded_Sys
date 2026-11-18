/* Pure IMU safety detector. Never drives; decides a candidate trigger only. */
#include "imu_safety.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define RAD_TO_DDEG 572.9577951f

static float magnitude_mg(const float accel_mg[3]);
static int32_t tilt_with_magnitude(const float accel_mg[3], float magnitude);

bool imu_safety_upright_ok(const float accel_mg[3])
{
    if (accel_mg == NULL) {
        return false;
    }
    /* Infinity must not become an accepted calibration pose. */
    return isfinite(accel_mg[0]) && isfinite(accel_mg[1]) &&
           isfinite(accel_mg[2]) &&
           accel_mg[2] >= IMU_SAFETY_UPRIGHT_Z_MG &&
           fabsf(accel_mg[0]) <= IMU_SAFETY_UPRIGHT_XY_MG &&
           fabsf(accel_mg[1]) <= IMU_SAFETY_UPRIGHT_XY_MG;
}

int32_t imu_safety_tilt_ddeg(const float accel_mg[3])
{
    if (accel_mg == NULL) {
        return 0;
    }
    return tilt_with_magnitude(accel_mg, magnitude_mg(accel_mg));
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

/* Tilt from +Z for a caller that already has |a|. One sqrtf and one atan2f
 * are saved by not repeating the magnitude. */
static int32_t tilt_with_magnitude(const float accel_mg[3], float magnitude)
{
    const float x = accel_mg[0];
    const float y = accel_mg[1];
    const float z = accel_mg[2];

    if (!isfinite(x) || !isfinite(y) || !isfinite(z) ||
        !isfinite(magnitude)) {
        return 0;
    }
    if (magnitude <= IMU_SAFETY_G_UNIT_MG) {
        return 0;
    }
    return (int32_t)(atan2f(sqrtf((x * x) + (y * y)), z) * RAD_TO_DDEG);
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

    const float input_magnitude = magnitude_mg(in->accel_mg);
    const bool vector_usable = isfinite(input_magnitude) &&
                               input_magnitude > 0.0f;
    const bool usable = in->accel_valid && in->calibrated && vector_usable;

    if (!usable) {
        /* No angle: an unusable sample cannot trigger. The flags still say
         * WHY it is unusable, so a reader can split the legacy INVALID
         * cause (plan Phase 1.3). */
        o->g_mg = 0.0f;
        o->tilt_ddeg = 0;
        o->vertical_mg = 0.0f;
        o->overturn_samples = 0u;
        o->lift_samples = 0u;
        o->clear_ms = 0u;
        s->freefall_samples = 0u;

        uint8_t blockers = 0u;
        if (!imu_safety_upright_ok(in->accel_mg)) {
            blockers |= IMU_SAFETY_BLOCK_NOT_UPRIGHT;
        }
        const float u_mg = input_magnitude;
        if (!isfinite(u_mg) ||
            (fabsf(u_mg - IMU_SAFETY_GRAVITY_MG) > IMU_SAFETY_NEAR_ONE_G_MG)) {
            blockers |= IMU_SAFETY_BLOCK_NOT_NEAR_ONE_G;
        }
        if (!in->wheels_still) {
            blockers |= IMU_SAFETY_BLOCK_WHEELS_MOVING;
        }
        if (gap) {
            blockers |= IMU_SAFETY_BLOCK_GAP;
        }
        o->clear_blockers = blockers;

        if (!in->accel_valid || !vector_usable) {
            const bool finite = isfinite(in->accel_mg[0]) &&
                                isfinite(in->accel_mg[1]) &&
                                isfinite(in->accel_mg[2]);
            const bool zero = (in->accel_mg[0] == 0.0f) &&
                              (in->accel_mg[1] == 0.0f) &&
                              (in->accel_mg[2] == 0.0f);
            o->flags = !in->read_failed && (!finite || zero || !vector_usable)
                           ? IMU_SAFETY_FLAG_ZERO_VECTOR
                           : IMU_SAFETY_FLAG_READ_FAILED;
        } else {
            o->flags = IMU_SAFETY_FLAG_NOT_CALIBRATED;
        }
        return;
    }

    const float g_mg = input_magnitude;
    const bool finite = isfinite(g_mg);

    o->g_mg = finite ? g_mg : 0.0f;

    const bool near_one_g =
        finite && (fabsf(g_mg - IMU_SAFETY_GRAVITY_MG) <=
                   IMU_SAFETY_NEAR_ONE_G_MG);
    const bool freefall = finite && g_mg < IMU_SAFETY_FREEFALL_MG;
    const bool upright = imu_safety_upright_ok(in->accel_mg);
    /* Inside the upright bounds the tilt cannot reach 43.3 deg (Z >= 600 mg,
     * |X|,|Y| <= 400 mg), so it can never cross the 75 deg overturn limit and
     * the angle is not needed. Computing it only outside the bounds removes one
     * atan2f and one sqrtf per sample from the upright path, which is most of
     * the running time. */
    const int32_t tilt = upright ? 0 : tilt_with_magnitude(in->accel_mg, g_mg);
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

    /* Why the clear window is not advancing, for the field refusal report.
     * Diagnostic only; the window test below is equivalent. */
    uint8_t blockers = 0u;
    if (!upright) {
        blockers |= IMU_SAFETY_BLOCK_NOT_UPRIGHT;
    }
    if (!near_one_g) {
        blockers |= IMU_SAFETY_BLOCK_NOT_NEAR_ONE_G;
    }
    if (!in->wheels_still) {
        blockers |= IMU_SAFETY_BLOCK_WHEELS_MOVING;
    }
    if (gap) {
        blockers |= IMU_SAFETY_BLOCK_GAP;
    }
    o->clear_blockers = blockers;

    /* The all-clear window measures the pose, so it must run even while a
     * trigger is latched; the safe restart needs it. A gap longer than the
     * observation cadence restarts the window: the pose was not observed in
     * that time, so it cannot count as continuously clear (RT-0094/F1). */
    if (blockers == 0u) {
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

    if ((age < 0) || (age > (int32_t)max_age_ms) ||
        (out->flags & IMU_SAFETY_FLAG_VALID) == 0u) {
        return (uint8_t)IMU_SAFETY_IMU_UNAVAILABLE;
    }
    return out->latched_trigger;
}

uint8_t imu_safety_unavailable_cause(const imu_safety_output_t *out, bool have,
                                     uint32_t now_ms, uint32_t max_age_ms)
{
    if (!have || (out == NULL)) {
        return (uint8_t)IMU_SAFETY_CAUSE_ABSENT;
    }
    if (out->flags & IMU_SAFETY_FLAG_ZERO_VECTOR) {
        return (uint8_t)IMU_SAFETY_CAUSE_ZERO_VECTOR;
    }
    if (out->flags & IMU_SAFETY_FLAG_READ_FAILED) {
        return (uint8_t)IMU_SAFETY_CAUSE_READ_FAILED;
    }
    if (out->flags & IMU_SAFETY_FLAG_NOT_CALIBRATED) {
        return (uint8_t)IMU_SAFETY_CAUSE_NOT_CALIBRATED;
    }
    if ((out->flags & IMU_SAFETY_FLAG_VALID) == 0u) {
        /* Legacy generic meaning, kept for any image or fixture that sets no
         * specific flag. */
        return (uint8_t)IMU_SAFETY_CAUSE_INVALID;
    }
    const int32_t age = (int32_t)(now_ms - out->observation_ms);

    if (age < 0 || age > (int32_t)max_age_ms) {
        /* Usable, but the writer has not published for too long. A busy
         * reader also lands here: it keeps the old copy, so the age grows. */
        return (uint8_t)IMU_SAFETY_CAUSE_STALE;
    }
    return (uint8_t)IMU_SAFETY_CAUSE_NONE;
}

/* Observe-only bias/filter/height ideas from Jaison, b2a6bb6.
 * Hump events report only. No kernel calls or driving outputs. */
#include "imu_core.h"
#include "imu_support.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define RAD_TO_DDEG 572.957795f
#define MG_TO_MM_S2 9.80665f
#define GRAVITY_MG 1000.0f

static bool finite_vector(const float v[3])
{
    return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

static float pitch(const float v[3])
{
    return atan2f(v[1], sqrtf(v[0] * v[0] + v[2] * v[2]));
}

static void reset_window(imu_core_t *core)
{
    core->still_count = 0u;
}

static void reset_support_window(imu_core_t *core)
{
    core->support_count = 0u;
    core->support_index = 0u;
    core->support_sum = 0.0f;
    core->support_bad_count = 0u;
    core->output.support_peak_eligible = false;
    imu_roll_step(&core->roll_estimator, 0.0f, 0.0f, IMU_TRACK_MM,
                   IMU_WHEELBASE_MM, 0.0f, false);
    core->output.roll_only = core->roll_estimator.output;
}

static void reset_integral(imu_core_t *core)
{
    core->output.height_mm = 0.0f;
    core->output.peak_mm = 0.0f;
    core->output.distance_mm = 0.0f;
    core->mark_travel = core->output.travel_mm;
}

void imu_core_mark(imu_core_t *core)
{
    if (core != NULL) {
        reset_integral(core);
        core->output.left_peak_mm = 0.0f;
        core->output.right_peak_mm = 0.0f;
        core->output.caster_peak_mm = 0.0f;
        reset_support_window(core);
        imu_roll_mark(&core->roll_estimator);
        core->output.roll_only = core->roll_estimator.output;
    }
}

void imu_core_init(imu_core_t *core)
{
    if (core != NULL) {
        memset(core, 0, sizeof(*core));
        imu_hump_init(&core->hump_detector);
    }
}

/* Feed one observation to the hump detector. valid=false still applies
 * run boundaries and marks a fault inside a run. */
static void hump(imu_core_t *core, const imu_core_input_t *in, float sine,
                 float ds, bool valid)
{
    imu_core_output_t *out = &core->output;
    const imu_hump_input_t h = {
        .height_mm = IMU_TRACK_MM * sine,
        .travel_mm = ds,
        .speed_mm_s = out->speed_mm_s,
        .yaw_rate_rad_s = out->yaw_rate_rad_s,
        .lateral_mg = out->lateral_fake_mm_s2 / MG_TO_MM_S2,
        .run_epoch = in->run_epoch,
        .valid = valid,
        .motion_valid = out->motion_valid};

    imu_hump_step(&core->hump_detector, &h);
    out->hump = core->hump_detector.output;
}

/* Candidate bias is the stationary mean minus level gravity. Check the
 * whole corrected axis envelope, NOT the tautologically unit mean. */
static bool calibration_good(imu_core_t *core)
{
    float low_squared = 0.0f;
    float high_squared = 0.0f;
    bool quiet = true;

    for (unsigned int axis = 0u; axis < 3u; ++axis) {
        const float expected = axis == 2u ? GRAVITY_MG : 0.0f;
        const float bias = core->mean[axis] - expected;
        const float low = core->minimum[axis] - bias;
        const float high = core->maximum[axis] - bias;
        const float near = low <= 0.0f && high >= 0.0f
                               ? 0.0f : fminf(fabsf(low), fabsf(high));
        const float far = fmaxf(fabsf(low), fabsf(high));
        const float variance = core->m2[axis] /
                               (float)(core->still_count - 1u);

        core->output.still_std_mg[axis] = sqrtf(fmaxf(variance, 0.0f));
        quiet = quiet && core->output.still_std_mg[axis] <=
                         IMU_STILL_STD_MG;
        low_squared += near * near;
        high_squared += far * far;
    }
    return quiet && low_squared >= 900.0f * 900.0f &&
           high_squared <= 1100.0f * 1100.0f;
}

static void calibrate(imu_core_t *core, const imu_core_input_t *in)
{
    if (!in->motion_valid || !in->wheels_still ||
        !in->calibration_allowed || (in->calibration_epoch & 1u) != 0u) {
        reset_window(core);
        return;
    }
    if (core->still_count == 0u) {
        core->still_start = in->now_ms;
        for (unsigned int axis = 0u; axis < 3u; ++axis) {
            core->mean[axis] = 0.0f;
            core->m2[axis] = 0.0f;
            core->minimum[axis] = in->accel_mg[axis];
            core->maximum[axis] = in->accel_mg[axis];
        }
    }
    ++core->still_count;
    for (unsigned int axis = 0u; axis < 3u; ++axis) {
        const float delta = in->accel_mg[axis] - core->mean[axis];

        core->mean[axis] += delta / (float)core->still_count;
        core->m2[axis] += delta * (in->accel_mg[axis] - core->mean[axis]);
        core->minimum[axis] = fminf(core->minimum[axis], in->accel_mg[axis]);
        core->maximum[axis] = fmaxf(core->maximum[axis], in->accel_mg[axis]);
    }
    if ((uint32_t)(in->now_ms - core->still_start) < IMU_STILL_MS ||
        core->still_count < IMU_STILL_MIN_SAMPLES) {
        return;
    }
    bool refresh_good = true;

    if (core->output.calibrated) {
        const float x = core->mean[0] - core->output.bias_mg[0];
        const float y = core->mean[1] - core->output.bias_mg[1];
        const float z = core->mean[2] - core->output.bias_mg[2];
        const float tilt = atan2f(sqrtf(x * x + y * y), z);

        core->output.refresh_tilt_ddeg = (int32_t)(tilt * RAD_TO_DDEG);
        refresh_good = tilt * RAD_TO_DDEG < IMU_REFRESH_TILT_DEG * 10.0f;
    }
    if (calibration_good(core) && refresh_good) {
        for (unsigned int axis = 0u; axis < 3u; ++axis) {
            core->output.bias_mg[axis] = core->mean[axis] -
                                       (axis == 2u ? GRAVITY_MG : 0.0f);
        }
        core->output.calibrated = true;
        ++core->output.calibrations;
        reset_integral(core);
        reset_support_window(core);
        /* Support peaks belong to MARK, not to a stationary refresh. */
        core->have_filter = false;
        core->output.forward_accel_mm_s2 = 0.0f;
    } else {
        ++core->output.calibration_rejects;
        if (!refresh_good) {
            ++core->output.refresh_rejects;
        }
    }
    reset_window(core);
}

/* Velocity uses a 100 ms count-distance window; acceleration is a signed
 * velocity derivative at window midpoints, then a 100 ms low-pass. */
static void velocity(imu_core_t *core, float ds, float dyaw, uint32_t dt,
                     uint32_t now)
{
    core->window_yaw += dyaw;
    core->window_distance += ds;
    core->window_ms += dt;
    if (core->window_ms >= IMU_ENCODER_WINDOW_MS) {
        const float v = core->window_distance * 1000.0f /
                        (float)core->window_ms;
        const float omega = core->window_yaw * 1000.0f /
                            (float)core->window_ms;
        const uint32_t midpoint = now - core->window_ms / 2u;

        if (core->have_speed) {
            const uint32_t delta = midpoint - core->velocity_ms;

            if (delta > 0u && delta <= 2u * IMU_ENCODER_WINDOW_MS) {
                const float a = (v - core->previous_speed) * 1000.0f /
                                (float)delta;
                const float alpha = (float)delta /
                                    (IMU_FILTER_TAU_MS + (float)delta);

                const float angular = (omega - core->previous_yaw_rate) *
                                      1000.0f / (float)delta;

                core->output.forward_accel_mm_s2 += alpha *
                    (a - core->output.forward_accel_mm_s2);
                core->output.yaw_accel_rad_s2 += alpha *
                    (angular - core->output.yaw_accel_rad_s2);
            }
        }
        core->output.speed_mm_s = v;
        core->output.yaw_rate_rad_s = omega;
        core->previous_yaw_rate = omega;
        core->previous_speed = v;
        core->velocity_ms = midpoint;
        core->have_speed = true;
        core->window_ms = 0u;
        core->window_distance = 0.0f;
        core->window_yaw = 0.0f;
    }
}

static void break_motion(imu_core_t *core)
{
    core->have_motion = false;
    core->have_speed = false;
    core->window_ms = 0u;
    core->window_distance = 0.0f;
    core->output.forward_accel_mm_s2 = 0.0f;
    core->output.speed_mm_s = 0.0f;
    core->output.yaw_rate_rad_s = 0.0f;
    core->output.yaw_accel_rad_s2 = 0.0f;
    core->window_yaw = 0.0f;
}

/* Return distance only for a distinct, bounded encoder interval. */
static float motion(imu_core_t *core, const imu_core_input_t *in,
                     bool *distinct)
{
    float ds = 0.0f;

    *distinct = false;
    core->output.motion_valid = in->motion_valid &&
                                isfinite(in->travel_mm) &&
                                isfinite(in->yaw_rad);
    if (!core->output.motion_valid) {
        ++core->output.motion_misses;
        break_motion(core);
        return 0.0f;
    }
    core->output.travel_mm = in->travel_mm;
    if (!core->have_reference) {
        core->mark_travel = in->travel_mm;
        core->have_reference = true;
    }
    core->output.distance_mm = in->travel_mm - core->mark_travel;
    if (core->have_motion) {
        const uint32_t dt = in->travel_ms - core->previous_travel_ms;

        ds = in->travel_mm - core->previous_travel;
        if (dt == 0u && ds == 0.0f &&
            in->yaw_rad == core->previous_yaw) {
            return 0.0f;
        }
        if (dt == 0u || dt > IMU_MAX_GAP_MS || !isfinite(ds)) {
            ++core->output.gap_resets;
            break_motion(core);
            ds = 0.0f;
        } else {
            *distinct = true;
            velocity(core, ds, in->yaw_rad - core->previous_yaw, dt,
                      in->travel_ms);
        }
    }
    core->previous_travel = in->travel_mm;
    core->previous_yaw = in->yaw_rad;
    core->previous_travel_ms = in->travel_ms;
    core->have_motion = true;
    return ds;
}

/* Quality tolerance affects only the primary smoothed peak, not heights,
 * secondary peaks, driving or any terrain state/event. */
static void support(imu_core_t *core, float pitch_sine, float roll_sine)
{
    imu_core_output_t *out = &core->output;
    const imu_support_t z = imu_support_from_sines(pitch_sine, roll_sine,
                            IMU_WHEELBASE_MM, IMU_TRACK_MM);

    out->left_mm = z.left_mm;
    out->right_mm = z.right_mm;
    out->caster_mm = z.caster_mm;
    out->left_residual_mm = fabsf(z.left_mm);
    if (!out->calibrated) {
        reset_support_window(core);
        return;
    }
    out->left_peak_mm = fmaxf(out->left_peak_mm, z.left_mm);
    out->caster_peak_mm = fmaxf(out->caster_peak_mm, z.caster_mm);
    if (!out->motion_valid) {
        reset_support_window(core);
        return;
    }
    const unsigned int index = core->support_index;
    const bool bad = out->left_residual_mm > IMU_SUPPORT_TOLERANCE_MM;

    if (core->support_count == IMU_SUPPORT_WINDOW_SAMPLES) {
        core->support_sum -= core->support_window[index];
        if (core->support_bad[index]) {
            --core->support_bad_count;
        }
    } else {
        ++core->support_count;
    }
    core->support_window[index] = z.right_mm;
    core->support_bad[index] = bad;
    core->support_sum += z.right_mm;
    if (bad) {
        ++core->support_bad_count;
    }
    core->support_index = (index + 1u) % IMU_SUPPORT_WINDOW_SAMPLES;
    out->right_smoothed_mm = core->support_sum / (float)core->support_count;
    out->support_peak_eligible = core->support_count ==
        IMU_SUPPORT_WINDOW_SAMPLES && core->support_bad_count == 0u;
    if (out->support_peak_eligible) {
        out->right_peak_mm = fmaxf(out->right_peak_mm,
                                  out->right_smoothed_mm);
    } else if (core->support_count == IMU_SUPPORT_WINDOW_SAMPLES) {
        ++out->support_peak_rejects;
    }
}

void imu_core_step(imu_core_t *core, const imu_core_input_t *in)
{
    uint32_t dt;
    float ds;
    bool distinct;
    bool sample_time;
    imu_core_output_t *out;

    if (core == NULL || in == NULL) {
        return;
    }
    out = &core->output;
    ++out->samples;
    dt = in->now_ms - core->previous_ms;
    sample_time = core->have_sample && dt > 0u && dt <= IMU_MAX_GAP_MS;
    if (!sample_time || core->epoch != in->calibration_epoch) {
        reset_window(core);
    }
    core->epoch = in->calibration_epoch;
    if (core->have_sample && !sample_time) {
        ++out->gap_resets;
        break_motion(core);
        core->have_filter = false;
        reset_support_window(core);
    }
    core->previous_ms = in->now_ms;
    core->have_sample = true;
    ds = motion(core, in, &distinct);
    out->valid = in->accel_valid && finite_vector(in->accel_mg) &&
                 (in->accel_mg[0] != 0.0f || in->accel_mg[1] != 0.0f ||
                  in->accel_mg[2] != 0.0f);
    if (!out->valid) {
        ++out->invalid_samples;
        reset_window(core);
        break_motion(core);
        core->have_filter = false;
        reset_support_window(core);
        hump(core, in, 0.0f, 0.0f, false);
        return;
    }
    ++out->valid_samples;
    /* The motion helper also validates finite travel; propagate that to
     * calibration instead of treating an invalid source as stationary. */
    if (out->motion_valid) {
        calibrate(core, in);
    } else {
        reset_window(core);
    }
    for (unsigned int axis = 0u; axis < 3u; ++axis) {
        out->accel_mg[axis] = in->accel_mg[axis] - out->bias_mg[axis];
    }
    const float p = pitch(out->accel_mg);
    const float r = atan2f(out->accel_mg[0], out->accel_mg[2]);
    /* +omega is LEFT, therefore centripetal acceleration is -X.
     * The forward lever arm contributes -X*alpha and -Y*omega^2.
     * Sensor height has no contribution in this planar yaw model. */
    const float omega = out->yaw_rate_rad_s;

    out->lateral_fake_mm_s2 = -out->speed_mm_s * omega -
                             IMU_FORWARD_MM * out->yaw_accel_rad_s2;
    out->forward_fake_mm_s2 = out->forward_accel_mm_s2 -
                             IMU_FORWARD_MM * omega * omega;
    const float corrected[3] = {
        out->accel_mg[0] - out->lateral_fake_mm_s2 / MG_TO_MM_S2,
        out->accel_mg[1] - out->forward_fake_mm_s2 / MG_TO_MM_S2,
        out->accel_mg[2]
    };
    const float cp = pitch(corrected);
    const float cr = atan2f(corrected[0], corrected[2]);
    const float alpha = (float)dt / (IMU_FILTER_TAU_MS + (float)dt);

    out->raw_pitch_ddeg = (int32_t)(pitch(in->accel_mg) * RAD_TO_DDEG);
    out->raw_roll_ddeg = (int32_t)(atan2f(in->accel_mg[0], in->accel_mg[2]) *
                                   RAD_TO_DDEG);
    out->pitch_ddeg = (int32_t)(p * RAD_TO_DDEG);
    out->roll_ddeg = (int32_t)(r * RAD_TO_DDEG);
    if (!core->have_filter) {
        core->pitch_rad = p;
        core->roll_rad = r;
        core->corrected_rad = cp;
        core->corrected_roll_rad = cr;
        core->have_filter = true;
    } else {
        core->pitch_rad += alpha * (p - core->pitch_rad);
        core->roll_rad += alpha * (r - core->roll_rad);
        core->corrected_rad += alpha * (cp - core->corrected_rad);
        core->corrected_roll_rad += alpha * (cr - core->corrected_roll_rad);
    }
    out->filtered_pitch_ddeg = (int32_t)(core->pitch_rad * RAD_TO_DDEG);
    out->filtered_roll_ddeg = (int32_t)(core->roll_rad * RAD_TO_DDEG);
    out->corrected_pitch_ddeg = (int32_t)(core->corrected_rad * RAD_TO_DDEG);
    out->corrected_roll_ddeg = (int32_t)(core->corrected_roll_rad *
                                       RAD_TO_DDEG);
    const float pitch_sine = sinf(core->corrected_rad);
    const float corrected_roll_sine = sinf(core->corrected_roll_rad);
    const float roll_sine = sinf(core->roll_rad);

    support(core, pitch_sine, corrected_roll_sine);
    /* Match the operator's roll-only offline model: bias-corrected roll
     * LPF, NOT yaw-corrected roll. Keep both corrected axes for diagnostics. */
    imu_roll_step(&core->roll_estimator, roll_sine,
                   core->corrected_rad, IMU_TRACK_MM, IMU_WHEELBASE_MM,
                   IMU_ROLL_PITCH_TOLERANCE_DEG / 57.2957795f,
                   out->calibrated && out->motion_valid);
    out->roll_only = core->roll_estimator.output;
    /* Step 3 events: the yaw-corrected roll removes the encoder v*omega and
     * lever-arm terms on curves; FILTERED is the step 2d primary input. */
    const float hump_sine = IMU_HUMP_SOURCE == IMU_HUMP_SOURCE_FILTERED
                                ? roll_sine : corrected_roll_sine;

    hump(core, in, hump_sine, ds, out->calibrated && sample_time);
    if (out->calibrated && sample_time && distinct) {
        out->height_mm += pitch_sine * ds;
        out->peak_mm = fmaxf(out->peak_mm, out->height_mm);
        ++out->integrated_samples;
    }
}

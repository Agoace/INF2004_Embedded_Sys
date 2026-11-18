/* Pure observer. Quality is diagnostic, never a height/peak gate. */
#include "imu_roll.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static void clear_windows(imu_roll_t *s)
{
    s->mean_count = 0u;
    s->mean_index = 0u;
    s->mean_sum = 0.0f;
    s->plateau_count = 0u;
    s->plateau_index = 0u;
    s->output.smooth_ready = false;
    s->output.plateau_ready = false;
    s->output.quality_good = false;
}

void imu_roll_init(imu_roll_t *s)
{
    if (s != NULL) {
        memset(s, 0, sizeof(*s));
    }
}

void imu_roll_mark(imu_roll_t *s)
{
    if (s != NULL) {
        clear_windows(s);
        s->output.peak_mm = 0.0f;
        s->output.plateau_mm = 0.0f;
        s->output.quality_samples = 0u;
        s->output.quality_matches = 0u;
        s->output.quality_ratio = 0.0f;
    }
}

static float median(const float *window)
{
    float sorted[IMU_ROLL_PLATEAU_SAMPLES];

    for (unsigned int i = 0u; i < IMU_ROLL_PLATEAU_SAMPLES; ++i) {
        unsigned int j = i;
        const float x = window[i];

        while (j > 0u && sorted[j - 1u] > x) {
            sorted[j] = sorted[j - 1u];
            --j;
        }
        sorted[j] = x;
    }
    const unsigned int mid = IMU_ROLL_PLATEAU_SAMPLES / 2u;

    return IMU_ROLL_PLATEAU_SAMPLES % 2u != 0u ? sorted[mid] :
           (sorted[mid - 1u] + sorted[mid]) * 0.5f;
}

void imu_roll_step(imu_roll_t *s, float roll_sine, float measured_pitch_rad,
                   float track_mm, float wb_mm, float tolerance_rad,
                   bool valid)
{
    if (s == NULL) {
        return;
    }
    if (!valid || !isfinite(roll_sine) || fabsf(roll_sine) > 1.0f ||
        !isfinite(measured_pitch_rad) || !isfinite(track_mm) ||
        !isfinite(wb_mm) || !isfinite(tolerance_rad) || track_mm <= 0.0f ||
        wb_mm <= 0.0f || tolerance_rad < 0.0f) {
        clear_windows(s);
        return;
    }
    imu_roll_output_t *o = &s->output;
    const float height = track_mm * roll_sine;
    const float ratio = fmaxf(-1.0f, fminf(1.0f, height / (2.0f * wb_mm)));

    o->height_mm = height;
    o->expected_pitch_rad = -asinf(ratio);
    o->quality_good = fabsf(measured_pitch_rad - o->expected_pitch_rad) <=
                      tolerance_rad;
    if (o->quality_samples != UINT32_MAX) {
        ++o->quality_samples;
        o->quality_matches += o->quality_good ? 1u : 0u;
        o->quality_ratio = (float)o->quality_matches /
                           (float)o->quality_samples;
    }
    if (s->mean_count == IMU_ROLL_MEAN_SAMPLES) {
        s->mean_sum -= s->mean_window[s->mean_index];
    } else {
        ++s->mean_count;
    }
    s->mean_window[s->mean_index] = height;
    s->mean_sum += height;
    s->mean_index = (s->mean_index + 1u) % IMU_ROLL_MEAN_SAMPLES;
    o->smooth_ready = s->mean_count == IMU_ROLL_MEAN_SAMPLES;
    if (!o->smooth_ready) {
        return;
    }
    o->smoothed_mm = s->mean_sum / (float)IMU_ROLL_MEAN_SAMPLES;
    o->peak_mm = fmaxf(o->peak_mm, o->smoothed_mm);
    s->plateau_window[s->plateau_index] = o->smoothed_mm;
    s->plateau_index = (s->plateau_index + 1u) % IMU_ROLL_PLATEAU_SAMPLES;
    if (s->plateau_count < IMU_ROLL_PLATEAU_SAMPLES) {
        ++s->plateau_count;
    }
    o->plateau_ready = s->plateau_count == IMU_ROLL_PLATEAU_SAMPLES;
    if (o->plateau_ready) {
        o->plateau_mm = fmaxf(o->plateau_mm, median(s->plateau_window));
    }
}

/* Synthetic geometry/filter fixtures, NOT hardware accuracy evidence. */
#include "../app_program/subsystem4/imu_core.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void sample(imu_roll_t *s, float h, float pitch)
{
    imu_roll_step(s, h / 111.0f, pitch, 111.0f, 80.0f, 0.174532925f, true);
}

static void contracts(void)
{
    imu_roll_t s;

    imu_roll_init(NULL);
    imu_roll_mark(NULL);
    imu_roll_step(NULL, 0, 0, 111, 80, 1, true);
    imu_roll_init(&s);
    for (unsigned int i = 0u; i < 100u; ++i) {
        sample(&s, 10.0f, 0.5f); /* Deliberately inconsistent pitch. */
    }
    assert(!s.output.quality_good && s.output.quality_ratio == 0.0f);
    assert(s.output.smooth_ready && s.output.plateau_ready);
    assert(fabsf(s.output.peak_mm - 10.0f) < 0.001f);
    assert(fabsf(s.output.plateau_mm - 10.0f) < 0.001f);
    for (unsigned int i = 0u; i < 100u; ++i) {
        sample(&s, 10.0f, -asinf(10.0f / 160.0f));
    }
    assert(fabsf(s.output.quality_ratio - 0.5f) < 0.0001f);
    imu_roll_step(&s, 0, 0, 111, 80, 1, false);
    assert(!s.output.smooth_ready && !s.output.plateau_ready);
    assert(s.output.peak_mm > 9.99f && s.output.quality_samples == 200u);
    imu_roll_mark(&s);
    assert(s.output.peak_mm == 0 && s.output.plateau_mm == 0);
    assert(s.output.quality_samples == 0u);
    sample(&s, NAN, 0);
    assert(s.mean_count == 0u);
    imu_roll_init(&s);
    for (unsigned int i = 0u; i < 250u; ++i) {
        sample(&s, i == 100u ? 20.0f : 0.0f, 0);
    }
    assert(fabsf(s.output.peak_mm - 2.0f) < 0.001f);
    const float spike_median = IMU_ROLL_PLATEAU_SAMPLES == 15u ? 2.0f :
        IMU_ROLL_PLATEAU_SAMPLES == 20u ? 1.0f : 0.0f;

    assert(fabsf(s.output.plateau_mm - spike_median) < 0.001f);
    for (int height = -20; height <= 20; height += 5) {
        imu_roll_init(&s);
        for (unsigned int i = 0u; i < 100u; ++i) {
            sample(&s, (float)height, -asinf((float)height / 160.0f));
        }
        assert(fabsf(s.output.height_mm - (float)height) < 0.001f);
        assert(fabsf(s.output.peak_mm - fmaxf(0, (float)height)) < 0.001f);
    }
}

static void profile(bool flat_top)
{
    imu_roll_t s;
    float roll = 0.0f;
    const float alpha = 10.0f / (IMU_FILTER_TAU_MS + 10.0f);
    const float crest_end = flat_top ? 145.0f : 100.0f;

    imu_roll_init(&s);
    for (int tick = -200; tick < 600; ++tick) {
        const float x = (float)tick; /* 1 mm/tick at 100 mm/s. */
        const float h = x < 0 ? 0 : x < 95 ? 10.0f * x / 95.0f :
            x <= crest_end ? 10.0f :
            fmaxf(0, 10.0f * (crest_end + 95.0f - x) / 95.0f);

        roll += alpha * (asinf(h / 111.0f) - roll);
        sample(&s, 111.0f * sinf(roll), -asinf(h / 160.0f));
    }
    printf("fixture %s N%u: mean_peak %.3f median_max %.3f bias %.3f mm\n",
           flat_top ? "flat500ms" : "triangle195mm",
           IMU_ROLL_PLATEAU_SAMPLES, (double)s.output.peak_mm,
           (double)s.output.plateau_mm,
           (double)(s.output.plateau_mm - 10.0f));
    assert(s.output.peak_mm > 8.5f && s.output.peak_mm <= 10.001f);
    assert(s.output.plateau_mm > 7.0f && s.output.plateau_mm <= 10.001f);
}

static void noise_and_step(void)
{
    imu_roll_t s;
    uint32_t seed = 12345u;
    float roll = 0.0f;
    unsigned int t10 = 0u, t50 = 0u, t90 = 0u, median90 = 0u;

    imu_roll_init(&s);
    for (unsigned int tick = 0u; tick < 1000u; ++tick) {
        seed = seed * 1664525u + 1013904223u;
        /* White noise AFTER roll LPF: measured driving roll sd about2mm.
         * Uniform [-sqrt(12),sqrt(12)] has sd2mm. Spectrum is NOT measured. */
        const float noise = ((float)(seed >> 8) / 16777215.0f - 0.5f) *
                            6.92820323f;
        sample(&s, noise, 0);
    }
    printf("fixture flat white sd2mm N%u: mean_peak %.3f median_max %.3f\n",
           IMU_ROLL_PLATEAU_SAMPLES, (double)s.output.peak_mm,
           (double)s.output.plateau_mm);
    imu_roll_init(&s);
    for (unsigned int tick = 0u; tick < 1000u; ++tick) {
        const float noise = 2.82842712f *
            sinf((float)tick * 6.28318531f / 40.0f);

        sample(&s, noise, 0);
    }
    printf("fixture flat sine400ms sd2mm N%u: median_max %.3f\n",
           IMU_ROLL_PLATEAU_SAMPLES, (double)s.output.plateau_mm);
    imu_roll_init(&s);
    for (unsigned int tick = 0u; tick < 200u; ++tick) {
        sample(&s, 0, 0);
    }
    for (unsigned int tick = 1u; tick < 200u; ++tick) {
        roll += (asinf(10.0f / 111.0f) - roll) / 11.0f;
        sample(&s, 111.0f * sinf(roll), 0);
        if (t10 == 0u && s.output.smoothed_mm >= 1) { t10 = tick * 10u; }
        if (t50 == 0u && s.output.smoothed_mm >= 5) { t50 = tick * 10u; }
        if (t90 == 0u && s.output.smoothed_mm >= 9) { t90 = tick * 10u; }
        if (median90 == 0u && s.output.plateau_mm >= 9) {
            median90 = tick * 10u;
        }
    }
    printf("fixture step N%u: mean t10/t50/t90 %u/%u/%u median t90 %u ms\n",
           IMU_ROLL_PLATEAU_SAMPLES, t10, t50, t90, median90);
    assert(t90 < 400u && median90 < 600u);
}

int main(void)
{
    contracts();
    profile(false);
    profile(true);
    noise_and_step();
    return 0;
}

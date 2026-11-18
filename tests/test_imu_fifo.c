/* FIFO/alias fixtures only: no sensor, SDK, kernel or motor. */
#include "../app_program/subsystem4/imu_fifo.h"
#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

static void alias_fixture(float frequency)
{
    const float pi = 3.14159265359f;
    /* Folded frequency after 100 Hz decimation, 0..50 Hz. */
    const float alias = fabsf(frequency - 100.0f *
                              floorf(frequency / 100.0f + 0.5f));
    double mean_sin = 0, mean_cos = 0, single_sin = 0, single_cos = 0;
    const unsigned int n = 1000u;

    for (unsigned int i = 0u; i < n; ++i) {
        float sum = 0;
        float first = 0;

        for (unsigned int k = 0u; k < 4u; ++k) {
            const float x = sinf(2 * pi * frequency *
                                 (float)(i * 4u + k) / 400.0f);
            if (k == 0u) { first = x; }
            sum += x;
        }
        const float phase = 2 * pi * alias * (float)i / 100.0f;

        mean_sin += (double)(sum * 0.25f * sinf(phase));
        mean_cos += (double)(sum * 0.25f * cosf(phase));
        single_sin += (double)(first * sinf(phase));
        single_cos += (double)(first * cosf(phase));
    }
    const double averaged = 2 * sqrt(mean_sin * mean_sin +
                                      mean_cos * mean_cos) / (double)n;
    const double single = 2 * sqrt(single_sin * single_sin +
                                    single_cos * single_cos) / (double)n;
    const double theory = fabs(sin((double)pi * (double)frequency / 100.0) /
        (4 * sin((double)pi * (double)frequency / 400.0)));

    printf("fixture %.0fHz -> %.0fHz: single %.4f mean4 %.4f theory %.4f\n",
           (double)frequency, (double)alias, single, averaged, theory);
    assert(fabs(single - 1) < 0.002 && fabs(averaged - theory) < 0.002);
}

int main(void)
{
    unsigned int count;
    int16_t raw[2][3] = {{16000, -16, 32767}, {16016, -32, -32768}};
    float mean[3];
    const uint8_t bytes[6] = {0, 0x80, 0xff, 0x7f, 0xf0, 0xff};
    int16_t decoded[3];

    assert(imu_fifo_count(IMU_FIFO_EMPTY, &count) && count == 0u);
    for (uint8_t i = 1u; i < 32u; ++i) {
        assert(imu_fifo_count(i, &count) && count == i);
    }
    assert(!imu_fifo_count(0, &count));
    assert(!imu_fifo_count(0x21u, &count));
    assert(!imu_fifo_count(0x40u, &count) && count == 32u);
    assert(!imu_fifo_count(0xffu, &count) && count == 32u);
    assert(!imu_fifo_count(1u, NULL));
    imu_fifo_words(bytes, decoded);
    assert(decoded[0] == -32768 && decoded[1] == 32767 && decoded[2] == -16);
    assert(imu_fifo_mean(raw, 2u, mean));
    assert(mean[0] == 1000.5f && mean[1] == -1.5f && mean[2] == -0.5f);
    assert(!imu_fifo_mean(raw, 0u, mean));
    assert(!imu_fifo_mean(raw, IMU_FIFO_MAX_BATCH + 1u, mean));
    alias_fixture(37.0f);   /* Below 50 Hz: attenuation, NOT aliasing. */
    alias_fixture(113.0f);  /* Folds to 13 Hz, inside the 12..25 Hz band. */
    alias_fixture(137.0f);  /* Folds to 37 Hz: incomplete suppression. */
    alias_fixture(187.0f);  /* Folds to 13 Hz from the other side. */
    return 0;
}

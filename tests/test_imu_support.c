#include "../app_program/subsystem4/imu_support.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define EPSILON_MM 0.0001f
#define DEG_TO_RAD 0.017453292519943295f

static void close_mm(float actual, float expected)
{
    assert(isfinite(actual));
    assert(fabsf(actual - expected) <= EPSILON_MM);
}

/* Four independent block placements. Alternate hypotheses stay signed. */
static void blocks(float height, float wheelbase, float track)
{
    const float axle_pitch = asinf(-height / (2.0f * wheelbase));
    const float wheel_roll = asinf(height / track);
    imu_support_t result = imu_support(axle_pitch, -wheel_roll,
                                       wheelbase, track);

    close_mm(result.left_mm, height);
    close_mm(result.right_mm, 0.0f);
    close_mm(result.caster_mm, -height * 0.5f);
    result = imu_support(axle_pitch, wheel_roll, wheelbase, track);
    close_mm(result.left_mm, 0.0f);
    close_mm(result.right_mm, height);
    close_mm(result.caster_mm, -height * 0.5f);
    result = imu_support(asinf(-height / wheelbase), 0.0f,
                         wheelbase, track);
    close_mm(result.left_mm, height);
    close_mm(result.right_mm, height);
    close_mm(result.caster_mm, -height);
    result = imu_support(asinf(height / wheelbase), 0.0f,
                         wheelbase, track);
    close_mm(result.left_mm, -height);
    close_mm(result.right_mm, -height);
    close_mm(result.caster_mm, height);
}

static void symmetry_and_reuse(void)
{
    for (int p = -8; p <= 8; ++p) {
        for (int r = -8; r <= 8; ++r) {
            const float pitch = (float)p * DEG_TO_RAD;
            const float roll = (float)r * DEG_TO_RAD;
            const imu_support_t angles = imu_support(pitch, roll,
                IMU_SUPPORT_WHEELBASE_MM, IMU_SUPPORT_TRACK_MM);
            const imu_support_t sines = imu_support_from_sines(
                sinf(pitch), sinf(roll), IMU_SUPPORT_WHEELBASE_MM,
                IMU_SUPPORT_TRACK_MM);
            const imu_support_t mirror = imu_support(pitch, -roll,
                IMU_SUPPORT_WHEELBASE_MM, IMU_SUPPORT_TRACK_MM);
            const imu_support_t inverted = imu_support(-pitch, -roll,
                IMU_SUPPORT_WHEELBASE_MM, IMU_SUPPORT_TRACK_MM);

            close_mm(angles.left_mm, sines.left_mm);
            close_mm(angles.right_mm, sines.right_mm);
            close_mm(angles.caster_mm, sines.caster_mm);
            close_mm(angles.left_mm, mirror.right_mm);
            close_mm(angles.right_mm, mirror.left_mm);
            close_mm(angles.caster_mm, mirror.caster_mm);
            close_mm(angles.left_mm, -inverted.left_mm);
            close_mm(angles.right_mm, -inverted.right_mm);
            close_mm(angles.caster_mm, -inverted.caster_mm);
        }
    }
}

static void specified_sample(void)
{
    const imu_support_t result = imu_support(-5.0f * DEG_TO_RAD,
        7.1f * DEG_TO_RAD, IMU_SUPPORT_WHEELBASE_MM, IMU_SUPPORT_TRACK_MM);

    /* This sample indicates RIGHT, not the course's primary LEFT wheel. */
    assert(result.right_mm > 13.8f && result.right_mm < 13.9f);
    assert(fabsf(result.left_mm) < 0.2f);
    close_mm(result.caster_mm, 80.0f * sinf(-5.0f * DEG_TO_RAD));
    printf("support sample: left %.3f right %.3f caster %.3f mm\n",
           (double)result.left_mm, (double)result.right_mm,
           (double)result.caster_mm);
}

static void sine_limits(void)
{
    const imu_support_t result = imu_support_from_sines(1.0f, -1.0f,
        IMU_SUPPORT_WHEELBASE_MM, IMU_SUPPORT_TRACK_MM);

    close_mm(result.left_mm, -24.5f);
    close_mm(result.right_mm, -135.5f);
    close_mm(result.caster_mm, 80.0f);
}

int main(void)
{
    for (unsigned int h = 0u; h <= 20u; h += 5u) {
        blocks((float)h, IMU_SUPPORT_WHEELBASE_MM, IMU_SUPPORT_TRACK_MM);
        blocks(-(float)h, IMU_SUPPORT_WHEELBASE_MM, IMU_SUPPORT_TRACK_MM);
        blocks((float)h, 100.0f, 160.0f);
        blocks(-(float)h, 100.0f, 160.0f);
    }
    symmetry_and_reuse();
    specified_sample();
    sine_limits();
    return 0;
}

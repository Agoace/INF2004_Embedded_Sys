/* Pure geometry only. Calibration and filtering belong to the caller. */
#include "imu_support.h"
#include <math.h>

imu_support_t imu_support_from_sines(float pitch_sine, float roll_sine,
                                     float wheelbase_mm, float track_mm)
{
    const float pitch_height = -wheelbase_mm * pitch_sine;
    const float roll_height = (track_mm * 0.5f) * roll_sine;
    const imu_support_t result = {
        .left_mm = pitch_height - roll_height,
        .right_mm = pitch_height + roll_height,
        .caster_mm = -pitch_height
    };

    return result;
}

imu_support_t imu_support(float pitch_rad, float roll_rad,
                          float wheelbase_mm, float track_mm)
{
    const float pitch_sine = sinf(pitch_rad);
    const float roll_sine = sinf(roll_rad);

    return imu_support_from_sines(pitch_sine, roll_sine,
                                   wheelbase_mm, track_mm);
}

/* Pure decode/axis mapping, adapted from Jaison's IMU-Telemetry at b2a6bb6.
 * Mounting signs require the operator tilt check, not an assumed calibration.
 */
#include "imu_decode.h"
#include <math.h>
#include <stddef.h>

const imu_axes_t imu_robot_axes = {
    .accel = {{0u, 1u, 2u}, {1, 1, 1}},
    .mag = {{0u, 1u, 2u}, {1, 1, 1}}
};

static int32_t signed16(uint16_t word)
{
    return (word >= 32768u) ? ((int32_t)word - 65536) : (int32_t)word;
}

static bool valid_map(const imu_axis_map_t *map)
{
    unsigned int used = 0u;

    for (unsigned int i = 0u; i < 3u; ++i) {
        if ((map->source[i] > 2u) ||
            ((map->sign[i] != 1) && (map->sign[i] != -1))) {
            return false;
        }
        used |= 1u << map->source[i];
    }
    return used == 7u;
}

bool imu_decode(const uint8_t accel[6], const uint8_t mag[6],
                const imu_axes_t *axes, imu_units_t *out)
{
    int32_t a[3];
    int32_t m[3];
    static const unsigned int mag_order[3] = {0u, 4u, 2u};
    imu_units_t result = {.mag_valid = true};

    if ((accel == NULL) || (mag == NULL) || (axes == NULL) || (out == NULL) ||
        !valid_map(&axes->accel) || !valid_map(&axes->mag)) {
        return false;
    }
    for (unsigned int i = 0u; i < 3u; ++i) {
        const unsigned int j = mag_order[i];
        const uint16_t aw = (uint16_t)((uint16_t)accel[2u * i] |
                           ((uint16_t)accel[(2u * i) + 1u] << 8));
        const uint16_t mw = (uint16_t)(((uint16_t)mag[j] << 8) | mag[j + 1u]);
        const int32_t raw_m = signed16(mw);

        /* Divide the unsigned word first: unused accel low bits are ignored,
         * including for negative samples (no signed right-shift assumption). */
        a[i] = ((aw & 0x8000u) != 0u) ? (int32_t)(aw / 16u) - 4096
                                         : (int32_t)(aw / 16u);
        m[i] = (raw_m * 1000) / ((i == 2u) ? 980 : 1100);
        result.mag_valid = result.mag_valid && (raw_m != -4096);
    }
    for (unsigned int i = 0u; i < 3u; ++i) {
        result.accel_mg[i] = a[axes->accel.source[i]] * axes->accel.sign[i];
        result.mag_mgauss[i] = m[axes->mag.source[i]] * axes->mag.sign[i];
    }
    *out = result;
    return true;
}

bool imu_tilt(const int32_t accel_mg[3], float *g_mg,
              float *pitch_deg, float *roll_deg)
{
    const float x = (float)accel_mg[0];
    const float y = (float)accel_mg[1];
    const float z = (float)accel_mg[2];
    const float radians_to_deg = 57.2957795f;

    *g_mg = sqrtf((x * x) + (y * y) + (z * z));
    if (*g_mg == 0.0f) {
        *pitch_deg = 0.0f;
        *roll_deg = 0.0f;
        return false;
    }
    *pitch_deg = atan2f(y, sqrtf((x * x) + (z * z))) * radians_to_deg;
    *roll_deg = atan2f(x, z) * radians_to_deg;
    return true;
}

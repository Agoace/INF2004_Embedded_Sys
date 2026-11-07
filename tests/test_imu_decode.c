/* Pure raw-byte checks. Register/axis reference: Jaison, b2a6bb6. */
#include "../app_program/subsystem4/imu_decode.h"
#include <assert.h>
#include <math.h>
#include <string.h>

static void near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.1f);
}

int main(void)
{
    /* Accel +1000, -1000, +2047; mag X=1100,Y=-1100,Z=980. */
    const uint8_t a[6] = {0x80u, 0x3eu, 0x80u, 0xc1u, 0xf0u, 0x7fu};
    const uint8_t m[6] = {0x04u, 0x4cu, 0x03u, 0xd4u, 0xfbu, 0xb4u};
    imu_units_t units;
    imu_units_t saved;
    imu_axes_t axes = imu_robot_axes;
    float g;
    float pitch;
    float roll;

    assert(imu_decode(a, m, &axes, &units));
    assert(units.mag_valid);
    assert(units.accel_mg[0] == 1000);
    assert(units.accel_mg[1] == -1000);
    assert(units.accel_mg[2] == 2047);
    assert(units.mag_mgauss[0] == 1000);
    assert(units.mag_mgauss[1] == -1000);
    assert(units.mag_mgauss[2] == 1000);
    axes.accel = (imu_axis_map_t){{2u, 0u, 1u}, {-1, 1, -1}};
    axes.mag = (imu_axis_map_t){{1u, 2u, 0u}, {1, -1, 1}};
    assert(imu_decode(a, m, &axes, &units));
    assert(units.accel_mg[0] == -2047);
    assert(units.accel_mg[1] == 1000 && units.accel_mg[2] == 1000);
    assert(units.mag_mgauss[0] == -1000);
    assert(units.mag_mgauss[1] == -1000 && units.mag_mgauss[2] == 1000);
    saved = units;
    axes.mag.source[0] = 3u;
    assert(!imu_decode(a, m, &axes, &units));
    assert(memcmp(&saved, &units, sizeof(units)) == 0);
    axes = imu_robot_axes;
    axes.accel.source[0] = 1u; /* duplicate source */
    assert(!imu_decode(a, m, &axes, &units));
    axes = imu_robot_axes;
    axes.mag.sign[0] = 0;
    assert(!imu_decode(a, m, &axes, &units));
    assert(!imu_decode(NULL, m, &imu_robot_axes, &units));
    {
        const uint8_t edges[6] = {0x0fu, 0x80u, 0xffu, 0xffu, 0u, 0u};
        const uint8_t overflow[6] = {0xf0u, 0u, 0x80u, 0u, 0x7fu, 0xffu};
        assert(imu_decode(edges, overflow, &imu_robot_axes, &units));
        assert(units.accel_mg[0] == -2048);
        assert(units.accel_mg[1] == -1); /* unused low bits ignored */
        assert(!units.mag_valid);
        assert(units.mag_mgauss[1] == (32767 * 1000) / 1100);
        assert(units.mag_mgauss[2] == (-32768 * 1000) / 980);
    }
    /* Reported 1375 mg is 0x55f0 left-aligned, not a hidden 1.375 gain. */
    assert(imu_decode((uint8_t[6]){0u, 0u, 0u, 0u, 0xf0u, 0x55u},
                      m, &imu_robot_axes, &units));
    assert(units.accel_mg[2] == 1375);
    assert(imu_tilt((int32_t[3]){0, 0, 1000}, &g, &pitch, &roll));
    near(g, 1000.0f);
    near(pitch, 0.0f);
    near(roll, 0.0f);
    assert(imu_tilt((int32_t[3]){0, 500, 866}, &g, &pitch, &roll));
    near(pitch, 30.0f);
    assert(imu_tilt((int32_t[3]){500, 0, 866}, &g, &pitch, &roll));
    near(roll, 30.0f);
    assert(imu_tilt((int32_t[3]){0, -500, 866}, &g, &pitch, &roll));
    near(pitch, -30.0f);
    assert(!imu_tilt((int32_t[3]){0, 0, 0}, &g, &pitch, &roll));
    return 0;
}

/* LSM303DLHC units and body axes. Port ideas: Jaison, b2a6bb6.
 * No gyro. This decoder stays raw; imu_core owns observer calibration. */
#ifndef IMU_DECODE_H
#define IMU_DECODE_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t source[3];
    int8_t sign[3];
} imu_axis_map_t;

typedef struct {
    imu_axis_map_t accel;
    imu_axis_map_t mag;
} imu_axes_t;

/* One mounting table: +X right, +Y forward, +Z up. Accel identity passed
 * S4-U03; magnetic orientation/calibration still require hardware checks. */
extern const imu_axes_t imu_robot_axes;

typedef struct {
    int32_t accel_mg[3];
    int32_t mag_mgauss[3];
    bool mag_valid; /* false if any raw axis is the -4096 overflow code */
} imu_units_t;

/* Accel: little-endian signed 12-bit left aligned, HR +/-2 g (1 mg/LSB).
 * Mag: big-endian X,Z,Y, +/-1.3 gauss (1100 XY, 980 Z LSB/gauss).
 * Invalid mapping returns false without changing out. */
bool imu_decode(const uint8_t accel[6], const uint8_t mag[6],
                const imu_axes_t *axes, imu_units_t *out);
/* Static gravity estimate only: positive pitch nose-up, roll right-up.
 * No filter or calibration. False on zero acceleration. */
bool imu_tilt(const int32_t accel_mg[3], float *g_mg,
              float *pitch_deg, float *roll_deg);
#endif

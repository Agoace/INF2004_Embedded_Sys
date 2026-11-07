/*
 * odometry.h - pure encoder odometry helpers for the telemetry observers.
 *
 * Everything here is derived from the wheel encoder counts and the speeds
 * that the motion task already publishes. The functions use the constants
 * that the motor controller already has (counts per metre and counts per
 * degree, motor_ctl_robot.h). The robot track width is not stored: the
 * counts per degree constant holds it (111 mm, 24.05 counts per degree).
 *
 * No hardware access. Host-tested by tests/test_odometry.c.
 */
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>

typedef struct {
    uint16_t counts_per_m;        /* encoder counts per metre of wheel travel */
    uint16_t counts_per_deg_x100; /* (right - left) counts per degree, x100 */
} odo_config_t;

/* One full turn in 0.01 degree units. */
#define ODO_FULL_TURN_CDEG 36000
/* The sine table has one entry per degree from 0 to 90. */
#define ODO_SIN_QUARTER_CDEG       9000
#define ODO_SIN_HALF_CDEG          18000
#define ODO_SIN_THREE_QUARTER_CDEG 27000
/* Sine and cosine are scaled by this value. */
#define ODO_Q15_ONE 32768

/* Heading change in 0.01 degree for wheel counts, positive = left. */
int32_t odo_heading_cdeg(const odo_config_t *cfg, int32_t left_counts,
                         int32_t right_counts);

/*
 * Turn rate in 0.1 degree per second from the wheel speeds in mm/s.
 * Positive = left (right wheel faster). Rounded to the nearest value and
 * limited to the int16 range.
 */
int16_t odo_turn_rate_ddps(const odo_config_t *cfg, int16_t left_mm_s,
                           int16_t right_mm_s);

/* Counts to millimetres, rounded down. */
uint32_t odo_counts_to_mm(const odo_config_t *cfg, uint64_t counts);

/* Sine and cosine of an angle in 0.01 degree, scaled by ODO_Q15_ONE. */
int32_t odo_sin_q15(int32_t cdeg);
int32_t odo_cos_q15(int32_t cdeg);

#endif

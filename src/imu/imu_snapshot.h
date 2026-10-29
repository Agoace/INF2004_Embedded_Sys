/**
 * @file imu_snapshot.h
 * @brief Named pack and unpack helpers for the two-word Manager snapshot.
 *
 * Layout (no packed structs, no compiler bit-fields):
 *
 * Word 1
 *   0..11   heading        unsigned deci-degrees, 0..3599
 *   12..21  pitch          signed two's-complement deci-degrees, -512..511
 *   22..31  turn rate      signed two's-complement degrees/s, -512..511
 *
 * Word 2
 *   0..15   highest hump peak   unsigned millimetres
 *   16..18  terrain state       imu_terrain_t: 0 INIT, 1 LEVEL, 2 ON_HUMP,
 *                               3 SENSOR_FAULT
 *   19      accelerometer valid
 *   20      magnetometer valid
 *   21      calibrated
 *   22      self-test passed
 *   23      fault latched
 *   24..31  sequence            increments once per completed 13 ms publish
 */
#ifndef IMU_SNAPSHOT_H
#define IMU_SNAPSHOT_H

#include "imu_types.h"

/** Pack decoded fields into the two words. Out-of-range values are clamped. */
void imu_snapshot_pack(const imu_snapshot_fields_t *fields, imu_snapshot_t *out);

/** Unpack the two words, sign-extending every signed field. */
void imu_snapshot_unpack(const imu_snapshot_t *snap, imu_snapshot_fields_t *fields);

#endif /* IMU_SNAPSHOT_H */

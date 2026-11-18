/* TEST-only synthetic input. It never reads a physical sensor. */
#ifndef IMU_REPLAY_H
#define IMU_REPLAY_H
#include "../subsystem4/imu_probe.h"
#define IMU_REPLAY_PHASES 5u
/* 40 s repeat: upright 12 s, tilt 8 s, upright 8 s, low-g 4 s,
 * upright 8 s. Four raw-equivalent samples per 100 Hz observation. */
unsigned int imu_replay_read(imu_probe_t *probe, imu_units_t *raw,
                            uint32_t elapsed_ms);
#endif

/* Bench-only synthetic input. It never reads a physical sensor. */
#ifndef IMU_REPLAY_H
#define IMU_REPLAY_H
#include "../subsystem4/imu_probe.h"
#ifndef IMU_SAFETY_BENCH_ROBOT
#define IMU_SAFETY_BENCH_ROBOT 0 /* Only the ROBOT synthetic profile sets 1. */
#endif
#define IMU_REPLAY_PHASES 5u
/* Phase labels, shared by both streams:
 * 0 upright idle, 1 tilt, 2 upright after tilt, 3 low-g lift,
 * 4 upright after lift. */

/* TEST profile. 40 s repeat: upright 12 s, tilt 8 s, upright 8 s,
 * low-g 4 s, upright 8 s. Four raw-equivalent samples per observation. */
unsigned int imu_replay_read(imu_probe_t *probe, imu_units_t *raw,
                            uint32_t elapsed_ms);

/* ROBOT profile. Upright until the Vehicle begins a calibration (a new
 * odd calibration epoch, or a whole start/end pair between two reads).
 * IMU_REPLAY_ROBOT_DELAY_MS later, one episode runs: starts alternate
 * OVERTURN (tilt to 85 deg, hold, return) and LIFT (0.3 g for 200 ms).
 * A start during a scheduled or running episode is ignored. */
#define IMU_REPLAY_ROBOT_DELAY_MS 2000u
#define IMU_REPLAY_TILT_HOLD_MS   3000u
#define IMU_REPLAY_TILT_RETURN_MS 320u
#define IMU_REPLAY_LIFT_MS        200u

typedef struct {
    uint32_t starts;     /* Calibration starts seen (epoch count). */
    uint32_t episodes;   /* Episodes scheduled since init. */
    uint32_t at_ms;      /* Episode start time. */
    uint8_t after;       /* Phase label while no episode runs. */
    bool begun;
    bool scheduled;
    bool running;
    bool lift;           /* Kind of the scheduled/running episode. */
} imu_replay_robot_t;

void imu_replay_robot_init(imu_replay_robot_t *r);
unsigned int imu_replay_robot_read(imu_replay_robot_t *r, imu_probe_t *probe,
                                   imu_units_t *raw, uint32_t now_ms,
                                   uint32_t calibration_epoch);
uint32_t imu_replay_robot_episodes(const imu_replay_robot_t *r);
#endif

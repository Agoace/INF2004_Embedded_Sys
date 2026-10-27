/**
 * @file motion_config.h
 * @brief Buddy 2 motion-control constants. Every value marked MEASURE is a
 *        placeholder until the team measures it on the loaded robot.
 */
#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

#include "robot_shared.h"

/* ---- Pins (NFR10, docs/CALIBRATION_RESULTS.md) ------------------------ */
#define MOTOR_1_FWD_GPIO        8u
#define MOTOR_1_REV_GPIO        9u
#define MOTOR_2_FWD_GPIO        10u
#define MOTOR_2_REV_GPIO        11u
#define ENC_1_A_GPIO            16u
#define ENC_1_B_GPIO            17u
#define ENC_2_A_GPIO            4u
#define ENC_2_B_GPIO            5u

/* Motor 2 counts negative when the robot drives forward
 * (docs/MOTOR_ROTATION_DRIFT.md). The ISR multiplies by these signs so that
 * forward is positive for both wheels. */
#define ENC_1_SIGN              (+1)
#define ENC_2_SIGN              (-1)

/* Which Robo Pico motor is on the LEFT side of the chassis.
 * MEASURE: run "rot 90" on the bench target. A positive angle must turn the
 * robot to the LEFT (counter-clockwise from above). If it turns right, set
 * this to 0. */
#define MOTION_MOTOR_1_IS_LEFT  1

/* Optional logic-analyser pin for the NFR16 stop test and the tick WCET.
 * GP22 is unused in the NFR10 pin table. Set to -1 to disable. */
#define MOTION_DEBUG_GPIO       22

/* ---- Geometry ---------------------------------------------------------- */
#define MOTION_COUNTS_PER_REV   ENC_COUNTS_PER_REV  /* 2535, measured        */
/* MEASURE: wheel diameter under load (tribunal: needed for cruise speed). */
#define MOTION_WHEEL_DIAMETER_MM    65
/* MEASURE: track width, centre of left tyre to centre of right tyre. */
#define MOTION_WHEELBASE_MM         120

/* Derived (integer): circumference in micrometres = pi * D * 1000. */
#define MOTION_CIRCUMFERENCE_UM \
    (((int32_t) MOTION_WHEEL_DIAMETER_MM * 314159) / 100)

/* ---- Timing ------------------------------------------------------------ */
#define MOTION_TICK_MS              20u     /* NFR4/NFR5 PID period          */
#define MOTION_SETTLE_RAMP_TICKS    3u      /* 3 x 20 ms = 60 ms ramp (6.2)  */
#define MOTION_SETTLE_TIMEOUT_MS    200u    /* Tribunal: SETTLE timeout      */
#define MOTION_STEER_STALE_MS       100u    /* Tribunal: stale-STEER timeout */
#define MOTION_KICK_PERMILLE        100     /* CALIBRATION_RESULTS: 10% kick */
#define MOTION_KICK_COUNTS          10      /* until 10 counts ...           */
#define MOTION_KICK_TIMEOUT_MS      250u    /* ... or 250 ms => stall        */
#define MOTION_STALL_TIMEOUT_MS     300u    /* No counts while driven        */
#define MOTION_STALL_MIN_PERMILLE   80      /* Only check above 8% duty      */
#define MOTION_TICK_OVERRUN_US      25000u  /* Gap that counts as overrun    */

/* ---- Speeds ------------------------------------------------------------ */
#define MOTION_DEFAULT_SPEED_MM_S   200     /* MEASURE: cruise speed         */
#define MOTION_MAX_SPEED_MM_S       600     /* ~70% of unloaded max          */
#define MOTION_SLOW_PERCENT         50      /* flg_mode SLOW scale           */
#define MOTION_MIN_CPT              12      /* Slowest approach speed, counts
                                             * per tick (~7% duty region).   */
#define MOTION_MAX_DISTANCE_MM      3000
#define MOTION_MAX_ANGLE_DEG        360

/* ---- Bounded reverse-follow recovery (motor-line-manager checkpoint) ----
 * Motion enforces these limits itself, so a caller cannot ask for an
 * unbounded reverse manoeuvre. */
#define MOTION_RECOVERY_SPEED_MM_S  100     /* Fixed reverse speed.          */
#define MOTION_RECOVERY_MAX_MM      100     /* Distance budget.              */
#define MOTION_RECOVERY_TIMEOUT_MS  3000u   /* Wall-clock guard.             */

/* ---- Default controller gains (Q8 fixed point: 256 = 1.0) --------------
 * Units: PWM permille per (count per 20 ms tick).
 * Feed-forward from the unloaded 5 V VIN table: 10% duty ~ 22 counts/tick,
 * 100% ~ 220 counts/tick, so about 4.5 permille per count/tick.
 * The A4 PID tuning report replaces these with measured values. */
#define MOTION_FF_Q8                1152    /* 4.5                           */
#define MOTION_FF_OFFSET_PERMILLE   10
#define MOTION_KP_Q8                768     /* 3.0                           */
#define MOTION_KI_Q8                128     /* 0.5 per tick                  */
#define MOTION_KSYNC_Q8             64      /* 0.25 cpt per count difference */
#define MOTION_ACCEL_Q8_PER_TICK    (4 * 256) /* +4 counts/tick per tick;
                                               * 0 = step (for A4 tests)     */
#define MOTION_DECEL_DIV            4       /* cap = remaining / 4 per tick  */
#define MOTION_COAST_LEAD_COUNTS    0       /* MEASURE under load            */
#define MOTION_INTEGRAL_LIMIT       (400 * 256)
#define MOTION_PERMILLE_MAX         1000

/* ---- Queues ------------------------------------------------------------ */
#define MOTION_CMD_QUEUE_LEN        4u      /* power of two                  */
#define MOTION_EVT_QUEUE_LEN        8u      /* power of two                  */

#endif /* MOTION_CONFIG_H */

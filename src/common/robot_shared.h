/**
 * @file robot_shared.h
 * @brief Team-wide shared definitions (Tribunal 1 readiness action:
 *        "Agree one shared header for packed messages, the fault bits and
 *        the 16-bit encoder delta helper").
 *
 * Status: PROPOSAL by Buddy 2. The motion fields are final for Buddy 2. The
 * other buddies add their own packed fields here when the team agrees the
 * layouts. Keep this header free of Pico SDK and RTOS includes so that every
 * task, the host unit tests and the telemetry packer can include it.
 */
#ifndef ROBOT_SHARED_H
#define ROBOT_SHARED_H

#include <stdbool.h>
#include <stdint.h>

/* ------------------------------------------------------------------------
 * Encoder counters (Tribunal 1 decision: wrapping counter, single writer).
 *
 * - The encoder ISR is the ONLY writer. No task resets a counter.
 * - Each counter is a 16-bit wrapping value. A 16-bit aligned load is
 *   atomic on the Cortex-M0+, so readers need no lock.
 * - "Forward" is positive for BOTH wheels. The ISR corrects the Motor 2
 *   sign, which counts negative when the robot drives forward.
 * - Readers keep their own last value and use enc_delta16(now, last).
 *
 * Limit: a reader must sample at least once per 32767 counts of travel,
 * otherwise the delta aliases. At the measured maximum of about 221 counts
 * per 20 ms (261 RPM unloaded) that is about 2.9 s. Every planned task
 * (1 ms, 13 ms, 20 ms, 100 ms, 110 ms) is far inside this limit.
 * ---------------------------------------------------------------------- */
#define ENC_COUNTS_PER_REV      2535u   /* docs/MOTOR_ROTATION_DRIFT.md */
#define ENC_MAX_SAFE_DELTA      32767

typedef enum
{
    MOTOR_LEFT  = 0,
    MOTOR_RIGHT = 1,
    MOTOR_COUNT = 2
} motor_id_t;

/**
 * @brief Signed travel between two wrapping 16-bit counter samples.
 *
 * Modular subtraction gives the correct signed result across wrap-around and
 * in reverse, if the true movement is within +/-32767 counts.
 */
static inline int16_t
enc_delta16 (uint16_t now, uint16_t last)
{
    return (int16_t) (uint16_t) (now - last);
}

/**
 * @brief Reader-owned odometer. Each consumer (Line, IMU, VC) keeps its own
 *        instance and calls odometer_update() at its own period. The shared
 *        counter itself is never reset.
 */
typedef struct
{
    uint16_t last;      /* Last raw counter sample. */
    int32_t  total;     /* Accumulated signed counts since odometer_mark(). */
} odometer_t;

static inline void
odometer_mark (odometer_t * p_odo, uint16_t now)
{
    p_odo->last  = now;
    p_odo->total = 0;
}

static inline int32_t
odometer_update (odometer_t * p_odo, uint16_t now)
{
    p_odo->total += enc_delta16(now, p_odo->last);
    p_odo->last   = now;
    return p_odo->total;
}

/* ------------------------------------------------------------------------
 * Fault bits (FR14). One bit per cause so several can be latched at once.
 * The Vehicle Controller publishes the OR of all bits as "f".
 * ---------------------------------------------------------------------- */
#define FAULT_NONE              (0u)
#define FAULT_ESTOP             (1u << 0)   /* Button GP20 or MQTT estop.  */
#define FAULT_MOTOR_STALL_L     (1u << 1)   /* Buddy 2                      */
#define FAULT_MOTOR_STALL_R     (1u << 2)   /* Buddy 2                      */
#define FAULT_ENCODER           (1u << 3)   /* Buddy 2: self-test failed    */
#define FAULT_LINE_SENSOR       (1u << 4)   /* Buddy 3                      */
#define FAULT_IMU               (1u << 5)   /* Buddy 4                      */
#define FAULT_COLLISION         (1u << 6)   /* Buddy 4                      */
#define FAULT_ULTRASONIC        (1u << 7)   /* Buddy 5                      */
#define FAULT_RECOVERY_TIMEOUT  (1u << 8)   /* Buddy 5 (VC)                 */
#define FAULT_STEER_STALE       (1u << 9)   /* Buddy 2: STEER not updated   */

/* ------------------------------------------------------------------------
 * Motion commands (mbf_motion_cmd payload). Discrete commands only.
 * STEER is NOT a message: it is a latest-value variable (Tribunal 1), so a
 * STOP can never queue behind a burst of STEER updates.
 * ---------------------------------------------------------------------- */
typedef enum
{
    MOTION_CMD_NONE = 0,
    MOTION_CMD_STOP,        /* Normal stop: PWM 0 at once, go to IDLE.       */
    MOTION_CMD_ESTOP,       /* PWM 0 at once, latch fault until RESET.       */
    MOTION_CMD_RESET,       /* Clear MOTOR_FAULT after the cause clears.     */
    MOTION_CMD_FOLLOW,      /* Line follow at speed, steered by STEER.       */
    MOTION_CMD_FWD,         /* Drive arg_a mm forward at speed.              */
    MOTION_CMD_REV,         /* Drive arg_a mm backward at speed.             */
    MOTION_CMD_TURN,        /* Pivot turn arg_a degrees (+ = left/CCW).      */
    MOTION_CMD_ROTATE,      /* In-place rotate arg_a degrees (+ = left/CCW). */
    MOTION_CMD_SPEED,       /* Open speed set-point per wheel (PID tests).   */
    MOTION_CMD_SELFTEST,    /* FR17 encoder self-test (short kicks).         */
    MOTION_CMD_COUNT
} motion_cmd_id_t;

/** 8-byte command; fits one mbf_motion_cmd message. */
typedef struct
{
    uint8_t  id;            /* motion_cmd_id_t                               */
    uint8_t  seq;           /* Sender sequence, echoed in MOTION_DONE.       */
    int16_t  arg_a;         /* mm, degrees, or left mm/s (SPEED)             */
    int16_t  speed_mm_s;    /* Cruise speed; 0 = use default. Right (SPEED)  */
    int16_t  reserved;
} motion_cmd_t;

/* ------------------------------------------------------------------------
 * Motion events (posted to mbf_ctrl_evt by the Motion Control Task).
 * ---------------------------------------------------------------------- */
/*  Event          arg                     detail          achieved
 *  DONE           MOTION_DONE_*           cmd id          mm (FWD/REV) or
 *                                                         signed degrees
 *  STOPPED        1 = ESTOP, 0 = STOP     -               -
 *  STALL          motor_id_t              FAULT_* bits    -
 *  STEER_STALE    -                       -               -
 *  SELFTEST       1 = pass, 0 = fail      pass mask       -
 *  REJECTED       cmd id                  MOTION_REJECT_* -              */
typedef enum
{
    MOTION_EVT_NONE = 0,
    MOTION_EVT_DONE,            /* FWD/REV/TURN/ROTATE finished.             */
    MOTION_EVT_STOPPED,         /* STOP or ESTOP applied (PWM is 0).         */
    MOTION_EVT_STALL,           /* Stall, fault latched, PWM is 0.           */
    MOTION_EVT_STEER_STALE,     /* FOLLOW lost fresh STEER; wheels held.     */
    MOTION_EVT_SELFTEST,        /* FR17 encoder self-test result.            */
    MOTION_EVT_REJECTED,        /* Command refused.                          */
    MOTION_EVT_COUNT
} motion_evt_id_t;

#define MOTION_DONE_OK              0u
#define MOTION_DONE_SETTLE_TIMEOUT  1u  /* Wheels still moving after ramp.  */

#define MOTION_REJECT_INVALID       0u  /* Bad id or argument.              */
#define MOTION_REJECT_FAULT         1u  /* MOTOR_FAULT: send RESET first.   */

/* Self-test pass mask bits (all set = 0x3F = pass). */
#define SELFTEST_L_FWD              (1u << 0)
#define SELFTEST_L_REV              (1u << 1)
#define SELFTEST_R_FWD              (1u << 2)
#define SELFTEST_R_REV              (1u << 3)
#define SELFTEST_L_PHASES           (1u << 4)
#define SELFTEST_R_PHASES           (1u << 5)

typedef struct
{
    uint8_t  id;            /* motion_evt_id_t                               */
    uint8_t  seq;           /* seq of the command this event belongs to.     */
    uint8_t  arg;           /* status / motor / pass flag                    */
    uint8_t  detail;        /* extra bits (self-test mask, stop reason)      */
    int16_t  achieved;      /* mm or degrees actually achieved               */
    uint16_t time_ms;       /* low 16 bits of ms timestamp                   */
} motion_evt_t;

/* ------------------------------------------------------------------------
 * Motion snapshot: two 32-bit single-writer words (minimum-bit requirement).
 * Written only by the Motion Control Task. An aligned 32-bit store is atomic
 * on the M0+, so each word is coherent; read W1 and W2 under mtx_snapshot if
 * you need them to match.
 *
 * W1: [8:0]   left speed,  signed 9-bit, counts per 20 ms (+/-255; max ~221)
 *     [17:9]  right speed, signed 9-bit, counts per 20 ms
 *     [21:18] motion state (motion_state_t, 4 bits)
 *     [22]    fault latched
 *     [23]    slow mode active
 *     [24]    steer stale
 *     [31:25] spare
 * W2: [15:0]  odometer in cm since RUN start (wraps at 655 m)
 *     [31:16] snapshot sequence (increments every 20 ms tick)
 * ---------------------------------------------------------------------- */
typedef enum
{
    MOTION_STATE_IDLE = 0,
    MOTION_STATE_SELECT,
    MOTION_STATE_FOLLOW,
    MOTION_STATE_DRIVE_DISTANCE,
    MOTION_STATE_TURNING,
    MOTION_STATE_SETTLE,
    MOTION_STATE_MOTOR_FAULT,
    MOTION_STATE_SPEED,         /* Test-only open speed set-point. */
    MOTION_STATE_SELFTEST,
    MOTION_STATE_COUNT
} motion_state_t;

#define MSNAP_W1_SPEED_BITS   9u
#define MSNAP_W1_SPEED_MASK   ((1u << MSNAP_W1_SPEED_BITS) - 1u)
#define MSNAP_W1_L_SHIFT      0u
#define MSNAP_W1_R_SHIFT      9u
#define MSNAP_W1_STATE_SHIFT  18u
#define MSNAP_W1_STATE_MASK   0xFu
#define MSNAP_W1_FAULT_BIT    (1u << 22)
#define MSNAP_W1_SLOW_BIT     (1u << 23)
#define MSNAP_W1_STALE_BIT    (1u << 24)

static inline uint32_t
msnap_pack_speed (int32_t counts_per_tick)
{
    if (counts_per_tick > 255)
    {
        counts_per_tick = 255;
    }
    else if (counts_per_tick < -255)
    {
        counts_per_tick = -255;
    }
    return ((uint32_t) counts_per_tick) & MSNAP_W1_SPEED_MASK;
}

static inline int16_t
msnap_unpack_speed (uint32_t w1, uint32_t shift)
{
    uint32_t raw = (w1 >> shift) & MSNAP_W1_SPEED_MASK;

    /* Sign-extend 9 bits. */
    return (int16_t) ((raw & 0x100u) ? (int32_t) raw - 512 : (int32_t) raw);
}

#endif /* ROBOT_SHARED_H */

/**
 * @file motion_control.c
 * @brief Motion Control core: the Section 6.2 statechart
 *        (IDLE > SELECT_MOTION > FOLLOW / DRIVE_DISTANCE / TURNING > SETTLE,
 *        MOTOR_FAULT), per-wheel PI speed control with feed-forward,
 *        straight-line synchronisation, start kick and stall detection.
 *
 * Integer and fixed-point only: no floating point, so the tick is cheap and
 * may also run from a timer IRQ in the bare-metal bench target.
 */
#include "motion_control.h"

#include <stddef.h>
#include <string.h>

#include "motion_config.h"
#include "motion_hal.h"
#include "motion_units.h"

#define STOP_REQ_STOP   (1u << 0)
#define STOP_REQ_ESTOP  (1u << 1)

#define TURN_ROTATE     0u
#define TURN_PIVOT      1u

#define SELFTEST_MIN_COUNTS 20
#define SELFTEST_ALL_PASS   0x3Fu

/* Faults that hold the motors at 0 until RESET. */
#define LATCHING_FAULTS (FAULT_ESTOP | FAULT_MOTOR_STALL_L | \
                         FAULT_MOTOR_STALL_R | FAULT_ENCODER)

typedef struct
{
    uint16_t last_count;
    int16_t  speed_cpt;         /* Counts measured in the last tick.      */
    int32_t  travel;            /* Signed counts since move start.        */
    int32_t  setpoint_q8;       /* Ramped set-point, counts/tick Q8.      */
    int32_t  integral_q8;       /* I-term, permille Q8.                   */
    int16_t  duty;              /* Last commanded permille.               */
    bool     kicking;
    uint32_t kick_ms;
    int32_t  kick_travel;
    uint32_t still_ms;
} wheel_t;

typedef struct
{
    int8_t   dir[MOTOR_COUNT];  /* +1, -1 or 0 (pivot inner wheel).        */
    int32_t  target_counts;     /* Progress needed (outer wheel / average). */
    int32_t  cruise_q8;
    uint8_t  turn_kind;
    int8_t   angle_sign;        /* +1 left, -1 right (achieved degrees).   */
    uint8_t  cmd_id;
    int16_t  speed_l_mm_s;      /* SPEED command set-points.               */
    int16_t  speed_r_mm_s;
    uint32_t recover_ms;        /* Time spent in RECOVERY.                 */
} move_t;

typedef struct
{
    uint32_t ticks_in;          /* Ticks spent in SETTLE.                  */
    int16_t  start_duty[MOTOR_COUNT];
} settle_t;

typedef struct
{
    uint8_t  phase;             /* 0..3: L+, L-, R+, R-; 4 = done.        */
    bool     resting;
    uint32_t phase_ms;
    int32_t  start_travel;
    uint32_t start_a;
    uint32_t start_b;
    uint8_t  mask;
} selftest_t;

/* ---- Shared with other contexts --------------------------------------- */
static volatile uint8_t     g_stop_req;
static volatile uint32_t    g_fault_bits;
static volatile uint32_t    g_steer_word;   /* [7:0] steer, [8] set, [31:16] stamp */
static volatile bool        g_slow;
static volatile uint32_t    g_snap_w1;
static volatile uint32_t    g_snap_w2;
static volatile uint8_t     g_state;

static motion_cmd_t         g_cmd_q[MOTION_CMD_QUEUE_LEN];
static volatile uint8_t     g_cmd_head;     /* Next write. */
static volatile uint8_t     g_cmd_tail;     /* Next read.  */

static motion_evt_t         g_evt_q[MOTION_EVT_QUEUE_LEN];
static volatile uint8_t     g_evt_head;
static volatile uint8_t     g_evt_tail;

static motion_notify_fn_t   g_notify;
static motion_log_fn_t      g_log_hook;
static motion_stats_t       g_stats;
static motion_gains_t       g_gains;

/* ---- Motion task private ----------------------------------------------- */
static wheel_t              g_wheel[MOTOR_COUNT];
static move_t               g_move;
static settle_t             g_settle;
static selftest_t           g_selftest;
static uint8_t              g_active_seq;
static bool                 g_steer_stale;
static int32_t              g_odo_sum;      /* Left + right counts. */
static volatile bool        g_odo_reset_req;
static uint32_t             g_last_tick_us;
static uint32_t             g_tick_count;

/* ======================================================================= */

static inline int32_t
abs32 (int32_t v)
{
    return (v < 0) ? -v : v;
}

static inline int32_t
clamp32 (int32_t v, int32_t lo, int32_t hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline uint16_t
stamp_now (void)
{
    /* 1.024 ms units; wraps cleanly because the us counter wraps at 2^32. */
    return (uint16_t) (motion_hal_time_us() >> 10);
}

static inline uint16_t
ms_now16 (void)
{
    return (uint16_t) (g_tick_count * MOTION_TICK_MS);
}

static void
post_event (uint8_t id, uint8_t arg, uint8_t detail, int16_t achieved)
{
    uint8_t const head = g_evt_head;
    uint8_t const next = (uint8_t) ((head + 1u) & (MOTION_EVT_QUEUE_LEN - 1u));

    if (next == g_evt_tail)
    {
        g_stats.evt_dropped++;
        return;
    }

    motion_evt_t * p_evt = &g_evt_q[head];

    p_evt->id       = id;
    p_evt->seq      = g_active_seq;
    p_evt->arg      = arg;
    p_evt->detail   = detail;
    p_evt->achieved = achieved;
    p_evt->time_ms  = ms_now16();
    g_evt_head      = next;
}

static void
set_state (motion_state_t state)
{
    g_state = (uint8_t) state;
}

static void
reset_wheel_control (void)
{
    for (uint32_t m = 0u; m < MOTOR_COUNT; m++)
    {
        g_wheel[m].setpoint_q8 = 0;
        g_wheel[m].integral_q8 = 0;
        g_wheel[m].duty        = 0;
        g_wheel[m].kicking     = false;
        g_wheel[m].still_ms    = 0u;
        g_wheel[m].travel      = 0;
    }
}

/**
 * The single PWM write path. The stop latch and the latched faults are
 * checked inside the same critical section as the write, so a STOP that
 * arrives at any point in the tick can never be overwritten by a non-zero
 * duty (Tribunal 1, R2).
 */
static void
apply_pwm (int16_t left, int16_t right)
{
    uint32_t const irq = motion_hal_irq_save();

    if ((0u != g_stop_req) || (0u != (g_fault_bits & LATCHING_FAULTS)))
    {
        motion_hal_pwm_zero_all();
        g_wheel[MOTOR_LEFT].duty  = 0;
        g_wheel[MOTOR_RIGHT].duty = 0;
    }
    else
    {
        motion_hal_pwm_write(MOTOR_LEFT, left);
        motion_hal_pwm_write(MOTOR_RIGHT, right);
        g_wheel[MOTOR_LEFT].duty  = left;
        g_wheel[MOTOR_RIGHT].duty = right;
    }

    motion_hal_irq_restore(irq);
}

static void
enter_fault (uint32_t fault_bit)
{
    uint32_t const irq = motion_hal_irq_save();

    g_fault_bits |= fault_bit;
    motion_hal_pwm_zero_all();
    g_cmd_tail = g_cmd_head;     /* Drop queued motion. */
    motion_hal_irq_restore(irq);

    reset_wheel_control();
    set_state(MOTION_STATE_MOTOR_FAULT);
}

/* ---- Wheel speed controller -------------------------------------------- */

/**
 * PI speed loop with feed-forward, set-point ramp, start kick and stall
 * detection for one wheel. Returns signed duty in permille.
 * Sets *p_stalled when the wheel is driven but does not turn.
 */
static int16_t
wheel_control (wheel_t * p_w, int32_t setpoint_q8, bool * p_stalled)
{
    int32_t const accel = g_gains.accel_q8_per_tick;
    int32_t const prev  = p_w->setpoint_q8;

    /* Set-point ramp limits acceleration only (0 = step, for the A4
     * step-response logs). Slowing down toward 0 is applied at once so the
     * distance deceleration cap is followed. */
    bool const slowing = ((setpoint_q8 >= 0) == (prev >= 0)) &&
                         (abs32(setpoint_q8) < abs32(prev));

    if ((0 == accel) || slowing || (abs32(setpoint_q8 - prev) <= accel))
    {
        p_w->setpoint_q8 = setpoint_q8;
    }
    else
    {
        p_w->setpoint_q8 = prev + ((setpoint_q8 > prev) ? accel : -accel);
    }

    int32_t const sp = p_w->setpoint_q8;

    if (0 == sp)
    {
        p_w->integral_q8 = 0;
        p_w->kicking     = false;
        p_w->still_ms    = 0u;
        return 0;
    }

    int32_t const sign = (sp > 0) ? 1 : -1;

    /* Start from rest: arm the kick (CALIBRATION_RESULTS start policy). */
    if ((0 == prev) || ((prev > 0) != (sp > 0)))
    {
        p_w->kicking     = true;
        p_w->kick_ms     = 0u;
        p_w->kick_travel = 0;
        p_w->integral_q8 = 0;
    }

    int32_t const err_q8 = sp - ((int32_t) p_w->speed_cpt * 256);
    int32_t const ff     = ((g_gains.ff_q8 * sp) / 65536) +
                           (sign * g_gains.ff_offset_permille);
    int32_t const p_term = (g_gains.kp_q8 * err_q8) / 65536;
    int32_t       out    = ff + p_term + (p_w->integral_q8 / 256);

    /* Anti-windup: integrate only when not saturated in the error sign. */
    bool const sat_hi = (out >= MOTION_PERMILLE_MAX) && (err_q8 > 0);
    bool const sat_lo = (out <= -MOTION_PERMILLE_MAX) && (err_q8 < 0);

    if (!sat_hi && !sat_lo)
    {
        p_w->integral_q8 = clamp32(p_w->integral_q8 +
                                   ((g_gains.ki_q8 * err_q8) / 256),
                                   -MOTION_INTEGRAL_LIMIT,
                                   MOTION_INTEGRAL_LIMIT);
    }

    out = clamp32(out, -MOTION_PERMILLE_MAX, MOTION_PERMILLE_MAX);

    if (p_w->kicking)
    {
        p_w->kick_travel += abs32(p_w->speed_cpt);
        p_w->kick_ms     += MOTION_TICK_MS;

        if (p_w->kick_travel >= MOTION_KICK_COUNTS)
        {
            p_w->kicking = false;
        }
        else if (p_w->kick_ms > MOTION_KICK_TIMEOUT_MS)
        {
            *p_stalled = true;
            return 0;
        }
        else if ((out * sign) < MOTION_KICK_PERMILLE)
        {
            out = sign * MOTION_KICK_PERMILLE;
        }
    }

    /* Stall: driven above the minimum duty but no counts for too long. */
    if ((!p_w->kicking) && (abs32(out) >= MOTION_STALL_MIN_PERMILLE) &&
        (0 == p_w->speed_cpt))
    {
        p_w->still_ms += MOTION_TICK_MS;

        if (p_w->still_ms >= MOTION_STALL_TIMEOUT_MS)
        {
            *p_stalled = true;
            return 0;
        }
    }
    else
    {
        p_w->still_ms = 0u;
    }

    return (int16_t) out;
}

/** Runs both wheel loops. Returns false (and latches the fault) on a stall. */
static bool
drive_wheels (int32_t sp_left_q8, int32_t sp_right_q8,
              int16_t * p_left, int16_t * p_right)
{
    bool stall_l = false;
    bool stall_r = false;

    *p_left  = wheel_control(&g_wheel[MOTOR_LEFT],  sp_left_q8,  &stall_l);
    *p_right = wheel_control(&g_wheel[MOTOR_RIGHT], sp_right_q8, &stall_r);

    if (stall_l || stall_r)
    {
        uint32_t const bits = (stall_l ? FAULT_MOTOR_STALL_L : 0u) |
                              (stall_r ? FAULT_MOTOR_STALL_R : 0u);

        enter_fault(bits);
        post_event(MOTION_EVT_STALL,
                   stall_l ? (uint8_t) MOTOR_LEFT : (uint8_t) MOTOR_RIGHT,
                   (uint8_t) bits, 0);
        *p_left  = 0;
        *p_right = 0;
        return false;
    }

    return true;
}

/* ---- Command selection (SELECT_MOTION) -------------------------------- */

static int32_t
cruise_q8_from (int16_t speed_mm_s)
{
    int32_t mm_s = (speed_mm_s <= 0) ? MOTION_DEFAULT_SPEED_MM_S : speed_mm_s;

    mm_s = clamp32(mm_s, 1, MOTION_MAX_SPEED_MM_S);

    return motion_mm_s_to_cpt_q8(mm_s);
}

static void
begin_move (void)
{
    reset_wheel_control();
}

static bool
select_motion (const motion_cmd_t * p_cmd)
{
    set_state(MOTION_STATE_SELECT);
    g_active_seq  = p_cmd->seq;
    g_move.cmd_id = p_cmd->id;
    g_move.cruise_q8 = cruise_q8_from(p_cmd->speed_mm_s);

    switch (p_cmd->id)
    {
        case MOTION_CMD_FOLLOW:
        {
            /* Keep the wheel state when switching FOLLOW speed. */
            set_state(MOTION_STATE_FOLLOW);
            return true;
        }

        case MOTION_CMD_FWD:
        case MOTION_CMD_REV:
        {
            if ((p_cmd->arg_a <= 0) || (p_cmd->arg_a > MOTION_MAX_DISTANCE_MM))
            {
                return false;
            }

            int8_t const d = (MOTION_CMD_FWD == p_cmd->id) ? 1 : -1;

            g_move.dir[MOTOR_LEFT]  = d;
            g_move.dir[MOTOR_RIGHT] = d;
            g_move.target_counts    = motion_mm_to_counts(p_cmd->arg_a);
            g_move.turn_kind        = TURN_ROTATE;
            begin_move();
            set_state(MOTION_STATE_DRIVE_DISTANCE);
            return true;
        }

        case MOTION_CMD_TURN:
        case MOTION_CMD_ROTATE:
        {
            int32_t const deg = p_cmd->arg_a;

            if ((0 == deg) || (abs32(deg) > MOTION_MAX_ANGLE_DEG))
            {
                return false;
            }

            bool const left = (deg > 0);

            g_move.angle_sign = left ? 1 : -1;

            if (MOTION_CMD_ROTATE == p_cmd->id)
            {
                /* In place: left turn = left wheel back, right wheel forward. */
                g_move.dir[MOTOR_LEFT]  = left ? -1 : 1;
                g_move.dir[MOTOR_RIGHT] = left ? 1 : -1;
                g_move.target_counts    = motion_rotate_counts(abs32(deg));
                g_move.turn_kind        = TURN_ROTATE;
            }
            else
            {
                /* Pivot about the inner wheel, which is held at 0. */
                g_move.dir[MOTOR_LEFT]  = left ? 0 : 1;
                g_move.dir[MOTOR_RIGHT] = left ? 1 : 0;
                g_move.target_counts    = motion_pivot_counts(abs32(deg));
                g_move.turn_kind        = TURN_PIVOT;
            }

            begin_move();
            set_state(MOTION_STATE_TURNING);
            return true;
        }

        case MOTION_CMD_SPEED:
        {
            if ((abs32(p_cmd->arg_a) > MOTION_MAX_SPEED_MM_S) ||
                (abs32(p_cmd->speed_mm_s) > MOTION_MAX_SPEED_MM_S))
            {
                return false;
            }

            g_move.speed_l_mm_s = p_cmd->arg_a;
            g_move.speed_r_mm_s = p_cmd->speed_mm_s;
            set_state(MOTION_STATE_SPEED);
            return true;
        }

        case MOTION_CMD_SELFTEST:
        {
            begin_move();
            (void) memset(&g_selftest, 0, sizeof(g_selftest));
            g_selftest.resting = true;  /* Start with one still tick. */
            set_state(MOTION_STATE_SELFTEST);
            return true;
        }

        case MOTION_CMD_RECOVER:
        {
            /* Bounded reverse follow. The distance and the speed are clamped
             * here, so no caller can ask for an unbounded reverse. */
            int32_t const mm  = p_cmd->arg_a;
            int32_t       spd = p_cmd->speed_mm_s;

            if ((mm <= 0) || (mm > MOTION_RECOVERY_MAX_MM))
            {
                return false;
            }

            if ((spd <= 0) || (spd > MOTION_RECOVERY_SPEED_MM_S))
            {
                spd = MOTION_RECOVERY_SPEED_MM_S;
            }

            /* Reverse the wheel set-points only. The bridge keeps publishing
             * the forward STEER, so the stored left/right bias unwinds and
             * the robot retraces the arc toward the line. */
            g_move.dir[MOTOR_LEFT]  = -1;
            g_move.dir[MOTOR_RIGHT] = -1;
            g_move.target_counts    = motion_mm_to_counts(mm);
            g_move.turn_kind        = TURN_ROTATE;
            g_move.cruise_q8        = motion_mm_s_to_cpt_q8(spd);
            g_move.recover_ms       = 0u;
            begin_move();
            set_state(MOTION_STATE_RECOVERY);
            return true;
        }

        default:
        {
            return false;
        }
    }
}

static bool
dequeue_command (motion_cmd_t * p_cmd)
{
    bool found = false;
    uint32_t const irq = motion_hal_irq_save();

    if (g_cmd_tail != g_cmd_head)
    {
        *p_cmd     = g_cmd_q[g_cmd_tail];
        g_cmd_tail = (uint8_t) ((g_cmd_tail + 1u) & (MOTION_CMD_QUEUE_LEN - 1u));
        found      = true;
    }

    motion_hal_irq_restore(irq);
    return found;
}

/** Commands are taken in IDLE, FOLLOW and SPEED (continuous states). A
 *  queued command waits while a move, SETTLE or self-test runs. In
 *  MOTOR_FAULT only RESET is accepted. */
static void
process_commands (void)
{
    motion_state_t const state = (motion_state_t) g_state;
    bool const can_take = (MOTION_STATE_IDLE == state) ||
                          (MOTION_STATE_FOLLOW == state) ||
                          (MOTION_STATE_SPEED == state) ||
                          (MOTION_STATE_MOTOR_FAULT == state);

    if (!can_take)
    {
        return;
    }

    motion_cmd_t cmd;

    if (!dequeue_command(&cmd))
    {
        return;
    }

    if (MOTION_STATE_MOTOR_FAULT == state)
    {
        if (MOTION_CMD_RESET == cmd.id)
        {
            g_fault_bits = 0u;
            g_active_seq = cmd.seq;
            reset_wheel_control();
            set_state(MOTION_STATE_IDLE);
        }
        else
        {
            g_active_seq = cmd.seq;
            post_event(MOTION_EVT_REJECTED, cmd.id, MOTION_REJECT_FAULT, 0);
        }

        return;
    }

    if (MOTION_CMD_RESET == cmd.id)
    {
        return;     /* Nothing to reset. */
    }

    if (!select_motion(&cmd))
    {
        g_active_seq = cmd.seq;
        post_event(MOTION_EVT_REJECTED, cmd.id, MOTION_REJECT_INVALID, 0);
        reset_wheel_control();
        set_state(MOTION_STATE_IDLE);
    }
}

/* ---- State handlers ---------------------------------------------------- */

static void
begin_settle (void)
{
    g_settle.ticks_in = 0u;
    g_settle.start_duty[MOTOR_LEFT]  = g_wheel[MOTOR_LEFT].duty;
    g_settle.start_duty[MOTOR_RIGHT] = g_wheel[MOTOR_RIGHT].duty;
    set_state(MOTION_STATE_SETTLE);
}

static int32_t
move_progress (void)
{
    int32_t const pl = g_wheel[MOTOR_LEFT].travel  * g_move.dir[MOTOR_LEFT];
    int32_t const pr = g_wheel[MOTOR_RIGHT].travel * g_move.dir[MOTOR_RIGHT];

    return (TURN_PIVOT == g_move.turn_kind) ? (pl + pr) : ((pl + pr) / 2);
}

static void
run_distance_or_turn (int16_t * p_l, int16_t * p_r)
{
    int32_t const remaining = g_move.target_counts - move_progress() -
                              MOTION_COAST_LEAD_COUNTS;

    if (remaining <= 0)
    {
        *p_l = g_wheel[MOTOR_LEFT].duty;
        *p_r = g_wheel[MOTOR_RIGHT].duty;
        begin_settle();
        return;
    }

    /* Deceleration cap so the robot arrives slowly (less coast). */
    int32_t cap = remaining / MOTION_DECEL_DIV;

    cap = (cap < MOTION_MIN_CPT) ? MOTION_MIN_CPT : cap;

    int32_t base = g_move.cruise_q8;

    if (g_slow)
    {
        base = (base * MOTION_SLOW_PERCENT) / 100;
    }

    base = (base < (cap * 256)) ? base : (cap * 256);

    int32_t sp_l = base * (g_move.dir[MOTOR_LEFT] != 0);
    int32_t sp_r = base * (g_move.dir[MOTOR_RIGHT] != 0);

    if (TURN_ROTATE == g_move.turn_kind)
    {
        /* Straight-line / symmetric-turn sync: equalise wheel progress. */
        int32_t const pl   = g_wheel[MOTOR_LEFT].travel  * g_move.dir[MOTOR_LEFT];
        int32_t const pr   = g_wheel[MOTOR_RIGHT].travel * g_move.dir[MOTOR_RIGHT];
        int32_t const sync = g_gains.ksync_q8 * (pl - pr);

        sp_l = clamp32(sp_l - sync, 0, 2 * base);
        sp_r = clamp32(sp_r + sync, 0, 2 * base);
    }

    (void) drive_wheels(sp_l * g_move.dir[MOTOR_LEFT],
                        sp_r * g_move.dir[MOTOR_RIGHT], p_l, p_r);
}

static int16_t
achieved_units (void)
{
    int32_t const progress = move_progress();

    if ((MOTION_CMD_FWD == g_move.cmd_id) || (MOTION_CMD_REV == g_move.cmd_id))
    {
        return (int16_t) motion_counts_to_mm(progress);
    }

    /* Signed degrees: invert motion_rotate_counts / motion_pivot_counts. */
    int64_t const denom = (int64_t) MOTION_WHEELBASE_MM * MOTION_COUNTS_PER_REV *
                          ((TURN_PIVOT == g_move.turn_kind) ? 2 : 1);

    return (int16_t) (((int64_t) progress * 360 * MOTION_WHEEL_DIAMETER_MM *
                       g_move.angle_sign) / denom);
}

static void
run_settle (int16_t * p_l, int16_t * p_r)
{
    g_settle.ticks_in++;

    if (g_settle.ticks_in < MOTION_SETTLE_RAMP_TICKS)
    {
        int32_t const k = (int32_t) (MOTION_SETTLE_RAMP_TICKS - g_settle.ticks_in);

        *p_l = (int16_t) ((g_settle.start_duty[MOTOR_LEFT]  * k) /
                          (int32_t) MOTION_SETTLE_RAMP_TICKS);
        *p_r = (int16_t) ((g_settle.start_duty[MOTOR_RIGHT] * k) /
                          (int32_t) MOTION_SETTLE_RAMP_TICKS);
        return;
    }

    *p_l = 0;
    *p_r = 0;

    bool const still = (0 == g_wheel[MOTOR_LEFT].speed_cpt) &&
                       (0 == g_wheel[MOTOR_RIGHT].speed_cpt);
    bool const timeout = (g_settle.ticks_in * MOTION_TICK_MS) >=
                         MOTION_SETTLE_TIMEOUT_MS;

    if (still || timeout)
    {
        post_event(MOTION_EVT_DONE,
                   (uint8_t) (still ? MOTION_DONE_OK : MOTION_DONE_SETTLE_TIMEOUT),
                   g_move.cmd_id, achieved_units());
        reset_wheel_control();
        set_state(MOTION_STATE_IDLE);
    }
}

/**
 * Read the latest-value STEER. Returns false when it is missing or older than
 * the stale timeout, and posts MOTION_EVT_STEER_STALE once for each episode.
 */
static bool
read_steer (int32_t * p_steer)
{
    uint32_t const word = g_steer_word;
    bool const     set  = (0u != (word & 0x100u));
    uint16_t const age  = (uint16_t) (stamp_now() - (uint16_t) (word >> 16));
    /* 1.024 ms units: 100 ms ~ 98 units. */
    bool const stale = (!set) || (age > ((MOTION_STEER_STALE_MS * 1000u) >> 10));

    if (stale)
    {
        if (!g_steer_stale)
        {
            g_steer_stale = true;
            post_event(MOTION_EVT_STEER_STALE, 0u, 0u, 0);
        }

        return false;
    }

    g_steer_stale = false;
    *p_steer = (int8_t) (word & 0xFFu);

    return true;
}

/**
 * Follow wheel set-points for one STEER value: outer wheel at base speed,
 * inner wheel slowed (0 at |steer| = 127). steer < 0 turns left.
 */
static void
steer_setpoints (int32_t steer, int32_t base, int32_t * p_l, int32_t * p_r)
{
    int32_t const inner = (base * (127 - abs32(steer))) / 127;

    *p_l = (steer < 0) ? inner : base;
    *p_r = (steer > 0) ? inner : base;
}

static int32_t
follow_base (void)
{
    int32_t const base = g_move.cruise_q8;

    return g_slow ? ((base * MOTION_SLOW_PERCENT) / 100) : base;
}

static void
run_follow (int16_t * p_l, int16_t * p_r)
{
    int32_t steer;

    if (!read_steer(&steer))
    {
        /* Hold: no fresh line information, do not drive blind. */
        (void) drive_wheels(0, 0, p_l, p_r);
        return;
    }

    int32_t sp_l;
    int32_t sp_r;

    steer_setpoints(steer, follow_base(), &sp_l, &sp_r);
    (void) drive_wheels(sp_l, sp_r, p_l, p_r);
}

/**
 * Bounded reverse follow (motor-line-manager checkpoint).
 *
 * The wheel set-points are the FOLLOW pair with dir = -1, so the same STEER
 * bias drives the robot backwards and unwinds instead of accumulating: a line
 * lost to the left is retraced back to the left. The manoeuvre ends at the
 * distance budget or at the wall-clock guard; either way Motion reports
 * MOTION_DONE_RECOVERY_LIMIT and stops, and the caller decides the fault.
 */
static void
run_recovery (int16_t * p_l, int16_t * p_r)
{
    g_move.recover_ms += MOTION_TICK_MS;

    int32_t const progress = move_progress();

    if ((progress >= g_move.target_counts) ||
        (g_move.recover_ms > MOTION_RECOVERY_TIMEOUT_MS))
    {
        post_event(MOTION_EVT_DONE, MOTION_DONE_RECOVERY_LIMIT, g_move.cmd_id,
                   (int16_t) motion_counts_to_mm(progress));
        reset_wheel_control();
        set_state(MOTION_STATE_IDLE);
        *p_l = 0;
        *p_r = 0;
        return;
    }

    int32_t steer;

    if (!read_steer(&steer))
    {
        (void) drive_wheels(0, 0, p_l, p_r);
        return;
    }

    int32_t mag_l;
    int32_t mag_r;

    steer_setpoints(steer, follow_base(), &mag_l, &mag_r);
    (void) drive_wheels(g_move.dir[MOTOR_LEFT]  * mag_l,
                        g_move.dir[MOTOR_RIGHT] * mag_r, p_l, p_r);
}

static void
run_speed (int16_t * p_l, int16_t * p_r)
{
    int32_t const l = motion_mm_s_to_cpt_q8(g_move.speed_l_mm_s);
    int32_t const r = motion_mm_s_to_cpt_q8(g_move.speed_r_mm_s);

    (void) drive_wheels(l, r, p_l, p_r);
}

static void
selftest_start_phase (void)
{
    motor_id_t const motor = (g_selftest.phase < 2u) ? MOTOR_LEFT : MOTOR_RIGHT;

    g_selftest.resting      = false;
    g_selftest.phase_ms     = 0u;
    g_selftest.start_travel = g_wheel[motor].travel;
    motion_hal_encoder_edges(motor, &g_selftest.start_a, &g_selftest.start_b);
}

/**
 * FR17 encoder self-test: drive each wheel at the kick duty for at most
 * 250 ms forward and then back. Pass bits: 0 L fwd, 1 L rev, 2 R fwd,
 * 3 R rev, 4 L both phases, 5 R both phases.
 */
static void
run_selftest (int16_t * p_l, int16_t * p_r)
{
    *p_l = 0;
    *p_r = 0;

    if (g_selftest.resting)
    {
        if ((0 == g_wheel[MOTOR_LEFT].speed_cpt) &&
            (0 == g_wheel[MOTOR_RIGHT].speed_cpt))
        {
            if (g_selftest.phase >= 4u)
            {
                bool const pass = (SELFTEST_ALL_PASS == g_selftest.mask);

                post_event(MOTION_EVT_SELFTEST, pass ? 1u : 0u,
                           g_selftest.mask, 0);

                if (pass)
                {
                    reset_wheel_control();
                    set_state(MOTION_STATE_IDLE);
                }
                else
                {
                    enter_fault(FAULT_ENCODER);
                }

                return;
            }

            selftest_start_phase();
        }
        else
        {
            return;
        }
    }

    motor_id_t const motor = (g_selftest.phase < 2u) ? MOTOR_LEFT : MOTOR_RIGHT;
    int32_t const    dir   = (0u == (g_selftest.phase & 1u)) ? 1 : -1;
    int32_t const    moved = (g_wheel[motor].travel - g_selftest.start_travel) * dir;

    g_selftest.phase_ms += MOTION_TICK_MS;

    bool const ok      = (moved >= SELFTEST_MIN_COUNTS);
    bool const timeout = (g_selftest.phase_ms > MOTION_KICK_TIMEOUT_MS);

    if (ok || timeout)
    {
        if (ok)
        {
            g_selftest.mask |= (uint8_t) (1u << g_selftest.phase);
        }

        if ((0u == (g_selftest.phase & 1u)) && ok)
        {
            uint32_t a;
            uint32_t b;

            motion_hal_encoder_edges(motor, &a, &b);

            if ((a != g_selftest.start_a) && (b != g_selftest.start_b))
            {
                g_selftest.mask |= (uint8_t) (1u << (4u + (uint32_t) motor));
            }
        }

        g_selftest.phase++;
        g_selftest.resting = true;
        return;
    }

    int16_t const duty = (int16_t) (dir * MOTION_KICK_PERMILLE);

    if (MOTOR_LEFT == motor)
    {
        *p_l = duty;
    }
    else
    {
        *p_r = duty;
    }
}

/* ---- Tick -------------------------------------------------------------- */

static void
measure_wheels (void)
{
    for (uint32_t m = 0u; m < MOTOR_COUNT; m++)
    {
        wheel_t * p_w = &g_wheel[m];
        uint16_t const now = motion_hal_encoder_count((motor_id_t) m);
        int16_t const  d   = enc_delta16(now, p_w->last_count);

        p_w->last_count = now;
        p_w->speed_cpt  = d;
        p_w->travel    += d;
    }

    if (g_odo_reset_req)
    {
        g_odo_reset_req = false;
        g_odo_sum       = 0;
    }

    /* Sum both wheels; halve only when publishing (no truncation drift). */
    g_odo_sum += (int32_t) g_wheel[MOTOR_LEFT].speed_cpt +
                 g_wheel[MOTOR_RIGHT].speed_cpt;
}

static void
handle_stop_requests (void)
{
    uint32_t const irq = motion_hal_irq_save();
    uint8_t const  req = g_stop_req;

    g_stop_req = 0u;
    motion_hal_irq_restore(irq);

    if (0u == req)
    {
        return;
    }

    if (0u != (req & STOP_REQ_ESTOP))
    {
        enter_fault(FAULT_ESTOP);
        post_event(MOTION_EVT_STOPPED, 1u, 0u, 0);
    }
    else if (MOTION_STATE_MOTOR_FAULT != (motion_state_t) g_state)
    {
        reset_wheel_control();
        set_state(MOTION_STATE_IDLE);
        post_event(MOTION_EVT_STOPPED, 0u, 0u, 0);
    }
    else
    {
        /* Already in fault: motors are already at 0. */
    }
}

static void
publish_snapshot (void)
{
    uint32_t w1 = 0u;

    w1 |= msnap_pack_speed(g_wheel[MOTOR_LEFT].speed_cpt)  << MSNAP_W1_L_SHIFT;
    w1 |= msnap_pack_speed(g_wheel[MOTOR_RIGHT].speed_cpt) << MSNAP_W1_R_SHIFT;
    w1 |= ((uint32_t) g_state & MSNAP_W1_STATE_MASK) << MSNAP_W1_STATE_SHIFT;
    w1 |= (0u != (g_fault_bits & LATCHING_FAULTS)) ? MSNAP_W1_FAULT_BIT : 0u;
    w1 |= g_slow ? MSNAP_W1_SLOW_BIT : 0u;
    w1 |= g_steer_stale ? MSNAP_W1_STALE_BIT : 0u;

    int32_t const cm = motion_counts_to_mm(g_odo_sum / 2) / 10;

    g_snap_w1 = w1;
    g_snap_w2 = ((uint32_t) (uint16_t) cm) |
                ((uint32_t) (uint16_t) g_tick_count << 16);
}

void
motion_tick (void)
{
    uint32_t const t0 = motion_hal_time_us();
    int16_t duty_l = 0;
    int16_t duty_r = 0;

    if ((0u != g_tick_count) &&
        ((uint32_t) (t0 - g_last_tick_us) > MOTION_TICK_OVERRUN_US))
    {
        g_stats.tick_overruns++;
    }

    g_last_tick_us = t0;
    g_tick_count++;

    measure_wheels();
    handle_stop_requests();
    process_commands();

    switch ((motion_state_t) g_state)
    {
        case MOTION_STATE_FOLLOW:
        {
            run_follow(&duty_l, &duty_r);
            break;
        }

        case MOTION_STATE_DRIVE_DISTANCE:
        case MOTION_STATE_TURNING:
        {
            run_distance_or_turn(&duty_l, &duty_r);
            break;
        }

        case MOTION_STATE_SETTLE:
        {
            run_settle(&duty_l, &duty_r);
            break;
        }

        case MOTION_STATE_SPEED:
        {
            run_speed(&duty_l, &duty_r);
            break;
        }

        case MOTION_STATE_RECOVERY:
        {
            run_recovery(&duty_l, &duty_r);
            break;
        }

        case MOTION_STATE_SELFTEST:
        {
            run_selftest(&duty_l, &duty_r);
            break;
        }

        case MOTION_STATE_IDLE:
        case MOTION_STATE_MOTOR_FAULT:
        default:
        {
            duty_l = 0;
            duty_r = 0;
            break;
        }
    }

    apply_pwm(duty_l, duty_r);
    publish_snapshot();

    if (NULL != g_log_hook)
    {
        motion_log_t rec;

        rec.tick  = g_tick_count;
        rec.state = g_state;

        for (uint32_t m = 0u; m < MOTOR_COUNT; m++)
        {
            rec.setpoint_cpt_q8[m] = (int16_t) clamp32(g_wheel[m].setpoint_q8,
                                                       -32768, 32767);
            rec.speed_cpt[m]       = g_wheel[m].speed_cpt;
            rec.duty_permille[m]   = g_wheel[m].duty;
            rec.travel[m]          = g_wheel[m].travel;
        }

        g_log_hook(&rec);
    }

    uint32_t const dt = motion_hal_time_us() - t0;

    g_stats.ticks        = g_tick_count;
    g_stats.tick_last_us = dt;

    if (dt > g_stats.tick_wcet_us)
    {
        g_stats.tick_wcet_us = dt;
    }
}

/* ---- Public API -------------------------------------------------------- */

void
motion_poll_commands (void)
{
    handle_stop_requests();
    process_commands();
}

void
motion_init (void)
{
    motion_hal_pwm_zero_all();

    (void) memset(g_wheel, 0, sizeof(g_wheel));
    (void) memset(&g_move, 0, sizeof(g_move));
    (void) memset(&g_stats, 0, sizeof(g_stats));

    for (uint32_t m = 0u; m < MOTOR_COUNT; m++)
    {
        g_wheel[m].last_count = motion_hal_encoder_count((motor_id_t) m);
    }

    g_gains.ff_q8              = MOTION_FF_Q8;
    g_gains.ff_offset_permille = MOTION_FF_OFFSET_PERMILLE;
    g_gains.kp_q8              = MOTION_KP_Q8;
    g_gains.ki_q8              = MOTION_KI_Q8;
    g_gains.ksync_q8           = MOTION_KSYNC_Q8;
    g_gains.accel_q8_per_tick  = MOTION_ACCEL_Q8_PER_TICK;

    g_stop_req    = 0u;
    g_fault_bits  = 0u;
    g_steer_word  = 0u;
    g_slow        = false;
    g_cmd_head    = 0u;
    g_cmd_tail    = 0u;
    g_evt_head    = 0u;
    g_evt_tail    = 0u;
    g_steer_stale = false;
    g_odo_sum       = 0;
    g_odo_reset_req = false;
    g_tick_count  = 0u;
    g_active_seq  = 0u;
    set_state(MOTION_STATE_IDLE);
    publish_snapshot();
}

void
motion_request_stop (bool estop)
{
    motion_hal_debug_pin(true);

    uint32_t const irq = motion_hal_irq_save();

    g_stop_req |= estop ? STOP_REQ_ESTOP : STOP_REQ_STOP;
    motion_hal_pwm_zero_all();
    g_cmd_tail = g_cmd_head;    /* Cancel everything queued before STOP. */
    g_stats.stop_requests++;
    motion_hal_irq_restore(irq);

    motion_hal_debug_pin(false);

    if (NULL != g_notify)
    {
        g_notify();
    }
}

bool
motion_command (const motion_cmd_t * p_cmd)
{
    if (NULL == p_cmd)
    {
        return false;
    }

    if (MOTION_CMD_STOP == p_cmd->id)
    {
        motion_request_stop(false);
        return true;
    }

    if (MOTION_CMD_ESTOP == p_cmd->id)
    {
        motion_request_stop(true);
        return true;
    }

    bool queued = false;
    uint32_t const irq  = motion_hal_irq_save();
    uint8_t const  next = (uint8_t) ((g_cmd_head + 1u) & (MOTION_CMD_QUEUE_LEN - 1u));

    if (next != g_cmd_tail)
    {
        g_cmd_q[g_cmd_head] = *p_cmd;
        g_cmd_head = next;
        queued     = true;
    }
    else
    {
        g_stats.cmd_dropped++;
    }

    motion_hal_irq_restore(irq);

    if (queued && (NULL != g_notify))
    {
        g_notify();
    }

    return queued;
}

void
motion_set_steer (int8_t steer)
{
    if (steer < -127)
    {
        steer = -127;
    }

    /* One aligned 32-bit store: atomic, no lock needed. */
    g_steer_word = ((uint32_t) stamp_now() << 16) | 0x100u | (uint8_t) steer;
}

void
motion_set_slow (bool slow)
{
    g_slow = slow;
}

bool
motion_get_event (motion_evt_t * p_evt)
{
    if (g_evt_tail == g_evt_head)
    {
        return false;
    }

    *p_evt     = g_evt_q[g_evt_tail];
    g_evt_tail = (uint8_t) ((g_evt_tail + 1u) & (MOTION_EVT_QUEUE_LEN - 1u));
    return true;
}

void
motion_get_snapshot (uint32_t * p_w1, uint32_t * p_w2)
{
    uint32_t const irq = motion_hal_irq_save();

    *p_w1 = g_snap_w1;
    *p_w2 = g_snap_w2;
    motion_hal_irq_restore(irq);
}

motion_state_t
motion_get_state (void)
{
    return (motion_state_t) g_state;
}

uint32_t
motion_get_fault_bits (void)
{
    return g_fault_bits | (g_steer_stale ? FAULT_STEER_STALE : 0u);
}

void
motion_odometer_reset (void)
{
    /* The tick applies it, so the tick stays the only writer of the sum. */
    g_odo_reset_req = true;
}

void
motion_get_gains (motion_gains_t * p_gains)
{
    *p_gains = g_gains;
}

void
motion_set_gains (const motion_gains_t * p_gains)
{
    uint32_t const irq = motion_hal_irq_save();

    g_gains = *p_gains;
    motion_hal_irq_restore(irq);
}

void
motion_get_stats (motion_stats_t * p_stats)
{
    uint32_t const irq = motion_hal_irq_save();

    *p_stats = g_stats;
    motion_hal_irq_restore(irq);
}

void
motion_set_notify (motion_notify_fn_t fn)
{
    g_notify = fn;
}

void
motion_set_log_hook (motion_log_fn_t fn)
{
    g_log_hook = fn;
}

const char *
motion_state_name (motion_state_t state)
{
    static const char * const names[MOTION_STATE_COUNT] =
    {
        "IDLE", "SELECT_MOTION", "FOLLOW", "DRIVE_DISTANCE", "TURNING",
        "SETTLE", "MOTOR_FAULT", "SPEED", "SELFTEST", "RECOVERY",
    };

    return (state < MOTION_STATE_COUNT) ? names[state] : "?";
}

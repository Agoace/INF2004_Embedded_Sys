/**
 * @file manager.c
 * @brief See manager.h.
 *
 * Communication with Motion uses only the public Motion API:
 *  - motion_command() / motion_request_stop() for discrete commands,
 *  - line_motion_bridge for the latest-value STEER,
 *  - motion_get_event() output, which the caller forwards here.
 */
#include "manager.h"

#include <stddef.h>
#include <string.h>

#include "line_motion_bridge.h"
#include "motion_control.h"

/* ---- Latest-value line mailbox ----------------------------------------- */

/*
 * One aligned 32-bit word: [2:0] black pattern, [5:3] unknown mask,
 * [8:6] fault mask. An aligned 32-bit store is atomic on the Cortex-M0+, so
 * the ISR or the Line task can overwrite it while the 1 ms step reads it.
 * g_line_published lets the Manager count samples that were overwritten
 * before it could use them.
 */
#define LINE_PACK(black, unknown, fault) \
    (((uint32_t) (black)   & 0x7u)       | \
     (((uint32_t) (unknown) & 0x7u) << 3) | \
     (((uint32_t) (fault)   & 0x7u) << 6))

static volatile uint32_t g_line_word;
static volatile uint32_t g_line_published;
static uint32_t          g_line_consumed;   /* Written by manager_step only. */

/* ---- Bounded event queue ---------------------------------------------- */

static manager_evt_t     g_evt_q[MANAGER_EVENT_QUEUE_LEN];
static volatile uint8_t  g_evt_head;        /* Next write.                   */
static volatile uint8_t  g_evt_tail;        /* Next read.                    */

/* ---- Manager state ----------------------------------------------------- */

static manager_state_t     g_state;
static manager_stats_t     g_stats;
static uint32_t            g_fault_bits;
static uint8_t             g_cmd_seq;

static bool                g_lost;
static uint32_t            g_lost_since_ms;
static uint8_t             g_tracking_samples;
static manager_line_dir_t  g_dir;

/* ======================================================================= */

static uint8_t
next_seq (void)
{
    g_cmd_seq++;
    return g_cmd_seq;
}

static void
post_motion_cmd (motion_cmd_id_t id, int16_t arg_a, int16_t speed_mm_s)
{
    motion_cmd_t cmd;

    cmd.id           = (uint8_t) id;
    cmd.seq          = next_seq();
    cmd.arg_a        = arg_a;
    cmd.speed_mm_s   = speed_mm_s;
    cmd.reserved     = 0;

    if (!motion_command(&cmd))
    {
        g_stats.motion_rejections++;
    }
}

static void
set_state (manager_state_t state)
{
    g_state = state;
}

static void
reset_run_tracking (void)
{
    g_lost             = false;
    g_lost_since_ms    = 0u;
    g_tracking_samples = 0u;
}

/* ---- State transitions ------------------------------------------------- */

/** Stop the wheels now and disarm line following. */
static void
stop_follow (void)
{
    /* STOP is the Motion fast path: PWM 0 in this context and the queued
     * commands are flushed, so no earlier FOLLOW can restart the wheels. */
    motion_request_stop(false);
    reset_run_tracking();
    set_state(MANAGER_LINE_FOLLOW_OFF);
}

/** Mark the travel origin and start following. */
static void
start_follow (void)
{
    line_bridge_mark();
    motion_odometer_reset();
    post_motion_cmd(MOTION_CMD_FOLLOW, 0, 0);
    g_dir = MANAGER_DIR_STRAIGHT;
    reset_run_tracking();
    set_state(MANAGER_LINE_FOLLOW_ON);
}

/**
 * Stop, then hand Motion the bounded reverse-follow manoeuvre. Motion keeps
 * the bridge's forward STEER and negates both wheel set-points, so the stored
 * left/right bias unwinds and the robot retraces the arc toward the line.
 */
static void
start_recovery (void)
{
    /* Stop first: the robot must not keep driving forward while the
     * reverse command waits for the next Motion tick. */
    motion_request_stop(false);

    motion_cmd_t cmd;

    cmd.id         = (uint8_t) MOTION_CMD_RECOVER;
    cmd.seq        = next_seq();
    cmd.arg_a      = (int16_t) MANAGER_RECOVERY_DISTANCE_MM;
    cmd.speed_mm_s = (int16_t) MANAGER_RECOVERY_SPEED_MM_S;
    cmd.reserved   = 0;

    if (!motion_command(&cmd))
    {
        g_stats.motion_rejections++;
        g_fault_bits |= FAULT_RECOVERY_TIMEOUT;
        reset_run_tracking();
        set_state(MANAGER_FAULT);
        return;
    }

    g_tracking_samples = 0u;
    g_stats.recoveries++;
    set_state(MANAGER_RECOVERY);
}

/** Three valid samples in recovery: resume forward following. */
static void
finish_recovery (void)
{
    /* End the reverse manoeuvre with the Motion fast path... */
    motion_request_stop(false);
    /* ...then re-arm forward travel. The bridge already holds the forward
     * direction for this sample, so nothing has to be un-inverted. */
    line_bridge_mark();
    motion_odometer_reset();
    post_motion_cmd(MOTION_CMD_FOLLOW, 0, 0);

    g_stats.reacquisitions++;
    reset_run_tracking();
    set_state(MANAGER_LINE_FOLLOW_ON);
}

static void
enter_fault (uint32_t fault_bits)
{
    g_fault_bits |= fault_bits;
    /* A latched Manager fault must never leave the wheels driven. */
    motion_request_stop(false);
    reset_run_tracking();
    set_state(MANAGER_FAULT);
}

/* ---- Line handling ----------------------------------------------------- */

/**
 * Classify the observation, then let the bridge refresh STEER.
 *
 * The bridge is the only writer of the L/C/R -> STEER mapping. The Manager
 * needs the status and the signed error to run the loss timer and to retain
 * the last valid direction, so it classifies once with the same call the
 * bridge uses internally.
 */
static line_status_t
observe_line (const line_observation_t * p_obs, int8_t * p_error)
{
    line_status_t const status =
        line_bridge_classify(p_obs->black_pattern, p_obs->unknown_mask,
                             p_obs->fault_mask, p_error);

    (void) line_bridge_update(p_obs->black_pattern, p_obs->unknown_mask,
                             p_obs->fault_mask);

    return status;
}

static void
handle_line_status (line_status_t status, int8_t error, uint32_t now_ms)
{
    switch (status)
    {
        case LINE_STATUS_FAULT:
        {
            g_stats.line_faults++;
            enter_fault(FAULT_LINE_SENSOR);
            break;
        }

        case LINE_STATUS_TRACKING:
        {
            /* Retain the newest valid forward direction. */
            if (error < 0)
            {
                g_dir = MANAGER_DIR_LEFT;
            }
            else if (error > 0)
            {
                g_dir = MANAGER_DIR_RIGHT;
            }
            else
            {
                g_dir = MANAGER_DIR_STRAIGHT;
            }

            g_lost = false;

            if (MANAGER_RECOVERY == g_state)
            {
                g_stats.recovery_samples++;
                g_tracking_samples++;

                if (g_tracking_samples >= MANAGER_RECOVERY_TRACKING_SAMPLES)
                {
                    finish_recovery();
                }
            }

            break;
        }

        case LINE_STATUS_LOST:
        {
            if (MANAGER_LINE_FOLLOW_ON != g_state)
            {
                break;      /* Only a forward follow can start a recovery. */
            }

            if (!g_lost)
            {
                g_lost          = true;
                g_lost_since_ms = now_ms;
            }
            else if ((now_ms - g_lost_since_ms) >= MANAGER_LINE_LOST_TIMEOUT_MS)
            {
                start_recovery();
            }

            break;
        }

        case LINE_STATUS_JUNCTION:
        case LINE_STATUS_SPLIT:
        case LINE_STATUS_UNKNOWN:
        default:
        {
            /* The bridge drives straight through; not a loss. Junction and
             * mission policy are out of scope for this checkpoint. */
            g_lost = false;
            break;
        }
    }
}

/** Consume the mailbox only when a newer observation has arrived. */
static void
update_line (uint32_t now_ms)
{
    uint32_t const published = g_line_published;

    if (published == g_line_consumed)
    {
        return;
    }

    uint32_t const delta = published - g_line_consumed;

    g_line_consumed    = published;
    g_stats.line_overwrites += delta - 1u;   /* 0 when every sample is used. */

    uint32_t const word = g_line_word;
    line_observation_t obs;

    obs.black_pattern = (uint8_t) (word & 0x7u);
    obs.unknown_mask  = (uint8_t) ((word >> 3) & 0x7u);
    obs.fault_mask    = (uint8_t) ((word >> 6) & 0x7u);
    obs.reserved      = 0u;

    /* Outside FOLLOW the Manager stays silent: no STEER is published, so
     * Motion cannot be driven by a stale line reading. */
    if ((MANAGER_LINE_FOLLOW_ON != g_state) && (MANAGER_RECOVERY != g_state))
    {
        return;
    }

    int8_t error = 0;
    line_status_t const status = observe_line(&obs, &error);

    handle_line_status(status, error, now_ms);
}

/* ---- Event handling ---------------------------------------------------- */

static void
handle_button (const button_event_t * p_evt)
{
    if (0u == p_evt->pressed)
    {
        return;
    }

    switch (g_state)
    {
        case MANAGER_LINE_FOLLOW_OFF:
        {
            start_follow();
            g_stats.button_toggles++;
            break;
        }

        case MANAGER_LINE_FOLLOW_ON:
        case MANAGER_RECOVERY:
        {
            stop_follow();
            g_stats.button_toggles++;
            break;
        }

        case MANAGER_SELFTEST:
        case MANAGER_FAULT:
        default:
        {
            /* Ignored by design: the self-test owns the wheels, and a latched
             * fault needs a reset, not another toggle. */
            g_stats.button_ignored++;
            break;
        }
    }
}

static void
handle_motion (const motion_evt_t * p_evt)
{
    switch (p_evt->id)
    {
        case MOTION_EVT_SELFTEST:
        {
            if (MANAGER_SELFTEST != g_state)
            {
                break;
            }

            if (1u == p_evt->arg)
            {
                reset_run_tracking();
                g_dir = MANAGER_DIR_STRAIGHT;
                set_state(MANAGER_LINE_FOLLOW_OFF);
            }
            else
            {
                /* Motion already latched FAULT_ENCODER and stopped. */
                reset_run_tracking();
                set_state(MANAGER_FAULT);
            }

            break;
        }

        case MOTION_EVT_STALL:
        {
            enter_fault(FAULT_NONE);    /* Motion holds FAULT_MOTOR_STALL_*. */
            break;
        }

        case MOTION_EVT_STOPPED:
        {
            /* This checkpoint has no physical ESTOP input, so arg = 1 can
             * only come from another writer: treat it as a fault. arg = 0 is
             * the Manager's own STOP and needs no action. */
            if (1u == p_evt->arg)
            {
                enter_fault(FAULT_ESTOP);
            }

            break;
        }

        case MOTION_EVT_DONE:
        {
            if ((MANAGER_RECOVERY == g_state) &&
                (MOTION_DONE_RECOVERY_LIMIT == p_evt->arg))
            {
                /* Recovery used the whole distance budget without finding
                 * the line again: latch the fault instead of retrying. */
                enter_fault(FAULT_RECOVERY_TIMEOUT);
            }

            break;
        }

        case MOTION_EVT_STEER_STALE:
        {
            g_stats.steer_stale++;
            break;
        }

        case MOTION_EVT_REJECTED:
        {
            g_stats.motion_rejections++;
            break;
        }

        default:
        {
            break;
        }
    }
}

static void
update_events (void)
{
    while (g_evt_tail != g_evt_head)
    {
        manager_evt_t evt = g_evt_q[g_evt_tail];

        g_evt_tail = (uint8_t) ((g_evt_tail + 1u) &
                                (MANAGER_EVENT_QUEUE_LEN - 1u));

        if (MANAGER_EVT_BUTTON == evt.kind)
        {
            handle_button(&evt.payload.button);
        }
        else
        {
            handle_motion(&evt.payload.motion);
        }
    }
}

/* ---- Public API -------------------------------------------------------- */

void
manager_init (uint32_t now_ms)
{
    (void) now_ms;

    g_line_word      = 0u;
    g_line_published = 0u;
    g_line_consumed  = 0u;
    g_evt_head       = 0u;
    g_evt_tail       = 0u;
    g_fault_bits     = 0u;
    g_cmd_seq        = 0u;
    g_dir            = MANAGER_DIR_STRAIGHT;

    (void) memset(&g_stats, 0, sizeof(g_stats));
    reset_run_tracking();
    set_state(MANAGER_SELFTEST);

    /* Boot: ask Motion for the encoder self-test (FR17). */
    post_motion_cmd(MOTION_CMD_SELFTEST, 0, 0);
}

void
manager_publish_line (const line_observation_t * p_obs)
{
    if (NULL == p_obs)
    {
        return;
    }

    g_line_word = LINE_PACK(p_obs->black_pattern, p_obs->unknown_mask,
                            p_obs->fault_mask);
    g_line_published++;
}

bool
manager_post_event (const manager_evt_t * p_evt)
{
    if (NULL == p_evt)
    {
        return false;
    }

    uint8_t const next = (uint8_t) ((g_evt_head + 1u) &
                                    (MANAGER_EVENT_QUEUE_LEN - 1u));

    if (next == g_evt_tail)
    {
        g_stats.events_dropped++;
        return false;
    }

    g_evt_q[g_evt_head] = *p_evt;
    g_evt_head          = next;

    return true;
}

bool
manager_post_button (manager_button_t button, uint32_t now_ms)
{
    (void) now_ms;

    manager_evt_t evt;

    evt.kind                = (uint8_t) MANAGER_EVT_BUTTON;
    evt.payload.button.button  = (uint8_t) button;
    evt.payload.button.pressed = 1u;
    evt.payload.button.reserved = 0u;

    return manager_post_event(&evt);
}

void
manager_step (uint32_t now_ms)
{
    g_stats.steps++;

    update_line(now_ms);
    update_events();
}

manager_state_t
manager_get_state (void)
{
    return g_state;
}

manager_line_dir_t
manager_get_line_direction (void)
{
    return g_dir;
}

uint32_t
manager_get_fault_bits (void)
{
    return g_fault_bits | motion_get_fault_bits();
}

void
manager_get_stats (manager_stats_t * p_stats)
{
    if (NULL == p_stats)
    {
        return;
    }

    *p_stats = g_stats;
}

const char *
manager_state_name (manager_state_t state)
{
    static const char * const names[MANAGER_STATE_COUNT] =
    {
        "SELFTEST", "LINE_FOLLOW_OFF", "LINE_FOLLOW_ON", "RECOVERY", "FAULT",
    };

    return (state < MANAGER_STATE_COUNT) ? names[state] : "?";
}

const char *
manager_dir_name (manager_line_dir_t dir)
{
    static const char * const names[MANAGER_DIR_COUNT] =
    {
        "Straight", "Left", "Right",
    };

    return (dir < MANAGER_DIR_COUNT) ? names[dir] : "?";
}

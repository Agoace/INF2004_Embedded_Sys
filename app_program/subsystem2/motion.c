/* Kernel side only. SDK register access is isolated in drv_motor_hw.c. */
#include "subsystem2.h"

/* Task timing and resource limits. */
#define ENCODER_LEFT_A_PIN   16u
#define ENCODER_LEFT_B_PIN   17u
#define MOTION_LATE_GAP_US   11000u
#define MOTION_TASK_PRIORITY 10
#define WATCHDOG_TIMEOUT_MS  100u

#include "motion.h"
#include "motion_diag.h"
#if APP_DIAG
#include "motion_diag_capture.h"
static md_capture_t s_diag;
#endif
#include "motor_ctl_robot.h"
#include "drv_motor_hw.h"
#include "../platform/drv_gpio_irq.h"
#include "../platform/drv_time.h"
#include "../platform/drv_watchdog.h"

static const mc_config_t config = MC_ROBOT_CONFIG;
static const int8_t QUADRATURE[16] = {
    0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0,
};
static volatile int32_t raw_count[MC_WHEELS];
static uint8_t previous_ab[MC_WHEELS];
static mc_t mc;
static ID mtx_motion;
static ID tid_motion;
static ID cyc_motion;
static ID tid_after_tick;
static int32_t sampled_counts[MC_WHEELS];
static uint32_t ticks;
static uint32_t tick_max_us;
static uint32_t last_run_us;
static uint32_t gap_min_us;
static uint32_t gap_max_us;
static uint32_t late_wakes;
static uint32_t missed_wakes;
static bool watchdog_armed;
static void motion_status_locked(motion_status_t *s);
#include "../platform/app_board.h"

typedef struct {
    motion_status_t status;
    uint32_t us;
} motion_board_t;

static APP_BOARD(motion_board_t) board;

/*
 * Publish a coherent motion snapshot after a completed tick.
 */
static void publish_motion(uint32_t us)
{
    motion_board_t v = {0};
    v.us = us;
    motion_status_locked(&v.status);
    app_board_publish(&board.sequence, board.words, &v, sizeof(v));
}
#if APP_TEST
static volatile bool test_requested;
static volatile bool test_suppressed;
static bool window_active;
static motion_timing_window_t window;
static uint32_t window_last_us;
static uint32_t window_missed_base;
static motion_timing_probe_t probes[MOTION_PROBE_COUNT];
static UINT probe_present;
#endif

/*
 * Process an encoder edge through the kernel-owned GPIO dispatcher.
 */
static void encoder_irq(unsigned int pin, uint32_t events, void *ctx)
{
    (void)events;
    (void)ctx;
    const unsigned int wheel =
        pin == ENCODER_LEFT_A_PIN || pin == ENCODER_LEFT_B_PIN ? MC_LEFT
                                                               : MC_RIGHT;
    const uint8_t ab = drv_motor_hw_ab(wheel);
    raw_count[wheel] += QUADRATURE[(previous_ab[wheel] << 2u) | ab];
    previous_ab[wheel] = ab;
}

/*
 * Copy both encoder counts with interrupt protection.
 */
static void copy_counts(int32_t counts[MC_WHEELS])
{
    UINT irq;
    DI(irq); /* motion_task and encoder IRQ both belong to physical core 0. */
    counts[MC_LEFT] = raw_count[MC_LEFT];
    counts[MC_RIGHT] = -raw_count[MC_RIGHT];
    EI(irq);
}

/*
 * Enter the motion API mutex and preserve kernel error results.
 */
static ER lock_motion(void)
{
    return mtx_motion > 0 ? tk_loc_mtx(mtx_motion, TMO_FEVR) : E_OBJ;
}

#if APP_DIAG
static ER diag_lock_motion(const char *owner)
{
    const ER result = lock_motion();
    if (result >= E_OK) {
        s_diag.locked_at = drv_time_us();
        s_diag.owner = owner;
    }
    return result;
}

static ER diag_unlock_motion(void)
{
    s_diag.prior_hold = drv_time_us() - s_diag.locked_at;
    s_diag.prior_owner = s_diag.owner;
    return tk_unl_mtx(mtx_motion);
}
#define lock_motion() diag_lock_motion(__func__)
#define unlock_motion() diag_unlock_motion()

void motion_diag_phase(uint32_t phase)
{
    s_diag.phase = phase;
}

void motion_diag_dump(int active)
{
    s_diag.dumping = active ? MD_DUMP_FLAG : 0u;
}

static void diag_print_point(const char *label, const md_point_t *p)
{
    tm_printf((UB *)"[MOTION-DIAG] %s entry=%lu gap=%lu release=%lu release_gap=%lu "
        "dispatch=%lu acquired=%lu wait=%lu phase=0x%lx prior=%s hold=%lu queued=%ld\n",
        label, (unsigned long)p->entry, (unsigned long)p->gap,
        (unsigned long)p->release, (unsigned long)p->release_gap,
        (unsigned long)(p->entry - p->release), (unsigned long)p->acquired,
        (unsigned long)(p->acquired - p->entry), (unsigned long)p->phase,
        p->prior_owner ? p->prior_owner : "none", (unsigned long)p->prior_hold,
        (long)p->queued);
}

void motion_diag_print(void)
{
    md_point_t latest, worst;
    uint32_t observations;
    UINT irq;
    DI(irq); /* GP20 only: bounded copy, no print while interrupts are masked. */
    latest = s_diag.latest;
    worst = s_diag.worst;
    observations = s_diag.observations;
    EI(irq);
    tm_printf((UB *)"[MOTION-DIAG] lifetime observations=%lu; DIAG=1 changes timing\n",
        (unsigned long)observations);
    if (observations != 0u) {
        diag_print_point("latest", &latest);
        diag_print_point("worst-gap", &worst);
    }
}
#else
#define unlock_motion() tk_unl_mtx(mtx_motion)
#endif

/*
 * Run the controller tick and feed the motion-only watchdog policy.
 */
static void motion_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    for (;;) {
        if (tk_slp_tsk(TMO_FEVR) < E_OK) {
            continue;
        }
        const INT queued = tk_can_wup(TSK_SELF);
        const uint32_t started = drv_time_us();
#if APP_DIAG
        md_point_t point = {.entry = started,
            .gap = ticks != 0u ? started - last_run_us : 0u,
            .release = s_diag.release, .release_gap = s_diag.release_gap,
            .phase = s_diag.phase | s_diag.dumping, .queued = queued};
#endif
        if (lock_motion() < E_OK) {
            continue;
        }
#if APP_DIAG
        point.acquired = s_diag.locked_at;
        point.prior_owner = s_diag.prior_owner;
        point.prior_hold = s_diag.prior_hold;
        md_observe(&s_diag, &point);
#endif
        if (ticks != 0u) {
            const uint32_t gap = started - last_run_us;
            if (gap_min_us == 0u || gap < gap_min_us) {
                gap_min_us = gap;
            }
            if (gap > gap_max_us) {
                gap_max_us = gap;
            }
            if (gap > MOTION_LATE_GAP_US) {
                ++late_wakes;
            }
        }
        last_run_us = started;
        if (queued > 0) {
            missed_wakes += (uint32_t)queued;
        }
        copy_counts(sampled_counts);
        mc_output_t out;
        mc_tick(&mc, sampled_counts, drv_time_ms(), &out);
        /* Same mutex as stop/estop: no old computed output can overwrite stop. */
        drv_motor_hw_write(out.duty_permille, out.servo_us);
        ++ticks;
        publish_motion(started);
        const uint32_t took = drv_time_us() - started;
        if (took > tick_max_us) {
            tick_max_us = took;
        }
#if APP_TEST
        if (window_active) {
            if (window.ticks != 0u) {
                const uint32_t gap = started - window_last_us;
                if (window.gaps == 0u || gap < window.gap_min_us) {
                    window.gap_min_us = gap;
                }
                if (gap > window.gap_max_us) {
                    window.gap_max_us = gap;
                }
                if (gap > MOTION_LATE_GAP_US) {
                    ++window.late_wakes;
                }
                ++window.gaps;
            }
            window_last_us = started;
            ++window.ticks;
            if (took > window.tick_max_us) {
                window.tick_max_us = took;
            }
        }
#endif
        const ID notify = tid_after_tick;
        (void)unlock_motion();
        /* Only completed control work feeds; never the cyclic handler or dump. */
        if (watchdog_armed) {
#if APP_TEST
            if (!test_suppressed) {
                drv_watchdog_feed();
                if (test_requested && drv_watchdog_test_begin() >= E_OK) {
                    test_suppressed = true;
                }
            }
#else
            drv_watchdog_feed();
#endif
        }
        if (notify > 0) {
            (void)tk_wup_tsk(notify);
        }
    }
}

/*
 * Wake the motion task for its next controller tick.
 */
static void motion_cyclic(void *exinf)
{
    (void)exinf;
#if APP_DIAG
    md_release(&s_diag, drv_time_us());
#endif
    (void)tk_wup_tsk(tid_motion);
}

/*
 * Initialize the controller and create its mutex and release objects.
 */
int motion_init(void)
{
#if TK_SUPPORT_SMP
    if (tk_get_prc() != 1) {
        return (int)E_CTX;
    }
#endif
    if (mtx_motion > 0) {
        return (int)E_OBJ;
    }
    /* Establish zero motor PWM before checking the clock; never enable motion
     * with a bad TIMER rate, even if usermain's earlier initialization failed. */
    drv_motor_hw_init(config.servo_centre_us);
    int er = drv_time_init();
    if (er >= E_OK) {
        er = drv_time_check();
    }
    if (er < E_OK) {
        tm_printf((UB *)"ERROR motion: TIMER clock check failed (%d); motors "
                        "disabled.\n",
                  er);
        return er;
    }
    T_CMTX m = {0};
    m.mtxatr = TA_INHERIT;
    mtx_motion = tk_cre_mtx(&m);
    if (mtx_motion < E_OK) {
        return (int)mtx_motion;
    }
    mc_init(&mc, &config, sampled_counts, drv_time_ms());
    previous_ab[MC_LEFT] = drv_motor_hw_ab(MC_LEFT);
    previous_ab[MC_RIGHT] = drv_motor_hw_ab(MC_RIGHT);
    er = drv_gpio_irq_init();
    if (er >= E_OK) {
        er = drv_gpio_irq_add(DRV_MOTOR_ENCODER_MASK, encoder_irq, 0);
    }
    if (er < E_OK) {
        (void)tk_del_mtx(mtx_motion);
        mtx_motion = 0;
        return er;
    }
    T_CCYC c = {0};
    c.cycatr = TA_HLNG;
    c.cychdr = (FP)motion_cyclic;
    c.cyctim = config.tick_ms;
    c.cycphs = config.tick_ms;
#if TK_SUPPORT_SMP
    c.cycatr |= TA_ASSPRC;
    c.assprc = TP_PRC1;
#endif
    cyc_motion = tk_cre_cyc(&c);
    if (cyc_motion < E_OK) {
        (void)tk_del_mtx(mtx_motion);
        mtx_motion = 0;
        return (int)cyc_motion;
    }
    return (int)E_OK;
}

/*
 * Start the motion task and its periodic release; retain watchdog policy.
 */
int motion_start(void)
{
    if (mtx_motion <= 0 || cyc_motion <= 0 || tid_motion > 0) {
        return (int)E_OBJ;
    }
    tid_motion = app_start_task((FP)motion_task, MOTION_TASK_PRIORITY, TP_PRC1,
                                "motion_task");
    if (tid_motion < E_OK) {
        return (int)tid_motion;
    }
    const ER cyclic = tk_sta_cyc(cyc_motion);
    if (cyclic < E_OK) {
        return (int)cyclic;
    }
    const int er = drv_watchdog_start(WATCHDOG_TIMEOUT_MS);
    if (er < E_OK) {
        tm_printf((UB *)"ERROR motion: watchdog start failed (%d); speed "
                        "commands disabled.\n",
                  er);
        return er;
    }
    watchdog_armed = true;
    return (int)E_OK;
}
#if APP_TEST
/*
 * Start a TEST timing window without resetting lifetime counters.
 */
int motion_timing_window_begin(void)
{
    if (!watchdog_armed) {
        return (int)E_OBJ;
    }
    const ER er = lock_motion();
    if (er < E_OK) {
        return (int)er;
    }
    int result = E_OBJ;
    if (!window_active) {
        window = (motion_timing_window_t){0};
        window_missed_base = missed_wakes;
        window_active = true;
        result = E_OK;
    }
    (void)unlock_motion();
    return result;
}

/*
 * Close the TEST timing window and return its observations.
 */
int motion_timing_window_end(motion_timing_window_t *out)
{
    if (!out) {
        return (int)E_PAR;
    }
    const ER er = lock_motion();
    if (er < E_OK) {
        return (int)er;
    }
    int result = E_OBJ;
    if (window_active) {
        window.missed_wakes = missed_wakes - window_missed_base;
        *out = window;
        window_active = false;
        result = E_OK;
    }
    (void)unlock_motion();
    return result;
}

/*
 * Store one TEST timing probe without printing on the critical path.
 */
int motion_timing_probe_capture(motion_probe_stage_t stage)
{
    if ((unsigned int)stage >= MOTION_PROBE_COUNT) {
        return (int)E_PAR;
    }
    const ER er = lock_motion();
    if (er < E_OK) {
        return (int)er;
    }
    probes[stage] = (motion_timing_probe_t){
        drv_time_us(), ticks,      gap_min_us,  gap_max_us,
        tick_max_us,   late_wakes, missed_wakes};
    probe_present |= 1u << (unsigned int)stage;
    return (int)unlock_motion();
}

/*
 * Copy a saved TEST probe for deferred reporting.
 */
int motion_timing_probe_read(motion_probe_stage_t stage,
                             motion_timing_probe_t *out)
{
    if (!out || (unsigned int)stage >= MOTION_PROBE_COUNT) {
        return (int)E_PAR;
    }
    if ((probe_present & (1u << (unsigned int)stage)) == 0u) {
        return (int)E_OBJ;
    }
    *out = probes[stage];
    return E_OK;
}

/*
 * Enable the deliberate TEST-only watchdog feed suppression.
 */
int motion_watchdog_test_suppress_feed(void)
{
    if (!watchdog_armed) {
        return (int)E_OBJ;
    }
    test_requested = true;
    return (int)E_OK;
}

/*
 * Report whether the TEST-only feed suppression has taken effect.
 */
bool motion_watchdog_test_feed_stopped(void)
{
    return test_suppressed;
}
#endif
/*
 * Register the task to wake after each completed controller tick.
 */
int motion_notify_after_tick(ID tid)
{
    if (tid < 0) {
        return (int)E_PAR;
    }
    const ER er = lock_motion();
    if (er < E_OK) {
        return (int)er;
    }
    tid_after_tick = tid;
    return (int)unlock_motion();
}

/*
 * Set wheel speed through the inherited motion mutex.
 */
int motion_set_speed(int16_t left, int16_t right)
{
    if (!watchdog_armed) {
        return (int)E_OBJ;
    }
    const ER er = lock_motion();
    if (er < E_OK) {
        return (int)er;
    }
    mc_set_speed(&mc, left, right, drv_time_ms());
    return (int)unlock_motion();
}

/*
 * Stop the controller through the inherited motion mutex.
 */
int motion_stop(void)
{
    const ER er = lock_motion();
    if (er < E_OK) {
        return (int)er;
    }
    mc_stop(&mc);
    drv_motor_hw_stop();
    return (int)unlock_motion();
}

/*
 * Latch the controller emergency stop through its mutex.
 */
int motion_estop(void)
{
    motion_diag_phase(MOTION_DIAG_STOP);
    const ER er = lock_motion();
    if (er < E_OK) {
        return (int)er;
    }
    mc_estop(&mc);
    drv_motor_hw_stop();
    return (int)unlock_motion();
}

/*
 * Clear motion faults under the controller mutex.
 */
int motion_clear_faults(void)
{
    const ER er = lock_motion();
    if (er < E_OK) {
        return (int)er;
    }
    mc_clear_faults(&mc);
    drv_motor_hw_stop();
    return (int)unlock_motion();
}

/*
 * Set the servo goal and return the accepted angle under the mutex.
 */
int motion_set_servo(int deg, int *accepted_deg)
{
    const ER er = lock_motion();
    if (er < E_OK) {
        return (int)er;
    }
    const int accepted = mc_set_servo(&mc, deg, drv_time_ms());
    if (accepted_deg) {
        *accepted_deg = accepted;
    }
    return (int)unlock_motion();
}

/*
 * Copy the controller state while the motion mutex is held.
 */
static void motion_status_locked(motion_status_t *s)
{
    for (unsigned int i = 0u; i < MC_WHEELS; ++i) {
        s->counts[i] = sampled_counts[i];
        s->travel_mm[i] = mc_travel_mm(&mc, i);
        s->speed_mm_s[i] = mc_speed_mm_s(&mc, i);
        s->duty_permille[i] = mc.w[i].duty;
    }
    s->heading_cdeg = mc_heading_cdeg(&mc);
    s->servo_us = mc.servo_us;
    s->servo_deg = (int8_t)mc_servo_deg(&mc);
    s->servo_settled = mc_servo_settled(&mc, drv_time_ms());
    s->faults = mc_faults(&mc);
    s->ticks = ticks;
    s->tick_max_us = tick_max_us;
    s->gap_min_us = gap_min_us;
    s->gap_max_us = gap_max_us;
    s->late_wakes = late_wakes;
    s->missed_wakes = missed_wakes;
    s->watchdog_armed = watchdog_armed;
}

/*
 * Copy a coherent controller status under the inherited mutex.
 */
int motion_status(motion_status_t *s)
{
    if (!s) {
        return (int)E_PAR;
    }
    const ER er = lock_motion();
    if (er < E_OK) {
        return (int)er;
    }
    motion_status_locked(s);
    return (int)unlock_motion();
}

/*
 * Attempt one read of the public motion board without waiting.
 */
int motion_snapshot_try_read(motion_status_t *out, uint32_t *timestamp_us)
{
    if (!out || !timestamp_us) {
        return E_PAR;
    }
    if (mtx_motion <= 0 || tid_motion <= 0) {
        return E_OBJ;
    }
    motion_board_t v;
    if (!app_board_try_read(&board.sequence, board.words, &v, sizeof(v))) {
        return E_TMOUT;
    }
    *out = v.status;
    *timestamp_us = v.us;
    return E_OK;
}

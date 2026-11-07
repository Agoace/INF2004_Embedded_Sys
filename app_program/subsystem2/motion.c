/* Kernel side only. SDK register access is isolated in drv_motor_hw.c. */
#include "subsystem2.h"
#include "motion.h"
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
static ID mtx_motion, tid_motion, cyc_motion, tid_after_tick;
static int32_t sampled_counts[MC_WHEELS];
static uint32_t ticks, tick_max_us, last_run_us, gap_min_us, gap_max_us;
static uint32_t late_wakes, missed_wakes;
static bool watchdog_armed;
static void motion_status_locked(motion_status_t *s);
#if TM_MQTT
#include "../platform/app_board.h"
typedef struct { motion_status_t status;uint32_t us; } motion_board_t;
static APP_BOARD(motion_board_t) board;
static void publish_motion(uint32_t us) {
    motion_board_t v={0};v.us=us;motion_status_locked(&v.status);
    app_board_publish(&board.sequence,board.words,&v,sizeof(v));
}
#endif
#if APP_TEST
static volatile bool test_requested, test_suppressed;
static bool window_active;
static motion_timing_window_t window;
static uint32_t window_last_us, window_missed_base;
static motion_timing_probe_t probes[MOTION_PROBE_COUNT];
static UINT probe_present;
#endif

static void encoder_irq(unsigned int pin, uint32_t events, void *ctx) {
    (void)events; (void)ctx;
    const unsigned int wheel = pin == 16u || pin == 17u ? MC_LEFT : MC_RIGHT;
    const uint8_t ab = drv_motor_hw_ab(wheel);
    raw_count[wheel] += QUADRATURE[(previous_ab[wheel] << 2u) | ab];
    previous_ab[wheel] = ab;
}
static void copy_counts(int32_t counts[MC_WHEELS]) {
    UINT irq;
    DI(irq); /* motion_task and encoder IRQ both belong to physical core 0. */
    counts[MC_LEFT] = raw_count[MC_LEFT];
    counts[MC_RIGHT] = -raw_count[MC_RIGHT];
    EI(irq);
}
static ER lock_motion(void) {
    return mtx_motion > 0 ? tk_loc_mtx(mtx_motion, TMO_FEVR) : E_OBJ;
}
static void motion_task(INT stacd, void *exinf) {
    (void)stacd; (void)exinf;
    for (;;) {
        if (tk_slp_tsk(TMO_FEVR) < E_OK) continue;
        const INT queued = tk_can_wup(TSK_SELF);
        const uint32_t started = drv_time_us();
        if (lock_motion() < E_OK) continue;
        if (ticks != 0u) {
            const uint32_t gap = started - last_run_us;
            if (gap_min_us == 0u || gap < gap_min_us) gap_min_us = gap;
            if (gap > gap_max_us) gap_max_us = gap;
            if (gap > 11000u) ++late_wakes;
        }
        last_run_us = started;
        if (queued > 0) missed_wakes += (uint32_t)queued;
        copy_counts(sampled_counts);
        mc_output_t out;
        mc_tick(&mc, sampled_counts, drv_time_ms(), &out);
        /* Same mutex as stop/estop: no old computed output can overwrite stop. */
        drv_motor_hw_write(out.duty_permille, out.servo_us);
        ++ticks;
#if TM_MQTT
        publish_motion(started);
#endif
        const uint32_t took = drv_time_us() - started;
        if (took > tick_max_us) tick_max_us = took;
#if APP_TEST
        if (window_active) {
            if (window.ticks != 0u) {
                const uint32_t gap = started - window_last_us;
                if (window.gaps == 0u || gap < window.gap_min_us) window.gap_min_us = gap;
                if (gap > window.gap_max_us) window.gap_max_us = gap;
                if (gap > 11000u) ++window.late_wakes;
                ++window.gaps;
            }
            window_last_us = started;
            ++window.ticks;
            if (took > window.tick_max_us) window.tick_max_us = took;
        }
#endif
        const ID notify = tid_after_tick;
        (void)tk_unl_mtx(mtx_motion);
        /* Only completed control work feeds; never the cyclic handler or dump. */
        if (watchdog_armed) {
#if APP_TEST
            if (!test_suppressed) {
                drv_watchdog_feed();
                if (test_requested && drv_watchdog_test_begin() >= E_OK) test_suppressed = true;
            }
#else
            drv_watchdog_feed();
#endif
        }
        if (notify > 0) (void)tk_wup_tsk(notify);
    }
}
static void motion_cyclic(void *exinf) {
    (void)exinf;
    (void)tk_wup_tsk(tid_motion);
}
int motion_init(void) {
#if TK_SUPPORT_SMP
    if (tk_get_prc() != 1) return (int)E_CTX;
#endif
    if (mtx_motion > 0) return (int)E_OBJ;
    /* Establish zero motor PWM before checking the clock; never enable motion
     * with a bad TIMER rate, even if usermain's earlier initialization failed. */
    drv_motor_hw_init(config.servo_centre_us);
    int er = drv_time_init();
    if (er >= E_OK) er = drv_time_check();
    if (er < E_OK) {
        tm_printf((UB*)"ERROR motion: TIMER clock check failed (%d); motors disabled.\n", er);
        return er;
    }
    T_CMTX m = {0};
    m.mtxatr = TA_INHERIT;
    mtx_motion = tk_cre_mtx(&m);
    if (mtx_motion < E_OK) return (int)mtx_motion;
    mc_init(&mc, &config, sampled_counts, drv_time_ms());
    previous_ab[MC_LEFT] = drv_motor_hw_ab(MC_LEFT);
    previous_ab[MC_RIGHT] = drv_motor_hw_ab(MC_RIGHT);
    er = drv_gpio_irq_init();
    if (er >= E_OK) er = drv_gpio_irq_add(DRV_MOTOR_ENCODER_MASK, encoder_irq, 0);
    if (er < E_OK) { (void)tk_del_mtx(mtx_motion); mtx_motion = 0; return er; }
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
        (void)tk_del_mtx(mtx_motion); mtx_motion = 0;
        return (int)cyc_motion;
    }
    return (int)E_OK;
}
int motion_start(void) {
    if (mtx_motion <= 0 || cyc_motion <= 0 || tid_motion > 0) return (int)E_OBJ;
    tid_motion = app_start_task((FP)motion_task, 10, TP_PRC1, "motion_task");
    if (tid_motion < E_OK) return (int)tid_motion;
    const ER cyclic = tk_sta_cyc(cyc_motion);
    if (cyclic < E_OK) return (int)cyclic;
    const int er = drv_watchdog_start(100u);
    if (er < E_OK) {
        tm_printf((UB*)"ERROR motion: watchdog start failed (%d); speed commands disabled.\n", er);
        return er;
    }
    watchdog_armed = true;
    return (int)E_OK;
}
#if APP_TEST
int motion_timing_window_begin(void) {
    if (!watchdog_armed) return (int)E_OBJ;
    const ER er = lock_motion();
    if (er < E_OK) return (int)er;
    int result = E_OBJ;
    if (!window_active) {
        window = (motion_timing_window_t){0};
        window_missed_base = missed_wakes;
        window_active = true;
        result = E_OK;
    }
    (void)tk_unl_mtx(mtx_motion);
    return result;
}
int motion_timing_window_end(motion_timing_window_t *out) {
    if (!out) return (int)E_PAR;
    const ER er = lock_motion();
    if (er < E_OK) return (int)er;
    int result = E_OBJ;
    if (window_active) {
        window.missed_wakes = missed_wakes - window_missed_base;
        *out = window;
        window_active = false;
        result = E_OK;
    }
    (void)tk_unl_mtx(mtx_motion);
    return result;
}
int motion_timing_probe_capture(motion_probe_stage_t stage) {
    if ((unsigned int)stage >= MOTION_PROBE_COUNT) return (int)E_PAR;
    const ER er = lock_motion();
    if (er < E_OK) return (int)er;
    probes[stage] = (motion_timing_probe_t){drv_time_us(),ticks,gap_min_us,
        gap_max_us,tick_max_us,late_wakes,missed_wakes};
    probe_present |= 1u << (unsigned int)stage;
    return (int)tk_unl_mtx(mtx_motion);
}
int motion_timing_probe_read(motion_probe_stage_t stage,motion_timing_probe_t *out) {
    if (!out || (unsigned int)stage >= MOTION_PROBE_COUNT) return (int)E_PAR;
    if ((probe_present & (1u << (unsigned int)stage)) == 0u) return (int)E_OBJ;
    *out = probes[stage];
    return E_OK;
}
int motion_watchdog_test_suppress_feed(void) {
    if (!watchdog_armed) return (int)E_OBJ;
    test_requested = true;
    return (int)E_OK;
}
bool motion_watchdog_test_feed_stopped(void) { return test_suppressed; }
#endif
int motion_notify_after_tick(ID tid) {
    if (tid < 0) return (int)E_PAR;
    const ER er = lock_motion();
    if (er < E_OK) return (int)er;
    tid_after_tick = tid;
    return (int)tk_unl_mtx(mtx_motion);
}
int motion_set_speed(int16_t left, int16_t right) {
    if (!watchdog_armed) return (int)E_OBJ;
    const ER er = lock_motion();
    if (er < E_OK) return (int)er;
    mc_set_speed(&mc, left, right, drv_time_ms());
    return (int)tk_unl_mtx(mtx_motion);
}
int motion_stop(void) {
    const ER er = lock_motion();
    if (er < E_OK) return (int)er;
    mc_stop(&mc); drv_motor_hw_stop();
    return (int)tk_unl_mtx(mtx_motion);
}
int motion_estop(void) {
    const ER er = lock_motion();
    if (er < E_OK) return (int)er;
    mc_estop(&mc); drv_motor_hw_stop();
    return (int)tk_unl_mtx(mtx_motion);
}
int motion_clear_faults(void) {
    const ER er = lock_motion();
    if (er < E_OK) return (int)er;
    mc_clear_faults(&mc); drv_motor_hw_stop();
    return (int)tk_unl_mtx(mtx_motion);
}
int motion_set_servo(int deg, int *accepted_deg) {
    const ER er = lock_motion();
    if (er < E_OK) return (int)er;
    const int accepted = mc_set_servo(&mc, deg, drv_time_ms());
    if (accepted_deg) *accepted_deg = accepted;
    return (int)tk_unl_mtx(mtx_motion);
}
static void motion_status_locked(motion_status_t *s) {
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
    s->ticks = ticks; s->tick_max_us = tick_max_us;
    s->gap_min_us = gap_min_us; s->gap_max_us = gap_max_us;
    s->late_wakes = late_wakes; s->missed_wakes = missed_wakes;
    s->watchdog_armed = watchdog_armed;
}
int motion_status(motion_status_t *s) {
    if (!s) return (int)E_PAR;
    const ER er=lock_motion();
    if (er<E_OK) return (int)er;
    motion_status_locked(s);
    return (int)tk_unl_mtx(mtx_motion);
}
int motion_snapshot_try_read(motion_status_t *out,uint32_t *timestamp_us) {
    if (!out || !timestamp_us) return E_PAR;
#if TM_MQTT
    if (mtx_motion<=0 || tid_motion<=0) return E_OBJ;
    motion_board_t v;
    if (!app_board_try_read(&board.sequence,board.words,&v,sizeof(v))) return E_TMOUT;
    *out=v.status;*timestamp_us=v.us;return E_OK;
#else
    return E_OBJ;
#endif
}

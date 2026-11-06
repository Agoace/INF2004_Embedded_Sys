/* Kernel side only. SDK register access is isolated in drv_motor_hw.c. */
#include "subsystem2.h"
#include "motion.h"
#include "motor_ctl_robot.h"
#include "drv_motor_hw.h"
#include "../platform/drv_gpio_irq.h"
#include "../platform/drv_time.h"

static const mc_config_t config = MC_ROBOT_CONFIG;
static const int8_t QUADRATURE[16] = {
    0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0,
};
static volatile int32_t raw_count[MC_WHEELS];
static uint8_t previous_ab[MC_WHEELS];
static mc_t mc;
static ID mtx_motion, tid_motion, cyc_motion;
static int32_t sampled_counts[MC_WHEELS];
static uint32_t ticks, tick_max_us, last_run_us, gap_min_us, gap_max_us;
static uint32_t late_wakes, missed_wakes;

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
        const uint32_t took = drv_time_us() - started;
        if (took > tick_max_us) tick_max_us = took;
        (void)tk_unl_mtx(mtx_motion);
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
    T_CMTX m = {0};
    m.mtxatr = TA_INHERIT;
    mtx_motion = tk_cre_mtx(&m);
    if (mtx_motion < E_OK) return (int)mtx_motion;
    mc_init(&mc, &config, sampled_counts, drv_time_ms());
    drv_motor_hw_init(mc.servo_us);
    previous_ab[MC_LEFT] = drv_motor_hw_ab(MC_LEFT);
    previous_ab[MC_RIGHT] = drv_motor_hw_ab(MC_RIGHT);
    int er = drv_gpio_irq_init();
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
    return (int)tk_sta_cyc(cyc_motion);
}
int motion_set_speed(int16_t left, int16_t right) {
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
int motion_status(motion_status_t *s) {
    if (!s) return (int)E_PAR;
    const ER er = lock_motion();
    if (er < E_OK) return (int)er;
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
    return (int)tk_unl_mtx(mtx_motion);
}

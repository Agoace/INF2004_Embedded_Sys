/* Kernel glue only. Pure modules and SDK adapters stay in separate files. */
#include "../app.h"
#include "vehicle_task.h"
#include "../subsystem2/motion.h"
#include "../subsystem2/drv_motor_hw.h"
#include "../subsystem3/drv_line.h"
#include "../subsystem5/drv_ultrasonic.h"
#include "../platform/drv_time.h"
#include "../platform/drv_watchdog.h"
#include "vehicle_inputs.h"
#include "vehicle_robot_config.h"
#include "vehicle_log.h"
#if !APP_TEST && !APP_BENCH
#define START_DELAY_MS 1000u
#define RUN_LIMIT_MS 90000u
#define EVENT_CAPACITY 512u
static lp_t lp;
static lt_t lt;
static ls_t ls;
static ss_t ss;
static dr_t dr;
static vsm_t vsm;
static uint16_t raw[3];
static vc_event_t line_event;
static bool active, cancelled, prev_run, prev_dump;
static uint32_t wake_lost, run_start, rows_used, events_used, events_lost, run_button_ms;
static vr_row_t rows[VR_ROWS];
typedef struct { uint32_t ms; uint16_t word; uint8_t kind, from, to, detail; } event_t;
static event_t events[EVENT_CAPACITY];
static drv_line_stats_t line_base;
static const char *reason = "no saved run";
#if TM_CONSOLE_USB_CDC
IMPORT volatile UW tm_usb_dropped;
static UW dropped(void) { return tm_usb_dropped; }
#else
static UW dropped(void) { return 0u; }
#endif
static bool pressed(unsigned int pin, bool *previous) {
    const bool down = drv_motor_hw_button_down(pin);
    const bool edge = down && !*previous; *previous = down; return edge;
}
static void event(uint8_t kind, uint8_t from, uint8_t to, uint16_t word) {
    if (!active) return;
    if (events_used == EVENT_CAPACITY) { ++events_lost; cancelled = true; reason = "event buffer full"; return; }
    events[events_used++] = (event_t){drv_time_ms()-run_start, word, kind, from, to, 0u};
}
static bool wait_tick(void) {
    const ER er = tk_slp_tsk(20);
    const INT extra = tk_can_wup(TSK_SELF);
    if (extra > 0) wake_lost += (uint32_t)extra;
    if (er < E_OK || (active && extra > 0)) { cancelled = true; reason = "lost motion wake"; }
    if (active && drv_time_ms()-run_button_ms >= 20u) {
        run_button_ms=drv_time_ms();
        if (pressed(21u, &prev_run)) {
            (void)motion_estop(); cancelled = true; reason = "GP21 emergency stop";
        }
    }
    if (active) {
        drv_line_stats_t st; drv_line_stats(&st);
        if (st.missed != line_base.missed || st.dropped != line_base.dropped) {
            (void)motion_estop(); cancelled = true; reason = "line samples lost";
        }
    }
    return er >= E_OK && !cancelled;
}
static void service_line(void) {
    drv_line_sample_t sample;
    while (drv_line_take(&sample)) {
        for (unsigned int i=0u;i<3u;++i) raw[i]=sample.raw[i];
        const uint32_t sample_ms = drv_time_ms() - (drv_time_us()-sample.us)/1000u;
        const vc_event_t e = lp_update(&lp, raw, sample_ms);
        if (vc_event_type(e)==VC_EVENT_LINE_LOST || vc_event_type(e)==VC_EVENT_LINE_FOUND) line_event=e;
    }
}
#define CAL_NOTE(...) ((void)0)
typedef enum { GOAL_OUT, GOAL_HOME } goal_t;

static int32_t abs32(int32_t v) {
    return v < 0 ? -v : v;
}

/* As line_spin.c: each wheel stops by itself at its goal, from home. */
static bool wheel_done(goal_t goal, int32_t from_home, int32_t at_start) {
    if (goal == GOAL_OUT) {
        const bool started_out = abs32(at_start) >= CAL_STEP_COUNTS / 2;
        const bool other_side = (from_home < 0) != (at_start < 0);
        return (!started_out || other_side) && abs32(from_home) >= CAL_STEP_COUNTS;
    }
    return from_home == 0 || (from_home < 0) != (at_start < 0);
}

/* direction +1: left wheel forward, right reverse (robot turns right). */
static bool cal_spin(int direction, goal_t goal, const int32_t home[2], bool stop_on_line) {
    motion_status_t s;
    (void)motion_status(&s);
    const int32_t start[2] = {s.counts[0] - home[0], s.counts[1] - home[1]};
    const int sign[2] = {direction, -direction};
    bool running[2] = {true, true};
    bool on_line = false;
    uint16_t peak = 0u;
    const uint32_t t0 = drv_time_ms();
    motion_clear_faults();
    drv_line_flush();
    while (running[0] || running[1]) {
        if (!wait_tick()) { (void)motion_stop(); return false; }
        const uint32_t now = drv_time_ms();

        drv_line_sample_t sample;
        while (drv_line_take(&sample)) {
            for (unsigned int i=0u;i<3u;++i) raw[i]=sample.raw[i];
            if (stop_on_line) {
                (void)lp_update(&lp, raw, drv_time_ms() - (drv_time_us()-sample.us)/1000u);
                const uint16_t c = lp.norm[LP_CENTRE];
                if (c > peak) peak = c;
                on_line = c >= CAL_CENTRE_STOP ||
                          (peak >= CAL_CENTRE_MIN && c + CAL_PEAK_DROP <= peak);
            } else {
                lp_calib_add(&lp, raw);
            }
        }
        (void)motion_status(&s);
        for (unsigned int w = 0u; w < 2u; ++w) {
            if (running[w] && (on_line || wheel_done(goal, s.counts[w] - home[w], start[w]))) {
                running[w] = false;
            }
        }
        if ((s.faults & MC_FAULT_LATCHED) != 0u || now - t0 >= CAL_STEP_TIMEOUT_MS) {
            motion_stop();
            CAL_NOTE("[spin] STOPPED: %s; from start L %ld  R %ld\n",
                   (s.faults & MC_FAULT_LATCHED) ? "motor fault" : "time limit",
                   (long)(s.counts[0] - home[0]), (long)(s.counts[1] - home[1]));
            return false;
        }
        motion_set_speed((int16_t)(running[0] ? sign[0] * CAL_SPEED_MM_S : 0),
                      (int16_t)(running[1] ? sign[1] * CAL_SPEED_MM_S : 0));
    }
    motion_stop();
    (void)motion_status(&s);
    CAL_NOTE("[spin] %+d done in %lu ms%s; from start L %ld  R %ld\n", direction,
           (unsigned long)(drv_time_ms() - t0), on_line ? ", stopped on the line" : "",
           (long)(s.counts[0] - home[0]), (long)(s.counts[1] - home[1]));
    return true;
}

/* Robot still: sample for the calibration. */
static void cal_pause(void) {
    const uint32_t end = drv_time_ms() + CAL_PAUSE_MS;
    while ((int32_t)(drv_time_ms() - end) < 0) {
        if (!wait_tick()) return;
        drv_line_sample_t sample;
        while (drv_line_take(&sample)) {
            for (unsigned int i=0u;i<3u;++i) raw[i]=sample.raw[i];
            lp_calib_add(&lp, raw);
        }
    }
}

static bool calibrate(void) {
    CAL_NOTE("\n[spin] Spin calibration at %d mm/s, %d counts (a quarter wheel turn).\n",
           CAL_SPEED_MM_S, CAL_STEP_COUNTS);
    motion_status_t s;
    (void)motion_status(&s);
    const int32_t home[2] = {s.counts[0], s.counts[1]};
    lp_calib_begin(&lp);
    bool ok = cal_spin(-1, GOAL_OUT, home, false);
    if (ok) {
        cal_pause();
        ok = cal_spin(+1, GOAL_OUT, home, false);
    }
    if (ok) {
        cal_pause();
    }
    if (!ok) {
        CAL_NOTE("[spin] Spin not complete. The calibration is not changed.\n");
        return false;
    }
    CAL_NOTE("[calib] seen range L %u-%u  C %u-%u\n", lp.cal_min[0], lp.cal_max[0], lp.cal_min[1],
           lp.cal_max[1]);
    unsigned int bad = 0u;
    const bool calibrated = lp_calib_end(&lp, &bad) == LP_CALIB_OK;
    if (calibrated) {
        CAL_NOTE("[calib] OK. Line reads %s. L floor %u line %u; C floor %u line %u\n",
               lp_line_reads_high(&lp) ? "HIGHER" : "LOWER", lp_floor(&lp, 0), lp_line(&lp, 0),
               lp_floor(&lp, 1), lp_line(&lp, 1));
    } else {
        CAL_NOTE("[calib] FAILED (bad sensors 0x%x).\n", bad);
    }
    ok = cal_spin(-1, GOAL_HOME, home, calibrated);
    return ok && calibrated && lp_calibrated(&lp);
}


/* Wrap module callbacks only to save every outcome, including multi-hop steps. */
typedef struct { vsm_module_t module; uint8_t id; } watched_t;
static watched_t watched[4];
static void watched_start(void *ctx, uint16_t payload, uint32_t now) {
    watched_t *w = ctx; w->module.start(w->module.ctx,payload,now);
}
static vsm_outcome_t watched_step(void *ctx,const vsm_inputs_t *in,uint32_t now,vsm_drive_t *drive) {
    watched_t *w=ctx;
    const vsm_outcome_t out=w->module.step(w->module.ctx,in,now,drive);
    if (vsm_outcome_type(out)!=VSM_OUT_RUNNING) event(1u,w->id,0u,out);
    return out;
}
static void install(unsigned int index,uint8_t id,vsm_module_t module) {
    watched[index]=(watched_t){module,id};
    (void)vsm_set_module(&vsm,(vsm_module_id_t)id,(vsm_module_t){watched_start,watched_step,&watched[index]});
}
static void save_row(uint32_t now,const motion_status_t *s,const vsm_drive_t *d,
                     ultrasonic_result_t echo,uint8_t bits) {
    if (rows_used==VR_ROWS) { cancelled=true; reason="row buffer full"; return; }
    const vr_values_t v={.elapsed_ms=now-run_start,.raw={raw[0],raw[1]},.echo=echo,
        .asked={d->left_mm_s,d->right_mm_s},.measured={s->speed_mm_s[0],s->speed_mm_s[1]},
        .position=lp.position,.servo=d->servo_deg,.state=vsm.state,.module=vsm.running,.events=bits};
    vr_pack(&rows[rows_used++],&v);
}
static void dump(void) {
    const UW before=dropped();
    motion_status_t s;
    (void)motion_status(&s);
    tm_printf((UB*)"[robot] KR1 reset %s; %s rows %lu events %lu lost_events %lu lost_wakes %lu faults %u gap %lu..%lu us\n",
        drv_watchdog_reset_name(),reason,(unsigned long)rows_used,(unsigned long)events_used,(unsigned long)events_lost,
        (unsigned long)wake_lost,(unsigned int)s.faults,(unsigned long)s.gap_min_us,(unsigned long)s.gap_max_us);
    drv_line_stats_t stats; drv_line_stats(&stats);
    tm_printf((UB*)"[robot] counts L/R %ld/%ld; line L floor/line %u/%u C %u/%u; sampled %lu missed %lu dropped %lu (lifetime)\n",
        (long)s.counts[0],(long)s.counts[1],(unsigned int)lp_floor(&lp,0u),(unsigned int)lp_line(&lp,0u),
        (unsigned int)lp_floor(&lp,1u),(unsigned int)lp_line(&lp,1u),(unsigned long)stats.samples,
        (unsigned long)stats.missed,(unsigned long)stats.dropped);
    for (uint32_t i=0u;i<rows_used;++i) {
        tm_printf((UB*)"KR %lu ",(unsigned long)i);
        for (unsigned int b=0u;b<VR_BYTES;++b) tm_printf((UB*)"%02x",(unsigned int)rows[i].bytes[b]);
        tm_printf((UB*)"\n");
        if ((i+1u)%8u==0u) (void)tk_dly_tsk(20u);
    }
    for (uint32_t i=0u;i<events_used;++i) {
        const event_t *e=&events[i];
        tm_printf((UB*)"VE %lu %u %u %u %u\n",(unsigned long)e->ms,(unsigned int)e->kind,
            (unsigned int)e->from,(unsigned int)e->to,(unsigned int)e->word);
        if ((i+1u)%8u==0u) (void)tk_dly_tsk(20u);
    }
    (void)tk_dly_tsk(20u);
    const UW after=dropped();
    tm_printf((UB*)"[robot] end tm_usb_dropped %lu (dump delta %lu)\n",
        (unsigned long)after,(unsigned long)(after-before));
    drv_line_flush(); (void)tk_can_wup(TSK_SELF);
}
static void run_once(void) {
    (void)motion_clear_faults(); vsm_reset(&vsm);
    line_event=VC_EVENT_NONE;
    run_start=drv_time_ms(); rows_used=0u;
    event(2u,VSM_MOD_NONE,VSM_MOD_LINE,VSM_RUNNING);
    (void)vsm_start(&vsm,VSM_MOD_LINE,0u,run_start);
    bool echo_new=false,ping_sent=false;
    ultrasonic_result_t echo=ULTRASONIC_RESULT_NONE;
    uint16_t ping_ms=VSM_PING_DEFAULT_MS;
    uint32_t last_ping=run_start, last_log=run_start-20u;
    int8_t last_servo=0;
    uint8_t pending_bits=0u;
    while (!cancelled) {
        if (!wait_tick()) break;
        const uint32_t now=drv_time_ms();
        service_line();
        if (drv_ultrasonic_service()) { echo=drv_ultrasonic_latest(); echo_new=true; }
        /* Preserve reference ordering: never ping over an unconsumed result. */
        if (ping_ms!=0u && !echo_new && now-last_ping>=ping_ms && !drv_ultrasonic_busy() && drv_ultrasonic_trigger()) {
            last_ping=now; ping_sent=true;
            if (!drv_ultrasonic_busy()) { echo=drv_ultrasonic_latest(); echo_new=true; }
        }
        motion_status_t s;
        if (motion_status(&s)<E_OK) { reason="motion status failed"; break; }
        if (vc_event_type(line_event)==VC_EVENT_LINE_LOST) pending_bits|=1u;
        if (vc_event_type(line_event)==VC_EVENT_LINE_FOUND) pending_bits|=2u;
        if (echo_new) pending_bits|=4u;
        if (ping_sent) pending_bits|=8u;
        const uint8_t state_before=vsm.state, module_before=vsm.running;
        const uint32_t handoffs=vsm.handoffs;
        ping_ms=VSM_PING_DEFAULT_MS;
        const vsm_inputs_t in={&lp,line_event,{s.counts[0],s.counts[1]},echo_new,echo,&ping_ms,ping_sent};
        line_event=VC_EVENT_NONE; echo_new=false; ping_sent=false;
        const vsm_drive_t d=vsm_step(&vsm,&in,(uint8_t)(s.faults & MC_FAULT_LATCHED),now);
        if (vsm.state!=state_before || vsm.running!=module_before)
            event(2u,module_before,vsm.running,(uint16_t)((uint16_t)vsm.state | ((uint16_t)vsm.fault<<8u)));
        if (s.faults!=0u) { pending_bits|=128u; event(3u,vsm.state,vsm.running,s.faults); }
        if (vsm.handoffs!=handoffs) pending_bits|=(uint8_t)((uint8_t)vsm_outcome_type(vsm.last.outcome)<<4u);
        if (d.action==VSM_ACT_STOP) (void)motion_stop();
        else {
            (void)motion_set_speed(d.left_mm_s,d.right_mm_s);
            if (d.servo_deg!=last_servo) { (void)motion_set_servo(d.servo_deg,0); last_servo=d.servo_deg; }
        }
        if (now-last_log>=20u) { save_row(now,&s,&d,echo,pending_bits); pending_bits=0u; last_log=now; }
        if (vsm.state!=VSM_RUNNING) { reason="shell left RUNNING"; break; }
        if (now-run_start>=RUN_LIMIT_MS) { reason="90 s time limit"; break; }
    }
    event(4u,vsm.state,vsm.running,vsm.fault_detail);
    (void)motion_stop(); (void)motion_set_servo(0,0); vsm_stop(&vsm);
}
static void vehicle_task(INT stacd,void *exinf) {
    (void)stacd; (void)exinf;
    lp_init(&lp); lp_set_pair(&lp,true,0);
    lt_init(&lt,&LINE_CFG); ls_init(&ls,&SEARCH_CFG); ss_init(&ss,&SCAN_CFG); dr_init(&dr,&DETOUR_CFG);
    vsm_init(&vsm,VSM_DEFAULT_ROUTES,VSM_DEFAULT_ROUTE_COUNT);
    install(0u,VSM_MOD_LINE,lt_module(&lt)); install(1u,VSM_MOD_SEARCH,ls_module(&ls));
    install(2u,VSM_MOD_SCAN,ss_module(&ss)); install(3u,VSM_MOD_DETOUR,dr_module(&dr));
    drv_motor_hw_buttons_init();
    prev_run=drv_motor_hw_button_down(21u); prev_dump=drv_motor_hw_button_down(20u);
    tm_printf((UB*)"[robot] GP21 calibrate then run / ESTOP; GP20 dump. D85. Cold-power S0-U01 required.\n");
    if (motion_notify_after_tick(tk_get_tid())<E_OK) {
        tm_printf((UB*)"ERROR robot: cannot register motion wake\n"); tk_ext_tsk();
    }
    uint32_t button_ms=drv_time_ms();
    for (;;) {
        cancelled=false;
        (void)wait_tick(); service_line(); (void)drv_ultrasonic_service();
        const uint32_t now=drv_time_ms();
        if (now-button_ms<20u) continue;
        button_ms=now;
        const bool run_edge=pressed(21u,&prev_run), dump_edge=pressed(20u,&prev_dump);
        if (dump_edge && !run_edge) dump();
        if (!run_edge) continue;
        cancelled=false; wake_lost=0u; events_used=0u; events_lost=0u; rows_used=0u;
        reason="calibration failed"; drv_line_flush(); drv_line_stats(&line_base);
        active=true; run_start=now; run_button_ms=now;
        if (calibrate() && !cancelled) {
            const uint32_t start=drv_time_ms();
            while (!cancelled && drv_time_ms()-start<START_DELAY_MS) { (void)wait_tick(); service_line(); }
            if (!cancelled) run_once();
        }
        (void)motion_stop(); (void)motion_set_servo(0,0);
        motion_status_t end;
        if (motion_status(&end)>=E_OK && end.faults!=0u) event(3u,vsm.state,vsm.running,end.faults);
        event(5u,vsm.state,vsm.running,cancelled ? 1u : 0u);
        active=false; cancelled=false; vsm_reset(&vsm);
        /* GP20 only dumps; never print automatically during calibration/run. */
    }
}
int vehicle_task_start(void) {
    motion_status_t status;
    int er=motion_status(&status);
    if (er>=E_OK && !status.watchdog_armed) er=(int)E_OBJ;
    if (er>=E_OK) er=drv_line_init();
    if (er>=E_OK) er=drv_ultrasonic_init();
    if (er<E_OK) return er;
    const ID tid=app_start_task((FP)vehicle_task,12,TP_PRC1,"vehicle_task");
    return tid<E_OK ? (int)tid : (int)E_OK;
}
_Static_assert(sizeof(event_t)==12u,"event record size");
#else
int vehicle_task_start(void) { return (int)E_OK; }
#endif

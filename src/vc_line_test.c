/*
 * Track test: line tracing through the Vehicle Controller shell and the
 * motor controller (docs/s3-line-barcode/LINE.md).
 *
 *   GP21  spin calibration, then the run after 1 s; press again to stop
 *   GP20  (idle) next steering D from D_STEPS; the value is printed
 *
 * Layers (docs/architecture/ADR_VEHICLE_CONTROLLER.md):
 *   detectors  line position (src/line_position.c) every 1 ms, ultrasonic
 *              reading at the period the running module asks (default
 *              100 ms), wheel counts (motor controller)
 *   modules    LINE (src/line_trace.c), SEARCH (src/line_search.c),
 *              SCAN (src/servo_scan.c) and DETOUR (src/detour_run.c,
 *              detour v1.8 in wheel speeds; docs/s5-obstacle/OBSTACLE.md)
 *   shell      src/vehicle_sm.c, every 10 ms (heartbeat flag)
 *   motors     src/motor_ctl_pico.c: the only owner of the motor PWM,
 *              the servo PWM and the encoders; wheel speeds in mm/s
 *
 * Timing: the heartbeat (src/heartbeat_pico.c) runs mc_tick() every 10 ms
 * in its interrupt (phase 0) and restarts the watchdog (100 ms). The shell
 * steps at phase 9, 1 ms before the next mc_tick. The line sample and
 * lp_update() run in the main loop on a 1 ms flag, as in detour_test (not
 * yet in the interrupt, docs/architecture/ADR_HEARTBEAT.md).
 *
 * A run stops on: GP21, the time limit RUN_LIMIT_MS, or the shell leaving
 * RUNNING (failed search, scan or detour, ultrasonic fault, wheel stall).
 *
 * Text log: each handoff, the scan profile, and the detour steps (pose,
 * sweep ends, back off), as detour_test.
 *
 * Power: a driving test. Use 5 V VIN with USB disconnected
 * (docs/archive/s2-motion/USB_POWER_SAFETY.md).
 *
 * Log: the steering log (docs/archive/STEER_WIFI_LOG.md) every 50 ms, live and
 * replayed after the run. Log header flag SL_FLAG_SPEED: the two wheel
 * fields are the asked wheel speeds in 4 mm/s units. Mode 3 (BYPS) is
 * SCAN or DETOUR; its position field is the scan point + 10 (SCAN) or the
 * detour step dt_step_t (DETOUR).
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "../app_program/subsystem5/detour_run.h"
#include "hardware/gpio.h"
#include "heartbeat_pico.h"
#include "../app_program/subsystem3/line_position.h"
#include "../app_program/subsystem3/line_search.h"
#include "line_sensors.h"
#include "../app_program/subsystem3/line_trace.h"
#include "motor_ctl_pico.h"
#include "../app_program/subsystem2/motor_ctl_robot.h"
#include "pico/stdlib.h"
#include "../app_program/subsystem5/servo_scan.h"
#include "steer_log.h"
#include "ultrasonic.h"
#include "../app_program/vehicle/vehicle_inputs.h"
#include "../app_program/vehicle/vehicle_sm.h"
#include "wifi_log.h"

#define ULTRASONIC_ECHO_PIN 2
#define ULTRASONIC_TRIGGER_PIN 3
#define SPIN_BUTTON_PIN 20
#define RUN_BUTTON_PIN 21

#define WATCHDOG_MS 100u
#define START_DELAY_MS 1000u
#define RUN_LIMIT_MS 90000u
#define LOG_STEPS 5u /* one log entry per 5 shell steps = 50 ms */
#define LOG_PERIOD_MS 50u
#define LOG_KEEP_MS 20000u
#define LOG_ENTRIES (LOG_KEEP_MS / LOG_PERIOD_MS)
#define LIVE_BATCH 5u
#define SPEED_LOG_UNIT 4 /* mm/s per log unit (int8: up to 508 mm/s) */
/* Speed loop I gain for this target (MC_ROBOT_CONFIG: 128). P9 tested 0:
 * no change on the straights, 6 mm/s slower, so 128 again. */
#ifndef VC_LINE_KI_Q8
#define VC_LINE_KI_Q8 128u
#endif

/*
 * Locked line tracing (docs/s3-line-barcode/LINE.md) mapped
 * to mm/s
 * through the duty map (docs/s3-line-barcode/LINE.md): base 25 % ->
 * 171,
 * min 12 % -> 62, max 50 % -> 378. Straight 450 counts across a junction;
 * side hint above 20 counts of turn.
 * P7 (docs/s3-line-barcode/LINE.md, after P6): matched to what P4 really
 * did, not
 * to the straight-run duty map. P4 drove 146 mm/s; P6 turned 17 % harder
 * than P4 for the same position. Base 146; gain, D and far gain x 0.85. */
/*
 * D test after P7 (docs/s3-line-barcode/LINE.md): GP20 picks the next D.
 * All other
 * settings stay. The first value is the P7 value. */
static const uint8_t D_STEPS[] = {85u, 130u, 50u};
#define D_STEP_COUNT (sizeof D_STEPS / sizeof D_STEPS[0])
static unsigned int d_step;

static const lt_config_t LINE_CFG = {
    .steer = {.base_mm_s = 146, .gain_pct = 85u, .kd_pct = 85u, .min_mm_s = 62,
              .max_mm_s = 378, .far_from = 50u, .far_gain_pct = 340u},
    .straight_counts = 450,
    .hint_counts = 20,
};
/* Search: 20 % -> 131 mm/s; an eighth wheel turn each way (2535 / 8). */
static const ls_config_t SEARCH_CFG = {.speed_mm_s = 131, .turn_counts = 2535 / 8};

/* Scan: the robot stands still 300 ms first (detour_test STOP_SETTLE_MS). */
static const ss_config_t SCAN_CFG = {.stop_ms = 300u};

/* Detour v1.8 (src/detour_test.c, docs/archive/OBSTACLE_DETOUR_HANDOFF.md) with
 * wheel speeds: straights and the non-sweep turns 131 mm/s (20 %); the
 * sweeps 40 deg/s = 39 mm/s per wheel, held by the motor controller (no
 * duty rate control). No heading hold on the straights (operator
 * 2026-11-04). Back off: a fixed back_off_deg 25 degrees (operator
 * 2026-11-05, after run R1; before: half of the turn, at most 30). */
static const dr_config_t DETOUR_CFG = {
    .dt = {.back_off_fn = NULL, /* fixed back_off_deg */
           .return_heading_deg = 90u,
           .return_max_mm = 1000u,
           .sweep_deg_s = 40u, /* pivot bench 2026-11-13: 19 mm/s sticks */
           .sweep_ping_ms = 60u,
           .sweep_seen_count = 2u,
           .sweep_min_duty = 0u, /* duty rate control: not used */
           .sweep_max_duty = 0u,
           .line_return_only = true,
           .turn_step_deg = 5u,
           .away_max_deg = 90u,
           .toward_max_deg = 180u, /* away_max_deg + return_heading_deg */
           .max_straights = 5u,
           .back_off_deg = 25u, /* operator 2026-11-05 */
           .look_ms = 150u,
           .vehicle_mm = 400u, /* operator 2026-11-05: 350 touched the box (R2), 450 went too far (R3) */
           .sweep_mm = 150u,
           .margin_mm = 30u,
           .sensor_ahead_mm = 65u,
           .line_ahead_mm = 123u,
           .seen_mm = 400u,
           .counts_per_deg_x100 = 2405u,
           .total_max_ms = 40000u,
           /* First straight: wall triangle d / cos(turn) + 50, at most
            * 700 mm (operator 2026-11-13), not vehicle_mm. */
           .first_extra_mm = 50u,
           .first_max_mm = 700u},
    .straight_mm_s = 131,
    .turn_mm_s = 131,
    .pause_ms = 250u,
};

/* Spin calibration (as src/line_spin.c, with wheel speeds). */
#define CAL_SPEED_MM_S 131 /* line_spin.c: 20 % */
#define CAL_STEP_COUNTS (2535 / 4)
#define CAL_STEP_TIMEOUT_MS 6000u
#define CAL_PAUSE_MS 200u
#define CAL_CENTRE_STOP 900u
#define CAL_CENTRE_MIN 500u
#define CAL_PEAK_DROP 50u

typedef enum {
    STOP_BUTTON,
    STOP_TIME_LIMIT,
    STOP_SHELL,
} stop_reason_t;
static const char *const STOP_NAMES[] = {"GP21 pressed", "time limit", "shell left RUNNING"};
static const char *const FAULT_NAMES[] = {"none",      "SAFETY (motor fault)", "FAILED",
                                          "NO_ROUTE",  "NO_MODULE",            "HANDOFF_LOOP"};
static const char *const MODULE_NAMES[] = {"NONE", "LINE", "SEARCH", "SCAN", "DETOUR"};
static const char *const OUT_NAMES[] = {"RUNNING", "OBSTACLE",  "SIDE",  "DONE_LINE",
                                        "DONE_NO_LINE", "LINE_LOST", "FOUND", "FAILED"};
static const char *const STEP_NAMES[] = {"BACK",    "TURN_AWAY", "STRAIGHT", "TURN_TOWARD",
                                         "ON_LINE", "ALIGN",     "DONE",     "FAILED"};
static const char *const DT_FAIL_NAMES[] = {"none",       "no side",   "turn limit", "time",
                                            "sensor",     "too near",  "no line",    "?"};
static const char *const SIDE_NAMES[] = {"NONE", "RIGHT", "LEFT", "?"};

static hb_t hb;
static mc_config_t mc_cfg = MC_ROBOT_CONFIG;
static int f_1ms, f_10ms, f_20ms;
static lp_t lp;
static lt_t lt;
static ls_t ls;
static ss_t ss;
static dr_t dr;
static vsm_t vsm;
static uint16_t raw[LP_SENSORS];
static vc_event_t line_event; /* last LOST or FOUND since the previous step */
static uint8_t run_number;

static struct {
    bool valid;
    stop_reason_t reason;
    uint32_t t0;
    uint32_t duration_ms;
    uint32_t missed; /* heartbeat missed frames during the run */
    int32_t counts[2];
    uint32_t handoffs;
    uint16_t searches;
    uint16_t scans;
    uint16_t detours;
    uint16_t detours_done;
    uint16_t pings;
    uint32_t log_total;
    sl_entry_t log[LOG_ENTRIES];
} run;

static bool pressed(unsigned int pin, bool *previous) {
    const bool down = !gpio_get(pin);
    const bool edge = down && !*previous;
    *previous = down;
    return edge;
}

static void idle_ms(uint32_t ms) {
    const uint32_t end = mcp_now_ms() + ms;
    while ((int32_t)(mcp_now_ms() - end) < 0) {
        wifi_log_poll(mcp_now_ms());
        sleep_ms(2);
    }
}

/* Line detector: one sample for each 1 ms flag. */
static void sample_line(uint32_t now) {
    line_sensors_read(raw);
    const vc_event_t e = lp_update(&lp, raw, now);
    const vc_event_type_t t = vc_event_type(e);
    if (t == VC_EVENT_LINE_LOST || t == VC_EVENT_LINE_FOUND) {
        line_event = e;
    }
}

static void service_line(void) {
    const uint32_t now = mcp_now_ms();
    if (hb_take(&hb, f_1ms) != 0u) {
        sample_line(now);
    }
}

/* ---------------------------------------------------------------------
 * Spin calibration with the motor controller.
 * ------------------------------------------------------------------- */
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
    mcp_status_t s;
    mcp_status(&s);
    const int32_t start[2] = {s.counts[0] - home[0], s.counts[1] - home[1]};
    const int sign[2] = {direction, -direction};
    bool running[2] = {true, true};
    bool on_line = false;
    uint16_t peak = 0u;
    const uint32_t t0 = mcp_now_ms();
    mcp_clear_faults();
    (void)hb_take(&hb, f_1ms);
    while (running[0] || running[1]) {
        const uint32_t now = mcp_now_ms();
        wifi_log_poll(now);
        if (hb_take(&hb, f_1ms) != 0u) {
            line_sensors_read(raw);
            if (stop_on_line) {
                (void)lp_update(&lp, raw, now);
                const uint16_t c = lp.norm[LP_CENTRE];
                if (c > peak) peak = c;
                on_line = c >= CAL_CENTRE_STOP ||
                          (peak >= CAL_CENTRE_MIN && c + CAL_PEAK_DROP <= peak);
            } else {
                lp_calib_add(&lp, raw);
            }
        }
        if (hb_take(&hb, f_10ms) == 0u) {
            continue;
        }
        mcp_status(&s);
        for (unsigned int w = 0u; w < 2u; ++w) {
            if (running[w] && (on_line || wheel_done(goal, s.counts[w] - home[w], start[w]))) {
                running[w] = false;
            }
        }
        if ((s.faults & MC_FAULT_LATCHED) != 0u || now - t0 >= CAL_STEP_TIMEOUT_MS) {
            mcp_stop();
            printf("[spin] STOPPED: %s; from start L %ld  R %ld\n",
                   (s.faults & MC_FAULT_LATCHED) ? "motor fault" : "time limit",
                   (long)(s.counts[0] - home[0]), (long)(s.counts[1] - home[1]));
            return false;
        }
        mcp_set_speed((int16_t)(running[0] ? sign[0] * CAL_SPEED_MM_S : 0),
                      (int16_t)(running[1] ? sign[1] * CAL_SPEED_MM_S : 0));
    }
    mcp_stop();
    mcp_status(&s);
    printf("[spin] %+d done in %lu ms%s; from start L %ld  R %ld\n", direction,
           (unsigned long)(mcp_now_ms() - t0), on_line ? ", stopped on the line" : "",
           (long)(s.counts[0] - home[0]), (long)(s.counts[1] - home[1]));
    return true;
}

/* Robot still: sample for the calibration. */
static void cal_pause(void) {
    const uint32_t end = mcp_now_ms() + CAL_PAUSE_MS;
    while ((int32_t)(mcp_now_ms() - end) < 0) {
        wifi_log_poll(mcp_now_ms());
        if (hb_take(&hb, f_1ms) != 0u) {
            line_sensors_read(raw);
            lp_calib_add(&lp, raw);
        }
    }
}

static bool calibrate(void) {
    printf("\n[spin] Spin calibration at %d mm/s, %d counts (a quarter wheel turn).\n",
           CAL_SPEED_MM_S, CAL_STEP_COUNTS);
    mcp_status_t s;
    mcp_status(&s);
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
        printf("[spin] Spin not complete. The calibration is not changed.\n");
        return false;
    }
    printf("[calib] seen range L %u-%u  C %u-%u\n", lp.cal_min[0], lp.cal_max[0], lp.cal_min[1],
           lp.cal_max[1]);
    unsigned int bad = 0u;
    const bool calibrated = lp_calib_end(&lp, &bad) == LP_CALIB_OK;
    if (calibrated) {
        printf("[calib] OK. Line reads %s. L floor %u line %u; C floor %u line %u\n",
               lp_line_reads_high(&lp) ? "HIGHER" : "LOWER", lp_floor(&lp, 0), lp_line(&lp, 0),
               lp_floor(&lp, 1), lp_line(&lp, 1));
    } else {
        printf("[calib] FAILED (bad sensors 0x%x).\n", bad);
    }
    ok = cal_spin(-1, GOAL_HOME, home, calibrated);
    return ok && calibrated && lp_calibrated(&lp);
}

/* ---------------------------------------------------------------------
 * Log.
 * ------------------------------------------------------------------- */
static void send_entries(uint32_t first, uint32_t count, uint8_t flags) {
    uint8_t packet[SL_LOG_HEADER_BYTES + SL_MAX_ENTRIES * SL_ENTRY_BYTES];
    if (count > SL_MAX_ENTRIES) count = SL_MAX_ENTRIES;
    size_t n = sl_write_log_header(packet, run_number, (uint8_t)(flags | SL_FLAG_SPEED),
                                   (uint16_t)first, (uint8_t)count);
    for (uint32_t i = 0u; i < count; ++i) {
        sl_pack_entry(&run.log[(first + i) % LOG_ENTRIES], &packet[n]);
        n += SL_ENTRY_BYTES;
    }
    (void)wifi_log_send(SL_TYPE_LOG, packet, n);
}

static struct {
    bool active;
    uint32_t next;
} replay;

static void replay_step(void) {
    if (!replay.active || !wifi_log_receiver_known()) {
        return;
    }
    const uint32_t left = run.log_total - replay.next;
    const uint32_t count = left < SL_MAX_ENTRIES ? left : SL_MAX_ENTRIES;
    const bool last = count == left;
    send_entries(replay.next, count, (uint8_t)(SL_FLAG_REPLAY | (last ? SL_FLAG_REPLAY_END : 0u)));
    replay.next += count;
    replay.active = !last;
}

static int8_t log_speed(int16_t mm_s) {
    const int v = mm_s >= 0 ? (mm_s + SPEED_LOG_UNIT / 2) / SPEED_LOG_UNIT
                            : -((-mm_s + SPEED_LOG_UNIT / 2) / SPEED_LOG_UNIT);
    return (int8_t)(v > 127 ? 127 : v < -127 ? -127 : v);
}

static void print_summary(void) {
    printf("[run] %lu ms, stopped by %s\n", (unsigned long)run.duration_ms, STOP_NAMES[run.reason]);
    if (vsm.state == VSM_FAULT) {
        printf("[run] shell FAULT %s, detail %u\n", FAULT_NAMES[vsm.fault % 6u], vsm.fault_detail);
    }
    printf("[run] last handoff at %lu ms: %s -> %s on %s (payload %u); handoffs %lu\n",
           (unsigned long)(vsm.last.t_ms - run.t0),
           MODULE_NAMES[vsm.last.from % 5u], MODULE_NAMES[vsm.last.to % 5u],
           OUT_NAMES[vsm_outcome_type(vsm.last.outcome) % 8u], vsm_outcome_payload(vsm.last.outcome),
           (unsigned long)run.handoffs);
    printf("[run] searches %u, scans %u, detours %u (%u done), pings %u, wheels L %ld  R %ld counts\n",
           run.searches, run.scans, run.detours, run.detours_done, run.pings, (long)run.counts[0],
           (long)run.counts[1]);
    mcp_status_t s;
    mcp_status(&s);
    printf("[run] heartbeat isr max %lu us, overruns gap %lu isr %lu, missed in run %lu; mc hook max "
           "%lu us; faults 0x%02x; Wi-Fi dropped %lu\n",
           (unsigned long)hb.isr_max_us, (unsigned long)hb.gap_overruns,
           (unsigned long)hb.isr_overruns, (unsigned long)run.missed,
           (unsigned long)s.tick_max_us, s.faults, (unsigned long)wifi_log_dropped());
    printf("[run] wheel loop: kp_q8 %u, ki_q8 %u, rise %u\n", mc_cfg.kp_q8, mc_cfg.ki_q8,
           mc_cfg.accel_cpt_q8);
    printf("[run] line: base %d min %d max %d mm/s, gain %u D %u far %u above %u; search %d mm/s %ld "
           "counts\n",
           lt.cfg.steer.base_mm_s, lt.cfg.steer.min_mm_s, lt.cfg.steer.max_mm_s,
           lt.cfg.steer.gain_pct, lt.cfg.steer.kd_pct, lt.cfg.steer.far_gain_pct,
           lt.cfg.steer.far_from, SEARCH_CFG.speed_mm_s, (long)SEARCH_CFG.turn_counts);
    printf("[run] detour: straight %d mm/s, turn %d mm/s, sweep %u deg/s = %d mm/s, pause %u ms, "
           "back off %u deg (fixed), past nearest %u mm\n",
           DETOUR_CFG.straight_mm_s, DETOUR_CFG.turn_mm_s, DETOUR_CFG.dt.sweep_deg_s, dr.sweep_mm_s,
           DETOUR_CFG.pause_ms, DETOUR_CFG.dt.back_off_deg, DETOUR_CFG.dt.vehicle_mm);
}

/* ---------------------------------------------------------------------
 * Text log of the handoffs and the obstacle modules (as detour_test).
 * ------------------------------------------------------------------- */
static struct {
    uint32_t handoffs;
    uint8_t step; /* last printed detour step; 0xFF: none */
    uint32_t sweep_serial;
    bool returning;
} shown;

static void print_reading(ultrasonic_result_t r) {
    const ultrasonic_status_t status = ultrasonic_result_status(r);
    if (status == ULTRASONIC_STATUS_VALID) {
        if (ultrasonic_result_too_near(r)) {
            printf(" TOO_NEAR");
        } else {
            printf(" %u", ultrasonic_result_distance(r));
        }
    } else {
        printf(status == ULTRASONIC_STATUS_NO_RETURN ? " NR"
               : status == ULTRASONIC_STATUS_NO_RISE ? " NO_RISE"
                                                     : " STUCK");
    }
}

static void report(uint32_t now) {
    if (vsm.handoffs != shown.handoffs) {
        const uint32_t more = vsm.handoffs - shown.handoffs - 1u;
        shown.handoffs = vsm.handoffs;
        const vsm_outcome_t o = vsm.last.outcome;
        const vsm_outcome_type_t type = vsm_outcome_type(o);
        printf("[vc] %lu ms: %s -> %s on %s (payload %u)", (unsigned long)(now - run.t0),
               MODULE_NAMES[vsm.last.from % 5u], MODULE_NAMES[vsm.last.to % 5u], OUT_NAMES[type % 8u],
               vsm_outcome_payload(o));
        printf(more ? " (+%lu earlier in this step)\n" : "\n", (unsigned long)more);
        if (vsm.last.from == VSM_MOD_SCAN && type == VSM_OUT_SIDE) {
            printf("[scan]");
            for (unsigned int i = 0u; i < OBSTACLE_SCAN_POINTS; ++i) {
                print_reading(ss.profile.point[i]);
            }
            printf("\n[scan] nearest %u mm, free step R %u L %u -> detour %s\n",
                   obstacle_profile_distance(ss.word), obstacle_profile_right(ss.word),
                   obstacle_profile_left(ss.word), SIDE_NAMES[ss_payload_side(vsm_outcome_payload(o))]);
        }
        if (vsm.last.from == VSM_MOD_DETOUR) {
            if (type == VSM_OUT_FAILED) {
                printf("[detour] failed: %s\n", DT_FAIL_NAMES[vsm_outcome_payload(o) % 8u]);
            } else if (type == VSM_OUT_DONE_LINE || type == VSM_OUT_DONE_NO_LINE) {
                ++run.detours_done;
            }
        }
        if (vsm.running == VSM_MOD_DETOUR) {
            shown.step = 0xFFu;
            shown.sweep_serial = 0u;
            shown.returning = false;
        }
    }
    if (vsm.running != VSM_MOD_DETOUR || !dr.started) {
        return;
    }
    const dt_t *dt = &dr.dt;
    if (dt->sweep_serial != shown.sweep_serial) {
        static const char *const ends[] = {"-", "seen", "lost", "return", "limit"};
        shown.sweep_serial = dt->sweep_serial;
        printf("[detour] sweep %s at heading %+d (coast %d.%d deg)\n", ends[dt->sweep_last_end % 5u],
               dt->sweep_heading, dt->sweep_coast_x10 / 10, dt->sweep_coast_x10 % 10);
    }
    if (dt->returning && !shown.returning) {
        printf("[detour] RETURN to line at heading %d\n", dt->heading);
    }
    shown.returning = dt->returning;
    if (dt->step != shown.step) {
        if (shown.step == DT_TURN_TOWARD && dt->step == DT_STRAIGHT && !dt->returning) {
            printf("[detour] back off %d of turned %d deg\n", dt->last_back_deg, dt->last_turned_deg);
        }
        shown.step = dt->step;
        printf("[detour] %s at %lu ms: pose (%ld, %ld) heading %+d\n", STEP_NAMES[dt->step & 7u],
               (unsigned long)(now - run.t0), (long)dt->x, (long)dt->y, dt->heading);
    }
}

/* ---------------------------------------------------------------------
 * The run.
 * ------------------------------------------------------------------- */
static void run_once(bool *previous_run) {
    run.valid = false;
    run.log_total = 0u;
    run.pings = 0u;
    replay.active = false;
    if (++run_number == 0u) run_number = 1u;
    printf("[run] start, run %u\n", run_number);

    mcp_clear_faults();
    const uint32_t handoffs0 = vsm.handoffs;
    const uint16_t searches0 = ls.searches;
    const uint16_t scans0 = ss.scans;
    const uint16_t detours0 = dr.detours;
    run.detours_done = 0u;
    mcp_status_t s;
    mcp_status(&s);
    const int32_t start[2] = {s.counts[0], s.counts[1]};
    int32_t last_log[2] = {start[0], start[1]};
    const uint32_t t0 = mcp_now_ms();
    run.t0 = t0;
    line_event = VC_EVENT_NONE;
    (void)vsm_start(&vsm, VSM_MOD_LINE, 0u, t0);
    shown.handoffs = vsm.handoffs;
    /* Drop old posts so that only the run's missed frames count. */
    (void)hb_take(&hb, f_1ms);
    (void)hb_take(&hb, f_10ms);
    (void)hb_take(&hb, f_20ms);
    const uint32_t missed0 = hb.missed_frames;

    bool echo_new = false;
    bool ping_sent = false;
    ultrasonic_result_t echo = 0u;
    uint16_t ping_ms = VSM_PING_DEFAULT_MS; /* asked by the running module */
    uint32_t last_ping_ms = t0;             /* let the servo centre first */
    unsigned int steps = 0u;
    int8_t last_servo = 0;
    bool stop = false;

    while (!stop) {
        const uint32_t now = mcp_now_ms();
        wifi_log_poll(now);
        service_line();

        if (ultrasonic_service()) {
            echo = ultrasonic_latest();
            echo_new = true;
        }
        /* No ping while a result waits for the step: then a ping and an
         * echo in one step are always in that order (vehicle_inputs.h). */
        if (ping_ms != 0u && !echo_new && now - last_ping_ms >= ping_ms && !ultrasonic_busy() &&
            ultrasonic_trigger()) {
            last_ping_ms = now;
            ping_sent = true;
            ++run.pings;
            if (!ultrasonic_busy()) { /* STUCK_HIGH: result at once */
                echo = ultrasonic_latest();
                echo_new = true;
            }
        }

        if (hb_take(&hb, f_20ms) != 0u && pressed(RUN_BUTTON_PIN, previous_run)) {
            run.reason = STOP_BUTTON;
            stop = true;
        }
        if (hb_take(&hb, f_10ms) == 0u) {
            continue;
        }

        mcp_status(&s);
        ping_ms = VSM_PING_DEFAULT_MS;
        const vsm_inputs_t in = {&lp, line_event, {s.counts[0], s.counts[1]}, echo_new, echo,
                                 &ping_ms, ping_sent};
        line_event = VC_EVENT_NONE;
        echo_new = false;
        ping_sent = false;
        const vsm_drive_t d = vsm_step(&vsm, &in, (uint8_t)(s.faults & MC_FAULT_LATCHED), now);
        report(now);
        if (d.action == VSM_ACT_STOP) {
            mcp_stop();
        } else {
            mcp_set_speed(d.left_mm_s, d.right_mm_s);
            if (d.servo_deg != last_servo) {
                (void)mcp_set_servo(d.servo_deg);
                last_servo = d.servo_deg;
            }
        }
        if (vsm.state != VSM_RUNNING) {
            run.reason = STOP_SHELL;
            stop = true;
        } else if (now - t0 >= RUN_LIMIT_MS) {
            run.reason = STOP_TIME_LIMIT;
            stop = true;
        }

        if (++steps % LOG_STEPS == 0u) {
            const uint8_t mode = vsm.running == VSM_MOD_SEARCH ? 1u
                                 : vsm.running != VSM_MOD_LINE ? 3u
                                 : lt.straight                 ? 2u
                                                               : 0u;
            const int8_t position = vsm.running == VSM_MOD_SCAN     ? (int8_t)(10 + ss.index)
                                    : vsm.running == VSM_MOD_DETOUR ? (int8_t)dr.dt.step
                                                                    : lp.position;
            run.log[run.log_total++ % LOG_ENTRIES] = (sl_entry_t){
                .position = position,
                .left_pct = log_speed(d.left_mm_s),
                .right_pct = log_speed(d.right_mm_s),
                .info = (uint8_t)(mode | ((lp.fade_side & 3u) << 2) | ((lp.fade_source & 3u) << 4) |
                                  ((lp.hint & 3u) << 6)),
                .raw = {raw[0], raw[1], raw[2]},
                .left_counts = (int16_t)(s.counts[0] - last_log[0]),
                .right_counts = (int16_t)(s.counts[1] - last_log[1]),
            };
            last_log[0] = s.counts[0];
            last_log[1] = s.counts[1];
            if (run.log_total % LIVE_BATCH == 0u) {
                send_entries(run.log_total - LIVE_BATCH, LIVE_BATCH, 0u);
            }
        }
    }

    mcp_stop();
    (void)mcp_set_servo(0);
    vsm_stop(&vsm);
    run.missed = hb.missed_frames - missed0;
    mcp_status(&s);
    run.duration_ms = mcp_now_ms() - t0;
    run.counts[0] = s.counts[0] - start[0];
    run.counts[1] = s.counts[1] - start[1];
    run.handoffs = vsm.handoffs - handoffs0;
    run.searches = (uint16_t)(ls.searches - searches0);
    run.scans = (uint16_t)(ss.scans - scans0);
    run.detours = (uint16_t)(dr.detours - detours0);
    run.valid = true;
    print_summary();
    vsm_reset(&vsm);
    const uint32_t kept = run.log_total < LOG_ENTRIES ? run.log_total : LOG_ENTRIES;
    replay.next = run.log_total - kept;
    replay.active = kept > 0u;
}

int main(void) {
    stdio_init_all();
    const unsigned int pins[2] = {SPIN_BUTTON_PIN, RUN_BUTTON_PIN};
    for (unsigned int i = 0u; i < 2u; ++i) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_pull_up(pins[i]);
    }
    line_sensors_init();
    ultrasonic_init(ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PIN, NULL);
    hb_init(&hb);
    /*
     * P9 (docs/s3-line-barcode/LINE.md) ran without the I term; now the
     * robot value.
     */
    mc_cfg.ki_q8 = VC_LINE_KI_Q8;
    const bool mc_ok = mcp_init(&hb, &mc_cfg); /* motors 0, servo centre; hook at phase 0 */
    f_1ms = hb_add_flag(&hb, 1u, 0u);
    /* Phase 9: the shell step runs 1 ms before the next mc_tick (phase 0),
     * so its command is used at once (phase 1 waited 9 ms; run P5). The
     * counts it reads are 9 ms old. */
    f_10ms = hb_add_flag(&hb, 10u, 9u);
    f_20ms = hb_add_flag(&hb, 20u, 2u);
    const bool hb_ok = f_1ms >= 0 && f_10ms >= 0 && f_20ms >= 0 && heartbeat_start(&hb, WATCHDOG_MS);

    lp_init(&lp);
    lp_set_pair(&lp, LINE_SENSORS_PAIR != 0, 0);
    lt_init(&lt, &LINE_CFG);
    ls_init(&ls, &SEARCH_CFG);
    vsm_init(&vsm, VSM_DEFAULT_ROUTES, VSM_DEFAULT_ROUTE_COUNT);
    (void)vsm_set_module(&vsm, VSM_MOD_LINE, lt_module(&lt));
    (void)vsm_set_module(&vsm, VSM_MOD_SEARCH, ls_module(&ls));
    ss_init(&ss, &SCAN_CFG);
    dr_init(&dr, &DETOUR_CFG);
    (void)vsm_set_module(&vsm, VSM_MOD_SCAN, ss_module(&ss));
    (void)vsm_set_module(&vsm, VSM_MOD_DETOUR, dr_module(&dr));

    wifi_log_init();
    idle_ms(2000u);
    printf("\nVehicle Controller test: line, search, scan, detour (mm/s). Last reset: %s. mcp_init %s, heartbeat %s.\n"
           "GP21: calibrate, then run %lu s (press again to stop). GP20: next D (now %u).\n",
           heartbeat_watchdog_reset() ? "watchdog" : "power or flash", mc_ok ? "ok" : "FAILED",
           hb_ok ? "ok" : "FAILED", (unsigned long)(RUN_LIMIT_MS / 1000u), lt.cfg.steer.kd_pct);
    if (!mc_ok || !hb_ok) {
        for (;;) idle_ms(1000u);
    }

    bool previous_spin = false;
    bool previous_run = false;
    for (;;) {
        wifi_log_poll(mcp_now_ms());
        replay_step();
        service_line();
        if (hb_take(&hb, f_20ms) == 0u) {
            continue;
        }
        const bool spin_edge = pressed(SPIN_BUTTON_PIN, &previous_spin);
        const bool run_edge = pressed(RUN_BUTTON_PIN, &previous_run);
        if (spin_edge) {
            d_step = (d_step + 1u) % D_STEP_COUNT;
            lt_config_t c = LINE_CFG;
            c.steer.kd_pct = D_STEPS[d_step];
            lt_init(&lt, &c);
            printf("[cfg] steering D %u (gain %u, far gain %u, base %d mm/s)\n", lt.cfg.steer.kd_pct,
                   lt.cfg.steer.gain_pct, lt.cfg.steer.far_gain_pct, lt.cfg.steer.base_mm_s);
        } else if (run_edge) {
            if (!calibrate()) {
                printf("[run] Calibration failed: not started.\n");
            } else {
                const uint32_t end = mcp_now_ms() + START_DELAY_MS;
                while ((int32_t)(mcp_now_ms() - end) < 0) {
                    wifi_log_poll(mcp_now_ms());
                    service_line();
                }
                run_once(&previous_run);
            }
        }
    }
}

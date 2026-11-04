/*
 * Track test: line tracing through the Vehicle Controller shell and the
 * motor controller (docs/VC_LINE_RUN.md).
 *
 *   GP21  spin calibration, then the run after 1 s; press again to stop
 *   GP20  spin calibration only
 *
 * Layers (docs/ADR_VEHICLE_CONTROLLER.md):
 *   detectors  line position (src/line_position.c) every 1 ms, ultrasonic
 *              reading every 100 ms, wheel counts (motor controller)
 *   modules    LINE (src/line_trace.c) and SEARCH (src/line_search.c);
 *              no SCAN or DETOUR: an obstacle stops the run (NO_MODULE)
 *   shell      src/vehicle_sm.c, every 10 ms (heartbeat flag)
 *   motors     src/motor_ctl_pico.c: the only owner of the motor PWM,
 *              the servo PWM and the encoders; wheel speeds in mm/s
 *
 * Timing: the heartbeat (src/heartbeat_pico.c) runs mc_tick() every 10 ms
 * in its interrupt (phase 0) and restarts the watchdog (100 ms). The shell
 * steps at phase 9, 1 ms before the next mc_tick. The line sample and
 * lp_update() run in the main loop on a 1 ms flag, as in detour_test (not
 * yet in the interrupt, docs/ADR_HEARTBEAT.md).
 *
 * A run stops on: GP21, the time limit RUN_LIMIT_MS, or the shell leaving
 * RUNNING (failed search, ultrasonic fault, obstacle, wheel stall).
 *
 * Power: a driving test. Use 5 V VIN with USB disconnected
 * (USB_POWER_SAFETY.md).
 *
 * Log: the steering log (docs/STEER_WIFI_LOG.md) every 50 ms, live and
 * replayed after the run. Log header flag SL_FLAG_SPEED: the two wheel
 * fields are the asked wheel speeds in 4 mm/s units.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "heartbeat_pico.h"
#include "line_position.h"
#include "line_search.h"
#include "line_sensors.h"
#include "line_trace.h"
#include "motor_ctl_pico.h"
#include "motor_ctl_robot.h"
#include "pico/stdlib.h"
#include "steer_log.h"
#include "ultrasonic.h"
#include "vehicle_inputs.h"
#include "vehicle_sm.h"
#include "wifi_log.h"

#define ULTRASONIC_ECHO_PIN 2
#define ULTRASONIC_TRIGGER_PIN 3
#define SPIN_BUTTON_PIN 20
#define RUN_BUTTON_PIN 21

#define WATCHDOG_MS 100u
#define START_DELAY_MS 1000u
#define RUN_LIMIT_MS 90000u
#define WATCH_PERIOD_MS 100u
#define LOG_STEPS 5u /* one log entry per 5 shell steps = 50 ms */
#define LOG_PERIOD_MS 50u
#define LOG_KEEP_MS 20000u
#define LOG_ENTRIES (LOG_KEEP_MS / LOG_PERIOD_MS)
#define LIVE_BATCH 5u
#define SPEED_LOG_UNIT 4 /* mm/s per log unit (int8: up to 508 mm/s) */

/* Locked line tracing (docs/TWO_SENSOR_LINE_TRACING.md) mapped to mm/s
 * through the duty map (docs/LINE_STEER_MM.md): base 25 % -> 171,
 * min 12 % -> 62, max 50 % -> 378. Straight 450 counts across a junction;
 * side hint above 20 counts of turn. */
static const lt_config_t LINE_CFG = {
    .steer = {.base_mm_s = 171, .gain_pct = 100u, .kd_pct = 100u, .min_mm_s = 62,
              .max_mm_s = 378, .far_from = 50u, .far_gain_pct = 400u},
    .straight_counts = 450,
    .hint_counts = 20,
};
/* Search: 20 % -> 131 mm/s; an eighth wheel turn each way (2535 / 8). */
static const ls_config_t SEARCH_CFG = {.speed_mm_s = 131, .turn_counts = 2535 / 8};

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

static hb_t hb;
static int f_1ms, f_10ms, f_20ms;
static lp_t lp;
static lt_t lt;
static ls_t ls;
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
    printf("[run] searches %u, pings %u, wheels L %ld  R %ld counts\n", run.searches, run.pings,
           (long)run.counts[0], (long)run.counts[1]);
    mcp_status_t s;
    mcp_status(&s);
    printf("[run] heartbeat isr max %lu us, overruns gap %lu isr %lu, missed in run %lu; mc hook max "
           "%lu us; faults 0x%02x; Wi-Fi dropped %lu\n",
           (unsigned long)hb.isr_max_us, (unsigned long)hb.gap_overruns,
           (unsigned long)hb.isr_overruns, (unsigned long)run.missed,
           (unsigned long)s.tick_max_us, s.faults, (unsigned long)wifi_log_dropped());
    printf("[run] line: base %d min %d max %d mm/s, gain %u D %u far %u above %u; search %d mm/s %ld "
           "counts\n",
           LINE_CFG.steer.base_mm_s, LINE_CFG.steer.min_mm_s, LINE_CFG.steer.max_mm_s,
           LINE_CFG.steer.gain_pct, LINE_CFG.steer.kd_pct, LINE_CFG.steer.far_gain_pct,
           LINE_CFG.steer.far_from, SEARCH_CFG.speed_mm_s, (long)SEARCH_CFG.turn_counts);
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
    mcp_status_t s;
    mcp_status(&s);
    const int32_t start[2] = {s.counts[0], s.counts[1]};
    int32_t last_log[2] = {start[0], start[1]};
    const uint32_t t0 = mcp_now_ms();
    run.t0 = t0;
    line_event = VC_EVENT_NONE;
    (void)vsm_start(&vsm, VSM_MOD_LINE, 0u, t0);
    /* Drop old posts so that only the run's missed frames count. */
    (void)hb_take(&hb, f_1ms);
    (void)hb_take(&hb, f_10ms);
    (void)hb_take(&hb, f_20ms);
    const uint32_t missed0 = hb.missed_frames;

    bool echo_new = false;
    ultrasonic_result_t echo = 0u;
    uint32_t next_ping_ms = t0 + WATCH_PERIOD_MS; /* let the servo centre first */
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
        if ((int32_t)(now - next_ping_ms) >= 0 && !ultrasonic_busy() && ultrasonic_trigger()) {
            next_ping_ms = now + WATCH_PERIOD_MS;
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
        const vsm_inputs_t in = {&lp, line_event, {s.counts[0], s.counts[1]}, echo_new, echo};
        line_event = VC_EVENT_NONE;
        echo_new = false;
        const vsm_drive_t d = vsm_step(&vsm, &in, (uint8_t)(s.faults & MC_FAULT_LATCHED), now);
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
            run.log[run.log_total++ % LOG_ENTRIES] = (sl_entry_t){
                .position = lp.position,
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
    static const mc_config_t cfg = MC_ROBOT_CONFIG;
    const bool mc_ok = mcp_init(&hb, &cfg); /* motors 0, servo centre; hook at phase 0 */
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

    wifi_log_init();
    idle_ms(2000u);
    printf("\nVehicle Controller line test (mm/s). Last reset: %s. mcp_init %s, heartbeat %s.\n"
           "GP21: calibrate, then run %lu s (press again to stop). GP20: calibrate only.\n",
           heartbeat_watchdog_reset() ? "watchdog" : "power or flash", mc_ok ? "ok" : "FAILED",
           hb_ok ? "ok" : "FAILED", (unsigned long)(RUN_LIMIT_MS / 1000u));
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
            (void)calibrate();
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

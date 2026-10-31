/*
 * Track test: line following with the obstacle detour (src/obstacle_detour.c):
 * straight drives and turns on the spot only, no arcs.
 *
 *   GP21  spin calibration, then the run after 1 s; press again to stop
 *   GP20  spin calibration only
 *
 * The run:
 *   FOLLOW  line following with the calm settings of steer_test, straight
 *           across junctions, the same search on LINE_LOST. The forward
 *           watch (src/obstacle.c) reads the ultrasonic sensor every
 *           100 ms, servo straight.
 *   STOP    forward watch STOP: motors off (coast), wait STOP_SETTLE_MS.
 *   SCAN    one servo scan (src/obstacle_scan.c); the profile gives the
 *           detour side and the obstacle distance.
 *   DETOUR  src/obstacle_detour.c: the sensor looks 30 degrees
 *           toward the obstacle. Turn away (5-degree steps, stop and look)
 *           until the obstacle passed out of view; then straights that
 *           end 350 mm (length + width) after their nearest reading, each
 *           followed by a turn toward the obstacle until it is seen and
 *           then back by the operator's formula; until the line is
 *           found in a straight. Then 123 mm on and a turn back along the
 *           line, and FOLLOW again with a new forward watch. The motors
 *           stop for MOVE_PAUSE_MS when the motion changes (not between
 *           the steps of one turn). Straights hold the heading with the
 *           wheel-count difference.
 *
 * A run stops on: GP21, a failed search, a stalled wheel, a sensor fault,
 * a failed detour, or the time limit RUN_LIMIT_MS.
 *
 * Power: a driving test. Use 5 V VIN with USB disconnected
 * (USB_POWER_SAFETY.md).
 *
 * Log: kept in RAM only (not in flash). The Wi-Fi log (docs/STEER_WIFI_LOG.md)
 * gets the entries live; when the run stops, the detour readings and the
 * whole kept log are sent again (replay). Mode 3 in the log is BYPS (STOP,
 * SCAN or DETOUR). Log header flag SL_FLAG_DETOUR: in the reading records,
 * state << 2 | phase is the detour step and steer is 0. Keep the robot on for about 2 s after it stops. A USB
 * terminal that connects after the run gets the summary and the log.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "drive.h"
#include "hardware/gpio.h"
#include "line_position.h"
#include "line_sensors.h"
#include "line_spin.h"
#include "line_steer.h"
#include "obstacle.h"
#include "obstacle_detour.h"
#include "obstacle_profile.h"
#include "obstacle_scan.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "servo.h"
#include "steer_log.h"
#include "ultrasonic.h"
#include "wifi_log.h"

#define ULTRASONIC_ECHO_PIN 2
#define ULTRASONIC_TRIGGER_PIN 3
#define SERVO_PIN 15
#define SPIN_BUTTON_PIN 20
#define RUN_BUTTON_PIN 21

/* Line following: the calm settings of steer_test (run 10). */
static const steer_config_t STEER_FOLLOW = {
    .base_pct = 25u,
    .gain_pct = 200u,
    .kd_pct = 200u,
    .min_pct = 12u,
    .max_pct = 50u,
    .right_trim_pct = 1,
};
/* First values: tune on the track. Geometry (docs/OBSTACLE_BYPASS.md):
 * vehicle length 190 mm, track 111 mm, servo axis 65 mm ahead of the rear
 * axle. See docs/CALIBRATION_RESULTS.md, Robot geometry: sweep 150 mm
 * uses the calculated 146.7 mm front-corner circle; line_ahead_mm is
 * 123 mm from the rear axle to the front-mounted line sensors. Turn counts:
 * 111 mm x pi / 180 = 1.937 mm per degree of (right - left) = 24.05
 * counts. */
/* Back off after TURN_TOWARD sees the obstacle. turned_deg: how far it
 * turned toward. Operator: edit the formula. */
static int back_off_for(int turned_deg) { return turned_deg / 2; }

static const dt_config_t DETOUR = {
    .back_off_fn = back_off_for,
    .return_heading_deg = 90u, /* operator: unseen at -90 degrees, return to line */
    .return_max_mm = 1000u,    /* operator: maximum return travel without a line */
    .turn_step_deg = 5u,
    .away_max_deg = 90u,
    .toward_max_deg = 150u,
    .max_straights = 5u,
    .back_off_deg = 0u, /* experiment: operator sim test; v1.5 robot value is 40 */
    .look_ms = 150u, /* the robot coasts, then stands still for the reading */
    .vehicle_mm = 350u, /* operator: length 190 + width 160 (v1, v1.1: 190) */
    .sweep_mm = 150u,
    .margin_mm = 30u,
    .sensor_ahead_mm = 65u,
    .line_ahead_mm = 123u, /* rear axle to line sensors, 65 + 58 mm; v1-v1.4: 100;
                           * docs/CALIBRATION_RESULTS.md, Robot geometry */
    .seen_mm = 400u,
    .counts_per_deg_x100 = 2405u,
    .total_max_ms = 25000u,
};

#define TURN_DUTY_PCT 20     /* on the spot, as the line search */
#define STRAIGHT_DUTY_PCT 20 /* bypass base of runs 5-18 */
#define HOLD_COUNTS_PER_PCT 8 /* heading hold: 1 % per 8 counts (1/3 degree) */
#define HOLD_MAX_PCT 8
#define MOVE_PAUSE_MS 250u
#define STOP_SETTLE_MS 300u
#define WATCH_PERIOD_MS 100u
#define SAMPLE_PERIOD_US 1000u
#define STEER_PERIOD_MS 10u
#define LOG_PERIOD_MS 50u
#define LOG_KEEP_MS 20000u
#define LOG_ENTRIES (LOG_KEEP_MS / LOG_PERIOD_MS)
#define BYPASS_RECORDS 256u
#define START_DELAY_MS 1000u
#define RUN_LIMIT_MS 30000u
#define STRAIGHT_COUNTS 450
#define STALL_GRACE_MS 1000u
#define STALL_TIMEOUT_MS 500u
#define BUTTON_POLL_MS 20u
#define REPORT_REPEAT_MS 5000u
#define HINT_COUNTS 20
#define SEARCH_DUTY_PCT 20
#define SEARCH_COUNTS (DRIVE_COUNTS_PER_WHEEL_TURN / 8)
#define LIVE_BATCH 5u

typedef enum {
    STOP_BUTTON,
    STOP_SEARCH_FAILED,
    STOP_STALL_LEFT,
    STOP_STALL_RIGHT,
    STOP_TIME_LIMIT,
    STOP_SENSOR_FAULT,
    STOP_DETOUR_FAILED,
} stop_reason_t;

static const char *const STOP_NAMES[] = {
    "GP21 pressed",   "search failed",      "left wheel stalled", "right wheel stalled",
    "time limit",     "ultrasonic fault",   "detour failed",
};
static const char *const FAIL_NAMES[] = {
    "-",          "no free side",     "turn limit (obstacle not seen / not lost)",
    "time limit", "ultrasonic fault", "obstacle too near",
    "line not found (straights)",
};
static const char *const STEP_NAMES[8] = {
    "BACK", "TURN_AWAY", "STRAIGHT", "TURN_TOWARD", "ON_LINE", "ALIGN", "DONE", "FAILED",
};

typedef enum {
    PHASE_FOLLOW,
    PHASE_SEARCH,
    PHASE_STOP,
    PHASE_SCAN,
    PHASE_DETOUR,
} phase_t;

#define MODE_FOLLOW 0u
#define MODE_SEARCH 1u
#define MODE_STRAIGHT 2u
#define MODE_BYPASS 3u /* STOP, SCAN or DETOUR */
static const char *const MODE_NAMES[4] = {"FOLW", "SRCH", "STRT", "BYPS"};

typedef struct {
    bool valid;
    stop_reason_t reason;
    uint8_t fail;  /* dt_fail_t of the last detour */
    uint32_t duration_ms;
    int32_t left_counts;
    int32_t right_counts;
    uint16_t stops;    /* forward watch STOP events */
    uint16_t detours; /* detours that found the line */
    uint16_t searches;
    uint32_t log_total;
    sl_entry_t log[LOG_ENTRIES];
    uint32_t bypass_total;
    sl_bypass_t bypass[BYPASS_RECORDS];
} run_t;

static run_t run;
static lp_t lp;
static uint8_t run_number;

static bool button_pressed(unsigned int pin, bool *previous) {
    const bool down = !gpio_get(pin);
    const bool edge = down && !*previous;
    *previous = down;
    return edge;
}

/* Servo pulse for an angle: 1500 us + angle x 1000/90 us, rounded (the
 * scan table, src/obstacle_scan.h). Positive = left = longer pulse. */
static uint16_t servo_pulse(int deg) {
    const int us = deg >= 0 ? (deg * 100 + 4) / 9 : -((-deg * 100 + 4) / 9);
    return (uint16_t)((int)SERVO_CENTRE_US + us);
}

static void print_reading(ultrasonic_result_t r) {
    const ultrasonic_status_t status = ultrasonic_result_status(r);
    if (status == ULTRASONIC_STATUS_VALID) {
        if (ultrasonic_result_too_near(r)) {
            printf("TOO_NEAR");
        } else {
            printf("%u", ultrasonic_result_distance(r));
        }
    } else {
        printf(status == ULTRASONIC_STATUS_NO_RETURN ? "NR"
               : status == ULTRASONIC_STATUS_NO_RISE ? "NO_RISE"
                                                     : "STUCK");
    }
}

static void print_summary(void) {
    printf("[run] %lu ms, stopped by %s", (unsigned long)run.duration_ms,
           STOP_NAMES[run.reason]);
    if (run.reason == STOP_DETOUR_FAILED) {
        printf(" (%s)", FAIL_NAMES[run.fail]);
    }
    printf("\n[run] obstacle stops %u, detours done %u, searches %u\n", run.stops,
           run.detours, run.searches);
    printf("[run] wheels L %ld  R %ld counts\n", (long)run.left_counts,
           (long)run.right_counts);
    printf("[run] detour v1.5-exp-tuning: turn steps %u deg (%u.%02u counts/deg), back off ",
           DETOUR.turn_step_deg, DETOUR.counts_per_deg_x100 / 100u,
           DETOUR.counts_per_deg_x100 % 100u);
    if (DETOUR.back_off_fn) {
        printf("fn, ");
    } else {
        printf("%u deg, ", DETOUR.back_off_deg);
    }
    printf("look %u ms, past nearest %u mm, seen < %u mm; turn %d%%, straight %d%%; "
           "return heading %u deg, return max %u mm\n",
           DETOUR.look_ms, DETOUR.vehicle_mm,
           DETOUR.seen_mm, TURN_DUTY_PCT, STRAIGHT_DUTY_PCT,
           DETOUR.return_heading_deg, DETOUR.return_max_mm);
}

static void print_log(void) {
    printf("\n[log] t_ms mode  pos  dutyL dutyR  rawL rawC rawR  cntL cntR\n");
    const uint32_t kept = run.log_total < LOG_ENTRIES ? run.log_total : LOG_ENTRIES;
    for (uint32_t i = run.log_total - kept; i < run.log_total; ++i) {
        const sl_entry_t *e = &run.log[i % LOG_ENTRIES];
        printf("[log] %6lu %s %+4d  %3d %3d  %4u %4u %4u  %4d %4d\n",
               (unsigned long)(i * LOG_PERIOD_MS), MODE_NAMES[e->info & 3u], e->position,
               e->left_pct, e->right_pct, e->raw[0], e->raw[1], e->raw[2], e->left_counts,
               e->right_counts);
    }
    printf("\n[detour] t_ms servo reading step\n");
    const uint32_t n = run.bypass_total < BYPASS_RECORDS ? run.bypass_total : BYPASS_RECORDS;
    for (uint32_t i = 0u; i < n; ++i) {
        const sl_bypass_t *b = &run.bypass[i];
        printf("[detour] %6u %+3d ", b->time_10ms * 10u, b->servo_deg);
        print_reading(b->reading);
        printf(" %s\n", STEP_NAMES[((b->state & 3u) << 2 | (b->phase & 3u)) & 7u]);
    }
}

/* Wi-Fi: send log entries [first, first + count) of the current run. */
static void send_entries(uint32_t first, uint32_t count, uint8_t flags) {
    uint8_t packet[SL_LOG_HEADER_BYTES + SL_MAX_ENTRIES * SL_ENTRY_BYTES];
    if (count > SL_MAX_ENTRIES) count = SL_MAX_ENTRIES;
    size_t n = sl_write_log_header(packet, run_number,
                                   (uint8_t)(flags | SL_FLAG_BYPASS | SL_FLAG_DETOUR),
                                   (uint16_t)first, (uint8_t)count);
    for (uint32_t i = 0u; i < count; ++i) {
        sl_pack_entry(&run.log[(first + i) % LOG_ENTRIES], &packet[n]);
        n += SL_ENTRY_BYTES;
    }
    (void)wifi_log_send(SL_TYPE_LOG, packet, n);
}

static void send_bypass(uint32_t first, uint32_t count) {
    uint8_t packet[SL_BYPASS_HEADER_BYTES + SL_MAX_BYPASS * SL_BYPASS_BYTES];
    if (count > SL_MAX_BYPASS) count = SL_MAX_BYPASS;
    size_t n = sl_write_bypass_header(packet, run_number, (uint16_t)first, (uint8_t)count);
    for (uint32_t i = 0u; i < count; ++i) {
        sl_pack_bypass(&run.bypass[first + i], &packet[n]);
        n += SL_BYPASS_BYTES;
    }
    (void)wifi_log_send(SL_TYPE_BYPASS, packet, n);
}

/* Replay: the bypass records first, then the kept log; the last log packet
 * tells the receiver to save the run. One packet per call. */
static struct {
    bool active;
    uint32_t next_bypass;
    uint32_t next;
} replay;

static void replay_start(void) {
    const uint32_t kept = run.log_total < LOG_ENTRIES ? run.log_total : LOG_ENTRIES;
    replay.next_bypass = 0u;
    replay.next = run.log_total - kept;
    replay.active = kept > 0u;
}

static void replay_step(void) {
    if (!replay.active || !wifi_log_receiver_known()) {
        return;
    }
    const uint32_t records =
        run.bypass_total < BYPASS_RECORDS ? run.bypass_total : BYPASS_RECORDS;
    if (replay.next_bypass < records) {
        const uint32_t left = records - replay.next_bypass;
        const uint32_t count = left < SL_MAX_BYPASS ? left : SL_MAX_BYPASS;
        send_bypass(replay.next_bypass, count);
        replay.next_bypass += count;
        return;
    }
    const uint32_t left = run.log_total - replay.next;
    const uint32_t count = left < SL_MAX_ENTRIES ? left : SL_MAX_ENTRIES;
    const bool last = count == left;
    send_entries(replay.next, count,
                 (uint8_t)(SL_FLAG_REPLAY | (last ? SL_FLAG_REPLAY_END : 0u)));
    replay.next += count;
    replay.active = !last;
}

typedef struct {
    int direction; /* +1 turn left first, -1 turn right first */
    unsigned int step;
    int32_t start_l;
    int32_t start_r;
} search_t;

/* Everything one run needs. */
typedef struct {
    phase_t phase;
    uint32_t t0;
    steer_t steer;
    obstacle_monitor_t watch;
    dt_t dt;
    search_t search;
    int duty[2];
    bool straight;
    bool was_hold;
    int32_t straight_from;
    int32_t heading;
    int32_t last_steer[2];
    uint32_t next_watch_ms;
    uint32_t stop_until_ms;
    uint8_t shown;      /* last printed detour step */
    dt_move_t move;     /* last motion (TURN or STRAIGHT) */
    int move_dir;
    uint32_t pause_until_ms; /* motors off between moves */
    int32_t hold_l, hold_r;  /* wheel counts at the start of the move */
    stop_reason_t reason;
    bool stop;
} ctx_t;

static void set_duty(ctx_t *x, int left, int right) {
    x->duty[0] = left;
    x->duty[1] = right;
    drive_set(left, right);
}

static void end_run(ctx_t *x, stop_reason_t reason) {
    x->reason = reason;
    x->stop = true;
}

static void start_follow(ctx_t *x, const int32_t c[2], uint32_t now) {
    x->phase = PHASE_FOLLOW;
    steer_set_config(&x->steer, &STEER_FOLLOW);
    steer_reset(&x->steer);
    obstacle_init(&x->watch);
    (void)servo_set_pulse_us(SERVO_CENTRE_US);
    x->heading = 0;
    x->last_steer[0] = c[0];
    x->last_steer[1] = c[1];
    x->was_hold = false;
    x->straight = false;
    x->next_watch_ms = now + WATCH_PERIOD_MS; /* let the servo centre first */
}

/* The same count-limited search after LINE_LOST or an off-line detour. */
static void start_search(ctx_t *x, const int32_t c[2], int direction) {
    x->search = (search_t){
        .direction = direction,
        .start_l = c[0],
        .start_r = c[1],
    };
    x->phase = PHASE_SEARCH;
    ++run.searches;
    set_duty(x, -direction * SEARCH_DUTY_PCT, direction * SEARCH_DUTY_PCT);
}

/* One ultrasonic result, by phase. */
static void on_result(ctx_t *x, ultrasonic_result_t r, uint32_t now) {
    if (x->phase == PHASE_FOLLOW) {
        const obstacle_event_word_t ev = obstacle_update(&x->watch, r);
        if (obstacle_event(ev) == OBSTACLE_EVENT_STOP) {
            set_duty(x, 0, 0);
            x->phase = PHASE_STOP;
            x->stop_until_ms = now + STOP_SETTLE_MS;
            ++run.stops;
            printf("[obst] STOP at %lu ms, %u mm\n", (unsigned long)(now - x->t0),
                   obstacle_event_distance(ev));
        } else if (obstacle_event(ev) == OBSTACLE_EVENT_SENSOR_FAULT) {
            end_run(x, STOP_SENSOR_FAULT);
        }
    } else if (x->phase == PHASE_DETOUR) {
        const int8_t at = (int8_t)(x->dt.side * x->dt.servo_deg); /* robot frame */
        const uint16_t best = x->dt.best_mm;
        const bool was_returning = x->dt.returning;
        dt_reading(&x->dt, r, now);
        if (!was_returning && x->dt.returning) {
            printf("[detour] RETURN to line at heading %d\n", x->dt.heading);
        }
        if (x->dt.best_mm != best) {
            printf("[detour] %s nearest %u mm at %ld mm: count again\n",
                   STEP_NAMES[x->dt.step & 7u], x->dt.best_mm, (long)x->dt.from_mm);
        }
        if (run.bypass_total < BYPASS_RECORDS) {
            const uint32_t t = (now - x->t0) / 10u;
            run.bypass[run.bypass_total] = (sl_bypass_t){
                .time_10ms = (uint16_t)(t > SL_BYPASS_TIME_MAX ? SL_BYPASS_TIME_MAX : t),
                .servo_deg = at,
                .reading = r,
                .state = (uint8_t)(x->dt.step >> 2),
                .steer_deg = 0,
                .phase = (uint8_t)(x->dt.step & 3u),
            };
        }
        ++run.bypass_total;
    }
}

static void start_detour(ctx_t *x, const int32_t c[2], uint32_t now) {
    const obstacle_profile_t *profile = obstacle_scan_profile();
    const obstacle_profile_word_t word = obstacle_profile_build(profile);
    const obstacle_side_t side = obstacle_profile_side(word, profile);
    printf("[scan]");
    for (unsigned int i = 0u; i < OBSTACLE_SCAN_POINTS; ++i) {
        printf(" ");
        print_reading(profile->point[i]);
    }
    printf("\n[scan] nearest %u mm, free step R %u L %u -> detour %s\n",
           obstacle_profile_distance(word), obstacle_profile_right(word),
           obstacle_profile_left(word),
           side == OBSTACLE_SIDE_LEFT    ? "LEFT"
           : side == OBSTACLE_SIDE_RIGHT ? "RIGHT"
                                         : "NONE");
    dt_init(&x->dt, &DETOUR, side, obstacle_profile_distance(word), c[0], c[1], now);
    x->shown = 0xFFu; /* print the first step */
    x->move = DT_MOVE_STOP;
    x->move_dir = 0;
    x->phase = PHASE_DETOUR;
}

/* Straight with a heading hold: the wheel that is ahead slows down. */
static void straight(ctx_t *x, const int32_t c[2], int dir) {
    int corr = (int)(((c[0] - x->hold_l) - (c[1] - x->hold_r)) / HOLD_COUNTS_PER_PCT);
    corr = corr > HOLD_MAX_PCT ? HOLD_MAX_PCT : corr < -HOLD_MAX_PCT ? -HOLD_MAX_PCT : corr;
    set_duty(x, dir * STRAIGHT_DUTY_PCT - corr, dir * STRAIGHT_DUTY_PCT + corr);
}

static void detour_step(ctx_t *x, const int32_t c[2], uint32_t now) {
    const dt_word_t w = dt_update(&x->dt, lp_line_seen(&lp), c[0], c[1], now);
    (void)servo_set_pulse_us(servo_pulse(dt_servo_deg(w)));
    const dt_step_t step = dt_step(&x->dt);
    /* Step changes also happen in dt_reading(): compare with the last print. */
    if ((uint8_t)step != x->shown) {
        if (x->shown == DT_TURN_TOWARD && step == DT_STRAIGHT && !x->dt.returning) {
            printf("[detour] back off %d of turned %d deg\n",
                   x->dt.last_back_deg, x->dt.last_turned_deg);
        }
        x->shown = (uint8_t)step;
        printf("[detour] %s at %lu ms: pose (%ld, %ld) heading %+d\n", STEP_NAMES[step & 7u],
               (unsigned long)(now - x->t0), (long)x->dt.x, (long)x->dt.y, x->dt.heading);
    }
    if (step == DT_DONE) {
        ++run.detours;
        lp_reset_tracking(&lp);
        uint16_t raw[LP_SENSORS];
        line_sensors_read(raw);
        (void)lp_update(&lp, raw, now);
        start_follow(x, c, now);
        if (!lp_line_seen(&lp)) {
            printf("[detour] DONE: line not seen, search toward the obstacle side\n");
            start_search(x, c, -x->dt.side);
        }
        return;
    }
    if (step == DT_FAILED) {
        run.fail = x->dt.fail;
        printf("[detour] failed: %s\n", FAIL_NAMES[x->dt.fail]);
        end_run(x, STOP_DETOUR_FAILED);
        return;
    }
    const dt_move_t move = dt_move(w);
    const int dir = dt_dir(w);
    if (move == DT_MOVE_STOP) { /* a look in a turn */
        set_duty(x, 0, 0);
        return;
    }
    if (move != x->move || dir != x->move_dir) {
        /* New motion (not the next step of the same turn): stop first. */
        x->move = move;
        x->move_dir = dir;
        x->pause_until_ms = now + MOVE_PAUSE_MS;
        set_duty(x, 0, 0);
    }
    if ((int32_t)(now - x->pause_until_ms) < 0) {
        x->hold_l = c[0]; /* the heading hold starts after the pause */
        x->hold_r = c[1];
        return;
    }
    if (move == DT_MOVE_TURN) {
        set_duty(x, -dir * TURN_DUTY_PCT, dir * TURN_DUTY_PCT);
    } else {
        straight(x, c, dir);
    }
}

static void follow_step(ctx_t *x, const int32_t c[2]) {
    x->heading += (c[0] - x->last_steer[0]) - (c[1] - x->last_steer[1]);
    x->last_steer[0] = c[0];
    x->last_steer[1] = c[1];
    if (lp_centred(&lp)) {
        x->heading = 0;
    }
    lp_set_hint(&lp, x->heading > HINT_COUNTS    ? VC_LINE_SIDE_LEFT
                     : x->heading < -HINT_COUNTS ? VC_LINE_SIDE_RIGHT
                                                 : VC_LINE_SIDE_UNKNOWN);
    const bool hold = lp_junction_hold(&lp);
    const int32_t travel = (c[0] + c[1]) / 2;
    if (hold && !x->was_hold) {
        x->straight_from = travel;
    }
    x->was_hold = hold;
    x->straight = hold && travel - x->straight_from < STRAIGHT_COUNTS;
    const steer_word_t w =
        x->straight ? steer_straight(&x->steer) : steer_update(&x->steer, lp.position);
    set_duty(x, (int)steer_left(w), (int)steer_right(w));
}

static void run_once(bool *previous_run) {
    run = (run_t){0};
    replay.active = false;
    ++run_number;
    if (run_number == 0u) run_number = 1u;
    printf("[run] start, run %u\n", run_number);

    static ctx_t x;
    x = (ctx_t){0};
    x.t0 = to_ms_since_boot(get_absolute_time());
    int32_t start[2];
    drive_counts(&start[0], &start[1]);
    steer_init(&x.steer, &STEER_FOLLOW);
    start_follow(&x, start, x.t0);

    int32_t last_log[2] = {start[0], start[1]};
    int32_t last_count[2] = {start[0], start[1]};
    uint32_t moved_ms[2] = {x.t0, x.t0};
    uint16_t raw[LP_SENSORS] = {0u, 0u, 0u};
    absolute_time_t next_sample = get_absolute_time();
    uint32_t next_steer_ms = x.t0;
    uint32_t next_log_ms = x.t0 + LOG_PERIOD_MS;
    uint32_t next_button_ms = x.t0;

    while (!x.stop) {
        const uint32_t now = to_ms_since_boot(get_absolute_time());
        int32_t c[2];
        drive_counts(&c[0], &c[1]);
        wifi_log_poll(now);

        if (time_reached(next_sample)) {
            next_sample = delayed_by_us(next_sample, SAMPLE_PERIOD_US);
            line_sensors_read(raw);
            const vc_event_t event = lp_update(&lp, raw, now);
            if (x.phase == PHASE_FOLLOW && vc_event_type(event) == VC_EVENT_LINE_LOST) {
                const vc_line_side_t side = (vc_line_side_t)vc_event_payload(event);
                start_search(&x, c, side == VC_LINE_SIDE_RIGHT ? -1 : +1);
            } else if (x.phase == PHASE_SEARCH && vc_event_type(event) == VC_EVENT_LINE_FOUND) {
                start_follow(&x, c, now);
                next_steer_ms = now;
            }
        }

        /* Ultrasonic: the scan owns it during SCAN. */
        if (x.phase == PHASE_SCAN) {
            const obstacle_scan_event_t ev = obstacle_scan_service();
            if (ev == OBSTACLE_SCAN_DONE) {
                start_detour(&x, c, now);
            } else if (ev == OBSTACLE_SCAN_FAULT) {
                end_run(&x, STOP_SENSOR_FAULT);
            }
        } else {
            if (ultrasonic_service()) {
                on_result(&x, ultrasonic_latest(), now);
            }
            bool want = false;
            if (x.phase == PHASE_FOLLOW) {
                want = (int32_t)(now - x.next_watch_ms) >= 0;
            } else if (x.phase == PHASE_DETOUR) {
                want = dt_wants_reading(&x.dt, now);
            }
            if (want && !ultrasonic_busy() && ultrasonic_trigger()) {
                x.next_watch_ms = now + WATCH_PERIOD_MS;
                if (x.phase == PHASE_DETOUR) {
                    dt_triggered(&x.dt);
                }
                if (!ultrasonic_busy()) { /* STUCK_HIGH: result at once */
                    on_result(&x, ultrasonic_latest(), now);
                }
            }
        }
        if (x.phase == PHASE_STOP && (int32_t)(now - x.stop_until_ms) >= 0 &&
            !ultrasonic_busy() && obstacle_scan_start()) {
            x.phase = PHASE_SCAN;
        }

        if (x.phase == PHASE_SEARCH) {
            const int32_t angle =
                ((c[1] - x.search.start_r) - (c[0] - x.search.start_l)) / 2 *
                x.search.direction;
            if (x.search.step == 0u && angle >= SEARCH_COUNTS) {
                x.search.step = 1u;
                set_duty(&x, x.search.direction * SEARCH_DUTY_PCT,
                         -x.search.direction * SEARCH_DUTY_PCT);
            } else if (x.search.step == 1u && angle <= -SEARCH_COUNTS) {
                end_run(&x, STOP_SEARCH_FAILED);
            }
        } else if ((int32_t)(now - next_steer_ms) >= 0) {
            next_steer_ms = now + STEER_PERIOD_MS;
            if (x.phase == PHASE_FOLLOW) {
                follow_step(&x, c);
            } else if (x.phase == PHASE_DETOUR) {
                detour_step(&x, c, now);
            }
        }

        bool stalled[2];
        for (unsigned int w = 0u; w < 2u; ++w) {
            if (c[w] != last_count[w] || x.duty[w] == 0) {
                last_count[w] = c[w];
                moved_ms[w] = now;
            }
            stalled[w] =
                now - x.t0 >= STALL_GRACE_MS && now - moved_ms[w] >= STALL_TIMEOUT_MS;
        }
        if (stalled[0] || stalled[1]) {
            end_run(&x, stalled[0] ? STOP_STALL_LEFT : STOP_STALL_RIGHT);
        }
        if (now - x.t0 >= RUN_LIMIT_MS) {
            end_run(&x, STOP_TIME_LIMIT);
        }

        if ((int32_t)(now - next_log_ms) >= 0) {
            next_log_ms += LOG_PERIOD_MS;
            const bool bypass = x.phase == PHASE_STOP || x.phase == PHASE_SCAN ||
                                x.phase == PHASE_DETOUR;
            run.log[run.log_total++ % LOG_ENTRIES] = (sl_entry_t){
                /* BYPS: the detour step (0 in STOP and SCAN). */
                .position = x.phase == PHASE_DETOUR ? (int8_t)x.dt.step
                            : bypass                ? 0
                                                    : lp.position,
                .left_pct = (int8_t)x.duty[0],
                .right_pct = (int8_t)x.duty[1],
                .info = (uint8_t)((bypass                    ? MODE_BYPASS
                                   : x.phase == PHASE_SEARCH ? MODE_SEARCH
                                   : x.straight              ? MODE_STRAIGHT
                                                             : MODE_FOLLOW) |
                                  ((lp.fade_side & 3u) << 2) |
                                  ((lp.fade_source & 3u) << 4) | ((lp.hint & 3u) << 6)),
                .raw = {raw[0], raw[1], raw[2]},
                .left_counts = (int16_t)(c[0] - last_log[0]),
                .right_counts = (int16_t)(c[1] - last_log[1]),
            };
            if (run.log_total % LIVE_BATCH == 0u) {
                send_entries(run.log_total - LIVE_BATCH, LIVE_BATCH, 0u);
            }
            last_log[0] = c[0];
            last_log[1] = c[1];
        }

        if ((int32_t)(now - next_button_ms) >= 0) {
            next_button_ms = now + BUTTON_POLL_MS;
            if (button_pressed(RUN_BUTTON_PIN, previous_run)) {
                end_run(&x, STOP_BUTTON);
            }
        }
    }

    drive_stop();
    obstacle_scan_abort();
    (void)servo_set_pulse_us(SERVO_CENTRE_US);
    const uint32_t end = to_ms_since_boot(get_absolute_time());
    int32_t c[2];
    drive_counts(&c[0], &c[1]);
    run.reason = x.reason;
    run.duration_ms = end - x.t0;
    run.left_counts = c[0] - start[0];
    run.right_counts = c[1] - start[1];
    run.valid = true;
    print_summary();
    replay_start();
}

int main(void) {
    stdio_init_all();
    wifi_log_init();
    line_sensors_init();
    drive_init();
    drive_stop();
    ultrasonic_init(ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PIN, NULL);
    obstacle_scan_init(SERVO_PIN); /* servo to the centre */
    const unsigned int buttons[2] = {SPIN_BUTTON_PIN, RUN_BUTTON_PIN};
    for (unsigned int i = 0u; i < 2u; ++i) {
        gpio_init(buttons[i]);
        gpio_set_dir(buttons[i], GPIO_IN);
        gpio_pull_up(buttons[i]);
    }
    sleep_ms(2000);
    printf("\nObstacle detour test. GP21: calibrate, then run %lu s (press again to "
           "stop). GP20: calibrate only.\n",
           (unsigned long)(RUN_LIMIT_MS / 1000u));
    lp_init(&lp);

    bool previous_spin = false;
    bool previous_run = false;
    bool was_connected = false;
    uint32_t next_report_ms = 0u;

    while (true) {
        const uint32_t now = to_ms_since_boot(get_absolute_time());
        wifi_log_poll(now);
        replay_step();

        if (button_pressed(SPIN_BUTTON_PIN, &previous_spin)) {
            (void)line_spin_calibrate(&lp);
            printf("\n");
        }
        if (button_pressed(RUN_BUTTON_PIN, &previous_run)) {
            if (line_spin_calibrate(&lp) != LINE_SPIN_OK || !lp_calibrated(&lp)) {
                printf("[run] Calibration failed: not started.\n");
            } else {
                sleep_ms(START_DELAY_MS);
                run_once(&previous_run);
                next_report_ms = to_ms_since_boot(get_absolute_time()) + REPORT_REPEAT_MS;
            }
        }

        const bool connected = stdio_usb_connected();
        if (connected && !was_connected && run.valid) {
            sleep_ms(500);
            print_summary();
            wifi_log_mirror_text(false);
            print_log();
            wifi_log_mirror_text(true);
            next_report_ms = now + REPORT_REPEAT_MS;
        }
        was_connected = connected;
        if (connected && run.valid && (int32_t)(now - next_report_ms) >= 0) {
            next_report_ms = now + REPORT_REPEAT_MS;
            print_summary();
        }

        uint16_t raw[LP_SENSORS];
        line_sensors_read(raw);
        (void)lp_update(&lp, raw, now);
        sleep_ms(BUTTON_POLL_MS);
    }
}

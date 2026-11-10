/* Exclusive operator pass. TEST20 only; normal robot has no call path. */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "../app.h"
#include "barcode_test.h"
#if APP_BARCODE_CAPTURE
#include "barcode.h"
#include "barcode_peak.h"
#include "barcode_capture.h"
#include "barcode_drive.h"
#include "barcode_hw.h"
#include "barcode_settings.h"
#include "drv_line.h"
#include "line_service.h"
#include "../platform/drv_time.h"
#include "../subsystem2/motion.h"

#if !APP_TEST || APP_BENCH || APP_IMU_STREAM || TM_MQTT || TK_SUPPORT_SMP
#error "BARCODE_CAPTURE requires exclusive single-core, non-MQTT TEST"
#endif
#if !TM_CONSOLE_USB_CDC
#error "BARCODE_CAPTURE deferred reports require CONSOLE=usb_cdc"
#endif

static const barcode_drive_config_t settings = {
    BARCODE_BOUNDED_DISTANCE_MM, BARCODE_PASS_SPEED_MM_S, BARCODE_COUNTS_PER_M};
static ID runner;
static barcode_workspace_t workspace;
static barcode_report_t decoded;
static barcode_peak_workspace_t peak_workspace;
static barcode_peak_report_t peak_report;
static barcode_result_t selected;
static uint16_t white_minimum;
static uint16_t white_maximum;
_Static_assert(sizeof(peak_workspace) + sizeof(peak_report) +
                       sizeof(workspace) + sizeof(decoded) + sizeof(selected) +
                       sizeof(barcode_config_t) +
                       sizeof(barcode_peak_config_t) +
                       BC_MAX_SAMPLES * sizeof(barcode_packed_t) +
                       BC_MAX_MOTION * sizeof(barcode_motion_t) <=
                   40000u,
               "Barcode primary storage exceeds 40,000 bytes");
static uint32_t motor_misses;
static uint32_t last_motor_us;
static uint32_t frame_count[2];

static void release(void *context)
{
    (void)context;
    (void)tk_wup_tsk(runner);
}

static bool motor_read(motion_status_t *value, uint32_t *stamp)
{
    return motion_snapshot_try_read(value, stamp) == E_OK &&
           drv_time_us() - *stamp <= 30000u;
}

typedef struct {
    int motor_rc;
    int line_rc;
    uint32_t motor_age_us;
    uint32_t line_age_us;
    unsigned int motor_fail;
    unsigned int line_fail;
    bool motor_seen;
    bool line_seen;
} health_t;

static health_t health;

/* E_TMOUT is a sequence-copy collision, not a sensor fault. Keep only the
 * last coherent board, bounded by age and three consecutive failed reads.
 * Read into static scratch so a failed call cannot damage the cached view. */
static uint32_t read_health(motion_status_t *motor, uint32_t *stamp,
                            line_snapshot_t *line, ER wake_rc, INT queued)
{
    static motion_status_t next_motor;
    static line_snapshot_t next_line;
    uint32_t next_stamp = 0u;
    health.motor_rc = motion_snapshot_try_read(&next_motor, &next_stamp);
    health.line_rc = line_snapshot_try_read(&next_line);
    if (health.motor_rc == E_OK) {
        *motor = next_motor;
        *stamp = next_stamp;
        health.motor_seen = true;
        health.motor_fail = 0u;
    } else if (health.motor_fail < BC_HEALTH_READ_LIMIT) {
        ++health.motor_fail;
    }
    if (health.line_rc == E_OK) {
        *line = next_line;
        health.line_seen = true;
        health.line_fail = 0u;
    } else if (health.line_fail < BC_HEALTH_READ_LIMIT) {
        ++health.line_fail;
    }
    const uint32_t now = drv_time_us();
    health.motor_age_us = health.motor_seen ? now - *stamp : UINT32_MAX;
    health.line_age_us =
        health.line_seen ? now - line->timestamp_us : UINT32_MAX;
    uint32_t mask = 0u;
    if (wake_rc < E_OK) {
        mask |= BC_FAULT_WAKE;
    }
    if (queued > 0) {
        mask |= BC_FAULT_QUEUED;
    }
    if (!health.motor_seen || health.motor_fail >= BC_HEALTH_READ_LIMIT ||
        (health.motor_rc != E_OK && health.motor_rc != E_TMOUT)) {
        mask |= BC_FAULT_MOTOR_READ;
    }
    if (health.motor_age_us > BC_HEALTH_MAX_AGE_US) {
        mask |= BC_FAULT_MOTOR_AGE;
    }
    if (!health.line_seen || health.line_fail >= BC_HEALTH_READ_LIMIT ||
        (health.line_rc != E_OK && health.line_rc != E_TMOUT)) {
        mask |= BC_FAULT_LINE_READ;
    }
    if (health.line_age_us > BC_HEALTH_MAX_AGE_US) {
        mask |= BC_FAULT_LINE_AGE;
    }
    if (!line->valid) {
        mask |= BC_FAULT_LINE_INVALID;
    }
    if (line->raw_bc > 4095u) {
        mask |= BC_FAULT_LINE_RAW;
    }
    if (motor->faults != 0u) {
        mask |= BC_FAULT_MOTOR_FLAGS;
    }
    /* motion_set_speed requires the already-armed global flag and never
     * clears it. A coherent false flag is NOT an arming grace period. */
    if (!motor->watchdog_armed) {
        mask |= BC_FAULT_WATCHDOG;
    }
    return mask;
}

static void fault_report(uint32_t mask, const motion_status_t *motor,
                         const line_snapshot_t *line, ER wake_rc, INT queued)
{
    tm_printf((UB *)"[BC_FAULT] mask 0x%08lx line_rc %ld line_age_us %lu "
                    "line_valid %u raw_bc %u line_fail %u motor_rc %ld "
                    "motor_age_us %lu motor_fail %u motor_faults 0x%08lx "
                    "watchdog_armed %u wake_rc %ld queued %ld\n",
              (unsigned long)mask, (long)health.line_rc,
              (unsigned long)health.line_age_us, (unsigned int)line->valid,
              (unsigned int)line->raw_bc, health.line_fail,
              (long)health.motor_rc, (unsigned long)health.motor_age_us,
              health.motor_fail, (unsigned long)motor->faults,
              (unsigned int)motor->watchdog_armed, (long)wake_rc, (long)queued);
}

static void motor_sample(const motion_status_t *value, uint32_t stamp,
                         bool force)
{
    if (force ||
        stamp - last_motor_us >= barcode_drive_motor_ms(&settings) * 1000u) {
        const barcode_motion_t row = {stamp, value->counts[0],
                                      value->counts[1]};
        barcode_capture_motor(&row);
        last_motor_us = stamp;
    }
}

static void result(const char *method, const barcode_result_t *r)
{
    tm_printf((UB *)"[BC_RESULT] FROZEN %s decoded %u text \"%s\" cmd %u "
                    "reason %s quality %u dirs %u frames %u separation %u\n",
              method, (unsigned int)r->decoded, r->text,
              (unsigned int)r->command, barcode_reason_name(r->reason),
              (unsigned int)r->quality, (unsigned int)r->directions,
              (unsigned int)r->frames, (unsigned int)r->separation_permille);
}

static void frame_result(const barcode_frame_t *frame, void *context)
{
    (void)context;
    const unsigned int method = frame->distance_method ? 1u : 0u;
    ++frame_count[method];
    tm_printf((UB *)"[BC_FRAME] %s index %lu text \"%s\" cmd %u quality %u "
                    "dirs %u start_us %lu end_us %lu start_um %ld end_um %ld "
                    "separation %u\n",
              method ? "DISTANCE" : "TIME", (unsigned long)frame_count[method],
              frame->value.text, (unsigned int)frame->value.command,
              (unsigned int)frame->value.quality,
              (unsigned int)frame->value.directions,
              (unsigned long)frame->start_us, (unsigned long)frame->end_us,
              (long)frame->start_um, (long)frame->end_um,
              (unsigned int)frame->value.separation_permille);
}

static void peak_frame(const barcode_frame_t *frame, void *context)
{
    (void)context;
    tm_printf((UB *)"[BC_PEAK_FRAME] %s text \"%s\" dirs %u start_us %lu "
                    "end_us %lu start_um %ld end_um %ld\n",
              frame->distance_method ? "DISTANCE" : "TIME", frame->value.text,
              (unsigned int)frame->value.directions,
              (unsigned long)frame->start_us, (unsigned long)frame->end_us,
              (long)frame->start_um, (long)frame->end_um);
}

static void peak_meta(const char *name, const barcode_peak_method_t *r)
{
    /* Kernel printf does not support floats. All values are milli-counts,
     * except geometry error (permille of the fitted physical narrow). */
    tm_printf((UB *)"[BC_PEAK_META] %s peaks %u baseline_milli %lu "
                    "noise_milli %lu gate_milli %lu min_prom_milli %lu "
                    "band_margin_milli %lu geometry_permille %lu\n",
              name, (unsigned int)r->peaks,
              (unsigned long)(r->baseline * 1000.0f),
              (unsigned long)(r->noise_sd * 1000.0f),
              (unsigned long)(r->threshold * 1000.0f),
              (unsigned long)(r->minimum_prominence * 1000.0f),
              (unsigned long)(r->amplitude_margin * 1000.0f),
              (unsigned long)(r->geometry_error * 1000.0f));
}

/* Keep PEAK-only locals out of the WIDTH call chain on the 1 KiB task. */
static void __attribute__((noinline))
decode_peaks(bool lost, const barcode_config_t *config)
{
    size_t n, m;
    const barcode_packed_t *rows = barcode_capture_rows(&n);
    const barcode_motion_t *motors = barcode_capture_motors(&m);
    static barcode_peak_config_t peak_config;
    peak_config.capture = *config;
    peak_config.speed_mm_s = BARCODE_PASS_SPEED_MM_S;
    peak_config.white_min = white_minimum;
    peak_config.white_max = white_maximum;
    barcode_peak_begin(NULL, lost ? 0u : n, rows, barcode_capture_origin(),
                       motors, m, &peak_config, &peak_workspace, &peak_report);
    barcode_peak_sink(&peak_workspace, peak_frame, NULL);
    uint32_t maximum = 0u, total = 0u;
    bool done;
    do {
        const uint32_t start = drv_time_us();
        done = barcode_peak_step(&peak_workspace);
        const uint32_t cost = drv_time_us() - start;
        total += cost;
        if (cost > maximum) {
            maximum = cost;
        }
        (void)tk_dly_tsk(1u);
        (void)tk_can_wup(TSK_SELF);
    } while (!done);
    if (lost) {
        peak_report.time.value = (barcode_result_t){.reason = BC_SAMPLE_LOSS};
        peak_report.distance.value = peak_report.time.value;
    }
    peak_meta("TIME", &peak_report.time);
    peak_meta("DISTANCE", &peak_report.distance);
    result("PEAK_TIME", &peak_report.time.value);
    result("PEAK_DISTANCE", &peak_report.distance.value);
    barcode_peak_select(&decoded, &peak_report, &selected);
    result("SELECTED", &selected);
    const char *choice = selected.reason == BC_AMBIGUOUS  ? "CONFLICT"
                         : !selected.decoded              ? "NONE"
                         : decoded.time.decoded           ? "WIDTH_TIME"
                         : decoded.distance.decoded       ? "WIDTH_DISTANCE"
                         : peak_report.time.value.decoded ? "PEAK_TIME"
                                                          : "PEAK_DISTANCE";
    tm_printf((UB *)"[BC_SELECT] %s; accepted payloads must agree; data only\n",
              choice);
    tm_printf((UB *)"[BC_PEAK_COST] step_max_us %lu total_us %lu\n",
              (unsigned long)maximum, (unsigned long)total);
}

static void decode(bool lost)
{
    size_t n;
    size_t m;
    const barcode_packed_t *rows = barcode_capture_rows(&n);
    const barcode_motion_t *motors = barcode_capture_motors(&m);
    /* One owner, after freeze; keep parameters off the 1 KiB task stack. */
    static barcode_config_t config;
    config = barcode_default_config;
    config.max_gap_us = barcode_drive_adc_ms(&settings) * 1000u + 1500u;
    /* Two busy polls plus a source/poll phase offset can delay retention
     * by three owner periods; this is not a fabricated source timestamp. */
    config.motion_gap_us = barcode_drive_motor_ms(&settings) * 1000u +
                           BC_HEALTH_READ_LIMIT * 10000u;
    if (barcode_capture_motor_overflow() != 0u) {
        config.counts_per_m = 0u;
    }
    barcode_begin_packed(lost ? NULL : rows, n, barcode_capture_origin(),
                         motors, m, &config, &workspace, &decoded);
    frame_count[0] = 0u;
    frame_count[1] = 0u;
    barcode_frame_sink(&workspace, frame_result, NULL);
    uint32_t maximum = 0u;
    uint32_t total = 0u;
    bool done;
    do {
        const uint32_t start = drv_time_us();
        done = barcode_step(&workspace);
        const uint32_t cost = drv_time_us() - start;
        total += cost;
        if (cost > maximum) {
            maximum = cost;
        }
        (void)tk_dly_tsk(1u);
        (void)tk_can_wup(TSK_SELF);
    } while (!done);
    if (lost) {
        decoded.time = (barcode_result_t){.reason = BC_SAMPLE_LOSS};
        decoded.distance = decoded.time;
    }
    tm_printf((UB *)"[BC_META] raw %u..%u contrast %u runs %u "
                    "decode_step_max_us %lu decode_total_us %lu\n",
              (unsigned int)decoded.minimum, (unsigned int)decoded.maximum,
              (unsigned int)decoded.contrast, (unsigned int)decoded.runs,
              (unsigned long)maximum, (unsigned long)total);
    result("TIME", &decoded.time);
    result("DISTANCE", &decoded.distance);
    decode_peaks(lost, &config);
}

/* Confirmed zero drive and frozen data precede this wait. Never poll the
 * run buttons here: presses cannot queue a pass. READY resynchronizes them.
 * Keep yielding so the lower-priority USB task can enumerate/open. */
static __attribute__((noinline)) void wait_for_usb(void)
{
    uint32_t since = 0u;
    bool connected = false;
    for (;;) {
        const bool ready = barcode_hw_usb_ready();
        const uint32_t now = drv_time_us();
        if (!ready) {
            connected = false;
        } else if (!connected) {
            since = now;
            connected = true;
        } else if (now - since >= 1000000u) {
            return;
        }
        (void)tk_dly_tsk(20u);
        (void)tk_can_wup(TSK_SELF);
    }
}

static void dump_button(bool *previous, bool *notified)
{
    const bool down = barcode_hw_drive_down();
    if (down && !*previous && !*notified) {
        tm_printf(
            (UB *)"[BC] GP21 ignored: dump in progress; wait for READY\n");
        *notified = true;
    }
    *previous = down;
}

static void dump(void)
{
    size_t n;
    size_t m;
    const barcode_packed_t *rows = barcode_capture_rows(&n);
    const barcode_motion_t *motors = barcode_capture_motors(&m);
    uint32_t us = barcode_capture_origin();
    bool previous = barcode_hw_drive_down();
    bool notified = false;
    const unsigned long total = (unsigned long)(n + m + decoded.runs);
    /* First hardware dump: ~40 s / 3209 rows, including tick rounding.
     * Estimate only; a slow/disconnected host can take longer. */
    tm_printf((UB *)"[BC] DUMP %lu rows ~%lu s: GP21 ignored until READY\n",
              total, (total * 13u + 999u) / 1000u);
    tm_printf((UB *)"[BC_DUMP] BEGIN adc %lu motor %lu origin_us %lu "
                    "adc_keep_ms %lu motor_keep_ms %lu; verify row counts\n",
              (unsigned long)n, (unsigned long)m, (unsigned long)us,
              (unsigned long)barcode_drive_adc_ms(&settings),
              (unsigned long)barcode_drive_motor_ms(&settings));
    tm_printf((UB *)"BC_ADC,index,us,raw_bc,raw_flags,delta_us\n");
    for (size_t i = 0u; i < n; ++i) {
        dump_button(&previous, &notified);
        us += rows[i].delta_us;
        tm_printf((UB *)"BC_ADC,%lu,%lu,%u,%u,%u\n", (unsigned long)i,
                  (unsigned long)us,
                  (unsigned int)(rows[i].raw_flags & BC_PACKED_RAW),
                  (unsigned int)(rows[i].raw_flags & UINT16_C(0xf000)),
                  (unsigned int)rows[i].delta_us);
        (void)tk_dly_tsk(1u); /* Stopped: let lower-priority USB drain. */
    }
    tm_printf((UB *)"BC_MOTOR,index,us,left_counts,right_counts\n");
    for (size_t i = 0u; i < m; ++i) {
        dump_button(&previous, &notified);
        tm_printf((UB *)"BC_MOTOR,%lu,%lu,%ld,%ld\n", (unsigned long)i,
                  (unsigned long)motors[i].us, (long)motors[i].left,
                  (long)motors[i].right);
        if (i % 16u == 15u) {
            (void)tk_dly_tsk(1u);
        }
    }
    tm_printf((UB *)"BC_RUN,index,time_us,distance_um_or_INT32_MIN\n");
    for (size_t i = 0u; i < decoded.runs; ++i) {
        dump_button(&previous, &notified);
        const float um = workspace.distance_mm[i] * 1000.0f;
        const int32_t distance = decoded.distance.reason != BC_MOTION &&
                                         um > -2147483648.0f &&
                                         um < 2147483648.0f
                                     ? (int32_t)um
                                     : INT32_MIN;
        tm_printf((UB *)"BC_RUN,%lu,%lu,%ld\n", (unsigned long)i,
                  (unsigned long)workspace.time_us[i], (long)distance);
        if (i % 16u == 15u) {
            (void)tk_dly_tsk(1u);
        }
    }
    tm_printf((UB *)"[BC_DUMP] END adc %lu motor %lu\n", (unsigned long)n,
              (unsigned long)m);
}

typedef struct {
    uint32_t changed;
    bool sample;
    bool stable;
    bool armed;
} button_t;

static button_t button_init(bool down)
{
    return (button_t){drv_time_ms(), down, down, !down};
}

static bool press(button_t *button, bool down)
{
    const uint32_t now = drv_time_ms();
    if (down != button->sample) {
        button->sample = down;
        button->changed = now;
    }
    if (now - button->changed < 30u || button->stable == down) {
        return false;
    }
    button->stable = down;
    if (!down) {
        button->armed = true;
        return false;
    }
    if (!button->armed) {
        return false;
    }
    button->armed = false;
    return true;
}

typedef struct {
    bool pending;
    bool accepted;
    bool used_estop;
    uint32_t after_us;
    uint32_t last_stamp;
    uint32_t last_notice_ms;
    unsigned int settled;
} stop_t;

/* Adapted from the checked stop_pending idea in the IMU stream, not linked
 * to S4. API success alone is not a stopped-wheel observation: require two
 * distinct fresh post-request snapshots with zero duty and speed <=10. */
static bool confirm_stop(stop_t *stop, const motion_status_t *motor, bool valid,
                         uint32_t stamp)
{
    bool accepted = motion_stop() >= E_OK;
    if (!accepted) {
        accepted = motion_estop() >= E_OK;
        stop->used_estop = true;
    }
    if (accepted && !stop->accepted) {
        stop->after_us = drv_time_us();
        stop->accepted = true;
    } else if (!accepted) {
        stop->accepted = false;
        stop->settled = 0u;
    }
    if (stop->accepted && valid && stamp != stop->last_stamp &&
        stamp != stop->after_us &&
        stamp - stop->after_us < UINT32_C(0x80000000)) {
        stop->last_stamp = stamp;
        const bool zero =
            motor->duty_permille[0] == 0 && motor->duty_permille[1] == 0 &&
            motor->speed_mm_s[0] >= -10 && motor->speed_mm_s[0] <= 10 &&
            motor->speed_mm_s[1] >= -10 && motor->speed_mm_s[1] <= 10;
        stop->settled = zero ? stop->settled + 1u : 0u;
        if (stop->settled >= 2u) {
            stop->pending = false;
            return true;
        }
    }
    if (drv_time_ms() - stop->last_notice_ms >= 1000u) {
        stop->last_notice_ms = drv_time_ms();
        tm_printf((UB *)"[BC_DRIVE] STOP UNCONFIRMED; retrying. "
                        "Remove VIN if wheels move. No rearm or dump.\n");
    }
    return false;
}

static void frozen_report(const barcode_drive_t *job,
                          const drv_line_stats_t *initial,
                          const drv_line_stats_t *current,
                          uint32_t observer_misses)
{
    const bool lost = current->missed != initial->missed ||
                      current->dropped != initial->dropped ||
                      barcode_capture_gaps() != 0u ||
                      barcode_capture_dropped() != 0u || barcode_capture_full();
    tm_printf((UB *)"[BC_DRIVE] STOP CONFIRMED direction %s reason %s "
                    "travel_mm %ld left_mm %ld right_mm %ld; decoding; "
                    "GP21 ignored until READY\n",
              job->backward ? "BACKWARD" : "FORWARD",
              barcode_drive_reason_name(job->reason), (long)job->travel_mm,
              (long)job->wheel_mm[0], (long)job->wheel_mm[1]);
    decode(lost);
    tm_printf((UB *)"[BC_STATS] missed %lu dropped %lu gaps %lu adc_full %u "
                    "capture_dropped %lu motor_misses %lu motion_overflow %lu "
                    "observer_misses %lu hook_max_us %lu\n",
              (unsigned long)(current->missed - initial->missed),
              (unsigned long)(current->dropped - initial->dropped),
              (unsigned long)barcode_capture_gaps(),
              (unsigned int)barcode_capture_full(),
              (unsigned long)barcode_capture_dropped(),
              (unsigned long)motor_misses,
              (unsigned long)barcode_capture_motor_overflow(),
              (unsigned long)observer_misses,
              (unsigned long)barcode_capture_max_us());
    static line_snapshot_t line;
    if (line_snapshot_try_read(&line) == E_OK) {
        tm_printf(
            (UB *)"[BC_LINE] isr_max_us %lu isr_total_us %lu "
                  "line_sample_max_us %lu line_total_us %lu processed %lu\n",
            (unsigned long)current->isr_max_us, (unsigned long)current->isr_us,
            (unsigned long)line.sample_max_us, (unsigned long)line.cpu_us,
            (unsigned long)line.processed);
    }
    dump();
}

void barcode_test(void)
{
    static const char *const names[] = {"FLOOR", "WHITE_PAPER", "BLACK_BAR"};
    const bool configuration_ok = barcode_drive_config_valid(&settings);
    barcode_hw_button_init();
    /* Initial TEST task has only 1 KiB stack. Keep owner buffers static;
     * no new task or template stack-size deviation is needed. */
    static button_t range_button;
    static button_t drive_button;
    range_button = button_init(barcode_hw_button_down());
    drive_button = button_init(barcode_hw_drive_down());
    runner = tk_get_tid();
    static const T_CCYC config = {
        .cycatr = TA_HLNG, .cychdr = (FP)release, .cyctim = 10u, .cycphs = 10u};
    const ID cyclic = tk_cre_cyc(&config);
    const bool cyclic_ok = cyclic > 0 && tk_sta_cyc(cyclic) >= E_OK;
    static barcode_drive_t job;
    static stop_t stop;
    job = (barcode_drive_t){.inhibited = !configuration_ok};
    stop = (stop_t){.pending = true};
    bool capturing = false;
    bool range_pending = false;
    uint32_t range_ms = 0u;
    uint32_t observer_misses = 0u;
    unsigned int stage = 0u;
    static drv_line_stats_t initial;
    static drv_line_stats_t current;
    static motion_status_t motor;
    static line_snapshot_t line;
    initial = (drv_line_stats_t){0};
    health = (health_t){0};
    static uint32_t stamp;
    stamp = 0u;
    tm_printf(
        (UB *)"[BC] B1c WIDTH+PEAK TEST ONLY: VIN REQUIRED. Lift wheels "
              "for first GP21 check. One F sheet per run; swap when idle.\n");
    tm_printf((UB *)"[BC] Frozen report waits for USB terminal DTR + 1 s. "
                    "Keep VIN on; GP21 ignored until READY.\n");
    tm_printf((UB *)"[BC] GP20: floor/white/black ranges, then redo ranges. "
                    "GP21: forward, then backward; while moving = STOP.\n");
    tm_printf(
        (UB *)"[BC] pass_mm %lu speed_mm_s %lu timeout_ms %lu "
              "adc_keep_ms %lu motor_keep_ms %lu; commands are data only\n",
        (unsigned long)settings.distance_mm, (unsigned long)settings.speed_mm_s,
        (unsigned long)barcode_drive_timeout_ms(&settings),
        (unsigned long)barcode_drive_adc_ms(&settings),
        (unsigned long)barcode_drive_motor_ms(&settings));
    tm_printf((UB *)"[BC] health_age_limit_us %lu read_fail_limit %u; "
                    "BC_FAULT masks enabled\n",
              (unsigned long)BC_HEALTH_MAX_AGE_US,
              (unsigned int)BC_HEALTH_READ_LIMIT);
    for (;;) {
        const ER wake_rc = cyclic_ok ? tk_slp_tsk(100u) : E_SYS;
        if (wake_rc < E_OK) {
            (void)tk_dly_tsk(10u);
        }
        const INT missed = tk_can_wup(TSK_SELF);
        const bool range_press = press(&range_button, barcode_hw_button_down());
        const bool drive_press = press(&drive_button, barcode_hw_drive_down());
        uint32_t fault_mask =
            read_health(&motor, &stamp, &line, wake_rc, missed);
        const bool motor_ok =
            (fault_mask & (BC_FAULT_MOTOR_READ | BC_FAULT_MOTOR_AGE)) == 0u;
        drv_line_stats(&current);
        if (capturing) {
            if (motor_ok) {
                motor_sample(&motor, stamp, false);
                barcode_drive_observe(&job, &settings, motor.counts[0],
                                      motor.counts[1]);
            }
            if (!motor_ok || health.motor_rc != E_OK) {
                ++motor_misses;
            }
            if (missed > 0) {
                observer_misses += (uint32_t)missed;
            }
        }
        if (stop.pending) {
            if (confirm_stop(&stop, &motor, motor_ok, stamp)) {
                job.inhibited = job.inhibited || stop.used_estop || !cyclic_ok;
                if (capturing) {
                    barcode_drive_observe(&job, &settings, motor.counts[0],
                                          motor.counts[1]);
                    barcode_capture_freeze();
                    (void)tk_dly_tsk(20u);
                    if (motor_read(&motor, &stamp)) {
                        motor_sample(&motor, stamp, true);
                        barcode_drive_observe(&job, &settings, motor.counts[0],
                                              motor.counts[1]);
                    } else {
                        ++motor_misses;
                    }
                    if (job.overrun) {
                        barcode_drive_cancel(&job, BC_DRIVE_OVERRUN);
                    }
                    wait_for_usb();
                    frozen_report(&job, &initial, &current, observer_misses);
                    capturing = false;
                    (void)tk_can_wup(TSK_SELF);
                    drive_button = button_init(barcode_hw_drive_down());
                    range_button = button_init(barcode_hw_button_down());
                }
                tm_printf((UB *)"[BC] STOP CONFIRMED; next %s; inhibited %u. "
                                "Save dump. GP20 ranges / GP21 pass.\n",
                          job.next_backward ? "BACKWARD" : "FORWARD",
                          (unsigned int)job.inhibited);
                if (stage == 3u) {
                    if (job.inhibited) {
                        tm_printf((UB *)"[BC] NOT READY: GP21 inhibited; "
                                        "reboot after fault\n");
                    } else {
                        tm_printf((UB *)"[BC] READY: GP21 = %s\n",
                                  job.next_backward ? "BACKWARD" : "FORWARD");
                    }
                }
            }
            continue;
        }
        if (job.active) {
            if (current.missed != initial.missed ||
                current.dropped != initial.dropped) {
                fault_mask |= BC_FAULT_ACQUISITION;
            }
            if (barcode_capture_gaps() != 0u) {
                fault_mask |= BC_FAULT_ADC_GAP;
            }
            if (barcode_capture_motor_overflow() != 0u) {
                fault_mask |= BC_FAULT_MOTION_FULL;
            }
            const int16_t speed = barcode_drive_step(
                &job, &settings, drv_time_ms(), motor.counts[0],
                motor.counts[1], drive_press, fault_mask != 0u);
            if (barcode_capture_full() || barcode_capture_dropped() != 0u) {
                barcode_drive_cancel(&job, BC_DRIVE_CAPACITY);
            }
            if (job.active && motion_set_speed(speed, speed) < E_OK) {
                barcode_drive_cancel(&job, BC_DRIVE_COMMAND);
            }
            if (!job.active) {
                stop = (stop_t){.pending = true};
                (void)confirm_stop(&stop, &motor, motor_ok, stamp);
                /* Stop request precedes diagnostics, even on API failure. */
                if (fault_mask != 0u) {
                    fault_report(fault_mask, &motor, &line, wake_rc, missed);
                }
                /* Even immediate API success needs later zero snapshots. */
            }
            continue;
        }
        if (stage < 3u || (range_press && !range_pending)) {
            if (stage == 3u) {
                stage = 0u;
            }
            if (range_press && !range_pending) {
                barcode_range_begin();
                range_ms = drv_time_ms();
                range_pending = true;
                tm_printf((UB *)"[BC] measuring %s for NEXT 1s; hold still\n",
                          names[stage]);
            }
            if (range_pending && drv_time_ms() - range_ms >= 1000u) {
                barcode_range_t range;
                barcode_range_end(&range);
                range_pending = false;
                tm_printf((UB *)"[BC_RANGE] %s n %lu raw %u..%u\n",
                          names[stage], (unsigned long)range.samples,
                          (unsigned int)range.minimum,
                          (unsigned int)range.maximum);
                if (range.samples >= 500u) {
                    if (stage == 0u) {
                        white_minimum = range.minimum;
                        white_maximum = range.maximum;
                    } else if (stage == 1u) {
                        if (range.minimum < white_minimum) {
                            white_minimum = range.minimum;
                        }
                        if (range.maximum > white_maximum) {
                            white_maximum = range.maximum;
                        }
                    }
                    ++stage;
                }
                tm_printf((UB *)"[BC] next %s\n",
                          stage < 3u ? names[stage] : "GP21 driven pass");
            }
            continue;
        }
        if (drive_press) {
            if (motor.duty_permille[0] != 0 || motor.duty_permille[1] != 0 ||
                motor.speed_mm_s[0] < -10 || motor.speed_mm_s[0] > 10 ||
                motor.speed_mm_s[1] < -10 || motor.speed_mm_s[1] > 10) {
                fault_mask |= BC_FAULT_START_MOVING;
            }
            if (!configuration_ok) {
                fault_mask |= BC_FAULT_CONFIG;
            }
            if (job.inhibited) {
                fault_mask |= BC_FAULT_INHIBITED;
            }
            const int16_t speed = barcode_drive_step(
                &job, &settings, drv_time_ms(), motor.counts[0],
                motor.counts[1], true, fault_mask != 0u);
            if (!job.active) {
                stop = (stop_t){.pending = true};
                (void)confirm_stop(&stop, &motor, motor_ok, stamp);
                fault_report(fault_mask, &motor, &line, wake_rc, missed);
                tm_printf((UB *)"[BC_DRIVE] START REFUSED reason %s; reboot "
                                "after a fault, never bypass it\n",
                          barcode_drive_reason_name(job.reason));
                continue;
            }
            initial = current;
            observer_misses = 0u;
            motor_misses = 0u;
            barcode_capture_begin(barcode_drive_adc_ms(&settings));
            motor_sample(&motor, stamp, true);
            capturing = true;
            if (motion_set_speed(speed, speed) < E_OK) {
                barcode_drive_cancel(&job, BC_DRIVE_COMMAND);
                stop = (stop_t){.pending = true};
                (void)confirm_stop(&stop, &motor, motor_ok, stamp);
            }
        }
    }
}
#endif

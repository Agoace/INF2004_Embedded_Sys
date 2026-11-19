/* Kernel navigation only: snapshot inputs, asynchronous services, motion outputs. */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>

/* Task timing and resource limits. */
#define VEHICLE_RESPONSE_LIMIT_US 10000u
#define LINE_MAX_AGE_US           10000u
#define VEHICLE_WAKE_TIMEOUT_MS   20
/* P1: a motion-stamp -> running gap above this is not one wake. */
#define VEHICLE_WAKE_LIMIT_US     100000u
#define BUTTON_INTERVAL_MS        20u
#define RUN_BUTTON_PIN            21u
#define DUMP_BUTTON_PIN           20u
#define LOG_PERIOD_MS             20u
#define SONAR_MODE_UNSET          255u
#define SONAR_PING_MIN_MS         60u
#define SONAR_REQUEST_DEADLINE_MS 100u
#define SONAR_WATCH_SLACK_MS      20u
#define VEHICLE_TASK_PRIORITY     13
#define ROW_FAULT_FLAG            128u

#include "../app.h"
#include "vehicle_task.h"
#include "imu_gate.h"
#include "../subsystem2/motion.h"
#include "../subsystem2/motion_diag.h"
#include "../subsystem2/drv_motor_hw.h"
#include "../subsystem3/line_service.h"
#include "../subsystem5/sonar_service.h"
#include "../subsystem1/telemetry.h"
#include "../platform/drv_time.h"
#include "../platform/run_timing.h"
#include "../platform/kernel_lock_stats.h"
#include "vehicle_inputs.h"
#include "vehicle_robot_config.h"
#include "vehicle_log.h"
#include "junction_line.h"
#include "junction_source.h"
#include "../subsystem3/barcode_live_source.h"
#include "vehicle_board.h"
#include "vehicle_pack.h"
#include "vehicle_fault.h"
#include "obstacle_report.h"
#include "obstacle_pack.h"
#include "run_stats.h"
#include "../platform/app_board.h"
#include "../subsystem4/imu_service.h"
#if TM_MQTT
#include "../subsystem1/robot_network.h"
#include "../subsystem1/robot_health.h"
#include "../subsystem1/network_queue.h"
#include "../subsystem2/motor_ctl_robot.h"
#endif
#if TM_WIFI_CYW43
#include "../../lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.h"
#endif
#if !APP_TEST && !APP_BENCH
#define START_DELAY_MS 1000u
#define RUN_LIMIT_MS   90000u
static lp_t lp;
static line_snapshot_t sensor;
static lt_t lt;
static jl_t junction_line;
static uint8_t junction_reported;
#if TM_MQTT
static uint8_t junction_angles_reported;
#endif
static ls_t ls;
static ss_t ss;
static dr_t dr;
static vsm_t vsm;
static fault_mask_board_t vehicle_fault_board;
static void publish_vehicle_faults(void);
static uint16_t raw[3];
static vc_event_t line_event;
static bool active;
static bool cancelled;
static bool prev_run;
static bool prev_dump;
static bool step_open;
/* Single writer; odd throughout line calibration, even otherwise.
 * Observer only. No IMU result can affect navigation or motion. */
static uint32_t calibration_epoch;
/* Single writer; odd from run start until the hump values are taken. */
static uint32_t run_epoch;
static uint32_t line_run_epoch;
static imu_hump_diag_board_t diag_copy;
static bool have_diag_copy;
static imu_hump_diag_board_t run_diag;
static APP_BOARD(imu_hump_diag_board_t) run_diag_board;
/* Newest IMU hump board copy and the result of the last finished run. */
static imu_hump_board_t hump_copy;
static bool have_hump_copy;
static imu_hump_summary_t run_hump;
static APP_BOARD(imu_hump_summary_t) run_hump_board;
static uint32_t wake_lost;
static uint32_t overruns;
static uint32_t response_max_us;
static uint32_t release_us;
static uint32_t release_tick;
/* P1 report-only step timing; Vehicle is the only writer. */
static uint32_t part_mark_us;  /* end of the previous step part */
static uint32_t part_slow_us;  /* slowest part of the open step */
static rt_max_t part_slow;
static uint32_t step_wake_us;  /* wake latency of the open step */
static uint32_t run_steps;     /* steps released in the current run */
static uint32_t bkl_reset_ms;  /* when the lock statistics were cleared */
static uint32_t run_start;
static uint32_t run_button_ms;
static uint32_t generation;
static uint32_t request_id;
static drv_line_stats_t line_base;
static const char *reason = "no saved run";
#if TM_MQTT
/* Why the run ended, as a robot/summary stop reason (RM_STOP_*). */
static uint8_t stop_code;
#define set_reason(code, text)                                                 \
    do {                                                                       \
        reason = (text);                                                       \
        stop_code = (code);                                                    \
    } while (0)
#else
#define set_reason(code, text) (reason = (text))
#endif
#if TM_MQTT
static APP_BOARD(vehicle_packed_t) vehicle_board;
static vehicle_snapshot_t vehicle;
/* CR1 A: 6-word packed layout, see obstacle_pack.h. */
static obstacle_board_t obstacle_board;
static uint32_t obstacle_scans;
static APP_BOARD(rm_detour_t) detour_board;
static APP_BOARD(rm_summary_t) summary_board;
static rs_t run_stats;
static barcode_live_counts_t run_barcode;
static bool have_run_barcode;

/* The LINE writer owns epoch counters. Never read its mutable producer. */
static void barcode_poll(uint32_t epoch)
{
    barcode_live_counts_t copy;
    if (barcode_live_counts_try_read(&copy) == E_OK && copy.epoch == epoch) {
        run_barcode = copy;
        have_run_barcode = true;
    }
}

static uint8_t pending_action;
static uint32_t event_sequence;
static uint32_t seen_commands[8];
static unsigned int seen_count;
static unsigned int seen_next;
static uint16_t last_network_fault;

/*
 * Queue a rare vehicle event without waiting for the network.
 */
static void live_event(uint8_t kind, uint8_t from, uint8_t to, uint16_t word,
                       uint8_t detail)
{
    const bool recovery = kind == 3u && (word & 0x8000u) != 0u;
    if (kind == 3u && !recovery && word == last_network_fault) {
        return;
    }
    if (kind == 3u && !recovery) {
        last_network_fault = word;
    }
    const rm_event_t e = {.kind = kind,
                          .from = from,
                          .to = to,
                          .word = word,
                          .detail = detail,
                          .uptime_ms = drv_time_ms(),
                          .sequence = ++event_sequence};
    (void)robot_event_push(&e); /* Never affect driving on a network loss. */
}

/*
 * Publish the last vehicle state and command for bounded readers.
 */
static void publish_vehicle(void)
{
    publish_vehicle_faults();
    vehicle.timestamp_us = drv_time_us();
    ++vehicle.sequence;
    vehicle.state = vsm.state;
    vehicle.module = vsm.running;
    vehicle.active = active;
    vehicle.calibrated = lp_calibrated(&lp);
    vehicle.handoffs = vsm.handoffs;
    vehicle.wake_lost = wake_lost;
    vehicle.overruns = overruns;
    vehicle.response_max_us = response_max_us;
    vehicle_packed_t packed;
    if (vehicle_pack(&packed, &vehicle)) {
        app_board_publish(&vehicle_board.sequence, vehicle_board.words, &packed,
                          sizeof(packed));
    }
}

/*
 * Attempt one read of the vehicle public board without waiting.
 */
int vehicle_snapshot_try_read(vehicle_snapshot_t *out)
{
    if (!out) {
        return E_PAR;
    }
    vehicle_packed_t packed;
    if (!app_board_try_read(&vehicle_board.sequence, vehicle_board.words, &packed,
                            sizeof(packed)) || !vehicle_unpack(out, &packed)) {
        return E_TMOUT;
    }
    return E_OK;
}

/*
 * Publish the profile of a scan that has just completed. Called from the
 * SCAN module wrapper, so it runs in the vehicle task at the moment the SIDE
 * outcome is produced.
 */
static void publish_obstacle(const ss_t *scan)
{
    rm_obstacle_t report;
    const obstacle_side_t side =
        obstacle_profile_side(scan->word, &scan->profile);

    ++obstacle_scans;
    obstacle_report_make(&scan->profile, scan->word, side, obstacle_scans,
                         drv_time_ms(), &report);
    report.flags = (uint8_t)(report.flags |
                             (scan->stage << RM_OBSTACLE_STAGE_SHIFT) |
                             ((uint8_t)scan->completed_attempt
                              << RM_OBSTACLE_ATTEMPT_SHIFT));
    /* The report comes from obstacle_report_make(); it always packs. */
    (void)obstacle_board_publish(&obstacle_board, &report);
    telemetry_message_t message = {.kind = TELEMETRY_SCAN};
    message.data.scan = (telemetry_scan_t){
        .scan_id = obstacle_scans, .duration_ms = scan->duration_ms,
        .fine_mask = scan->fine_mask, .coarse_hits = scan->coarse_hits,
        .attempt = scan->completed_attempt};
    /* Bounded value copy; print at dump only. */
    (void)telemetry_send(&message);
}

/* Report one scan retry edge. It is an observation only. */
static void scan_retry_event(const ss_t *scan)
{
    const uint16_t word = (uint16_t)(scan->attempt |
                                     ((uint16_t)RM_SCAN_RETRY_NO_SIDE << 8u));

    live_event(RM_EVENT_SCAN_RETRY, VSM_MOD_SCAN, VSM_MOD_SCAN, word,
               scan->notice);
    if (scan->notice == SS_NOTICE_REVERSE_BEGIN) {
        rs_scan_reattempt(&run_stats);
    }
}

/*
 * SCAN module step. It publishes every completed profile and every retry edge.
 */
static vsm_outcome_t scan_step_observed(void *ctx, const vsm_inputs_t *in,
                                        uint32_t now_ms, vsm_drive_t *drive)
{
    ss_t *scan = ctx;
    const uint16_t completed = scan->completed_scans;
    const vsm_outcome_t outcome = ss_step(ctx, in, now_ms, drive);

    if (scan->completed_scans != completed) {
        publish_obstacle(scan);
    }
    if (scan->notice != SS_NOTICE_NONE) {
        scan_retry_event(scan);
    }
    return outcome;
}

/* Publish a finished detour summary (observer output only). */
static void publish_detour(const rm_detour_t *detour)
{
    app_board_publish(&detour_board.sequence, detour_board.words, detour,
                      sizeof(*detour));
}

/* Publish the summary of the run that has just ended. */
static void publish_run_summary(void)
{
    rm_summary_t summary;
    rm_detour_t aborted;

    if (rs_run_end(&run_stats, drv_time_ms(), stop_code, vsm.fault,
                   vsm.fault_detail, &summary, &aborted)) {
        publish_detour(&aborted);
    }
    if (have_run_barcode) {
        summary.flags |= RM_SUMMARY_BARCODE_IMPLEMENTED;
        summary.barcode_ok = (uint8_t)(run_barcode.frames > UINT8_MAX ?
                                      UINT8_MAX : run_barcode.frames);
        summary.barcode_failed = (uint8_t)(run_barcode.failures > UINT8_MAX ?
                                          UINT8_MAX : run_barcode.failures);
    }
    summary.hump_peak = run_hump.peak_01mm;
    summary.hump_count = run_hump.count;
    summary.hump_flags = run_hump.flags;
    summary.hump_gated = run_hump.gated;
    app_board_publish(&summary_board.sequence, summary_board.words, &summary,
                      sizeof(summary));
}

/* Copy the newest obstacle report; see obstacle_report.h. */
int vehicle_obstacle_try_read(rm_obstacle_t *out)
{
    rm_obstacle_t report;

    if (!out) {
        return E_PAR;
    }
    if (!obstacle_board_try_read(&obstacle_board, &report)) {
        return (obstacle_board.sequence == 0u) ? E_NOEXS : E_TMOUT;
    }
    *out = report;
    return E_OK;
}

/* Copy the newest detour summary; see run_stats.h. */
int vehicle_detour_try_read(rm_detour_t *out)
{
    rm_detour_t value;

    if (!out) {
        return E_PAR;
    }
    if (!app_board_try_read(&detour_board.sequence, detour_board.words, &value,
                            sizeof(value))) {
        return (detour_board.sequence == 0u) ? E_NOEXS : E_TMOUT;
    }
    *out = value;
    return E_OK;
}

/* Copy the newest run summary; see run_stats.h. */
int vehicle_summary_try_read(rm_summary_t *out)
{
    rm_summary_t value;

    if (!out) {
        return E_PAR;
    }
    if (!app_board_try_read(&summary_board.sequence, summary_board.words,
                            &value, sizeof(value))) {
        return (summary_board.sequence == 0u) ? E_NOEXS : E_TMOUT;
    }
    *out = value;
    return E_OK;
}

/*
 * Record the commanded wheel speeds, then call the motion owner.
 */
static int vehicle_speed(int16_t l, int16_t r)
{
    const int er = motion_set_speed(l, r);
    if (er >= E_OK) {
        vehicle.left_mm_s = l;
        vehicle.right_mm_s = r;
    }
    return er;
}

/*
 * Clear the observed wheel command and stop through the motion owner.
 */
static int vehicle_stop(void)
{
    vehicle.left_mm_s = 0;
    vehicle.right_mm_s = 0;
    return motion_stop();
}

/*
 * Record the accepted servo command from the motion owner.
 */
static int vehicle_servo(int deg, int *accepted)
{
    int actual = 0;
    const int er = motion_set_servo(deg, &actual);
    if (er >= E_OK) {
        vehicle.servo_deg = (int8_t)actual;
    }
    if (accepted) {
        *accepted = actual;
    }
    return er;
}

/*
 * Discard queued remote commands when an emergency stop is handled.
 */
static void discard_commands(void)
{
    robot_command_t old;
    for (unsigned int i = 0u; i < 9u && robot_command_pop(&old) > 0; ++i) {
        robot_command_lost();
    }
    pending_action = 0u;
}

/*
 * Clear pending commands and request the motion-owned emergency stop.
 */
static int vehicle_estop(void)
{
    discard_commands();
    vehicle.left_mm_s = 0;
    vehicle.right_mm_s = 0;
    live_event(3u, vsm.state, vsm.running, MC_FAULT_ESTOP, 0u);
    return motion_estop();
}

/*
 * Apply remote-command policy in vehicle context, never in Wi-Fi.
 */
static void handle_command(void)
{
    robot_command_t c;
    if (robot_command_pop(&c) != (int)sizeof(c)) {
        return;
    }
    uint8_t result = 0u;
    bool duplicate = false;
    for (unsigned int i = 0u; i < seen_count; ++i) {
        if (seen_commands[i] == c.request_id) {
            duplicate = true;
        }
    }
    if (c.opcode != RM_CMD_ESTOP && duplicate) {
        result = 4u;
    } else {
        seen_commands[seen_next] = c.request_id;
        seen_next = (seen_next + 1u) % 8u;
        if (seen_count < 8u) {
            ++seen_count;
        }
        telemetry_stats_t log;
        telemetry_stats(&log);
        result = vehicle_remote_policy(c.opcode, active, pending_action != 0u,
                                       log.busy, lp_calibrated(&lp));
        if (result == 0u) {
            if (c.opcode == RM_CMD_ESTOP) {
                (void)vehicle_estop();
                if (active) {
                    cancelled = true;
                    set_reason(RM_STOP_MQTT_ESTOP, "remote stop (best effort)");
                }
            } else if (c.opcode == RM_CMD_DUMP) {
                if (robot_network_dump(c.request_id) < E_OK) {
                    result = 3u;
                }
            } else {
                pending_action = c.opcode;
            }
        }
    }
    vehicle.command_id = c.request_id;
    vehicle.command_opcode = c.opcode;
    vehicle.command_status = result;
    live_event(6u, vsm.state, vsm.running,
               (uint16_t)((uint16_t)c.opcode | ((uint16_t)result << 8u)),
               result);
}
#else
#define vehicle_speed     motion_set_speed
#define vehicle_stop      motion_stop
#define vehicle_estop     motion_estop
#define vehicle_servo     motion_set_servo
#define publish_vehicle() publish_vehicle_faults()
#endif

/*
 * P1: close one part of the open step and keep the slowest part.
 */
static void part_end(rt_max_t part)
{
    const uint32_t now = drv_time_us();
    const uint32_t us = now - part_mark_us;
    part_mark_us = now;
    rt_max(part, us);
    rt_tl(RT_TL_VEH_PART, now, (uint32_t)part);
    if (us > part_slow_us) {
        part_slow_us = us;
        part_slow = part;
    }
}

/*
 * P1: telemetry_send with its call time in the [kcall] veh_send latch.
 */
static int timed_send(const telemetry_message_t *message)
{
    const uint32_t start = drv_time_us();
    const int er = telemetry_send(message);
    rt_call(RT_K_VEH_SEND, drv_time_us() - start);
    return er;
}

/*
 * Detect one falling button edge from the active-low input.
 */
static bool pressed(unsigned int pin, bool *previous)
{
    const bool down = drv_motor_hw_button_down(pin);
    const bool edge = down && !*previous;
    *previous = down;
    return edge;
}

/*
 * Send a nonblocking log message and preserve the local failure policy.
 */
static bool send(telemetry_message_t *message)
{
    if (timed_send(message) >= E_OK) {
        return true;
    }
    cancelled = true;
    set_reason(RM_STOP_TELEMETRY, "telemetry queue unavailable");
    return false;
}

/*
 * Store a run-relative event and optionally queue a live observation.
 */
static void event_detail(uint8_t kind, uint8_t from, uint8_t to,
                          uint16_t word, uint8_t detail)
{
#if TM_MQTT
    live_event(kind == 4u ? 5u : kind, from, to, word, detail);
    if (kind == 1u && from == VSM_MOD_SCAN) {
        live_event(4u, from, to, word, 0u);
    }
#else
    (void)detail;
#endif
    if (!active) {
        return;
    }
    telemetry_message_t m = {.kind = TELEMETRY_EVENT};
    m.data.event = (telemetry_event_t){.ms = drv_time_ms() - run_start,
                                       .word = word,
                                       .kind = kind,
                                       .from = from,
                                       .to = to};
    (void)send(&m);
}

static void event(uint8_t kind, uint8_t from, uint8_t to, uint16_t word)
{
    event_detail(kind, from, to, word, 0u);
}

/*
 * Copy the IMU hump board once. Never waits: a busy board keeps the last
 * copy, whose IMU timestamp decides later whether it is still usable.
 */
static void diagnostic_read(void)
{
    imu_hump_diag_board_t r;

    if (imu_hump_diag_try_read(&r) == E_OK) {
        diag_copy = r;
        have_diag_copy = true;
    }
}

static void hump_read(void)
{
    diagnostic_read();
    imu_hump_board_t r;
    if (imu_hump_try_read(&r) == E_OK) {
        hump_copy = r;
        have_hump_copy = true;
    }
}

#if TM_MQTT
static uint16_t hump_emitted;
#endif

/*
 * Observe the hump board during a run. With MQTT, each newly closed event
 * of THIS run from a fresh copy becomes one live HUMP event (kind 9).
 */
static void hump_poll(uint32_t epoch)
{
    hump_read();
#if TM_MQTT
    const imu_hump_board_t *h = &hump_copy;
    const uint16_t count = imu_hump_board_count(h);
    if (!have_hump_copy || imu_hump_board_epoch(h) != epoch ||
        count == hump_emitted ||
        drv_time_ms() - imu_hump_board_time(h) > IMU_HUMP_MAX_AGE_MS) {
        return;
    }
    hump_emitted = count;
    live_event(RM_EVENT_HUMP, 0u, 0u, imu_hump_board_last_01mm(h),
               imu_hump_detail(count, imu_hump_board_last_left(h)));
#else
    (void)epoch;
#endif
}

/*
 * Take the run's hump values, then end the IMU run. A stale, busy-only or
 * other-run copy gives the unavailable value, never an old number.
 */
static void hump_finish(uint32_t epoch)
{
    /* Capture final owner values before this common run exit goes even.
     * Keep vsm_stop below the summary/events so their shell state is unchanged. */
    publish_vehicle_faults();
#if TM_MQTT
    robot_health_faults_stop(epoch);
#endif
    hump_read();
    run_hump = imu_hump_board_summarize(&hump_copy, have_hump_copy, epoch,
                                        drv_time_ms(), IMU_HUMP_MAX_AGE_MS);
    run_diag = diag_copy;
    imu_hump_diag_board_finish(&run_diag, epoch, drv_time_ms(),
        have_diag_copy && have_hump_copy &&
        imu_hump_diag_board_time(&diag_copy) ==
            imu_hump_board_time(&hump_copy) &&
        run_hump.peak_01mm != IMU_HUMP_PEAK_NONE);
    app_board_publish(&run_diag_board.sequence, run_diag_board.words,
                      &run_diag, sizeof(run_diag));
    app_board_publish(&run_hump_board.sequence, run_hump_board.words,
                      &run_hump, sizeof(run_hump));
    __atomic_store_n(&line_run_epoch, 0u, __ATOMIC_SEQ_CST);
    __atomic_store_n(&run_epoch, epoch + 1u, __ATOMIC_SEQ_CST);
    rt_bkl_t lock = {0};
    (void)kls_read(&lock);
    lock.run_ms = drv_time_ms() - bkl_reset_ms;
    rt_bkl(&lock);
    rt_begin(epoch + 1u);
}

/*
 * Print the run's hump result once on the USB console.
 */
static void hump_print(void)
{
    const imu_hump_summary_t *h = &run_hump;
    if (h->peak_01mm == IMU_HUMP_PEAK_NONE) {
        tm_printf((UB *)"[robot] hump unavailable flags %02x "
                  "source %u %s\n", (unsigned int)h->flags,
                  (unsigned int)IMU_HUMP_SOURCE, IMU_HUMP_SOURCE_NAME);
        return;
    }
    tm_printf((UB *)"[robot] hump highest_mm %u.%u side %s count %u "
                    "flags %02x gated %u source %u %s\n",
              (unsigned int)(h->peak_01mm / 10u),
              (unsigned int)(h->peak_01mm % 10u),
              h->count == 0u ? "none"
              : (h->flags & IMU_HUMP_FLAG_LEFT) != 0u ? "left" : "right",
              (unsigned int)h->count, (unsigned int)h->flags,
              (unsigned int)h->gated, (unsigned int)IMU_HUMP_SOURCE,
              IMU_HUMP_SOURCE_NAME);
}

/*
 * Measure the previous vehicle response and enforce its deadline.
 */
static void finish_step(void)
{
    if (!step_open) {
        return;
    }
    publish_vehicle();
    const uint32_t elapsed = drv_time_us() - release_us;
    part_end(RT_STEP_PUBLISH);
    if (elapsed > response_max_us) {
        response_max_us = elapsed;
    }
    motion_status_t s;
    const bool late = elapsed >= VEHICLE_RESPONSE_LIMIT_US;
    const bool have = !late && motion_status(&s) >= E_OK;
    if (!late) {
        part_end(RT_STEP_STATUS);
    }
    const bool over = late || (have && s.ticks != release_tick);
    /* P2: record the finish before rt_overrun() freezes the timeline. */
    rt_tl(RT_TL_VEH_FINISH, drv_time_us(), over ? 1u : 0u);
    if (over) {
        /* P1: a late step did not read the status; use the bounded board
         * copy for the tick count. 0xFFFFFFFF: not available. */
        uint32_t ticks = 0xFFFFFFFFu;
        uint32_t stamp;
        if (have) {
            ticks = s.ticks - release_tick;
        } else if (motion_snapshot_try_read(&s, &stamp) >= E_OK) {
            ticks = s.ticks - release_tick;
        }
        const rt_overrun_t record = {
            .step = run_steps, .elapsed_us = elapsed, .tick_delta = ticks,
            .slow_part = (uint32_t)part_slow, .slow_us = part_slow_us,
            .wake_us = step_wake_us, .uptime_ms = drv_time_ms()};
        rt_overrun(&record);
        ++overruns;
        if (active) {
            cancelled = true;
            set_reason(RM_STOP_DEADLINE, "vehicle deadline missed");
            (void)vehicle_stop();
        }
    }
    step_open = false;
}

/*
 * Copy line inputs and enforce the existing acquisition freshness rules.
 */
static void read_line(void)
{
    if (line_snapshot_read(&sensor, true) < E_OK) {
        if (active) {
            cancelled = true;
            set_reason(RM_STOP_LINE_STALE, "line snapshot failed");
        }
        return;
    }
    lp = sensor.line;
    for (unsigned int i = 0u; i < 3u; ++i) {
        raw[i] = sensor.raw[i];
    }
    if (sensor.event != VC_EVENT_NONE) {
        line_event = sensor.event;
    }
    if (active && (!sensor.valid ||
                   drv_time_us() - sensor.timestamp_us > LINE_MAX_AGE_US)) {
        (void)vehicle_estop();
        cancelled = true;
        set_reason(RM_STOP_LINE_STALE, "line snapshot stale");
    }
    if (active && (sensor.acquisition.missed != line_base.missed ||
                   sensor.acquisition.dropped != line_base.dropped)) {
        (void)vehicle_estop();
        cancelled = true;
        set_reason(RM_STOP_LINE_LOST_SAMPLES, "line samples lost");
    }
}

/* Port of smojjofy 48b5f9b..038ff23: integer-only, bounded Vehicle glue. */
static ig_t imu_gate;

/* Vehicle task only: mirror existing state, never latch or clear a fault. */
static void publish_vehicle_faults(void)
{
    const uint32_t value = vehicle_fault_value(imu_gate.latched, vsm.state,
                                               vsm.fault);
    (void)fault_mask_publish(&vehicle_fault_board, FAULT_OWNER_VEHICLE, value);
}
static bool imu_announced;

static const char *imu_stop_text(uint8_t trigger)
{
    if (trigger == IMU_SAFETY_LIFT) { return "IMU lift detected"; }
    if (trigger == IMU_SAFETY_OVERTURN) { return "IMU overturn detected"; }
    return "IMU unavailable";
}
#if TM_MQTT
static uint8_t imu_stop_code(uint8_t trigger)
{
    if (trigger == IMU_SAFETY_LIFT) { return RM_STOP_IMU_LIFT; }
    if (trigger == IMU_SAFETY_OVERTURN) { return RM_STOP_IMU_OVERTURN; }
    return RM_STOP_IMU_UNAVAILABLE;
}
#endif

/* Kind 10 RECOVERY: bit 15, latched trigger in bits 8-14, gate reason in
 * bits 0-7 (0 CLEARED), detail = latched cause. See mqtt_wire.h. */
static void imu_recovery_word(uint8_t latched, uint8_t gate, uint8_t cause)
{
    const uint16_t word = (uint16_t)(RM_IMU_RECOVERY |
                          ((uint32_t)latched << 8u) | gate);
    event_detail(RM_EVENT_IMU_SAFETY, vsm.state, vsm.running, word, cause);
}

static void imu_recovery(uint8_t gate)
{
    imu_recovery_word(imu_gate.latched, gate, imu_gate.cause);
}

static bool imu_check_tick(void)
{
    imu_safety_report_t report;
    const bool have = imu_safety_try_read(&report) >= E_OK;
    const uint32_t now = drv_time_ms();
    const uint8_t was_latched = imu_gate.latched;
    const uint8_t was_cause = imu_gate.cause;
    const ig_action_t action = ig_step(&imu_gate, have ? &report : NULL,
                                        now, active);
    if (action.stop_edge) {
        const uint8_t module = vsm.running;
        (void)vehicle_estop();
        vsm_set_fault(&vsm, VSM_FAULT_IMU, imu_gate.latched);
        cancelled = true;
        imu_announced = false;
        set_reason(imu_stop_code(imu_gate.latched),
                   imu_stop_text(imu_gate.latched));
        event_detail(RM_EVENT_IMU_SAFETY, vsm.state, module,
                     imu_gate.latched, imu_gate.cause);
        publish_vehicle();
    }
    if (action.cleared) {
        if (!active) { vsm_reset(&vsm); }
        /* The only success signal: name the latch that was cleared. */
        imu_recovery_word(was_latched, RM_IMU_CLEARED, was_cause);
    }
    if (action.request_clear) {
        ig_queued(&imu_gate, imu_safety_clear_request(), now);
    }
    if (action.recovery != 0u) { imu_recovery(action.recovery); }
    publish_vehicle_faults();
    if (ig_stop_required(&imu_gate)) {
        if (active) { cancelled = true; }
        return false;
    }
    return imu_safety_evaluate(&imu_gate.report.value, imu_gate.have, now,
                               IMU_SAFETY_MAX_AGE_MS) == IMU_SAFETY_NONE;
}

/*
 * Wait for the motion release and collect the next vehicle inputs.
 */
static bool wait_tick(void)
{
    finish_step();
    if (active && cancelled) {
        return false;
    }
    const ER er = tk_slp_tsk(VEHICLE_WAKE_TIMEOUT_MS);
    const uint32_t woke = drv_time_us();
    rt_tl(RT_TL_VEH_WAKE, woke, 0u);
    const uint32_t wake = woke - rt_motion_stamp_us();
    step_wake_us = 0u;
    if (er >= E_OK && wake <= VEHICLE_WAKE_LIMIT_US) {
        step_wake_us = wake;
        rt_call(RT_K_VEH_WAKE, wake);
        rt_max(RT_STEP_WAKE, wake);
    }
    const INT extra = tk_can_wup(TSK_SELF);
    const uint32_t can_done = drv_time_us();
    rt_call(RT_K_VEH_CAN, can_done - woke);
    if (extra > 0) {
        wake_lost += (uint32_t)extra;
    }
    if (er < E_OK || (active && extra > 0)) {
        cancelled = true;
        set_reason(RM_STOP_MOTION_WAKE, "lost motion wake");
    }
    motion_status_t s;
    if (motion_status(&s) >= E_OK) {
        release_tick = s.ticks;
    } else {
        cancelled = true;
        set_reason(RM_STOP_MOTION_STATUS, "motion status failed");
    }
    release_us = drv_time_us();
    rt_call(RT_K_VEH_STATUS, release_us - can_done);
    part_mark_us = release_us;
    part_slow_us = 0u;
    part_slow = RT_STEP_READ_LINE;
    ++run_steps;
    step_open = true;
    /* Always park before checking safety. A fault must not create a spin. */
    if (!imu_check_tick() && active) {
        part_end(RT_STEP_COMMAND);
        return false;
    }
    read_line();
    part_end(RT_STEP_READ_LINE);
#if TM_MQTT
    handle_command();
#endif
    telemetry_stats_t log;
    telemetry_stats(&log);
    if (active && log.storage_full) {
        cancelled = true;
        set_reason(RM_STOP_TELEMETRY, "telemetry storage full");
    }
    if (active && drv_time_ms() - run_button_ms >= BUTTON_INTERVAL_MS) {
        run_button_ms = drv_time_ms();
        if (pressed(RUN_BUTTON_PIN, &prev_run)) {
            (void)vehicle_estop();
            cancelled = true;
            set_reason(RM_STOP_GP21, "GP21 emergency stop");
        }
    }
    part_end(RT_STEP_COMMAND);
    return er >= E_OK && !cancelled;
}

typedef enum {
    GOAL_OUT,
    GOAL_HOME
} goal_t;

/*
 * Return the magnitude of an in-range signed navigation value.
 */
static int32_t abs32(int32_t v)
{
    return v < 0 ? -v : v;
}

/*
 * Test whether a calibration wheel has reached its count goal.
 */
static bool wheel_done(goal_t goal, int32_t from_home, int32_t at_start)
{
    if (goal == GOAL_OUT) {
        const bool started_out = abs32(at_start) >= CAL_STEP_COUNTS / 2;
        const bool other_side = (from_home < 0) != (at_start < 0);
        return (!started_out || other_side) &&
               abs32(from_home) >= CAL_STEP_COUNTS;
    }
    return from_home == 0 || (from_home < 0) != (at_start < 0);
}

/* Spin remains a vehicle behaviour; acquisition/calibration data belongs to S3. */
static bool cal_spin(int direction, goal_t goal, const int32_t home[2],
                     bool stop_on_line)
{
    motion_status_t s;
    (void)motion_status(&s);
    const int32_t start[2] = {s.counts[0] - home[0], s.counts[1] - home[1]};
    const int sign[2] = {direction, -direction};
    bool running[2] = {true, true};
    const uint32_t t0 = drv_time_ms();
    if (!ig_stop_required(&imu_gate)) { (void)motion_clear_faults(); }
    while (running[0] || running[1]) {
        if (!wait_tick()) {
            (void)vehicle_stop();
            return false;
        }
        const uint32_t now = drv_time_ms();
        const uint16_t c = lp.norm[LP_CENTRE];
        const uint16_t peak = sensor.norm_peak;
        const bool on_line =
            stop_on_line &&
            (c >= CAL_CENTRE_STOP ||
             (peak >= CAL_CENTRE_MIN && c + CAL_PEAK_DROP <= peak));
        (void)motion_status(&s);
        for (unsigned int w = 0u; w < 2u; ++w) {
            if (running[w] &&
                (on_line ||
                 wheel_done(goal, s.counts[w] - home[w], start[w]))) {
                running[w] = false;
            }
        }
        if ((s.faults & MC_FAULT_LATCHED) != 0u ||
            now - t0 >= CAL_STEP_TIMEOUT_MS) {
            (void)vehicle_stop();
            return false;
        }
        (void)vehicle_speed(
            (int16_t)(running[0] ? sign[0] * CAL_SPEED_MM_S : 0),
            (int16_t)(running[1] ? sign[1] * CAL_SPEED_MM_S : 0));
    }
    (void)vehicle_stop();
    return true;
}

/*
 * Wait through the existing calibration pause without bypassing checks.
 */
static void cal_pause(void)
{
    const uint32_t end = drv_time_ms() + CAL_PAUSE_MS;
    while ((int32_t)(drv_time_ms() - end) < 0) {
        if (!wait_tick()) {
            return;
        }
    }
}

/*
 * Run the existing line calibration and retain its failure policy.
 */
static bool calibrate_body(void)
{
    motion_diag_phase(MOTION_DIAG_CALIBRATE);
    motion_status_t s;
    (void)motion_status(&s);
    const int32_t home[2] = {s.counts[0], s.counts[1]};
    if (line_calibration_begin() < E_OK) {
        return false;
    }
    bool ok = cal_spin(-1, GOAL_OUT, home, false);
    if (ok) {
        cal_pause();
        ok = !cancelled && cal_spin(+1, GOAL_OUT, home, false);
    }
    if (ok) {
        cal_pause();
    }
    /* End acquisition calibration even on an aborted spin, retaining old ends on failure. */
    if (!ok || cancelled) {
        (void)line_calibration_abort();
        return false;
    }
    const bool calibrated = line_calibration_end() == LP_CALIB_OK;
    read_line();
    ok = cal_spin(-1, GOAL_HOME, home, calibrated);
    read_line();
    return ok && calibrated && lp_calibrated(&lp);
}

/* Preserve every existing calibration exit and motion decision. */
static bool calibrate(void)
{
    const uint32_t epoch = __atomic_load_n(&calibration_epoch,
                                            __ATOMIC_SEQ_CST);

    __atomic_store_n(&calibration_epoch, epoch + 1u, __ATOMIC_SEQ_CST);
    const bool ok = calibrate_body();

    __atomic_store_n(&calibration_epoch, epoch + 2u, __ATOMIC_SEQ_CST);
    return ok;
}

typedef struct {
    vsm_module_t module;
    uint8_t id;
} watched_t;

static watched_t watched[4];

/* Live only: never add junction phase events to the RAM event log. */
static void barcode_dump_request(void)
{
    const telemetry_message_t m = {.kind = TELEMETRY_BARCODE_DUMP};
    (void)timed_send(&m);
}

static void junction_observe(void)
{
    telemetry_message_t m = {.kind = TELEMETRY_JUNCTION_SOURCE};
    if (junction_source_report(&m.data.junction_source)) {
        (void)timed_send(&m);
    }
    if (junction_source_counts(&m.data.junction_counts)) {
        m.kind = TELEMETRY_JUNCTION_COUNTS;
        (void)timed_send(&m);
    }
    const ja_t *action = &junction_line.action;
#if TM_MQTT
    if (action->phase == JA_PENDING && junction_reported != JA_PENDING) {
        junction_angles_reported = 0u;
    }
    const uint8_t marks = action->result.marks;
    const uint8_t command = action->result.command;
    if ((marks & JA_MARK_FIRST) != 0u &&
        (junction_angles_reported & JA_MARK_FIRST) == 0u) {
        live_event(RM_EVENT_JUNCTION, VSM_MOD_LINE, VSM_MOD_LINE,
            (uint16_t)(int16_t)action->result.first_line_cdeg,
            (uint8_t)(RM_JUNCTION_FIRST | command));
    }
    if ((marks & JA_MARK_ACCEPTED) != 0u &&
        (junction_angles_reported & JA_MARK_ACCEPTED) == 0u) {
        live_event(RM_EVENT_JUNCTION, VSM_MOD_LINE, VSM_MOD_LINE,
            (uint16_t)(int16_t)action->result.accepted_line_cdeg,
            (uint8_t)(RM_JUNCTION_ACCEPTED | command));
    }
    junction_angles_reported = marks;
#endif
    if (action->phase == junction_reported) {
        return;
    }
    junction_reported = action->phase;
    /* Set busy before ACK: a consumed command still owns the action job. */
    junction_source_set_busy(action->phase == JA_PENDING ||
                               ja_active(action));
#if TM_MQTT
    const uint16_t word = (uint16_t)(action->result.command |
                                      ((uint16_t)action->phase << 8u));
    live_event(RM_EVENT_JUNCTION, VSM_MOD_LINE, VSM_MOD_LINE, word,
               action->result.reason);
#endif
    if (action->consumed != 0u) {
        junction_source_acknowledge(action->consumed);
    }
    if (action->phase == JA_DONE || action->phase == JA_FAILED ||
        action->phase == JA_UNUSED) {
        m.kind = TELEMETRY_JUNCTION;
        m.data.junction = action->result;
        (void)timed_send(&m);
    }
}

/*
 * Start a navigation module and observe detour entry when enabled.
 */
static void watched_start(void *ctx, uint16_t payload, uint32_t now)
{
    watched_t *w = ctx;
    junction_source_route(w->id, now);
#if TM_MQTT
    if (w->id == VSM_MOD_DETOUR) {
        rs_detour_begin(&run_stats, now, (uint8_t)ss_payload_side(payload),
                        obstacle_scans);
    }
#endif
    w->module.start(w->module.ctx, payload, now);
}

/*
 * Run an unchanged module and record its completed outcome.
 */
static vsm_outcome_t watched_step(void *ctx, const vsm_inputs_t *in,
                                  uint32_t now, vsm_drive_t *drive)
{
    watched_t *w = ctx;
    ja_pending_route(&junction_line.action, w->id, in->counts, now);
    const vsm_outcome_t out = w->module.step(w->module.ctx, in, now, drive);
    if (vsm_outcome_type(out) != VSM_OUT_RUNNING) {
        event(1u, w->id, 0u, out);
#if TM_MQTT
        rm_detour_t detour;

        if (rs_outcome(&run_stats, w->id, vsm_outcome_type(out),
                       vsm_outcome_payload(out), now, &detour)) {
            publish_detour(&detour);
        }
#endif
    }
    return out;
}

/*
 * Register an outcome-observing wrapper around a navigation module.
 */
static void install(unsigned int index, uint8_t id, vsm_module_t module)
{
    watched[index] = (watched_t){module, id};
    (void)vsm_set_module(
        &vsm, (vsm_module_id_t)id,
        (vsm_module_t){watched_start, watched_step, &watched[index]});
}

/*
 * Pack the current vehicle step into the existing KR1 RAM-log row.
 */
static void save_row(uint32_t now, const motion_status_t *s,
                     const vsm_drive_t *d, ultrasonic_result_t echo,
                     uint8_t bits)
{
    const bool reversing =
        (vsm.running == VSM_MOD_SCAN) && (ss.phase == SS_REVERSE);
    const vr_values_t v = {.elapsed_ms = now - run_start,
                           .raw = {raw[0], raw[1]},
                           .echo = echo,
                           .asked = {d->left_mm_s, d->right_mm_s},
                           .measured = {s->speed_mm_s[0], s->speed_mm_s[1]},
                           .position = lp.position,
                           .servo = d->servo_deg,
                           .state = vsm.state,
                           .module = reversing ? VR_MODULE_SCAN_REVERSE
                                               : vsm.running,
                           .events = bits};
    vr_row_t row;
    vr_pack(&row, &v);
    telemetry_message_t m = {.kind = TELEMETRY_ROW};
    for (unsigned int i = 0u; i < VR_BYTES; ++i) {
        m.data.row[i] = row.bytes[i];
    }
    (void)send(&m);
}

/*
 * Collect the existing run configuration for the local log header.
 */
static void info(telemetry_info_t *i)
{
    *i = (telemetry_info_t){0};
    for (unsigned int n = 0u; n + 1u < sizeof(i->reason) && reason[n]; ++n) {
        i->reason[n] = reason[n];
    }
    motion_status_t s = {0};
    (void)motion_status(&s);
    i->wake_lost = wake_lost;
    i->overruns = overruns;
    i->response_max_us = response_max_us;
    i->gap_min_us = s.gap_min_us;
    i->gap_max_us = s.gap_max_us;
    i->faults = s.faults;
    for (unsigned int w = 0u; w < 2u; ++w) {
        i->counts[w] = s.counts[w];
        i->floor[w] = lp_floor(&lp, w);
        i->line[w] = lp_line(&lp, w);
    }
    i->sampled = sensor.acquisition.samples;
    i->missed = sensor.acquisition.missed;
    i->dropped = sensor.acquisition.dropped;
    i->raw_bc = sensor.raw_bc;
    i->isr_max_us = sensor.acquisition.isr_max_us;
    i->line_max_us = sensor.sample_max_us;
}

/*
 * Request sonar cancellation without waiting for the service.
 */
static void sonar_stop(void)
{
    const sonar_request_t r = {
        .generation = ++generation, .id = ++request_id, .mode = SONAR_STOP};
    if (sonar_request(&r) < E_OK && active) {
        cancelled = true;
        set_reason(RM_STOP_SONAR, "sonar cancel unavailable");
    }
}

/*
 * Submit a sonar request and preserve the existing local failure policy.
 */
static bool request(sonar_request_t *r)
{
    if (sonar_request(r) >= E_OK) {
        return true;
    }
    cancelled = true;
    set_reason(RM_STOP_SONAR, "sonar request unavailable");
    return false;
}

/*
 * Run the existing navigation session with read-only summary observers.
 */
static void run_once(void)
{
    if (!ig_stop_required(&imu_gate)) {
        (void)motion_clear_faults();
        vsm_reset(&vsm);
        publish_vehicle_faults();
    }
    read_line();
    line_event = VC_EVENT_NONE;
    motion_status_t initial = {0};
    uint32_t initial_stamp = 0u;
    if (JUNCTION_TEST_CMD != 0 &&
        (motion_snapshot_try_read(&initial, &initial_stamp) < E_OK ||
         drv_time_us() - initial_stamp > VEHICLE_RESPONSE_LIMIT_US)) {
        set_reason(RM_STOP_MOTION_STATUS, "junction START motion snapshot");
        return;
    }
    int32_t final_counts[2] = {initial.counts[0], initial.counts[1]};
    __atomic_store_n(&run_start, drv_time_ms(), __ATOMIC_SEQ_CST);
    const uint32_t epoch =
        __atomic_load_n(&run_epoch, __ATOMIC_SEQ_CST) | 1u;
    __atomic_store_n(&run_epoch, epoch, __ATOMIC_SEQ_CST);
    run_steps = 0u;
    rt_begin(epoch);
    (void)kls_reset();
    bkl_reset_ms = drv_time_ms();
#if TM_MQTT
    hump_emitted = 0u;
    stop_code = RM_STOP_UNKNOWN;
    rs_run_begin(&run_stats, run_start);
    run_barcode = (barcode_live_counts_t){0};
    have_run_barcode = false;
#endif
    ja_init(&junction_line.action, SEARCH_CFG.speed_mm_s, epoch);
    junction_reported = JA_IDLE;
    const uint32_t read_age_ms = JUNCTION_TEST_CMD != 0
        ? (drv_time_us() - initial_stamp) / 1000u : 0u;
    junction_source_start(JUNCTION_TEST_CMD, epoch, run_start - read_age_ms,
                            initial.counts);
    ja_command_t command;
#if JUNCTION_TEST_CMD != 0
    if (junction_source_read(&command)) {
        (void)ja_offer(&junction_line.action, &command);
    }
#endif
    junction_observe();
    event(2u, VSM_MOD_NONE, VSM_MOD_LINE, VSM_RUNNING);
    (void)vsm_start(&vsm, VSM_MOD_LINE, 0u, run_start);
    __atomic_store_n(&line_run_epoch, epoch, __ATOMIC_SEQ_CST);
    ++generation;
    uint8_t mode = SONAR_MODE_UNSET;
    uint16_t interval = 0u;
    bool outstanding = false;
    ultrasonic_result_t echo = ULTRASONIC_RESULT_NONE;
    uint32_t seen_ping = 0u;
    uint32_t seen_result = 0u;
    uint32_t last_ping = run_start;
    uint32_t last_log = run_start - LOG_PERIOD_MS;
    uint32_t request_deadline = 0u;
    uint32_t watch_deadline = 0u;
    int8_t last_servo = 0;
    uint8_t pending_bits = 0u;
    while (!cancelled) {
        if (!wait_tick()) {
            break;
        }
        hump_poll(epoch);
#if TM_MQTT
        barcode_poll(epoch);
#endif
        const uint32_t now = drv_time_ms();
        sonar_snapshot_t sonar = {0};
        bool fresh = false;
        if (sonar_result_read(&sonar, &fresh) < E_OK) {
            set_reason(RM_STOP_SONAR, "sonar snapshot failed");
            break;
        }
        const bool matching = sonar.generation == generation &&
                              sonar.id == request_id &&
                              sonar.angle == last_servo;
        const bool ping_sent = sonar.ping_generation == generation &&
                               sonar.ping_id == request_id &&
                               sonar.ping_angle == last_servo &&
                               sonar.ping_sequence != seen_ping &&
                               (int32_t)(sonar.trigger_ms - run_start) >= 0;
        if (ping_sent) {
            last_ping = sonar.trigger_ms;
        }
        bool echo_new = matching && fresh &&
                        sonar.result_sequence != seen_result &&
                        sonar.status == SONAR_ECHO;
        if (matching && fresh && sonar.status == SONAR_DEADLINE) {
            set_reason(RM_STOP_SONAR, "sonar request deadline");
            break;
        }
        if (echo_new && (int32_t)(now - sonar.deadline_ms) > 0) {
            set_reason(RM_STOP_SONAR, "stale sonar result");
            break;
        }
        if (echo_new) {
            echo = sonar.echo;
            outstanding = false;
            if (mode == SONAR_WATCH) {
                watch_deadline = sonar.deadline_ms;
            }
            if (mode == SONAR_WATCH) {
                sonar_request_t ack = {.generation = generation,
                                       .id = request_id,
                                       .mode = SONAR_ACK};
                if (!request(&ack)) {
                    break;
                }
            }
        }
        if (mode == SONAR_WATCH && (int32_t)(now - watch_deadline) > 0) {
            set_reason(RM_STOP_SONAR, "sonar watch stale");
            break;
        }
        if (mode == SONAR_ONCE && outstanding &&
            (int32_t)(now - request_deadline) > 0) {
            set_reason(RM_STOP_SONAR, "sonar request deadline");
            break;
        }
        seen_ping = sonar.ping_sequence;
        seen_result = sonar.result_sequence;
        part_end(RT_STEP_SONAR);
        motion_status_t s;
        if (motion_status(&s) < E_OK) {
            set_reason(RM_STOP_MOTION_STATUS, "motion status failed");
            break;
        }
#if JUNCTION_TEST_CMD != 0
        if (junction_source_read(&command)) {
            (void)ja_offer(&junction_line.action, &command);
        }
#endif
        final_counts[0] = s.counts[0];
        final_counts[1] = s.counts[1];
        junction_line.sample = (ja_sample_t){
            .line_stamp = sensor.timestamp_us,
            .line_age_ms = (drv_time_us() - sensor.timestamp_us) / 1000u,
            .valid = sensor.valid
        };
        if (vc_event_type(line_event) == VC_EVENT_LINE_LOST) {
            pending_bits |= 1u;
        }
        if (vc_event_type(line_event) == VC_EVENT_LINE_FOUND) {
            pending_bits |= 2u;
        }
        if (echo_new) {
            pending_bits |= 4u;
        }
        if (ping_sent) {
            pending_bits |= 8u;
        }
#if TM_MQTT
        {
            const int16_t commanded[2] = {vehicle.left_mm_s,
                                          vehicle.right_mm_s};
            const int16_t measured[2] = {s.speed_mm_s[0], s.speed_mm_s[1]};

            rs_step(&run_stats, now, s.counts, measured, commanded, s.faults);
        }
#endif
        const uint8_t state_before = vsm.state;
        const uint8_t module_before = vsm.running;
        const uint32_t handoffs = vsm.handoffs;
        uint16_t ping_ms = VSM_PING_DEFAULT_MS;
        const vsm_inputs_t in = {
            &lp,      line_event, {s.counts[0], s.counts[1]}, echo_new, echo,
            &ping_ms, ping_sent};
        line_event = VC_EVENT_NONE;
        const vsm_drive_t d =
            vsm_step(&vsm, &in, (uint8_t)(s.faults & MC_FAULT_LATCHED), now);
        publish_vehicle_faults();
        if (vsm.state == VSM_RUNNING &&
            (vsm.running == VSM_MOD_LINE || vsm.running == VSM_MOD_SEARCH)) {
            ja_cross_progress(&junction_line.action, s.counts, now);
        }
        /* Resolve encounters/routes BEFORE admitting a new live frame.
         * Fake commands retain the original pre-step B2a timing. */
#if JUNCTION_TEST_CMD == 0
        const uint32_t source_us = drv_time_us();
        const uint32_t source_ms = drv_time_ms();
        const ja_sample_t source_sample = {
            .line_stamp = sensor.timestamp_us,
            .line_age_ms = (source_us - sensor.timestamp_us) / 1000u,
            .valid = sensor.valid};
        junction_source_tick(vsm.state == VSM_RUNNING ? vsm.running :
                                VSM_MOD_NONE,
                                &lp, &source_sample, s.counts, source_ms,
                                &junction_line.action);
        if (junction_source_read(&command)) {
            const bool accepted = ja_offer(&junction_line.action, &command);
            junction_source_offer_result(command.sequence, accepted);
        }
#endif
        junction_observe();
        __atomic_store_n(&line_run_epoch,
            vsm.state == VSM_RUNNING && vsm.running == VSM_MOD_LINE &&
                    !ja_searching(&junction_line.action)
                ? epoch : 0u, __ATOMIC_SEQ_CST);
        motion_diag_phase(MOTION_DIAG_RUN | ((uint32_t)vsm.running << 8u));
        if (vsm.state != state_before || vsm.running != module_before) {
            event(
                2u, module_before, vsm.running,
                (uint16_t)((uint16_t)vsm.state | ((uint16_t)vsm.fault << 8u)));
        }
        if (s.faults != 0u) {
            pending_bits |= ROW_FAULT_FLAG;
            event(3u, vsm.state, vsm.running, s.faults);
        }
        if (vsm.handoffs != handoffs) {
            pending_bits |=
                (uint8_t)((uint8_t)vsm_outcome_type(vsm.last.outcome) << 4u);
            generation += vsm.handoffs - handoffs;
            mode = SONAR_MODE_UNSET;
            outstanding = false;
        }
        if (d.action == VSM_ACT_STOP) {
            (void)vehicle_stop();
        } else {
            (void)vehicle_speed(d.left_mm_s, d.right_mm_s);
            if (d.servo_deg != last_servo) {
                (void)vehicle_servo(d.servo_deg, 0);
                last_servo = d.servo_deg;
            }
        }
        part_end(RT_STEP_VSM);
        if (now - last_log >= LOG_PERIOD_MS) {
            save_row(now, &s, &d, echo, pending_bits);
            pending_bits = 0u;
            last_log = now;
        }
        if (vsm.state != VSM_RUNNING) {
            if ((vsm.fault == VSM_FAULT_FAILED) &&
                (module_before == VSM_MOD_SCAN) &&
                (vsm.fault_detail == SS_FAIL_REVERSE_TIMEOUT)) {
                set_reason(RM_STOP_SCAN_REVERSE_TIMEOUT,
                           "scan reverse timed out");
            } else {
                if (vsm.fault == VSM_FAULT_FAILED &&
                    vsm.fault_detail > JA_FAIL_BASE &&
                    vsm.fault_detail <= JA_FAIL_BASE + JA_TOTAL_LIMIT) {
                    set_reason(RM_STOP_SHELL,
                        ja_reason_name((uint8_t)(vsm.fault_detail -
                                                  JA_FAIL_BASE)));
                } else {
                    set_reason(RM_STOP_SHELL, "shell left RUNNING");
                }
            }
            break;
        }
        const uint8_t desired =
            ping_ms == 0u
                ? SONAR_STOP
                : (vsm.running == VSM_MOD_SCAN ? SONAR_ONCE : SONAR_WATCH);
        const uint16_t rate =
            ping_ms < SONAR_PING_MIN_MS ? SONAR_PING_MIN_MS : ping_ms;
        if (desired != mode || (desired == SONAR_WATCH && rate != interval) ||
            (desired == SONAR_ONCE && !outstanding)) {
            sonar_request_t r = {.generation = generation,
                                 .id = ++request_id,
                                 .mode = desired,
                                 .interval_ms = rate,
                                 .deadline_ms = now + SONAR_REQUEST_DEADLINE_MS,
                                 .not_before_ms = last_ping + ping_ms,
                                 .angle = last_servo};
            if (!request(&r)) {
                break;
            }
            mode = desired;
            interval = rate;
            outstanding = desired == SONAR_ONCE;
            request_deadline = r.deadline_ms;
            watch_deadline = now + (uint32_t)rate * 2u + SONAR_WATCH_SLACK_MS;
        }
        if (now - run_start >= RUN_LIMIT_MS) {
            set_reason(RM_STOP_TIME_LIMIT, "90 s time limit");
            break;
        }
        part_end(RT_STEP_LOG);
    }
    /* Close the last driving step here, before the stop work (prints,
     * actuator stops). Measured later, that work would count as a deadline
     * miss and replace the real stop reason. */
    finish_step();
    ja_finish(&junction_line.action, final_counts, drv_time_ms());
#if TM_MQTT
    /* Take the last same-run copy before STOP changes producer context. */
    barcode_poll(epoch);
#endif
    junction_source_stop(drv_time_ms());
    junction_observe();
    barcode_dump_request();
    hump_finish(epoch);
    event(4u, vsm.state, vsm.running, vsm.fault_detail);
#if TM_MQTT
    publish_run_summary();
#endif
    (void)vehicle_stop();
    (void)vehicle_servo(0, 0);
    vsm_stop(&vsm);
    publish_vehicle_faults();
    motion_diag_phase(MOTION_DIAG_STOP);
    sonar_stop();
    hump_print();
    imu_hump_diag_board_print(&run_diag);
}

/*
 * Own navigation, calibration, local buttons and actuator requests.
 */
static void vehicle_task(INT code, void *ctx)
{
    (void)code;
    (void)ctx;
    ig_init(&imu_gate);
    lt_init(&lt, &LINE_CFG);
    jl_init(&junction_line, &lt, SEARCH_CFG.speed_mm_s);
    ls_init(&ls, &SEARCH_CFG);
    ss_init(&ss, &SCAN_CFG);
    dr_init(&dr, &DETOUR_CFG);
    dr_set_scan_word(&dr, &ss.word); /* minimum turn before LOST */
#if TM_MQTT
    {
        static const mc_config_t motion_config = MC_ROBOT_CONFIG;
        const rs_config_t stats_config = {
            {motion_config.counts_per_m, motion_config.counts_per_deg_x100}};

        rs_init(&run_stats, &stats_config);
    }
#endif
    vsm_init(&vsm, VSM_DEFAULT_ROUTES, VSM_DEFAULT_ROUTE_COUNT);
    install(0u, VSM_MOD_LINE, jl_module(&junction_line));
    install(1u, VSM_MOD_SEARCH, ls_module(&ls));
#if TM_MQTT
    install(2u, VSM_MOD_SCAN,
            (vsm_module_t){ss_start, scan_step_observed, &ss});
#else
    install(2u, VSM_MOD_SCAN, ss_module(&ss));
#endif
    install(3u, VSM_MOD_DETOUR, dr_module(&dr));
    drv_motor_hw_buttons_init();
    prev_run = drv_motor_hw_button_down(RUN_BUTTON_PIN);
    prev_dump = drv_motor_hw_button_down(DUMP_BUTTON_PIN);
    tm_printf((UB *)"[robot] GP21 calibrate then run / ESTOP; GP20 telemetry "
                    "dump. D85.\n");
    if (motion_notify_after_tick(tk_get_tid()) < E_OK) {
        tm_printf((UB *)"ERROR robot: cannot register motion wake\n");
        tk_ext_tsk();
    }
    publish_vehicle();
    uint32_t button_ms = drv_time_ms();
    for (;;) {
        motion_diag_phase(MOTION_DIAG_IDLE);
        cancelled = false;
        (void)wait_tick();
        if (imu_gate.latched != IMU_SAFETY_NONE && !imu_announced) {
            imu_announced = true; /* Stopped, once per stop, never per tick. */
            tm_printf((UB *)"[IMU_SAFETY] stop %u cause %u\n",
                      (unsigned int)imu_gate.latched,
                      (unsigned int)imu_gate.cause);
        }
        const uint32_t now = drv_time_ms();
        if (now - button_ms < BUTTON_INTERVAL_MS) {
            continue;
        }
        button_ms = now;
        const bool run_edge = pressed(RUN_BUTTON_PIN, &prev_run);
        const bool dump_edge = pressed(DUMP_BUTTON_PIN, &prev_dump);
        telemetry_stats_t log;
        telemetry_stats(&log);
        if (dump_edge && !run_edge && !log.busy) {
            junction_observe(); /* Resend stopped counters after queue loss. */
            telemetry_message_t m = {.kind = TELEMETRY_DUMP};
            info(&m.data.info);
            (void)timed_send(&m);
        }
#if TM_MQTT
        const uint8_t action = run_edge ? 0u : pending_action;
        if ((!run_edge && action == 0u) || log.busy) {
            continue;
        }
        pending_action = 0u;
        vehicle.phase = 1u;
        last_network_fault = 0u;
#else
        if (!run_edge || log.busy) {
            continue;
        }
#endif
        const bool queued = run_edge && ig_press(&imu_gate, now);
        if (queued) {
            ig_queued(&imu_gate, imu_safety_clear_request(), now);
            imu_recovery(IG_PENDING);
        }
        if (!ig_start_allowed(&imu_gate, drv_time_ms())) {
            /* One observation per request: a queued clear already sent
             * PENDING for this press. */
            if (!queued) {
                imu_recovery(imu_gate.pending ? IG_PENDING :
                             ig_refusal(&imu_gate, drv_time_ms()));
            }
            set_reason(imu_stop_code(imu_gate.latched),
                       imu_stop_text(imu_gate.latched));
#if TM_MQTT
            vehicle.phase = 0u;
            publish_vehicle();
#endif
            continue;
        }
        telemetry_message_t begin = {.kind = TELEMETRY_BEGIN};
        if (!send(&begin)) {
            continue;
        }
        cancelled = false;
        wake_lost = 0u;
        overruns = 0u;
        response_max_us = 0u;
        reason = "calibration failed";
        line_base = sensor.acquisition;
        active = true;
        run_start = now;
        run_button_ms = now;
#if TM_WIFI_CYW43
        const bool wifi_window = cyw43_utk_timing_window_begin() >= E_OK;
#endif
#if TM_MQTT
        const bool calibrated =
            action == RM_CMD_START ? lp_calibrated(&lp) : calibrate();
        const bool run = action != RM_CMD_CALIBRATE;
        if (calibrated && !run && !cancelled) {
            reason = "calibration complete";
        }
        if (calibrated && run && !cancelled) {
            vehicle.phase = 2u;
#else
        if (calibrate() && !cancelled) {
#endif
            motion_diag_phase(MOTION_DIAG_START_DELAY);
            const uint32_t start = drv_time_ms();
            while (!cancelled && drv_time_ms() - start < START_DELAY_MS) {
                (void)wait_tick();
            }
            if (!cancelled) {
#if TM_MQTT
                vehicle.phase = 3u;
#endif
                run_once();
            }
        }
        (void)vehicle_stop();
        (void)vehicle_servo(0, 0);
        sonar_stop();
        motion_status_t end;
        if (motion_status(&end) >= E_OK && end.faults != 0u) {
            event(3u, vsm.state, vsm.running, end.faults);
        }
        event(5u, vsm.state, vsm.running, cancelled ? 1u : 0u);
        finish_step();
        telemetry_message_t m = {.kind = TELEMETRY_INFO};
        info(&m.data.info);
        (void)send(&m);
        active = false;
        cancelled = false;
        if (!ig_stop_required(&imu_gate)) { vsm_reset(&vsm); }
        publish_vehicle_faults();
#if TM_MQTT
        vehicle.phase = 0u;
        publish_vehicle();
#endif
#if TM_WIFI_CYW43
        if (wifi_window) {
            T_CYW43_TIMING w;
            if (cyw43_utk_timing_window_end(&w) >= E_OK) {
                tm_printf((UB *)"[robot] WiFi window wall polls %lu gaps %lu "
                                "max_gap_us %lu gaps_over_10000 %lu "
                                "work_max_us %lu probe_max_us %lu\n",
                          (unsigned long)w.polls, (unsigned long)w.gaps,
                          (unsigned long)w.gap_max_us,
                          (unsigned long)w.gaps_over_10000,
                          (unsigned long)w.work_max_us,
                          (unsigned long)w.probe_max_us);
            }
        }
#endif
    }
}

int vehicle_task_start(void)
{
    motion_status_t s;
    line_snapshot_t line;
    sonar_snapshot_t sonar;
    bool fresh;
    int er = motion_status(&s);
    if (er >= E_OK && !s.watchdog_armed) {
        er = E_OBJ;
    }
    if (er >= E_OK && (!line_service_running() || !sonar_service_running() ||
                       !telemetry_running())) {
        er = E_OBJ;
    }
    if (er >= E_OK) {
        er = line_snapshot_read(&line, false);
    }
    if (er >= E_OK) {
        er = sonar_result_read(&sonar, &fresh);
    }
    if (er < E_OK) {
        return er;
    }
    const ID tid = app_start_task((FP)vehicle_task, VEHICLE_TASK_PRIORITY,
                                  TP_PRC1, "vehicle_task");
    return tid < 0 ? (int)tid : E_OK;
}

_Static_assert(VR_BYTES == TELEMETRY_BYTES && VR_ROWS == TELEMETRY_ROWS,
               "KR1 storage contract");
#if TM_MQTT
_Static_assert(RM_HUMP_FLAGS_MASK == IMU_HUMP_FLAGS_MASK &&
               RM_HUMP_IMPLEMENTED == IMU_HUMP_FLAG_IMPLEMENTED &&
               RM_HUMP_LEFT == IMU_HUMP_FLAG_LEFT &&
               RM_HUMP_PEAK_NONE == IMU_HUMP_PEAK_NONE &&
               RM_HUMP_DETAIL_LEFT == IMU_HUMP_DETAIL_LEFT,
               "summary v3 hump contract");
#endif
#else
int vehicle_task_start(void)
{
    return E_OK;
}
#endif
/* Read-only observer: no board exists in TEST/BENCH roles. */
bool vehicle_fault_mask_try_read(uint32_t *out)
{
#if !APP_TEST && !APP_BENCH
    return fault_mask_try_read(&vehicle_fault_board, FAULT_OWNER_VEHICLE, out);
#else
    (void)out;
    return false;
#endif
}

/* Available without MQTT. TEST/BENCH cannot run vehicle calibration. */
uint32_t vehicle_calibration_epoch(void)
{
#if !APP_TEST && !APP_BENCH
    return __atomic_load_n(&calibration_epoch, __ATOMIC_SEQ_CST);
#else
    return 0u;
#endif
}

uint32_t vehicle_run_epoch(void)
{
#if !APP_TEST && !APP_BENCH
    return __atomic_load_n(&run_epoch, __ATOMIC_SEQ_CST);
#else
    return 0u;
#endif
}

uint32_t vehicle_run_start_ms(void)
{
#if !APP_TEST && !APP_BENCH
    return __atomic_load_n(&run_start, __ATOMIC_SEQ_CST);
#else
    return 0u;
#endif
}

uint32_t vehicle_line_run_epoch(void)
{
#if !APP_TEST && !APP_BENCH
    return __atomic_load_n(&line_run_epoch, __ATOMIC_SEQ_CST);
#else
    return 0u;
#endif
}

int vehicle_hump_diag_try_read(imu_hump_diag_board_t *out)
{
    if (out == NULL) {
        return E_PAR;
    }
#if !APP_TEST && !APP_BENCH
    imu_hump_diag_board_t value;
    if (!app_board_try_read(&run_diag_board.sequence, run_diag_board.words,
                            &value, sizeof(value))) {
        const uint32_t sequence =
            __atomic_load_n(&run_diag_board.sequence, __ATOMIC_SEQ_CST);
        return sequence == 0u ? E_NOEXS : E_TMOUT;
    }
    *out = value;
    return E_OK;
#else
    return E_NOEXS;
#endif
}

int vehicle_hump_try_read(imu_hump_summary_t *out)
{
    if (!out) {
        return E_PAR;
    }
#if !APP_TEST && !APP_BENCH
    imu_hump_summary_t value;
    if (!app_board_try_read(&run_hump_board.sequence, run_hump_board.words,
                            &value, sizeof(value))) {
        const uint32_t sequence =
            __atomic_load_n(&run_hump_board.sequence, __ATOMIC_SEQ_CST);
        return sequence == 0u ? E_NOEXS : E_TMOUT;
    }
    *out = value;
    return E_OK;
#else
    return E_NOEXS;
#endif
}

#if !TM_MQTT || APP_TEST || APP_BENCH
int vehicle_snapshot_try_read(vehicle_snapshot_t *out)
{
    return out ? E_OBJ : E_PAR;
}

int vehicle_obstacle_try_read(rm_obstacle_t *out)
{
    return out ? E_NOEXS : E_PAR;
}

int vehicle_detour_try_read(rm_detour_t *out)
{
    return out ? E_NOEXS : E_PAR;
}

int vehicle_summary_try_read(rm_summary_t *out)
{
    return out ? E_NOEXS : E_PAR;
}
#endif

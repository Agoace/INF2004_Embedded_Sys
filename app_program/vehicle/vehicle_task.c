/* Kernel navigation only: snapshot inputs, asynchronous services, motion outputs. */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "../app.h"
#include "vehicle_task.h"
#include "../subsystem2/motion.h"
#include "../subsystem2/drv_motor_hw.h"
#include "../subsystem3/line_service.h"
#include "../subsystem5/sonar_service.h"
#include "../subsystem1/telemetry.h"
#include "../platform/drv_time.h"
#include "vehicle_inputs.h"
#include "vehicle_robot_config.h"
#include "vehicle_log.h"
#include "vehicle_board.h"
#include "obstacle_report.h"
#if TM_MQTT
#include "../subsystem1/robot_network.h"
#include "../subsystem1/network_queue.h"
#include "../platform/app_board.h"
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
static ls_t ls;
static ss_t ss;
static dr_t dr;
static vsm_t vsm;
static uint16_t raw[3];
static vc_event_t line_event;
static bool active;
static bool cancelled;
static bool prev_run;
static bool prev_dump;
static bool step_open;
static uint32_t wake_lost;
static uint32_t overruns;
static uint32_t response_max_us;
static uint32_t release_us;
static uint32_t release_tick;
static uint32_t run_start;
static uint32_t run_button_ms;
static uint32_t generation;
static uint32_t request_id;
static drv_line_stats_t line_base;
static const char *reason = "no saved run";
#if TM_MQTT
static APP_BOARD(vehicle_snapshot_t) vehicle_board;
static vehicle_snapshot_t vehicle;
static APP_BOARD(rm_obstacle_t) obstacle_board;
static uint32_t obstacle_scans;
static uint8_t pending_action;
static uint32_t event_sequence;
static uint32_t seen_commands[8];
static unsigned int seen_count;
static unsigned int seen_next;
static uint16_t last_network_fault;

static void live_event(uint8_t kind, uint8_t from, uint8_t to, uint16_t word,
                       uint8_t detail)
{
    if (kind == 3u && word == last_network_fault) {
        return;
    }
    if (kind == 3u) {
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

static void publish_vehicle(void)
{
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
    app_board_publish(&vehicle_board.sequence, vehicle_board.words, &vehicle,
                      sizeof(vehicle));
}

int vehicle_snapshot_try_read(vehicle_snapshot_t *out)
{
    if (!out) {
        return E_PAR;
    }
    vehicle_snapshot_t v;
    if (!app_board_try_read(&vehicle_board.sequence, vehicle_board.words, &v,
                            sizeof(v))) {
        return E_TMOUT;
    }
    *out = v;
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
    app_board_publish(&obstacle_board.sequence, obstacle_board.words, &report,
                      sizeof(report));
}

/*
 * SCAN module step. It runs the unchanged scan and only observes its outcome.
 */
static vsm_outcome_t scan_step_observed(void *ctx, const vsm_inputs_t *in,
                                        uint32_t now_ms, vsm_drive_t *drive)
{
    const vsm_outcome_t outcome = ss_step(ctx, in, now_ms, drive);

    if (vsm_outcome_type(outcome) == VSM_OUT_SIDE) {
        publish_obstacle((const ss_t *)ctx);
    }
    return outcome;
}

/* Copy the newest obstacle report; see obstacle_report.h. */
int vehicle_obstacle_try_read(rm_obstacle_t *out)
{
    rm_obstacle_t report;

    if (!out) {
        return E_PAR;
    }
    if (!app_board_try_read(&obstacle_board.sequence, obstacle_board.words,
                            &report, sizeof(report))) {
        return (obstacle_board.sequence == 0u) ? E_NOEXS : E_TMOUT;
    }
    *out = report;
    return E_OK;
}

static int vehicle_speed(int16_t l, int16_t r)
{
    const int er = motion_set_speed(l, r);
    if (er >= E_OK) {
        vehicle.left_mm_s = l;
        vehicle.right_mm_s = r;
    }
    return er;
}

static int vehicle_stop(void)
{
    vehicle.left_mm_s = 0;
    vehicle.right_mm_s = 0;
    return motion_stop();
}

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

static void discard_commands(void)
{
    robot_command_t old;
    for (unsigned int i = 0u; i < 9u && robot_command_pop(&old) > 0; ++i) {
        robot_command_lost();
    }
    pending_action = 0u;
}

static int vehicle_estop(void)
{
    discard_commands();
    vehicle.left_mm_s = 0;
    vehicle.right_mm_s = 0;
    live_event(3u, vsm.state, vsm.running, MC_FAULT_ESTOP, 0u);
    return motion_estop();
}

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
                    reason = "remote stop (best effort)";
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
#define publish_vehicle() ((void)0)
#endif

static bool pressed(unsigned int pin, bool *previous)
{
    const bool down = drv_motor_hw_button_down(pin);
    const bool edge = down && !*previous;
    *previous = down;
    return edge;
}

static bool send(telemetry_message_t *message)
{
    if (telemetry_send(message) >= E_OK) {
        return true;
    }
    cancelled = true;
    reason = "telemetry queue unavailable";
    return false;
}

static void event(uint8_t kind, uint8_t from, uint8_t to, uint16_t word)
{
#if TM_MQTT
    live_event(kind == 4u ? 5u : kind, from, to, word, 0u);
    if (kind == 1u && from == VSM_MOD_SCAN) {
        live_event(4u, from, to, word, 0u);
    }
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

static void finish_step(void)
{
    if (!step_open) {
        return;
    }
    publish_vehicle();
    const uint32_t elapsed = drv_time_us() - release_us;
    if (elapsed > response_max_us) {
        response_max_us = elapsed;
    }
    motion_status_t s;
    if (elapsed >= 10000u ||
        (motion_status(&s) >= E_OK && s.ticks != release_tick)) {
        ++overruns;
        if (active) {
            cancelled = true;
            reason = "vehicle deadline missed";
            (void)vehicle_stop();
        }
    }
    step_open = false;
}

static void read_line(void)
{
    if (line_snapshot_read(&sensor, true) < E_OK) {
        if (active) {
            cancelled = true;
            reason = "line snapshot failed";
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
    if (active &&
        (!sensor.valid || drv_time_us() - sensor.timestamp_us > 10000u)) {
        (void)vehicle_estop();
        cancelled = true;
        reason = "line snapshot stale";
    }
    if (active && (sensor.acquisition.missed != line_base.missed ||
                   sensor.acquisition.dropped != line_base.dropped)) {
        (void)vehicle_estop();
        cancelled = true;
        reason = "line samples lost";
    }
}

static bool wait_tick(void)
{
    finish_step();
    if (active && cancelled) {
        return false;
    }
    const ER er = tk_slp_tsk(20);
    const INT extra = tk_can_wup(TSK_SELF);
    if (extra > 0) {
        wake_lost += (uint32_t)extra;
    }
    if (er < E_OK || (active && extra > 0)) {
        cancelled = true;
        reason = "lost motion wake";
    }
    motion_status_t s;
    if (motion_status(&s) >= E_OK) {
        release_tick = s.ticks;
    } else {
        cancelled = true;
        reason = "motion status failed";
    }
    release_us = drv_time_us();
    step_open = true;
    read_line();
#if TM_MQTT
    handle_command();
#endif
    telemetry_stats_t log;
    telemetry_stats(&log);
    if (active && log.storage_full) {
        cancelled = true;
        reason = "telemetry storage full";
    }
    if (active && drv_time_ms() - run_button_ms >= 20u) {
        run_button_ms = drv_time_ms();
        if (pressed(21u, &prev_run)) {
            (void)vehicle_estop();
            cancelled = true;
            reason = "GP21 emergency stop";
        }
    }
    return er >= E_OK && !cancelled;
}

typedef enum {
    GOAL_OUT,
    GOAL_HOME
} goal_t;

static int32_t abs32(int32_t v)
{
    return v < 0 ? -v : v;
}

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
    (void)motion_clear_faults();
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

static void cal_pause(void)
{
    const uint32_t end = drv_time_ms() + CAL_PAUSE_MS;
    while ((int32_t)(drv_time_ms() - end) < 0) {
        if (!wait_tick()) {
            return;
        }
    }
}

static bool calibrate(void)
{
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

typedef struct {
    vsm_module_t module;
    uint8_t id;
} watched_t;

static watched_t watched[4];

static void watched_start(void *ctx, uint16_t payload, uint32_t now)
{
    watched_t *w = ctx;
    w->module.start(w->module.ctx, payload, now);
}

static vsm_outcome_t watched_step(void *ctx, const vsm_inputs_t *in,
                                  uint32_t now, vsm_drive_t *drive)
{
    watched_t *w = ctx;
    const vsm_outcome_t out = w->module.step(w->module.ctx, in, now, drive);
    if (vsm_outcome_type(out) != VSM_OUT_RUNNING) {
        event(1u, w->id, 0u, out);
    }
    return out;
}

static void install(unsigned int index, uint8_t id, vsm_module_t module)
{
    watched[index] = (watched_t){module, id};
    (void)vsm_set_module(
        &vsm, (vsm_module_id_t)id,
        (vsm_module_t){watched_start, watched_step, &watched[index]});
}

static void save_row(uint32_t now, const motion_status_t *s,
                     const vsm_drive_t *d, ultrasonic_result_t echo,
                     uint8_t bits)
{
    const vr_values_t v = {.elapsed_ms = now - run_start,
                           .raw = {raw[0], raw[1]},
                           .echo = echo,
                           .asked = {d->left_mm_s, d->right_mm_s},
                           .measured = {s->speed_mm_s[0], s->speed_mm_s[1]},
                           .position = lp.position,
                           .servo = d->servo_deg,
                           .state = vsm.state,
                           .module = vsm.running,
                           .events = bits};
    vr_row_t row;
    vr_pack(&row, &v);
    telemetry_message_t m = {.kind = TELEMETRY_ROW};
    for (unsigned int i = 0u; i < VR_BYTES; ++i) {
        m.data.row[i] = row.bytes[i];
    }
    (void)send(&m);
}

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

static void sonar_stop(void)
{
    const sonar_request_t r = {
        .generation = ++generation, .id = ++request_id, .mode = SONAR_STOP};
    if (sonar_request(&r) < E_OK && active) {
        cancelled = true;
        reason = "sonar cancel unavailable";
    }
}

static bool request(sonar_request_t *r)
{
    if (sonar_request(r) >= E_OK) {
        return true;
    }
    cancelled = true;
    reason = "sonar request unavailable";
    return false;
}

static void run_once(void)
{
    (void)motion_clear_faults();
    vsm_reset(&vsm);
    read_line();
    line_event = VC_EVENT_NONE;
    run_start = drv_time_ms();
    event(2u, VSM_MOD_NONE, VSM_MOD_LINE, VSM_RUNNING);
    (void)vsm_start(&vsm, VSM_MOD_LINE, 0u, run_start);
    ++generation;
    uint8_t mode = 255u;
    uint16_t interval = 0u;
    bool outstanding = false;
    ultrasonic_result_t echo = ULTRASONIC_RESULT_NONE;
    uint32_t seen_ping = 0u;
    uint32_t seen_result = 0u;
    uint32_t last_ping = run_start;
    uint32_t last_log = run_start - 20u;
    uint32_t request_deadline = 0u;
    uint32_t watch_deadline = 0u;
    int8_t last_servo = 0;
    uint8_t pending_bits = 0u;
    while (!cancelled) {
        if (!wait_tick()) {
            break;
        }
        const uint32_t now = drv_time_ms();
        sonar_snapshot_t sonar = {0};
        bool fresh = false;
        if (sonar_result_read(&sonar, &fresh) < E_OK) {
            reason = "sonar snapshot failed";
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
            reason = "sonar request deadline";
            break;
        }
        if (echo_new && (int32_t)(now - sonar.deadline_ms) > 0) {
            reason = "stale sonar result";
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
            reason = "sonar watch stale";
            break;
        }
        if (mode == SONAR_ONCE && outstanding &&
            (int32_t)(now - request_deadline) > 0) {
            reason = "sonar request deadline";
            break;
        }
        seen_ping = sonar.ping_sequence;
        seen_result = sonar.result_sequence;
        motion_status_t s;
        if (motion_status(&s) < E_OK) {
            reason = "motion status failed";
            break;
        }
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
        if (vsm.state != state_before || vsm.running != module_before) {
            event(
                2u, module_before, vsm.running,
                (uint16_t)((uint16_t)vsm.state | ((uint16_t)vsm.fault << 8u)));
        }
        if (s.faults != 0u) {
            pending_bits |= 128u;
            event(3u, vsm.state, vsm.running, s.faults);
        }
        if (vsm.handoffs != handoffs) {
            pending_bits |=
                (uint8_t)((uint8_t)vsm_outcome_type(vsm.last.outcome) << 4u);
            generation += vsm.handoffs - handoffs;
            mode = 255u;
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
        if (now - last_log >= 20u) {
            save_row(now, &s, &d, echo, pending_bits);
            pending_bits = 0u;
            last_log = now;
        }
        if (vsm.state != VSM_RUNNING) {
            reason = "shell left RUNNING";
            break;
        }
        const uint8_t desired =
            ping_ms == 0u
                ? SONAR_STOP
                : (vsm.running == VSM_MOD_SCAN ? SONAR_ONCE : SONAR_WATCH);
        const uint16_t rate = ping_ms < 60u ? 60u : ping_ms;
        if (desired != mode || (desired == SONAR_WATCH && rate != interval) ||
            (desired == SONAR_ONCE && !outstanding)) {
            sonar_request_t r = {.generation = generation,
                                 .id = ++request_id,
                                 .mode = desired,
                                 .interval_ms = rate,
                                 .deadline_ms = now + 100u,
                                 .not_before_ms = last_ping + ping_ms,
                                 .angle = last_servo};
            if (!request(&r)) {
                break;
            }
            mode = desired;
            interval = rate;
            outstanding = desired == SONAR_ONCE;
            request_deadline = r.deadline_ms;
            watch_deadline = now + (uint32_t)rate * 2u + 20u;
        }
        if (now - run_start >= RUN_LIMIT_MS) {
            reason = "90 s time limit";
            break;
        }
    }
    event(4u, vsm.state, vsm.running, vsm.fault_detail);
    (void)vehicle_stop();
    (void)vehicle_servo(0, 0);
    vsm_stop(&vsm);
    sonar_stop();
}

static void vehicle_task(INT code, void *ctx)
{
    (void)code;
    (void)ctx;
    lt_init(&lt, &LINE_CFG);
    ls_init(&ls, &SEARCH_CFG);
    ss_init(&ss, &SCAN_CFG);
    dr_init(&dr, &DETOUR_CFG);
    vsm_init(&vsm, VSM_DEFAULT_ROUTES, VSM_DEFAULT_ROUTE_COUNT);
    install(0u, VSM_MOD_LINE, lt_module(&lt));
    install(1u, VSM_MOD_SEARCH, ls_module(&ls));
#if TM_MQTT
    install(2u, VSM_MOD_SCAN,
            (vsm_module_t){ss_start, scan_step_observed, &ss});
#else
    install(2u, VSM_MOD_SCAN, ss_module(&ss));
#endif
    install(3u, VSM_MOD_DETOUR, dr_module(&dr));
    drv_motor_hw_buttons_init();
    prev_run = drv_motor_hw_button_down(21u);
    prev_dump = drv_motor_hw_button_down(20u);
    tm_printf((UB *)"[robot] GP21 calibrate then run / ESTOP; GP20 telemetry "
                    "dump. D85.\n");
    if (motion_notify_after_tick(tk_get_tid()) < E_OK) {
        tm_printf((UB *)"ERROR robot: cannot register motion wake\n");
        tk_ext_tsk();
    }
    publish_vehicle();
    uint32_t button_ms = drv_time_ms();
    for (;;) {
        cancelled = false;
        (void)wait_tick();
        const uint32_t now = drv_time_ms();
        if (now - button_ms < 20u) {
            continue;
        }
        button_ms = now;
        const bool run_edge = pressed(21u, &prev_run);
        const bool dump_edge = pressed(20u, &prev_dump);
        telemetry_stats_t log;
        telemetry_stats(&log);
        if (dump_edge && !run_edge && !log.busy) {
            telemetry_message_t m = {.kind = TELEMETRY_DUMP};
            info(&m.data.info);
            (void)telemetry_send(&m);
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
        vsm_reset(&vsm);
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
    const ID tid =
        app_start_task((FP)vehicle_task, 13, TP_PRC1, "vehicle_task");
    return tid < 0 ? (int)tid : E_OK;
}

_Static_assert(VR_BYTES == TELEMETRY_BYTES && VR_ROWS == TELEMETRY_ROWS,
               "KR1 storage contract");
#else
int vehicle_task_start(void)
{
    return E_OK;
}
#endif
#if !TM_MQTT || APP_TEST || APP_BENCH
int vehicle_snapshot_try_read(vehicle_snapshot_t *out)
{
    return out ? E_OBJ : E_PAR;
}

int vehicle_obstacle_try_read(rm_obstacle_t *out)
{
    return out ? E_NOEXS : E_PAR;
}
#endif

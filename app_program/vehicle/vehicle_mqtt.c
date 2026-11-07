/* Network-side adapters pull public boards; sensors never know MQTT. */
#if TM_MQTT
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "../app.h"
#include "vehicle_board.h"
#include "vehicle_log.h"
#include "obstacle_report.h"
#include "odometry.h"
#include "run_stats.h"
#include "../subsystem1/robot_network.h"
#include "../subsystem2/motion.h"
#include "../subsystem2/motor_ctl_robot.h"
#include "../subsystem3/line_service.h"
#include "../subsystem5/sonar_service.h"
#include "../platform/drv_time.h"

/* Motor controller settings: counts per metre and per degree of turn. */
static const mc_config_t motion_config = MC_ROBOT_CONFIG;

typedef struct {
    bool have;
    uint32_t sequence;
    uint32_t observed_ms;
    uint32_t base_age;
} age_t;

static uint16_t age(age_t *a, uint32_t sequence, uint32_t timestamp_us,
                    uint32_t now, uint32_t us, bool success)
{
    if (success && (!a->have || sequence != a->sequence)) {
        a->have = true;
        a->sequence = sequence;
        a->observed_ms = now;
        a->base_age = (us - timestamp_us) / 1000u;
    }
    if (!a->have) {
        return 65535u;
    }
    const uint64_t elapsed =
        (uint64_t)a->base_age + (uint32_t)(now - a->observed_ms);
    return elapsed > 65535u ? 65535u : (uint16_t)elapsed;
}

void robot_network_state(rm_state_t *out)
{
    static line_snapshot_t line;
    static motion_status_t motion;
    static vehicle_snapshot_t vehicle;
    static age_t la;
    static age_t ma;
    static age_t va;
    static uint32_t motion_us;
    const bool l = line_snapshot_try_read(&line) >= E_OK;
    const bool m = motion_snapshot_try_read(&motion, &motion_us) >= E_OK;
    const bool v = vehicle_snapshot_try_read(&vehicle) >= E_OK;
    sonar_snapshot_t sonar;
    const bool s = sonar_snapshot_try_read(&sonar) >= E_OK;
    /* Writers may preempt this reader. Observe time AFTER the copies, so a
     * newly published timestamp cannot appear to be in our future. */
    const uint32_t now = drv_time_ms();
    const uint32_t us = drv_time_us();
    if (!l) {
        robot_network_busy(ROBOT_BOARD_LINE);
    }
    if (!m) {
        robot_network_busy(ROBOT_BOARD_MOTION);
    }
    if (!v) {
        robot_network_busy(ROBOT_BOARD_VEHICLE);
    }
    *out = (rm_state_t){0};
    out->line_age_ms = age(&la, line.sequence, line.timestamp_us, now, us, l);
    out->motion_age_ms = age(&ma, motion.ticks, motion_us, now, us, m);
    out->vehicle_age_ms =
        age(&va, vehicle.sequence, vehicle.timestamp_us, now, us, v);
    if (line.valid) {
        out->flags |= 1u;
    }
    if (l && line.valid && out->line_age_ms <= 10u) {
        out->flags |= 2u;
    }
    if (m && motion.ticks > 0u && out->motion_age_ms <= 20u) {
        out->flags |= 4u;
    }
    if (v && out->vehicle_age_ms <= 30u) {
        out->flags |= 8u;
    }
    if (!l) {
        out->flags |= 16u;
    }
    if (!m) {
        out->flags |= 32u;
    }
    if (!v) {
        out->flags |= 64u;
    }
    if (vehicle.active) {
        out->flags |= 128u;
    }
    out->state = vehicle.state;
    out->module = vehicle.module;
    out->motion_faults = motion.faults;
    out->raw_bc = line.raw_bc;
    out->norm_l = line.line.norm[0];
    out->norm_c = line.line.norm[1];
    out->count_l = motion.counts[0];
    out->count_r = motion.counts[1];
    out->dist_left_mm = motion.travel_mm[MC_LEFT];
    out->dist_right_mm = motion.travel_mm[MC_RIGHT];
    out->dist_avg_mm = rm_average_mm(out->dist_left_mm, out->dist_right_mm);
    const odo_config_t odometry = {motion_config.counts_per_m,
                                   motion_config.counts_per_deg_x100};
    out->duty_left_permille = motion.duty_permille[MC_LEFT];
    out->duty_right_permille = motion.duty_permille[MC_RIGHT];
    out->heading_cdeg = motion.heading_cdeg;
    out->turn_rate_ddps =
        odo_turn_rate_ddps(&odometry, motion.speed_mm_s[MC_LEFT],
                           motion.speed_mm_s[MC_RIGHT]);
    ultrasonic_result_t echo = ULTRASONIC_RESULT_NONE;
    if (s) {
        if (sonar.valid && sonar.status == SONAR_ECHO &&
            now - sonar.timestamp_ms <= 100u) {
            echo = sonar.echo;
        }
    } else {
        robot_network_busy(ROBOT_BOARD_SONAR);
    }
    const vr_values_t values = {
        .elapsed_ms = now,
        .raw = {line.raw[0], line.raw[1]},
        .echo = echo,
        .asked = {vehicle.left_mm_s, vehicle.right_mm_s},
        .measured = {motion.speed_mm_s[0], motion.speed_mm_s[1]},
        .position = line.line.position,
        .servo = vehicle.servo_deg,
        .state = vehicle.state,
        .module = vehicle.module};
    vr_row_t row;
    vr_pack(&row, &values);
    for (unsigned int i = 0u; i < 16u; ++i) {
        out->kr1[i] = row.bytes[i];
    }
}

int robot_network_sonar(uint32_t *cursor, rm_sonar_t *out, uint32_t *missed)
{
    sonar_history_entry_t entry;
    const int er = sonar_history_read(cursor, &entry, missed);
    if (er < E_OK) {
        return er;
    }
    const sonar_snapshot_t *s = &entry.result;
    *out = (rm_sonar_t){.status = s->status,
                        .valid = s->valid,
                        .pending = s->pending,
                        .angle = s->angle,
                        .ping_angle = s->ping_angle,
                        .history_sequence = entry.sequence,
                        .generation = s->generation,
                        .request_id = s->id,
                        .trigger_ms = s->trigger_ms,
                        .timestamp_ms = s->timestamp_ms,
                        .echo = s->echo,
                        .ping_sequence = s->ping_sequence};
    return E_OK;
}

/*
 * Pull the newest obstacle report if it is new. The vehicle board holds only
 * the last report, so a gap in scan_id shows missed reports.
 * Returns E_OK for a new report (cursor moves to its scan_id), E_NOEXS if
 * nothing is new, or an error if the board was busy.
 */
int robot_network_obstacle(uint32_t *cursor, rm_obstacle_t *out)
{
    rm_obstacle_t report;
    const int result = vehicle_obstacle_try_read(&report);

    if (result == E_NOEXS) {
        return E_NOEXS;
    }
    if (result < E_OK) {
        robot_network_busy(ROBOT_BOARD_VEHICLE);
        return result;
    }
    if (report.scan_id == *cursor) {
        return E_NOEXS;
    }
    *out = report;
    *cursor = report.scan_id;
    return E_OK;
}

/* Pull the newest detour summary if it is new; see robot_network.h. */
int robot_network_detour(uint32_t *cursor, rm_detour_t *out)
{
    rm_detour_t value;
    const int result = vehicle_detour_try_read(&value);

    if (result == E_NOEXS) {
        return E_NOEXS;
    }
    if (result < E_OK) {
        robot_network_busy(ROBOT_BOARD_VEHICLE);
        return result;
    }
    if (value.sequence == *cursor) {
        return E_NOEXS;
    }
    *out = value;
    *cursor = value.sequence;
    return E_OK;
}

/* Pull the newest run summary if it is new; see robot_network.h. */
int robot_network_summary(uint32_t *cursor, rm_summary_t *out)
{
    rm_summary_t value;
    const int result = vehicle_summary_try_read(&value);

    if (result == E_NOEXS) {
        return E_NOEXS;
    }
    if (result < E_OK) {
        robot_network_busy(ROBOT_BOARD_VEHICLE);
        return result;
    }
    if (value.run_id == *cursor) {
        return E_NOEXS;
    }
    *out = value;
    *cursor = value.run_id;
    return E_OK;
}

void robot_health_collect(rm_heartbeat_t *out)
{
    static motion_status_t motion;
    static vehicle_snapshot_t vehicle;
    uint32_t us;
    if (motion_snapshot_try_read(&motion, &us) < E_OK) {
        robot_network_busy(ROBOT_BOARD_MOTION);
    }
    if (vehicle_snapshot_try_read(&vehicle) < E_OK) {
        robot_network_busy(ROBOT_BOARD_VEHICLE);
    }
    out->motion_late = motion.late_wakes;
    out->motion_missed = motion.missed_wakes;
    out->vehicle_overruns = vehicle.overruns;
}
#endif

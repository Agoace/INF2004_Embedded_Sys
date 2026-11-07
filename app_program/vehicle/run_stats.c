/*
 * run_stats.c - pure observer for the run summary and the detour summary.
 * See run_stats.h.
 */
#include "run_stats.h"

#include <string.h>

#define RS_HALF_COUNT_UNITS 2u
#define RS_MM_PER_M         1000u
#define RS_Q15_SHIFT        15

static int64_t abs64(int64_t value)
{
    return (value < 0) ? -value : value;
}

static uint16_t sat16(uint32_t value)
{
    return (value > 0xFFFFu) ? 0xFFFFu : (uint16_t)value;
}

static uint16_t inc16(uint16_t value)
{
    return (value == 0xFFFFu) ? value : (uint16_t)(value + 1u);
}

/* Sum of left + right steps (half counts) to millimetres of travel. */
static uint32_t half_counts_to_mm(const rs_t *rs, uint64_t half_counts)
{
    const uint64_t divisor =
        (uint64_t)rs->cfg.odo.counts_per_m * RS_HALF_COUNT_UNITS;

    return (uint32_t)(half_counts * RS_MM_PER_M / divisor);
}

void rs_init(rs_t *rs, const rs_config_t *cfg)
{
    memset(rs, 0, sizeof(*rs));
    rs->cfg = *cfg;
}

void rs_run_begin(rs_t *rs, uint32_t now_ms)
{
    const rs_config_t cfg = rs->cfg;
    const uint32_t runs = rs->runs + 1u;
    const uint32_t detours = rs->detour_sequence;

    rs_init(rs, &cfg);
    rs->runs = runs;
    rs->detour_sequence = detours;
    rs->running = true;
    rs->start_ms = now_ms;
}

/* Largest wheel error while the command was steady. */
static void track_speed_error(rs_t *rs, uint32_t now_ms,
                              const int16_t measured[2],
                              const int16_t command[2])
{
    if (command[0] != rs->command[0] || command[1] != rs->command[1]) {
        rs->command[0] = command[0];
        rs->command[1] = command[1];
        rs->command_since_ms = now_ms;
        return;
    }
    if ((uint32_t)(now_ms - rs->command_since_ms) < RS_STEADY_MS) {
        return;
    }
    for (unsigned int wheel = 0u; wheel < 2u; ++wheel) {
        const uint32_t error =
            (uint32_t)abs64((int64_t)command[wheel] - (int64_t)measured[wheel]);

        rs->speed_error_valid = true;
        if (error > rs->speed_error_max) {
            rs->speed_error_max = sat16(error);
        }
    }
}

/* Add one step of travel to the open detour. */
static void integrate_detour(rs_t *rs, const int32_t counts[2],
                             int32_t step_sum)
{
    const int32_t heading =
        odo_heading_cdeg(&rs->cfg.odo, counts[0] - rs->detour_origin[0],
                         counts[1] - rs->detour_origin[1]);

    rs->detour_half_counts += (uint64_t)abs64(step_sum);
    rs->detour_y += (int64_t)step_sum * odo_sin_q15(heading);
    if (abs64(rs->detour_y) > rs->detour_y_max) {
        rs->detour_y_max = abs64(rs->detour_y);
    }
    ++rs->detour_steps;
}

void rs_step(rs_t *rs, uint32_t now_ms, const int32_t counts[2],
             const int16_t measured_mm_s[2], const int16_t command_mm_s[2],
             uint8_t motion_faults)
{
    if (!rs->running) {
        return;
    }
    rs->motion_fault_bits |= motion_faults;
    if (!rs->origin_set) {
        rs->origin_set = true;
        rs->last[0] = counts[0];
        rs->last[1] = counts[1];
        rs->command[0] = command_mm_s[0];
        rs->command[1] = command_mm_s[1];
        rs->command_since_ms = now_ms;
        return;
    }
    const int32_t step_sum =
        (counts[0] - rs->last[0]) + (counts[1] - rs->last[1]);

    rs->path_half_counts += (uint64_t)abs64(step_sum);
    rs->last[0] = counts[0];
    rs->last[1] = counts[1];
    if (rs->detour_active) {
        integrate_detour(rs, counts, step_sum);
    }
    track_speed_error(rs, now_ms, measured_mm_s, command_mm_s);
}

void rs_detour_begin(rs_t *rs, uint32_t now_ms, uint8_t side, uint32_t scan_id)
{
    if (!rs->running || rs->detour_active) {
        return;
    }
    rs->detour_active = true;
    rs->detour_side = side;
    rs->detour_scan_id = scan_id;
    rs->detour_start_ms = now_ms;
    rs->detour_origin[0] = rs->last[0];
    rs->detour_origin[1] = rs->last[1];
    rs->detour_half_counts = 0u;
    rs->detour_steps = 0u;
    rs->detour_y = 0;
    rs->detour_y_max = 0;
    ++rs->detours_started;
}

/* Close the open detour into *out. */
static void close_detour(rs_t *rs, uint8_t outcome, uint16_t payload,
                         uint32_t now_ms, rm_detour_t *out)
{
    const uint64_t y_counts = (uint64_t)rs->detour_y_max /
                              ((uint64_t)RS_HALF_COUNT_UNITS << RS_Q15_SHIFT);
    const uint32_t duration = now_ms - rs->detour_start_ms;

    memset(out, 0, sizeof(*out));
    out->outcome = outcome;
    out->side = rs->detour_side;
    out->flags = (rs->detour_steps > 0u) ? RM_DETOUR_LATERAL_VALID : 0u;
    out->sequence = ++rs->detour_sequence;
    out->start_ms = rs->detour_start_ms;
    out->end_ms = now_ms;
    out->duration_ms = duration;
    out->distance_mm = half_counts_to_mm(rs, rs->detour_half_counts);
    out->max_lateral_mm = sat16(odo_counts_to_mm(&rs->cfg.odo, y_counts));
    out->payload = payload;
    out->scan_id = rs->detour_scan_id;
    rs->detour_total_ms += duration;
    rs->detour_active = false;
}

bool rs_outcome(rs_t *rs, uint8_t module_id, vsm_outcome_type_t type,
                uint16_t payload, uint32_t now_ms, rm_detour_t *detour)
{
    if (!rs->running) {
        return false;
    }
    if (type == VSM_OUT_LINE_LOST) {
        rs->line_losses = inc16(rs->line_losses);
    }
    if (type == VSM_OUT_OBSTACLE) {
        rs->obstacles = inc16(rs->obstacles);
    }
    if (module_id != VSM_MOD_DETOUR || !rs->detour_active) {
        return false;
    }
    if (type == VSM_OUT_DONE_LINE) {
        rs->detours_done = inc16(rs->detours_done);
        rs->detours_succeeded = inc16(rs->detours_succeeded);
        close_detour(rs, RM_DETOUR_DONE_LINE, payload, now_ms, detour);
        return true;
    }
    if (type == VSM_OUT_DONE_NO_LINE) {
        rs->detours_done = inc16(rs->detours_done);
        close_detour(rs, RM_DETOUR_DONE_NO_LINE, payload, now_ms, detour);
        return true;
    }
    if (type == VSM_OUT_FAILED) {
        close_detour(rs, RM_DETOUR_FAILED, payload, now_ms, detour);
        return true;
    }
    return false;
}

uint8_t rs_stop_source(uint8_t stop_reason)
{
    switch (stop_reason) {
    case RM_STOP_UNKNOWN:
        return RM_STOP_SOURCE_NONE;
    case RM_STOP_GP21:
        return RM_STOP_SOURCE_GP21;
    case RM_STOP_MQTT_ESTOP:
        return RM_STOP_SOURCE_MQTT;
    case RM_STOP_TIME_LIMIT:
        return RM_STOP_SOURCE_LIMIT;
    default:
        return RM_STOP_SOURCE_FAULT;
    }
}

bool rs_run_end(rs_t *rs, uint32_t now_ms, uint8_t stop_reason,
                uint8_t shell_fault, uint16_t shell_fault_detail,
                rm_summary_t *summary, rm_detour_t *aborted)
{
    bool was_open = false;

    if (rs->detour_active) {
        close_detour(rs, RM_DETOUR_ABORTED, 0u, now_ms, aborted);
        was_open = true;
    }
    memset(summary, 0, sizeof(*summary));
    summary->flags =
        (uint8_t)((rs->speed_error_valid ? RM_SUMMARY_SPEED_ERROR_VALID : 0u) |
                  (was_open ? RM_SUMMARY_DETOUR_ABORTED : 0u));
    summary->stop_source = rs_stop_source(stop_reason);
    summary->stop_reason = stop_reason;
    summary->run_id = rs->runs;
    summary->start_ms = rs->start_ms;
    summary->duration_ms = now_ms - rs->start_ms;
    summary->distance_mm = half_counts_to_mm(rs, rs->path_half_counts);
    summary->line_losses = rs->line_losses;
    summary->obstacles = rs->obstacles;
    summary->detours_done = rs->detours_done;
    summary->detours_succeeded = rs->detours_succeeded;
    summary->detour_total_ms = rs->detour_total_ms;
    summary->detours_started = rs->detours_started;
    summary->max_speed_error_mm_s =
        rs->speed_error_valid ? rs->speed_error_max : RM_SUMMARY_NONE_U16;
    summary->motion_fault_bits = rs->motion_fault_bits;
    summary->shell_fault = shell_fault;
    summary->shell_fault_detail = shell_fault_detail;
    summary->hump_peak = RM_SUMMARY_NONE_U16;
    rs->running = false;
    return was_open;
}

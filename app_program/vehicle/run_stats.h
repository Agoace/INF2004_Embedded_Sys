/*
 * run_stats.h - pure observer for the run summary and the detour summary.
 *
 * The vehicle task feeds this unit the data it already has (wheel counts,
 * measured and commanded wheel speed, module outcomes). The unit only counts
 * and measures. It never returns a drive command and never changes a module
 * input, so it cannot change how the robot drives.
 *
 * Odometry comes from the wheel encoders only (odometry.h). The heading
 * origin of a detour is the robot heading when the DETOUR module starts.
 *
 * No hardware access. Host-tested by tests/test_run_stats.c.
 */
#ifndef RUN_STATS_H
#define RUN_STATS_H

#include <stdbool.h>
#include <stdint.h>

#include "odometry.h"
#include "../subsystem1/mqtt_wire.h"
#include "vehicle_sm.h"

/* The wheel speed command must stay the same this long before its error counts. */
#define RS_STEADY_MS 300u

typedef struct {
    odo_config_t odo;
} rs_config_t;

typedef struct {
    rs_config_t cfg;
    uint32_t runs;            /* runs since init */
    uint32_t detour_sequence; /* detours since init */
    bool running;
    bool origin_set;
    uint32_t start_ms;
    int32_t last[2];           /* wheel counts of the last step */
    uint64_t path_half_counts; /* sum of |left + right| steps */
    uint16_t line_losses;
    uint16_t obstacles;
    uint16_t detours_started;
    uint16_t detours_done;
    uint16_t detours_succeeded;
    uint32_t detour_total_ms;
    uint8_t motion_fault_bits;
    int16_t command[2];
    uint32_t command_since_ms;
    uint16_t speed_error_max;
    bool speed_error_valid;
    bool detour_active;
    uint8_t detour_side;
    uint32_t detour_scan_id;
    uint32_t detour_start_ms;
    int32_t detour_origin[2];
    uint64_t detour_half_counts;
    uint32_t detour_steps;
    int64_t detour_y;     /* sum of step * sin(heading), q15 half counts */
    int64_t detour_y_max; /* largest |detour_y| */
} rs_t;

void rs_init(rs_t *rs, const rs_config_t *cfg);

/* A run starts. Clears the run totals. */
void rs_run_begin(rs_t *rs, uint32_t now_ms);

/*
 * One vehicle step, before the shell runs. counts: wheel counts, measured:
 * wheel speed (mm/s), command: the wheel speed commanded in the previous
 * step, motion_faults: MC_FAULT_* bits.
 */
void rs_step(rs_t *rs, uint32_t now_ms, const int32_t counts[2],
             const int16_t measured_mm_s[2], const int16_t command_mm_s[2],
             uint8_t motion_faults);

/*
 * The DETOUR module starts (start payload from the SCAN module). Uses the
 * wheel counts of the latest rs_step(). A second start while a detour is
 * open is ignored.
 */
void rs_detour_begin(rs_t *rs, uint32_t now_ms, uint8_t side, uint32_t scan_id);

/*
 * A module made an outcome (not VSM_OUT_RUNNING). Counts line losses and
 * obstacles. When the DETOUR module ends its job, fills *detour and returns
 * true.
 */
bool rs_outcome(rs_t *rs, uint8_t module_id, vsm_outcome_type_t type,
                uint16_t payload, uint32_t now_ms, rm_detour_t *detour);

/*
 * The run ended. Fills the summary. If a detour was open it ends with
 * RM_DETOUR_ABORTED; then fills *aborted and returns true.
 */
bool rs_run_end(rs_t *rs, uint32_t now_ms, uint8_t stop_reason,
                uint8_t shell_fault, uint16_t shell_fault_detail,
                rm_summary_t *summary, rm_detour_t *aborted);

/* Source class (RM_STOP_SOURCE_*) of a stop reason (RM_STOP_*). */
uint8_t rs_stop_source(uint8_t stop_reason);

/*
 * Copy the newest detour summary or run summary from the vehicle task. E_OK
 * for a copy, E_NOEXS if none exists yet, E_TMOUT if the board was busy, E_PAR
 * for a null pointer. A failed read leaves *out unchanged. The vehicle task
 * keeps only the newest one, so a gap in sequence or run_id shows a miss.
 */
int vehicle_detour_try_read(rm_detour_t *out);
int vehicle_summary_try_read(rm_summary_t *out);

#endif

/*
 * run_timing.h - P1 report-only per-run timing latches (core 0).
 *
 * Each latch word has ONE writer task or handler. Vehicle starts a run with
 * rt_begin(odd epoch) and stops it with rt_begin(even epoch). A writer that
 * sees a new odd epoch clears only its own words first, so no word is ever
 * written by two contexts. Readers copy the words with atomic loads; a copy
 * taken during a run is not a coherent snapshot, only a set of maxima.
 * An even epoch freezes the values for the post-STOP dump.
 *
 * All values are wall-clock microseconds from the 1 MHz TIMER. A higher-
 * priority interrupt or task inside a measured interval is included.
 */
#ifndef RUN_TIMING_H
#define RUN_TIMING_H

#include <stdint.h>

/* Max-only latches. The STEP_* values also name the slowest step part. */
typedef enum {
    RT_ISR_LATE,      /* alarm_irq entry minus its target time */
    RT_ISR_ADC,       /* drv_line_hw_read in alarm_irq */
    RT_ISR_WUP,       /* whole alarm_irq tk_wup_tsk, also in RT_K_ISR_WUP */
    RT_LINE_SAMPLE,   /* line_task one sample, with the barcode sample */
    RT_LINE_LPU,      /* lp_update */
    RT_LINE_BARCODE,  /* barcode_live_source_sample */
    RT_STEP_WAKE,     /* motion publish -> vehicle running */
    RT_STEP_READ_LINE,
    RT_STEP_COMMAND,  /* handle_command, telemetry stats, button */
    RT_STEP_SONAR,    /* hump poll, sonar read and checks */
    RT_STEP_VSM,      /* motion status through actuator requests */
    RT_STEP_LOG,      /* save_row, sonar request, limits */
    RT_STEP_PUBLISH,  /* finish_step publish_vehicle */
    RT_STEP_STATUS,   /* finish_step motion_status */
    RT_MAX_COUNT
} rt_max_t;

/* Per-call {count, sum, max} sites. */
typedef enum {
    RT_K_ISR_WUP,     /* alarm_irq tk_wup_tsk(listener) */
    RT_K_LINE_WAKE,   /* alarm_irq exit -> line_task after tk_slp_tsk */
    RT_K_LINE_CAN,    /* line_task tk_can_wup */
    RT_K_LINE_LOC,    /* line_task tk_loc_mtx */
    RT_K_LINE_UNL,    /* line_task tk_unl_mtx */
    RT_K_MOT_WUP,     /* motion_task tk_wup_tsk(notify) */
    RT_K_VEH_WAKE,    /* motion publish -> vehicle after tk_slp_tsk */
    RT_K_VEH_CAN,     /* vehicle tk_can_wup */
    RT_K_VEH_STATUS,  /* vehicle motion_status in wait_tick */
    RT_K_VEH_SEND,    /* vehicle telemetry_send */
    RT_CALL_COUNT
} rt_call_t;

typedef struct {
    uint32_t count;
    uint32_t sum_us;
    uint32_t max_us;
} rt_call_stats_t;

/* First overrun of a run (vehicle deadline check). step == 0: none. */
typedef struct {
    uint32_t step;       /* vehicle step number in the run, from 1 */
    uint32_t elapsed_us; /* release to finish_step */
    uint32_t tick_delta; /* motion ticks since release */
    uint32_t slow_part;  /* rt_max_t of the slowest part of that step */
    uint32_t slow_us;
    uint32_t wake_us;    /* wake latency of that step */
    uint32_t uptime_ms;
} rt_overrun_t;

/* Kernel big-lock statistics of one run, both cores (SMP builds only). */
typedef struct {
    uint32_t valid;       /* 0: no SMP kernel or no record */
    uint32_t acquisitions;
    uint32_t contentions;
    uint32_t max_wait_us;
    uint32_t max_hold_us;
    uint32_t max_depth;
    uint32_t detached;    /* lock released around user callbacks */
    uint32_t run_ms;      /* time from reset to copy */
} rt_bkl_t;

typedef struct {
    uint32_t epoch;
    uint32_t load_epoch;  /* run of the load figures below */
    uint32_t load_samples;
    uint16_t load_peak[2]; /* permille; 65535 = unknown */
    rt_bkl_t bkl;
    uint32_t max[RT_MAX_COUNT];
    rt_call_stats_t call[RT_CALL_COUNT];
    rt_overrun_t first;
} rt_snapshot_t;

/* Vehicle only: odd epoch starts a run, even epoch freezes it. */
void rt_begin(uint32_t epoch);
/* Any reader: the current epoch (odd: run, even: frozen). */
uint32_t rt_epoch(void);
/* Health only: the per-run peak load (cpu_peak_t fields). Always stored. */
void rt_load(uint32_t epoch, uint32_t samples, uint16_t peak0,
             uint16_t peak1);
/* Vehicle only, before the STOP rt_begin. Ignored while frozen. */
void rt_bkl(const rt_bkl_t *record);
/* Owner of `which` only. Ignored while frozen. */
void rt_max(rt_max_t which, uint32_t us);
/* Owner of `which` only. Ignored while frozen. */
void rt_call(rt_call_t which, uint32_t us);
/* Vehicle only. Keeps the first record of the run. */
void rt_overrun(const rt_overrun_t *record);
/* Motion only: TIMER time when its notify wake call returned. Always
 * written, also while frozen; Vehicle reads it for the wake latency. */
void rt_motion_stamp(uint32_t us);
uint32_t rt_motion_stamp_us(void);
/* Any reader. */
void rt_snapshot(rt_snapshot_t *out);
/* Print [load_run] [bkl] [isr_max] [line_max] [step_max] [kcall]
 * [first_overrun] lines. */
void rt_report(void);

#endif

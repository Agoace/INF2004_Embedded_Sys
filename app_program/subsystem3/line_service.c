/*
 * line_service.c - application telemetry and service interface.
 */
#if APP_BARCODE_CAPTURE
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#endif
#include "subsystem3.h"

/* Task timing and resource limits. */
#define LINE_US_PER_MS     1000u
#define LINE_TASK_PRIORITY 11

#include "line_service.h"
#include "barcode_live_source.h"
#if APP_BARCODE_CAPTURE
#include "barcode_capture.h"
#endif
#include "../platform/drv_time.h"
static ID mutex;
static ID task_id;
static bool ready;
static bool calibrating;
static line_snapshot_t snapshot;
#include "../platform/app_board.h"
static APP_BOARD(line_snapshot_t) board;
static barcode_calibration_t barcode_calibration;
#if !APP_TEST && !APP_BENCH
static barcode_cal_report_t live_calibration;
#endif
static APP_BOARD(barcode_cal_report_t) barcode_calibration_board;

/* Publish only lifecycle boundaries, not another per-sample board. */
static void publish_barcode_calibration(void)
{
    app_board_publish(&barcode_calibration_board.sequence,
                      barcode_calibration_board.words,
                      &barcode_calibration.report,
                      sizeof(barcode_calibration.report));
}

/* A separate board leaves the existing LINE snapshot and wire sizes intact. */
int line_barcode_calibration_try_read(barcode_cal_report_t *out)
{
    if (!out) {
        return E_PAR;
    }
    if (!ready) {
        return E_OBJ;
    }
    barcode_cal_report_t report;
    if (!app_board_try_read(&barcode_calibration_board.sequence,
                           barcode_calibration_board.words, &report,
                           sizeof(report))) {
        return E_TMOUT;
    }
    *out = report;
    return E_OK;
}

/* Called after unlocking, by the calibration caller, not acquisition/ISR. */
static void log_barcode_calibration(const barcode_cal_report_t *report)
{
    const char *mode = "INVALID";
    if (report->mode == BC_CAL_LEARNED) {
        mode = "LEARNED";
    } else if (report->mode == BC_CAL_FALLBACK) {
        mode = "FALLBACK";
    } else if (report->mode == BC_CAL_ABORTED) {
        mode = "ABORTED";
    }
    tm_printf((UB *)"[BC_CAL] gen %lu samples %lu GP28 %lu..%lu "
              "mode %s faults %02lx\n",
              (unsigned long)report->generation,
              (unsigned long)report->samples,
              (unsigned long)report->minimum,
              (unsigned long)report->maximum, mode,
              (unsigned long)report->faults);
    tm_printf((UB *)"[BC_CAL] white %lu/%lu black %lu/%lu hold_us %lu "
              "gap_us %lu rise %lu fall %lu; shadow only\n",
              (unsigned long)report->white_mean,
              (unsigned long)report->white_samples,
              (unsigned long)report->black_mean,
              (unsigned long)report->black_samples,
              (unsigned long)report->black_hold_us,
              (unsigned long)report->max_gap_us,
              (unsigned long)report->rise,
              (unsigned long)report->fall);
}

/*
 * Publish the latest line result for bounded network reads.
 */
static void publish_line(void)
{
    app_board_publish(&board.sequence, board.words, &snapshot,
                      sizeof(snapshot));
}

/*
 * Attempt one read of the public line board without waiting.
 */
int line_snapshot_try_read(line_snapshot_t *out)
{
    if (!out) {
        return E_PAR;
    }
    if (!ready) {
        return E_OBJ;
    }
    line_snapshot_t v;
    if (!app_board_try_read(&board.sequence, board.words, &v, sizeof(v))) {
        return E_TMOUT;
    }
    *out = v;
    return E_OK;
}

/*
 * Process queued line samples and publish the current detector state.
 */
static void line_task(INT code, void *ctx)
{
    (void)code;
    (void)ctx;
    for (;;) {
        if (tk_slp_tsk(TMO_FEVR) < E_OK) {
            continue;
        }
        (void)tk_can_wup(TSK_SELF);
        drv_line_sample_t sample;
        while (drv_line_take(&sample)) {
            const uint32_t start = drv_time_us();
            if (tk_loc_mtx(mutex, TMO_FEVR) < E_OK) {
                continue;
            }
            snapshot.raw[0] = sample.raw[0];
            snapshot.raw[1] = sample.raw[1];
            snapshot.raw[2] = 0u;
            snapshot.raw_bc = sample.raw[2];
            if (calibrating) {
                lp_calib_add(&snapshot.line, snapshot.raw);
                barcode_calibration_add(&barcode_calibration, sample.us,
                                        sample.raw[2]);
            } else {
                const uint32_t ms =
                    drv_time_ms() -
                    (drv_time_us() - sample.us) / LINE_US_PER_MS;
                const vc_event_t event =
                    lp_update(&snapshot.line, snapshot.raw, ms);
                if (vc_event_type(event) == VC_EVENT_LINE_LOST ||
                    vc_event_type(event) == VC_EVENT_LINE_FOUND) {
                    snapshot.event = event;
                }
                if (snapshot.line.norm[LP_CENTRE] > snapshot.norm_peak) {
                    snapshot.norm_peak = snapshot.line.norm[LP_CENTRE];
                }
            }
            snapshot.timestamp_us = sample.us;
            snapshot.valid = true;
            ++snapshot.sequence;
            ++snapshot.processed;
            drv_line_stats(&snapshot.acquisition);
#if !APP_TEST && !APP_BENCH
            live_calibration = barcode_calibration.report;
            const uint32_t missed = snapshot.acquisition.missed;
            const uint32_t dropped = snapshot.acquisition.dropped;
#endif
            publish_line();
            const uint32_t cost = drv_time_us() - start;
            snapshot.cpu_us += cost;
            if (cost > snapshot.sample_max_us) {
                snapshot.sample_max_us = cost;
            }
            (void)tk_unl_mtx(mutex);
#if !APP_TEST && !APP_BENCH
            /* Same acquisition owner, every GP28 sample. No decoder work,
             * motion wait, logging or extra board work inside LINE mutex. */
            barcode_live_source_sample(sample.us, sample.raw[2],
                                       &live_calibration, missed, dropped);
#endif
#if APP_BARCODE_CAPTURE
            /* Bounded TEST-only copy, never a second acquisition consumer.
             * No decoder, print or motion lock in this priority-11 path. */
            const uint32_t capture_start = drv_time_us();
            barcode_capture_observe(sample.us, sample.raw[2]);
            barcode_capture_cost(drv_time_us() - capture_start);
#endif
        }
    }
}

/*
 * Create the line service objects and initialize detector state.
 */
int line_service_init(void)
{
    if (ready) {
        return E_OK;
    }
    if (mutex <= 0) {
        const T_CMTX c = {.mtxatr = TA_INHERIT};
        mutex = tk_cre_mtx(&c);
    }
    if (mutex < 0) {
        return (int)mutex;
    }
    lp_init(&snapshot.line);
    lp_set_pair(&snapshot.line, true, 0);
    const int er = drv_line_init();
    if (er >= E_OK) {
        /* S3 initializes its invalid shadow record before either task runs.
         * Vehicle remains the sole future control-board writer. */
        const int source = barcode_live_source_init();
        if (source < E_OK) {
            return source;
        }
        ready = true;
        publish_line();
        publish_barcode_calibration();
    }
    return er;
}

/*
 * Start the line task and its acquisition source.
 */
int line_service_start(void)
{
    if (!ready) {
        return E_OBJ;
    }
    if (task_id > 0) {
        return E_OBJ;
    }
    task_id =
        app_start_task((FP)line_task, LINE_TASK_PRIORITY, TP_PRC1, "line_task");
    if (task_id < 0) {
        return (int)task_id;
    }
    drv_line_listen((int)task_id);
    return E_OK;
}

/*
 * Report whether the line task has started.
 */
bool line_service_running(void)
{
    return ready && task_id > 0;
}

/*
 * Copy detector state under the line service mutex.
 */
int line_snapshot_read(line_snapshot_t *out, bool consume)
{
    if (!out) {
        return E_PAR;
    }
    if (!ready) {
        return E_OBJ;
    }
    const ER er = tk_loc_mtx(mutex, TMO_FEVR);
    if (er < 0) {
        return (int)er;
    }
    *out = snapshot;
    if (consume) {
        snapshot.event = VC_EVENT_NONE;
        publish_line();
    }
    return (int)tk_unl_mtx(mutex);
}

/*
 * Ask the line owner to begin collecting calibration extrema.
 */
int line_calibration_begin(void)
{
    if (!ready) {
        return E_OBJ;
    }
    const ER er = tk_loc_mtx(mutex, TMO_FEVR);
    if (er < 0) {
        return (int)er;
    }
    drv_line_flush();
    lp_calib_begin(&snapshot.line);
    barcode_calibration_begin(&barcode_calibration);
    publish_barcode_calibration();
    calibrating = true;
    snapshot.event = VC_EVENT_NONE;
    snapshot.norm_peak = 0u;
    snapshot.valid = false;
    publish_line();
    return (int)tk_unl_mtx(mutex);
}

/*
 * Finish calibration through the line owner and copy its result.
 */
lp_calib_result_t line_calibration_end(void)
{
    if (!ready || tk_loc_mtx(mutex, TMO_FEVR) < 0) {
        return LP_CALIB_NO_DATA;
    }
    unsigned int bad = 0u;
    const lp_calib_result_t result = lp_calib_end(&snapshot.line, &bad);
    barcode_calibration_end(&barcode_calibration, result == LP_CALIB_OK);
    const barcode_cal_report_t report = barcode_calibration.report;
    publish_barcode_calibration();
    calibrating = false;
    snapshot.norm_peak = 0u;
    publish_line();
    (void)tk_unl_mtx(mutex);
    log_barcode_calibration(&report);
    return result;
}

/*
 * Abort calibration without replacing retained calibration values.
 */
int line_calibration_abort(void)
{
    if (!ready) {
        return E_OBJ;
    }
    const ER er = tk_loc_mtx(mutex, TMO_FEVR);
    if (er < 0) {
        return (int)er;
    }
    barcode_calibration_end(&barcode_calibration, false);
    const barcode_cal_report_t report = barcode_calibration.report;
    publish_barcode_calibration();
    calibrating = false;
    snapshot.norm_peak = 0u;
    snapshot.event = VC_EVENT_NONE;
    publish_line();
    const ER unlock = tk_unl_mtx(mutex);
    log_barcode_calibration(&report);
    return (int)unlock;
}

/*
 * Reset detector tracking memory under the service mutex.
 */
int line_tracking_reset(void)
{
    if (!ready) {
        return E_OBJ;
    }
    const ER er = tk_loc_mtx(mutex, TMO_FEVR);
    if (er < 0) {
        return (int)er;
    }
    lp_reset_tracking(&snapshot.line);
    snapshot.event = VC_EVENT_NONE;
    snapshot.norm_peak = 0u;
    publish_line();
    return (int)tk_unl_mtx(mutex);
}

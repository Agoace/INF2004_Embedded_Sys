/* TEST runner below motion/sensors. No ADC IRQ work or driving client. */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "../app.h"
#include "barcode_test.h"
#if APP_BARCODE_CAPTURE
#include "barcode.h"
#include "barcode_capture.h"
#include "barcode_hw.h"
#include "drv_line.h"
#include "line_service.h"
#include "../platform/drv_time.h"
#include "../subsystem2/motion.h"

#if !APP_TEST || APP_BENCH || APP_IMU_STREAM || TM_MQTT || TK_SUPPORT_SMP
#error "BARCODE_CAPTURE requires exclusive single-core, non-MQTT TEST"
#endif

static ID runner;
static barcode_workspace_t workspace;
static barcode_report_t decoded;
static uint32_t motor_misses;
static uint32_t decode_max_us;
static uint32_t decode_total_us;

static void release(void *context)
{
    (void)context;
    (void)tk_wup_tsk(runner);
}

static void motor_sample(void)
{
    motion_status_t value;
    uint32_t stamp;
    if (motion_snapshot_try_read(&value, &stamp) != E_OK ||
        (uint32_t)(drv_time_us() - stamp) > 30000u) {
        ++motor_misses;
        return;
    }
    const barcode_motion_t row = {stamp, value.counts[0], value.counts[1]};
    barcode_capture_motor(&row);
}

static void result(const char *stage, const char *method,
                   const barcode_result_t *r)
{
    tm_printf((UB *)"[BC_RESULT] %s %s decoded %u text \"%s\" cmd %u "
                    "reason %s quality %u dirs %u frames %u "
                    "separation_permille %u\n",
              stage, method, (unsigned int)r->decoded, r->text,
              (unsigned int)r->command, barcode_reason_name(r->reason),
              (unsigned int)r->quality, (unsigned int)r->directions,
              (unsigned int)r->frames, (unsigned int)r->separation_permille);
}

static void decode(bool lost)
{
    size_t n;
    size_t m;
    const barcode_sample_t *rows = barcode_capture_rows(&n);
    const barcode_motion_t *motors = barcode_capture_motors(&m);
    barcode_begin(rows, n, motors, m, &barcode_default_config, &workspace,
                  &decoded);
    decode_max_us = 0u;
    decode_total_us = 0u;
    bool done;
    do {
        const uint32_t start = drv_time_us();
        done = barcode_step(&workspace);
        const uint32_t cost = drv_time_us() - start;
        decode_total_us += cost;
        if (cost > decode_max_us) {
            decode_max_us = cost;
        }
        (void)tk_dly_tsk(1u);
        (void)tk_can_wup(TSK_SELF);
    } while (!done);
    if (lost) {
        decoded.time = (barcode_result_t){.reason = BC_SAMPLE_LOSS};
        decoded.distance = decoded.time;
    } else if (barcode_capture_motor_overflow() != 0u) {
        decoded.distance = (barcode_result_t){.reason = BC_MOTION};
    }
    tm_printf((UB *)"[BC_META] raw %u..%u contrast %u runs %u\n",
              (unsigned int)decoded.minimum, (unsigned int)decoded.maximum,
              (unsigned int)decoded.contrast, (unsigned int)decoded.runs);
    result("FROZEN", "TIME", &decoded.time);
    result("FROZEN", "DISTANCE", &decoded.distance);
}

static void dump(void)
{
    size_t n;
    size_t m;
    const barcode_sample_t *rows = barcode_capture_rows(&n);
    const barcode_motion_t *motors = barcode_capture_motors(&m);
    tm_printf((UB *)"[BC_DUMP] BEGIN adc %lu motor %lu; verify row counts\n",
              (unsigned long)n, (unsigned long)m);
    tm_printf((UB *)"BC_ADC,index,us,raw_bc,flags\n");
    for (size_t i = 0u; i < n; ++i) {
        tm_printf((UB *)"BC_ADC,%lu,%lu,%u,%u\n", (unsigned long)i,
                  (unsigned long)rows[i].us, (unsigned int)rows[i].raw,
                  (unsigned int)rows[i].flags);
        if (i % 16u == 15u) {
            (void)tk_dly_tsk(1u);
        }
    }
    tm_printf((UB *)"BC_MOTOR,index,us,left_counts,right_counts\n");
    for (size_t i = 0u; i < m; ++i) {
        tm_printf((UB *)"BC_MOTOR,%lu,%lu,%ld,%ld\n", (unsigned long)i,
                  (unsigned long)motors[i].us, (long)motors[i].left,
                  (long)motors[i].right);
        if (i % 16u == 15u) {
            (void)tk_dly_tsk(1u);
        }
    }
    tm_printf((UB *)"BC_RUN,index,time_us,distance_um_or_INT32_MIN\n");
    for (size_t i = 0u; i < decoded.runs; ++i) {
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

static bool press(button_t *button)
{
    const bool down = barcode_hw_button_down();
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

void barcode_test(void)
{
    static const char *const names[] = {"FLOOR", "WHITE_PAPER", "BLACK_BAR"};
    T_CCYC config = {
        .cycatr = TA_HLNG, .cychdr = (FP)release, .cyctim = 10u, .cycphs = 10u};
    if (motion_stop() < E_OK) {
        (void)motion_estop();
        tm_printf((UB *)"[BC] ABORT: cannot confirm initial motor stop\n");
        return;
    }
    barcode_hw_button_init();
    const bool held = barcode_hw_button_down();
    button_t button = {.sample = held, .stable = held, .armed = !held};
    runner = tk_get_tid();
    const ID cyclic = tk_cre_cyc(&config);
    if (cyclic < E_OK || tk_sta_cyc(cyclic) < E_OK) {
        if (cyclic >= E_OK) {
            (void)tk_del_cyc(cyclic);
        }
        tm_printf((UB *)"[BC] ABORT: cannot start observer cyclic\n");
        return;
    }
    tm_printf((UB *)"[BC] HAND PUSH ONLY. No GP21 or speed commands. "
                    "USB powers encoders; zero duty is coast, not brake.\n");
    tm_printf((UB *)"[BC] Place FLOOR; GP20 records NEXT 1s. Then place "
                    "WHITE_PAPER/BLACK_BAR and press for each. Hold still.\n");
    tm_printf((UB *)"[BC] Next GP20 arms 4096 ms capture; press again to "
                    "freeze. Push straight over 2x print first.\n");
    tm_printf(
        (UB *)"[BC] Both width methods reported; hand-push encoder "
              "slip is NOT a distance calibration. Commands are data only.\n");
    unsigned int stage = 0u;
    bool capturing = false;
    uint32_t live_ms = 0u;
    uint32_t observer_misses = 0u;
    uint32_t last_live = drv_time_ms();
    drv_line_stats_t initial = {0};
    bool range_pending = false;
    uint32_t range_ms = 0u;
    for (;;) {
        if (tk_slp_tsk(100u) < E_OK) {
            barcode_capture_freeze();
            tm_printf((UB *)"[BC] ABORT: observer wake timeout\n");
            break;
        }
        const INT missed = tk_can_wup(TSK_SELF);
        const bool pushed = press(&button);
        if (stage < 3u) {
            if (pushed && !range_pending) {
                barcode_range_begin();
                range_ms = drv_time_ms();
                range_pending = true;
                tm_printf((UB *)"[BC] measuring %s; hold still for 1s\n",
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
                    ++stage;
                }
                if (stage < 3u) {
                    tm_printf((UB *)"[BC] place %s; GP20 records NEXT 1s\n",
                              names[stage]);
                } else {
                    tm_printf(
                        (UB *)"[BC] contrast ranges recorded; "
                              "GP20 arms pass (no automatic contrast PASS)\n");
                }
            }
        } else if (!capturing && pushed) {
            barcode_capture_begin();
            drv_line_stats(&initial);
            motor_misses = 0u;
            observer_misses = 0u;
            motor_sample();
            live_ms = drv_time_ms();
            capturing = true;
            tm_printf((UB *)"[BC] CAPTURE armed; hand push now\n");
        } else if (capturing) {
            motor_sample();
            if (missed > 0) {
                observer_misses += (uint32_t)missed;
            }
            if (pushed) {
                barcode_capture_freeze();
            }
            drv_line_stats_t current;
            drv_line_stats(&current);
            const bool lost = current.missed != initial.missed ||
                              current.dropped != initial.dropped;
            if (!barcode_capture_active()) {
                /* Keep the final ADC endpoint bracketed without inventing
                 * future encoder motion. Capture storage stays frozen. */
                (void)tk_dly_tsk(20u);
                motor_sample();
                barcode_capture_freeze();
                decode(lost);
                tm_printf((UB *)"[BC_STATS] missed %lu dropped %lu "
                                "motor_misses %lu observer_misses %lu "
                                "hook_max_us %lu elapsed_ms %lu "
                                "decode_step_max_us %lu decode_total_us %lu "
                                "adc_full %u motion_overflow %lu gaps %lu\n",
                          (unsigned long)(current.missed - initial.missed),
                          (unsigned long)(current.dropped - initial.dropped),
                          (unsigned long)motor_misses,
                          (unsigned long)observer_misses,
                          (unsigned long)barcode_capture_max_us(),
                          (unsigned long)(drv_time_ms() - live_ms),
                          (unsigned long)decode_max_us,
                          (unsigned long)decode_total_us,
                          (unsigned int)barcode_capture_full(),
                          (unsigned long)barcode_capture_motor_overflow(),
                          (unsigned long)barcode_capture_gaps());
                line_snapshot_t line;
                if (line_snapshot_try_read(&line) == E_OK) {
                    tm_printf((UB *)"[BC_LINE] isr_max_us %lu "
                                    "isr_total_us %lu line_sample_max_us %lu "
                                    "line_total_us %lu processed %lu\n",
                              (unsigned long)current.isr_max_us,
                              (unsigned long)current.isr_us,
                              (unsigned long)line.sample_max_us,
                              (unsigned long)line.cpu_us,
                              (unsigned long)line.processed);
                }
                dump();
                /* Dump deliberately yields; those queued observer wakes
                 * are not acquisition loss for the next hand-push pass. */
                (void)tk_can_wup(TSK_SELF);
                capturing = false;
                tm_printf((UB *)"[BC] frozen; GP20 starts NEW pass. "
                                "Save complete dump before rearming.\n");
            }
        }
        if (!capturing && drv_time_ms() - last_live >= 1000u) {
            line_snapshot_t line;
            if (line_snapshot_try_read(&line) == E_OK) {
                tm_printf((UB *)"[BC_RAW] raw_bc %u; phase %u\n",
                          (unsigned int)line.raw_bc, stage);
            }
            last_live = drv_time_ms();
        }
    }
    barcode_range_end(NULL);
    (void)tk_stp_cyc(cyclic);
    (void)tk_del_cyc(cyclic);
    (void)tk_can_wup(TSK_SELF);
}
#endif

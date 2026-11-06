/* Floor bench: same B3 steps and row format, no output during a run. */
#include "subsystem2.h"
#include "motion.h"
#include "drv_motor_hw.h"
#include "../platform/drv_time.h"
#if !APP_TEST
#define MAX_ROWS 2400u
#define START_DELAY_MS 1000u
#define DUMP_ROWS_PER_CHUNK 8u
typedef struct { const char *name; uint16_t ms; int16_t left, right; } step_t;
static const step_t STEPS[] = {
    {"rest", 500u, 0, 0},
    {"fwd 150", 1500u, 150, 150},     {"rest", 700u, 0, 0},
    {"rev 150", 1500u, -150, -150},   {"rest", 700u, 0, 0},
    {"fwd 300", 1000u, 300, 300},     {"rest", 700u, 0, 0},
    {"rev 300", 1000u, -300, -300},   {"rest", 700u, 0, 0},
    {"spin L 20", 3000u, -20, 20},    {"rest", 700u, 0, 0}, /* about 60 degrees */
    {"spin R 20", 3000u, 20, -20},    {"rest", 700u, 0, 0},
    {"fwd 40", 2000u, 40, 40},        {"rest", 700u, 0, 0},
    {"rev 40", 2000u, -40, -40},      {"rest", 700u, 0, 0},
};
#define STEP_COUNT (sizeof STEPS / sizeof STEPS[0])
typedef struct {
    int32_t count[MC_WHEELS];
    int16_t target[MC_WHEELS], speed[MC_WHEELS], duty[MC_WHEELS];
    uint8_t step, faults;
} row_t;
static row_t rows[MAX_ROWS];
static unsigned int row_count;
static uint32_t skipped_rows;
static int32_t heading_end;
static const char *stop_reason = "no run saved";
#if TM_CONSOLE_USB_CDC
IMPORT volatile UW tm_usb_dropped;
static UW dropped(void) { return tm_usb_dropped; }
#else
static UW dropped(void) { return 0u; }
#endif
static bool pressed(unsigned int pin, bool *previous) {
    const bool down = drv_motor_hw_button_down(pin);
    const bool edge = down && !*previous;
    *previous = down;
    return edge;
}
static void dump(void) {
    motion_status_t s;
    if (motion_status(&s) < E_OK) return;
    const UW dropped_before = dropped();
    tm_printf((UB*)"\n[bench] stop: %s. rows %u, skipped %lu, faults 0x%02x\n",
              stop_reason, row_count, (unsigned long)skipped_rows, (unsigned int)s.faults);
    tm_printf((UB*)"[bench] heading end %+ld cdeg; counts L/R %ld/%ld\n",
              (long)heading_end, (long)s.counts[0], (long)s.counts[1]);
    tm_printf((UB*)"[bench] ticks %lu max %lu us, gap %lu..%lu us, late %lu missed %lu\n",
              (unsigned long)s.ticks, (unsigned long)s.tick_max_us,
              (unsigned long)s.gap_min_us, (unsigned long)s.gap_max_us,
              (unsigned long)s.late_wakes, (unsigned long)s.missed_wakes);
    (void)tk_dly_tsk(20);
    tm_printf((UB*)"[bench] step target L/R speed 2nd half L/R duty 2nd half L/R heading change\n");
    unsigned int start = 0u;
    while (start < row_count) {
        const uint8_t st = rows[start].step;
        unsigned int end = start;
        while (end < row_count && rows[end].step == st) ++end;
        const unsigned int half = start + (end - start) / 2u;
        long speed[2] = {0, 0}, duty[2] = {0, 0};
        for (unsigned int i = half; i < end; ++i)
            for (unsigned int w = 0u; w < MC_WHEELS; ++w) {
                speed[w] += rows[i].speed[w]; duty[w] += rows[i].duty[w];
            }
        const long n = (long)(end - half);
        const long dl = (long)(rows[end - 1u].count[0] - rows[start].count[0]);
        const long dr = (long)(rows[end - 1u].count[1] - rows[start].count[1]);
        tm_printf((UB*)"[bench] %-12s %+5d/%+5d %+5ld/%+5ld %+4ld/%+4ld permille %+6ld cdeg\n",
                  STEPS[st].name, STEPS[st].left, STEPS[st].right,
                  n ? speed[0] / n : 0, n ? speed[1] / n : 0,
                  n ? duty[0] / n : 0, n ? duty[1] / n : 0, (dr - dl) * 10000L / 2405L);
        (void)tk_dly_tsk(20);
        start = end;
    }
    tm_printf((UB*)"[bench] rows: i step tgtL tgtR spdL spdR dutyL dutyR cntL cntR faults\n");
    for (unsigned int i = 0u; i < row_count; ++i) {
        const row_t *x = &rows[i];
        tm_printf((UB*)"MB %u %u %d %d %d %d %d %d %ld %ld %u\n", i, (unsigned int)x->step,
                  x->target[0], x->target[1], x->speed[0], x->speed[1], x->duty[0], x->duty[1],
                  (long)x->count[0], (long)x->count[1], (unsigned int)x->faults);
        if ((i + 1u) % DUMP_ROWS_PER_CHUNK == 0u) (void)tk_dly_tsk(20);
    }
    (void)tk_dly_tsk(20);
    const UW after = dropped();
    tm_printf((UB*)"[bench] rows end. tm_usb_dropped %lu (dump delta %lu)\n",
              (unsigned long)after, (unsigned long)(after - dropped_before));
}
static void bench_task(INT stacd, void *exinf) {
    (void)stacd; (void)exinf;
    enum { IDLE, WAIT, RUN } mode = IDLE;
    bool prev_run = drv_motor_hw_button_down(21u);
    bool prev_dump = drv_motor_hw_button_down(20u);
    uint32_t wait_until = 0u, step_start = 0u, button_ms = drv_time_ms();
    uint32_t previous_tick = 0u;
    unsigned int step = 0u;
    tm_printf((UB*)"[bench] GP21: start after 1 s / estop. GP20: dump saved RAM log.\n");
    if (motion_notify_after_tick(tk_get_tid()) < E_OK) {
        tm_printf((UB*)"[bench] tick notification registration failed; task exits.\n");
        tk_ext_tsk();
    }
    for (;;) {
        /* tk_dly_tsk(10) adds a kernel tick: it would lose alternate rows. */
        (void)tk_slp_tsk(20);
        const INT queued = tk_can_wup(TSK_SELF);
        const uint32_t now = drv_time_ms();
        if (now - button_ms >= 20u) {
            button_ms = now;
            const bool run_edge = pressed(21u, &prev_run);
            const bool dump_edge = pressed(20u, &prev_dump);
            if (run_edge && mode == IDLE) {
                row_count = 0u; skipped_rows = 0u;
                stop_reason = "waiting to start";
                wait_until = now + START_DELAY_MS; mode = WAIT;
            } else if (run_edge) {
                (void)motion_estop(); mode = IDLE;
                stop_reason = "GP21 emergency stop";
                motion_status_t end;
                if (motion_status(&end) >= E_OK) heading_end = end.heading_cdeg;
            } else if (dump_edge && mode == IDLE) {
                dump();
            }
        }
        if (mode == WAIT && (int32_t)(now - wait_until) >= 0) {
            (void)motion_clear_faults();
            step = 0u; step_start = now; previous_tick = 0u; mode = RUN;
        }
        if (mode == RUN) {
            motion_status_t s;
            if (motion_status(&s) < E_OK) { (void)motion_stop(); mode = IDLE; stop_reason = "status failed"; }
            else if (s.ticks != previous_tick) {
                if (previous_tick != 0u) {
                    const uint32_t gaps = s.ticks - previous_tick - 1u;
                    const uint32_t extra = queued > 0 ? (uint32_t)queued : 0u;
                    /* Both measure lost rows; do not count the same loss twice. */
                    skipped_rows += gaps > extra ? gaps : extra;
                }
                previous_tick = s.ticks;
                while (step < STEP_COUNT && now - step_start >= STEPS[step].ms) {
                    step_start += STEPS[step].ms; ++step;
                }
                const char *why = 0;
                if ((s.faults & MC_FAULT_LATCHED) != 0u) why = "latched motion fault";
                else if (step >= STEP_COUNT) why = "list done";
                else if (row_count == MAX_ROWS) why = "row buffer full";
                if (why) {
                    (void)motion_stop(); mode = IDLE;
                    stop_reason = why; heading_end = s.heading_cdeg;
                } else {
                    (void)motion_set_speed(STEPS[step].left, STEPS[step].right);
                    row_t *x = &rows[row_count++];
                    for (unsigned int w = 0u; w < MC_WHEELS; ++w) {
                        x->count[w] = s.counts[w]; x->speed[w] = s.speed_mm_s[w];
                        x->duty[w] = s.duty_permille[w];
                    }
                    x->target[0] = STEPS[step].left; x->target[1] = STEPS[step].right;
                    x->step = (uint8_t)step; x->faults = s.faults;
                }
            }
        }
    }
}
int motion_bench_start(void) {
    drv_motor_hw_buttons_init();
    const ID tid = app_start_task((FP)bench_task, 11, TP_PRC1, "motion_bench");
    return tid < E_OK ? (int)tid : (int)E_OK;
}
#else
/* Never let button commands compete with the lifted-wheel TEST sequence. */
int motion_bench_start(void) { return (int)E_OK; }
#endif

#ifndef MOTION_H
#define MOTION_H
#include <tk/tkernel.h>
#include "motor_ctl.h"
/* Task-context API only. Control/status calls take inherited mtx_motion.
 * TEST-only buffered readers and watchdog handshakes are described below. */
typedef struct {
    int32_t counts[MC_WHEELS]; /* coherent counts copied at the last tick */
    int32_t travel_mm[MC_WHEELS];
    int16_t speed_mm_s[MC_WHEELS];
    int16_t duty_permille[MC_WHEELS];
    int32_t heading_cdeg;
    uint16_t servo_us;
    int8_t servo_deg;
    bool servo_settled;
    uint8_t faults;
    uint32_t ticks, tick_max_us;
    uint32_t gap_min_us, gap_max_us;
    uint32_t late_wakes; /* gaps greater than 11 ms */
    uint32_t missed_wakes; /* queued wake requests discarded by tk_can_wup */
    bool watchdog_armed; /* hardware enabled; TEST may deliberately suppress feeds */
} motion_status_t;
int motion_init(void);
int motion_start(void);
/* One client: positive task ID to wake after each tick, or 0 to disable.
 * The client must stay alive until disabled; a wake already in flight can arrive. */
int motion_notify_after_tick(ID tid);
int motion_set_speed(int16_t left_mm_s, int16_t right_mm_s);
int motion_stop(void);
int motion_estop(void);
int motion_clear_faults(void);
/* Returns E_OK or a kernel error; optional output receives accepted angle. */
int motion_set_servo(int deg, int *accepted_deg);
int motion_status(motion_status_t *status);
/* Bounded read-only last-tick copy, independent of any transport. No inherited mutex/wait.
 * E_TMOUT busy/not yet published, E_OBJ not started; failed output unchanged. */
int motion_snapshot_try_read(motion_status_t *status,uint32_t *timestamp_us);
int motion_bench_start(void);
#if APP_TEST
/* Only complete gaps between starts inside the window are scored. Lifetime
 * counters are never reset. missed_wakes is the end-minus-begin delta. */
typedef struct {
    uint32_t ticks, gaps, gap_min_us, gap_max_us, tick_max_us;
    uint32_t late_wakes, missed_wakes;
} motion_timing_window_t;
int motion_timing_window_begin(void);
int motion_timing_window_end(motion_timing_window_t *out);
typedef enum {
    MOTION_PROBE_BEFORE_S0_PRINT, MOTION_PROBE_AFTER_S0_PRINT,
    MOTION_PROBE_S1_BURST_BEGIN, MOTION_PROBE_S1_BURST_END,
    MOTION_PROBE_AFTER_S1_YIELD, MOTION_PROBE_AFTER_S1_PRINT,
    MOTION_PROBE_S2_ENTRY, MOTION_PROBE_S2_EXIT, MOTION_PROBE_COUNT
} motion_probe_stage_t;
typedef struct {
    uint32_t us, ticks, gap_min_us, gap_max_us, tick_max_us, late_wakes, missed_wakes;
} motion_timing_probe_t;
/* One TEST runner captures under mtx_motion, then reads immutable copies
 * after the window. Neither function prints or resets lifetime counters. */
int motion_timing_probe_capture(motion_probe_stage_t stage);
int motion_timing_probe_read(motion_probe_stage_t stage,motion_timing_probe_t *out);
/* TEST-only: motion acknowledges just after its final feed. */
int motion_watchdog_test_suppress_feed(void);
bool motion_watchdog_test_feed_stopped(void);
#endif
#endif

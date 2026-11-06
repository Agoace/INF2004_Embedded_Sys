#ifndef MOTION_H
#define MOTION_H
#include <tk/tkernel.h>
#include "motor_ctl.h"
/* Task-context API only. Every call takes priority-inheritance mtx_motion. */
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
int motion_bench_start(void);
#endif

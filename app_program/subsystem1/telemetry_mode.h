/*
 * telemetry_mode.h - choose the MQTT publication rates (RUN or IDLE).
 *
 * Only the publication schedule changes. Sensors, the motion task, the
 * vehicle task and their priorities do not change. Commands, events, run
 * summaries, status and DUMP refill are the same in both modes.
 *
 * Inputs come from the vehicle public board. With no vehicle consumer
 * (TEST and BENCH images: the board read returns E_OBJ) the mode is always
 * RUN, so TEST acceptance (for example S1-U05 rates) stays unchanged.
 *
 * Pure logic, no kernel or network calls. Host-tested by
 * tests/test_telemetry_mode.c.
 */
#ifndef TELEMETRY_MODE_H
#define TELEMETRY_MODE_H

#include <stdbool.h>
#include <stdint.h>

/* RUN stays on this long after the vehicle stops. One period gives the
 * vehicle task time to write the last rows, the run-end event and the
 * summary, and gives the live log at least 19 more 100 ms batch periods to
 * publish the final rows. The value is 2 s. */
#define PM_RUN_GRACE_MS      2000u

/* Publication periods, milliseconds. */
#define PM_RUN_STATE_MS      200u
#define PM_RUN_HEARTBEAT_MS  1000u
#define PM_IDLE_STATE_MS     1000u
#define PM_IDLE_HEARTBEAT_MS 5000u

typedef enum {
    PM_MODE_RUN = 0,
    PM_MODE_IDLE = 1
} pm_mode_t;

typedef enum {
    PM_BOARD_OK = 0,   /* the vehicle board was read */
    PM_BOARD_BUSY = 1, /* the writer was busy: keep the last decision */
    PM_BOARD_NONE = 2  /* no vehicle consumer (TEST, BENCH): RUN */
} pm_board_t;

typedef struct {
    bool seen_busy;   /* the vehicle was busy at least once */
    bool busy;        /* the last sample was busy */
    pm_mode_t mode;   /* the last decision */
    uint32_t idle_at; /* time of the last busy-to-idle edge */
} pm_tracker_t;

void pm_init(pm_tracker_t *t);

/*
 * One sample. phase: 0 idle, 1 calibrating, 2 start delay, 3 running;
 * active: the vehicle run flag. The vehicle is busy when phase != 0 or
 * active. IDLE needs a readable board, a not-busy vehicle, and
 * PM_RUN_GRACE_MS since the last busy sample (boot counts as idle).
 */
pm_mode_t pm_update(pm_tracker_t *t, pm_board_t board, uint8_t phase,
                    bool active, uint32_t now_ms);

static inline uint32_t pm_state_period_ms(pm_mode_t mode)
{
    return (mode == PM_MODE_IDLE) ? PM_IDLE_STATE_MS : PM_RUN_STATE_MS;
}

static inline uint32_t pm_heartbeat_period_ms(pm_mode_t mode)
{
    return (mode == PM_MODE_IDLE) ? PM_IDLE_HEARTBEAT_MS : PM_RUN_HEARTBEAT_MS;
}

/* Live log and sonar history are published only in RUN. */
static inline bool pm_streams_live(pm_mode_t mode)
{
    return mode == PM_MODE_RUN;
}

#endif

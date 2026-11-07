/*
 * robot_health.c - health task (priority 31) that publishes uptime, reset
 * reason, minimum stack margin and network counters once per second.
 *
 * Wi-Fi code never samples stacks or counters itself. It reads the board
 * that this task fills, using a bounded read.
 */
#if TM_MQTT
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "../app.h"
#include "robot_health.h"
#include "robot_network.h"
#include "telemetry.h"
#include "../platform/app_board.h"
#include "../platform/app_task.h"
#include "../platform/drv_time.h"
#include "../platform/drv_watchdog.h"

/* Default extra health provider. Integration code can override it. */
void __attribute__((weak)) robot_health_collect(rm_heartbeat_t *out)
{
    (void)out;
}

/* Health task and cyclic timing. */
#define HEALTH_PERIOD_MS     1000u
#define HEALTH_TASK_PRIORITY 31
#define HEALTH_STACK_SLOTS   7u

static ID health_task;
static ID health_cyclic;

typedef struct {
    rm_heartbeat_t value;
    uint32_t us;
} health_snapshot_t;

static APP_BOARD(health_snapshot_t) board;

/* Build one health snapshot and publish it to the board. */
static void refresh(void)
{
    health_snapshot_t v = {0};
    v.us = drv_time_us();
    v.value.uptime_ms = drv_time_ms();
    v.value.reset_reason = (uint8_t)drv_watchdog_last_reset();
    v.value.stack_min_free = RM_NO_DATA_U16;
    app_stack_t stacks[HEALTH_STACK_SLOTS];
    const unsigned int n = app_task_stacks(stacks, HEALTH_STACK_SLOTS);
    for (unsigned int i = 0u; i < n; ++i) {
        if (stacks[i].free < v.value.stack_min_free) {
            v.value.stack_min_free = (uint16_t)stacks[i].free;
        }
    }
    telemetry_stats_t st;
    telemetry_stats(&st);
    v.value.telemetry_lost = st.lost;
    robot_health_collect(&v.value);
    robot_network_metrics(&v.value);
    app_board_publish(&board.sequence, board.words, &v, sizeof(v));
}

/* Task body: refresh, then sleep until the next cyclic wake-up. */
static void health_entry(INT code, void *ctx)
{
    (void)code;
    (void)ctx;
    for (;;) {
        refresh();
        (void)tk_slp_tsk(TMO_FEVR);
        (void)tk_can_wup(TSK_SELF);
    }
}

/* Cyclic handler: wake the health task. */
static void health_tick(void *ctx)
{
    (void)ctx;
    if (health_task > 0) {
        (void)tk_wup_tsk(health_task);
    }
}

/* Create the 1 s cyclic handler. Returns E_OK or a kernel error. */
int robot_health_init(void)
{
    if (health_cyclic > 0) {
        return E_OK;
    }
    T_CCYC c = {0};
    c.cycatr = TA_HLNG;
    c.cychdr = (FP)health_tick;
    c.cyctim = HEALTH_PERIOD_MS;
    c.cycphs = HEALTH_PERIOD_MS;
#if TK_SUPPORT_SMP
    c.cycatr |= TA_ASSPRC;
    c.assprc = TP_PRC1;
#endif
    health_cyclic = tk_cre_cyc(&c);
    return health_cyclic < 0 ? (int)health_cyclic : E_OK;
}

/* Start the health task on core 0, then the cyclic. E_OBJ if unready. */
int robot_health_start(void)
{
    if (health_cyclic <= 0 || health_task > 0) {
        return E_OBJ;
    }
    health_task = app_start_task((FP)health_entry, HEALTH_TASK_PRIORITY,
                                 TP_PRC1, "health_task");
    return health_task < 0 ? (int)health_task : (int)tk_sta_cyc(health_cyclic);
}

/*
 * Copy the latest snapshot and its capture time (us).
 * Returns E_OK, E_TMOUT when unavailable or busy, or E_PAR.
 */
int robot_health_try_read(rm_heartbeat_t *out, uint32_t *timestamp_us)
{
    if (!out || !timestamp_us) {
        return E_PAR;
    }
    health_snapshot_t v;
    if (!app_board_try_read(&board.sequence, board.words, &v, sizeof(v))) {
        return E_TMOUT;
    }
    *out = v.value;
    *timestamp_us = v.us;
    return E_OK;
}
#endif

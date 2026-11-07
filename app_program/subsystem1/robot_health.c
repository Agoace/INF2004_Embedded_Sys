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

void __attribute__((weak)) robot_health_collect(rm_heartbeat_t *out)
{
    (void)out;
}

static ID health_task, health_cyclic;

typedef struct {
    rm_heartbeat_t value;
    uint32_t us;
} health_snapshot_t;

static APP_BOARD(health_snapshot_t) board;

static void refresh(void)
{
    health_snapshot_t v = {0};
    v.us = drv_time_us();
    v.value.uptime_ms = drv_time_ms();
    v.value.reset_reason = (uint8_t)drv_watchdog_last_reset();
    v.value.stack_min_free = 65535u;
    app_stack_t stacks[7];
    const unsigned int n = app_task_stacks(stacks, 7u);
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

static void health_tick(void *ctx)
{
    (void)ctx;
    if (health_task > 0) {
        (void)tk_wup_tsk(health_task);
    }
}

int robot_health_init(void)
{
    if (health_cyclic > 0) {
        return E_OK;
    }
    T_CCYC c = {0};
    c.cycatr = TA_HLNG;
    c.cychdr = (FP)health_tick;
    c.cyctim = 1000u;
    c.cycphs = 1000u;
#if TK_SUPPORT_SMP
    c.cycatr |= TA_ASSPRC;
    c.assprc = TP_PRC1;
#endif
    health_cyclic = tk_cre_cyc(&c);
    return health_cyclic < 0 ? (int)health_cyclic : E_OK;
}

int robot_health_start(void)
{
    if (health_cyclic <= 0 || health_task > 0) {
        return E_OBJ;
    }
    health_task = app_start_task((FP)health_entry, 31, TP_PRC1, "health_task");
    return health_task < 0 ? (int)health_task : (int)tk_sta_cyc(health_cyclic);
}

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

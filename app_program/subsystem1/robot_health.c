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
#include "network_pack.h"
#include "robot_network.h"
#include "telemetry.h"
#include "../platform/app_board.h"
#include "../platform/app_task.h"
#include "../platform/drv_time.h"
#include "../platform/drv_watchdog.h"
#include "../platform/idle_meter.h"
#include "../platform/cpu_load.h"
#include "../platform/run_timing.h"
#include "../platform/fault_pack.h"
#include "../subsystem2/motion.h"

/* Optional in isolated TEST/BENCH images. Missing means unknown, not zero.
 * The full robot must bind these to its real single-writer owner modules. */
extern bool vehicle_fault_mask_try_read(uint32_t *) __attribute__((weak));
extern bool sonar_fault_mask_try_read(uint32_t *) __attribute__((weak));
extern bool telemetry_fault_mask_try_read(uint32_t *) __attribute__((weak));
extern bool robot_network_fault_mask_try_read(uint32_t *) __attribute__((weak));
extern int motion_snapshot_try_read(motion_status_t *, uint32_t *) __attribute__((weak));

/* Default extra health provider. Integration code can override it. */
void __attribute__((weak)) robot_health_collect(rm_heartbeat_t *out)
{
    (void)out;
}

/* Health task and cyclic timing. */
#define HEALTH_PERIOD_MS     1000u
#define HEALTH_TASK_PRIORITY 31
#define HEALTH_STACK_SLOTS   APP_TASK_MAX_SLOTS

static ID health_task;
static ID health_cyclic;

typedef struct {
    network_health_packed_t value;
    uint32_t us;
    fault_snapshot_packed_t faults; /* CR1 F pack: 8 B, fault_pack.h */
    uint32_t fault_epoch; /* Raw START/STOP phase; not just normalized identity. */
} health_snapshot_t;

static APP_BOARD(health_snapshot_t) board;
_Static_assert(sizeof(board) == 124u, "Packed Health plus fault/phase board allocation");

/* Vehicle writes this STOP capture; Health only reads. No masks are ORed
 * across owners here: each owner keeps its own field and read quality for
 * the sole combiner. CR1 F pack: 12 B payload (fault_pack.h). */
static APP_BOARD(fault_stop_packed_t) stop_capture;
_Static_assert(sizeof(stop_capture) == 16u, "STOP capture allocation");
/* Owner words refused by the STOP capture. Writer: robot_health_faults_stop
 * (Vehicle task only). */
static uint32_t stop_refused;
static fault_snapshot_packed_t fault_views; /* Health sole writer, 8 B. */

static const uint32_t *const owner_masks = fault_pack_owner_masks;

/* Copy each owner word. A word with a bit outside its owner mask is not
 * stored: the owner stays unknown (valid clear, value 0) and *refused
 * counts it. Unknown is never an implicit all-clear. */
static void capture_owner_values(fault_stop_values_t *out, uint32_t *refused)
{
    *out=(fault_stop_values_t){0};
    out->us=drv_time_us();
    motion_status_t motion;uint32_t us;
    if (motion_snapshot_try_read != 0 && motion_snapshot_try_read(&motion,&us)==E_OK) {
        if (((uint32_t)motion.faults & ~FAULT_OWNER_MOTION)==0u) {
            out->owner[0]=(uint32_t)motion.faults;
            out->valid=FAULT_OWNER_MOTION;
        } else {
            ++*refused;
        }
    }
    bool (*const getters[4])(uint32_t *)={vehicle_fault_mask_try_read,
        sonar_fault_mask_try_read,telemetry_fault_mask_try_read,
        robot_network_fault_mask_try_read};
    for (unsigned int i=1u;i<5u;++i) {
        uint32_t word;
        if (getters[i-1u]!=0 && getters[i-1u](&word)) {
            if ((word & ~owner_masks[i])==0u) {
                out->owner[i]=word;
                out->valid|=owner_masks[i];
            } else {
                ++*refused;
            }
        }
    }
}

void robot_health_faults_stop(uint32_t odd_epoch)
{
    if ((odd_epoch & 1u)==0u) return;
    fault_stop_values_t values;
    uint32_t refused=0u;
    capture_owner_values(&values,&refused);
    values.epoch=odd_epoch;
    fault_stop_packed_t packed;
    if (!fault_stop_pack(&packed,&values)) {
        /* Not reachable: every owner is masked above. Store all unknown. */
        packed=(fault_stop_packed_t){0u,odd_epoch,values.us};
        ++refused;
    }
    if (refused!=0u) {
        __atomic_store_n(&stop_refused,
            __atomic_load_n(&stop_refused,__ATOMIC_RELAXED)+refused,__ATOMIC_SEQ_CST);
    }
    app_board_publish(&stop_capture.sequence,stop_capture.words,&packed,sizeof(packed));
}

uint32_t robot_health_faults_stop_refused(void)
{
    return __atomic_load_n(&stop_refused,__ATOMIC_SEQ_CST);
}

static void collect_fault_views(fault_mask_snapshot_t *views)
{
    fault_snapshot_unpack(views,&fault_views);
    const fault_mask_snapshot_t before=*views;
    const uint32_t epoch=rt_epoch();
    const bool active=(epoch & 1u)!=0u;
    /* STOP is odd+1, not a new run. At wrapped STOP0 keep the last identity. */
    const uint32_t run_epoch=active?epoch:(epoch==0u?views->epoch:epoch-1u);
    fault_mask_collect_begin(views,run_epoch);
    const uint32_t previous_run=views->run;
    fault_stop_values_t current;
    uint32_t refused=0u; /* now view: shown as valid clear, not counted */
    capture_owner_values(&current,&refused);
    for (unsigned int i=0u;i<5u;++i) {
        const uint32_t *word=(current.valid & owner_masks[i])==owner_masks[i]?
            &current.owner[i]:0;
        (void)fault_mask_combine(views,owner_masks[i],word);
    }
    if (!active) {
        /* Do not add post-STOP conditions to the frozen run. Merge only the
         * immutable capture made at STOP, even if an owner has since cleared. */
        views->run=previous_run;
        fault_stop_packed_t packed;
        if (app_board_try_read(&stop_capture.sequence,stop_capture.words,&packed,
                               sizeof(packed)) && packed.epoch==run_epoch) {
            fault_stop_values_t final;
            fault_stop_unpack(&final,&packed);
            for (unsigned int i=0u;i<5u;++i) {
                if ((final.valid & owner_masks[i])==owner_masks[i]) {
                    views->run|=final.owner[i];
                }
            }
        }
    }
    if (rt_epoch()!=epoch) {
        *views=before;
        views->valid=0u; /* Cross-epoch collection is not a current sample. */
    }
    /* Masks come from fault_mask_combine, so they are within 10 bits. */
    (void)fault_snapshot_pack(&fault_views,views);
}
/* P1: per-run peak load; Health is the only writer. */
static cpu_peak_t run_peak;

/* Build one health snapshot and publish it to the board. */
static void refresh(void)
{
    struct { rm_heartbeat_t value; uint32_t us; } v = {0};
    v.us = drv_time_us();
    v.value.uptime_ms = drv_time_ms();
    v.value.reset_reason = (uint8_t)drv_watchdog_last_reset();
    v.value.stack_min_free = RM_NO_DATA_U16;
    /* RT-0069/C2: Health is the single 1 Hz sampler for both meters. */
    idle_meter_sample_all();
    v.value.cpu_load_permille = idle_meter_load_permille();
#if TK_SUPPORT_SMP
    const uint16_t load1 = idle_meter_load_permille_core1();
#else
    const uint16_t load1 = CPU_LOAD_UNKNOWN;
#endif
    cpu_peak_update(&run_peak, rt_epoch(), v.value.cpu_load_permille, load1);
    rt_load(run_peak.epoch, run_peak.samples, run_peak.peak[0],
            run_peak.peak[1]);
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
    health_snapshot_t packed;
    packed.us = v.us;
    packed.fault_epoch = rt_epoch();
    fault_mask_snapshot_t views;
    collect_fault_views(&views);
    packed.faults = fault_views;
    if (network_health_pack(&packed.value, &v.value)) {
        app_board_publish(&board.sequence, board.words, &packed, sizeof(packed));
    }
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
    cpu_peak_init(&run_peak);
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
    if (!network_health_unpack(out, &v.value)) return E_SYS;
    *timestamp_us = v.us;
    return E_OK;
}

int robot_health_faults_try_read(fault_mask_snapshot_t *out, uint32_t *timestamp_us)
{
    if (!out || !timestamp_us) return E_PAR;
    health_snapshot_t v;
    if (!app_board_try_read(&board.sequence,board.words,&v,sizeof(v))) return E_TMOUT;
    if (v.fault_epoch!=rt_epoch()) return E_TMOUT;
    fault_snapshot_unpack(out,&v.faults);
    *timestamp_us=v.us;
    return E_OK;
}
#endif

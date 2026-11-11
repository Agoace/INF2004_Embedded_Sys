/*
 * app_task.c - creates registered application tasks on caller-owned static
 * stacks and measures their stack use.
 *
 * Each slot has a static, 8-byte aligned stack that is filled with a known
 * pattern before the task starts. The unused part of the pattern shows how
 * much stack remains.
 *
 * Placement.  The slot table below is the single owner of where a registered
 * task may run.  Movement and hardware owners stay on core 0.  The observers
 * that the core-migration plan moves later are moved by editing their slot
 * entry and their call site together, so a call-site edit alone cannot move a
 * task: app_task_start() rejects a core its slot does not allow.
 */
#include "app_task.h"
#include "../app.h"   /* app_core_t, app_prc_core0/1: the placement seam */
#include <stdbool.h>
#include <tm/tmonitor.h>
#include <tk/syslib.h>
/* Fill pattern and stack sizes in 64-bit words (bytes = words * 8). */
#define FILL                  0xa5u
#define MOTION_STACK_WORDS    384
#define LINE_STACK_WORDS      256
#define SONAR_STACK_WORDS     256
#define IMU_STACK_WORDS       256
#define VEHICLE_STACK_WORDS   512
#define TELEMETRY_STACK_WORDS 512
#define BENCH_STACK_WORDS     512
#define HEALTH_STACK_WORDS    256
#define IDLE_STACK_WORDS      96
#define STACK_LOW_MARGIN      256u

static uint64_t motion_stack[MOTION_STACK_WORDS];
static uint64_t line_stack[LINE_STACK_WORDS];
static uint64_t sonar_stack[SONAR_STACK_WORDS];
#if !APP_BENCH
static uint64_t imu_stack[IMU_STACK_WORDS];
#endif
static uint64_t vehicle_stack[VEHICLE_STACK_WORDS];
static uint64_t telemetry_stack[TELEMETRY_STACK_WORDS];
#if APP_BENCH
static uint64_t bench_stack[BENCH_STACK_WORDS];
#endif
#if TM_MQTT
static uint64_t health_stack[HEALTH_STACK_WORDS];
static uint64_t idle_stack[IDLE_STACK_WORDS];
#endif
typedef struct {
    const char *name;
    uint64_t *stack;
    uint32_t size;
    uint32_t high;
    ID id;
    uint8_t core;   /* physical core this task is allowed to run on */
} slot_t;

#define SLOT(n, s, c) {n, s, sizeof(s), 0u, 0, c}
static slot_t slots[] = {SLOT("motion_task", motion_stack, APP_CORE_0),
                         SLOT("line_task", line_stack, APP_CORE_0),
                         SLOT("sonar_task", sonar_stack, APP_CORE_0),
#if !APP_BENCH
                         SLOT("imu_task", imu_stack, APP_CORE_0),
#endif
                         SLOT("vehicle_task", vehicle_stack, APP_CORE_0),
                         SLOT("telemetry_task", telemetry_stack, APP_CORE_0)
#if APP_BENCH
                             ,
                         SLOT("motion_bench", bench_stack, APP_CORE_0)
#endif
#if TM_MQTT
                             ,
                         SLOT("health_task", health_stack, APP_CORE_0),
                         SLOT("idle_meter", idle_stack, APP_CORE_0)
#endif
};
/* Compare two C strings. Returns true when they are equal. */
static bool equal(const char *a, const char *b)
{
    while (*a && *a == *b) {
        ++a;
        ++b;
    }
    return *a == *b;
}

/*
 * PRC value for a physical core.  TP_PRC1 is core 0, so the value is TP_PRC1
 * plus the core index.  A single-core build has only core 0, so a core-1
 * request is answered with core 0 rather than an unmapped PRC.
 */
UW app_core_prc(app_core_t core)
{
#if TK_SUPPORT_SMP
    if (APP_CORE_1 == core && TK_MAX_CORE > 1) {
        return TP_PRC1 + 1u;
    }
#else
    (void)core;
#endif
    return TP_PRC1;
}

/* PRC for core 0.  Cyclic handlers use this for their own core. */
UW app_prc_core0(void)
{
    return app_core_prc(APP_CORE_0);
}

/* PRC for core 1, or core 0 when the build has one core. */
UW app_prc_core1(void)
{
    return app_core_prc(APP_CORE_1);
}

/*
 * Create and start the task registered under name.
 * entry, priority and core are the usual kernel task parameters.
 * Returns the task ID (> 0) or a negative error: E_CTX for the wrong core or a
 * core the slot does not allow, E_PAR for an unknown name or core, E_OBJ if the
 * slot is already running.
 */
ID app_task_start(FP entry, PRI priority, UW core, const char *name)
{
#if TK_SUPPORT_SMP
    /* Creating a task must happen on core 0, as the kernel requires.  The task
     * itself may be placed on core 1 when the build has a second core. */
    if (tk_get_prc() != TP_PRC1) {
        return E_CTX;
    }
    if (core != TP_PRC1 && !((TK_MAX_CORE > 1) && (core == TP_PRC1 + 1u))) {
        return E_PAR;
    }
#else
    if (core != TP_PRC1) {
        return E_CTX;
    }
#endif
    if (!name) {
        return E_PAR;
    }
    slot_t *s = 0;
    for (unsigned int i = 0u; i < sizeof(slots) / sizeof(slots[0]); ++i) {
        if (equal(name, slots[i].name)) {
            s = &slots[i];
            break;
        }
    }
    if (!s) {
        return E_PAR;
    }
    if (s->id > 0) {
        return E_OBJ;
    }
    /* The slot table owns the placement, so a call site cannot move a task that
     * the migration has not moved yet. */
    if (app_core_prc((app_core_t)s->core) != core) {
        return E_CTX;
    }
    unsigned char *bytes = (unsigned char *)s->stack;
    for (uint32_t i = 0u; i < s->size; ++i) {
        bytes[i] = FILL;
    }
    T_CTSK c = {0};
    c.task = entry;
    c.itskpri = priority;
    c.stksz = (SZ)s->size;
    c.tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF;
    c.bufptr = s->stack;
#if TK_SUPPORT_SMP
    /* Honour the requested core.  This used to be a fixed TP_PRC1, which would
     * have pinned every task to core 0 no matter what the caller asked for. */
    c.tskatr |= TA_ASSPRC;
    c.assprc = core;
#endif
    const ID id = tk_cre_tsk(&c);
    if (id < 0) {
        return id;
    }
    const ER er = tk_sta_tsk(id, 0);
    if (er < 0) {
        (void)tk_del_tsk(id);
        return er;
    }
    s->id = id;
    return id;
}

/*
 * Copy stack use of started tasks to out (at most capacity rows).
 * Updates each high-water mark. Returns the number of rows written.
 */
unsigned int app_task_stacks(app_stack_t *out, unsigned int capacity)
{
    unsigned int n = 0u;
    for (unsigned int i = 0u;
         i < sizeof(slots) / sizeof(slots[0]) && n < capacity; ++i) {
        slot_t *s = &slots[i];
        if (s->id <= 0) {
            continue;
        }
        const volatile unsigned char *b =
            (const volatile unsigned char *)s->stack;
        uint32_t free = 0u;
        while (free < s->size && b[free] == FILL) {
            ++free;
        }
        const uint32_t used = s->size - free;
        UINT irq;
        DI(irq);
        if (used > s->high) {
            s->high = used;
        }
        out[n++] = (app_stack_t){s->name, s->size, s->high, s->size - s->high};
        EI(irq);
    }
    return n;
}

/* Print one line per started task: size, high water, free. */
void app_task_report(void)
{
    app_stack_t rows[sizeof(slots) / sizeof(slots[0])];
    const unsigned int n =
        app_task_stacks(rows, (unsigned int)(sizeof(slots) / sizeof(slots[0])));
    for (unsigned int i = 0u; i < n; ++i) {
        tm_printf((UB *)"[stack] %s size %lu high_water %lu free %lu %s\n",
                  rows[i].name, (unsigned long)rows[i].size,
                  (unsigned long)rows[i].used, (unsigned long)rows[i].free,
                  rows[i].free < STACK_LOW_MARGIN ? "LOW MARGIN" : "OK");
    }
}

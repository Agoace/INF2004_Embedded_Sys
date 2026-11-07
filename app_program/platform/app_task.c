#include "app_task.h"
#include <stdbool.h>
#include <tm/tmonitor.h>
#include <tk/syslib.h>
#define FILL 0xa5u
static uint64_t motion_stack[384];
static uint64_t line_stack[256];
static uint64_t sonar_stack[256];
static uint64_t vehicle_stack[512];
static uint64_t telemetry_stack[512];
static uint64_t bench_stack[512];
#if TM_MQTT
static uint64_t health_stack[256];
#endif
typedef struct {
    const char *name;
    uint64_t *stack;
    uint32_t size;
    uint32_t high;
    ID id;
} slot_t;

#define SLOT(n, s) {n, s, sizeof(s), 0u, 0}
static slot_t slots[] = {SLOT("motion_task", motion_stack),
                         SLOT("line_task", line_stack),
                         SLOT("sonar_task", sonar_stack),
                         SLOT("vehicle_task", vehicle_stack),
                         SLOT("telemetry_task", telemetry_stack),
                         SLOT("motion_bench", bench_stack)
#if TM_MQTT
                             ,
                         SLOT("health_task", health_stack)
#endif
};
static bool equal(const char *a, const char *b)
{
    while (*a && *a == *b) {
        ++a;
        ++b;
    }
    return *a == *b;
}

ID app_task_start(FP entry, PRI priority, UW core, const char *name)
{
#if TK_SUPPORT_SMP
    if (tk_get_prc() != 1 || core != TP_PRC1) {
        return E_CTX;
    }
#else
    (void)core;
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
    c.tskatr |= TA_ASSPRC;
    c.assprc = TP_PRC1;
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

void app_task_report(void)
{
    app_stack_t rows[sizeof(slots) / sizeof(slots[0])];
    const unsigned int n =
        app_task_stacks(rows, (unsigned int)(sizeof(slots) / sizeof(slots[0])));
    for (unsigned int i = 0u; i < n; ++i) {
        tm_printf((UB *)"[stack] %s size %lu high_water %lu free %lu %s\n",
                  rows[i].name, (unsigned long)rows[i].size,
                  (unsigned long)rows[i].used, (unsigned long)rows[i].free,
                  rows[i].free < 256u ? "LOW MARGIN" : "OK");
    }
}

/* Kernel-only, core-0 producer/consumer queue. */
#include <tk/tkernel.h>
#include <tk/syslib.h>
#include "drv_line.h"
#include "drv_line_hw.h"
#include "../platform/drv_time.h"
#define CAPACITY 128u
static drv_line_sample_t queue[CAPACITY];
static volatile unsigned int head, tail;
static drv_line_stats_t stats;
static uint32_t next_us;
static bool ready;
static void alarm_irq(UINT intno) {
    drv_line_hw_ack();
    drv_line_sample_t sample = {.us = drv_time_us()};
    const int er = drv_line_hw_read(sample.raw);
    ++stats.samples;
    if (er != 0) ++stats.missed;
    else {
        for (unsigned int i = 0u; i < 2u; ++i) {
            if (sample.raw[i] < stats.min[i]) stats.min[i] = sample.raw[i];
            if (sample.raw[i] > stats.max[i]) stats.max[i] = sample.raw[i];
        }
        const unsigned int next = (head + 1u) % CAPACITY;
        if (next == tail) ++stats.dropped;
        else { queue[head] = sample; head = next; }
    }
    next_us += 1000u;
    const uint32_t now = drv_time_us();
    while ((int32_t)(next_us - now) <= 0) { next_us += 1000u; ++stats.missed; }
    drv_line_hw_alarm(next_us);
    ClearInt(intno);
}
int drv_line_init(void) {
#if TK_SUPPORT_SMP
    if (tk_get_prc() != 1) return (int)E_CTX;
#endif
    if (ready) return (int)E_OK;
    if (drv_time_init() < E_OK || drv_line_hw_init() != 0) return (int)E_SYS;
    DisableInt(0u);
    const T_DINT d = {.intatr = TA_HLNG, .inthdr = (FP)alarm_irq};
    const ER er = tk_def_int(0u, &d);
    if (er < E_OK) return (int)er;
    stats.min[0] = stats.min[1] = 4095u;
    drv_line_hw_ack(); ClearInt(0u);
    next_us = drv_time_us() + 1000u;
    drv_line_hw_alarm(next_us);
    EnableInt(0u, 2);
    ready = true;
    return (int)E_OK;
}
bool drv_line_take(drv_line_sample_t *sample) {
    UINT irq; DI(irq);
    const bool available = tail != head;
    if (available) { *sample = queue[tail]; tail = (tail + 1u) % CAPACITY; }
    EI(irq); return available;
}
void drv_line_flush(void) { UINT irq; DI(irq); tail = head; EI(irq); }
void drv_line_stats(drv_line_stats_t *s) { UINT irq; DI(irq); *s = stats; EI(irq); }

/*
 * Bench build of the robot idle meter. idle_meter.c compiles only when
 * TM_MQTT=1, and BENCH_KERNEL=1 forbids the radio, so this file compiles the
 * same source with TM_MQTT forced on. The robot slot table has no meter slot
 * without MQTT, so the task is created here with the robot meter stack size
 * and priority. Only the core-0 meter runs (a split image reads core 1 as
 * 65535).
 */
#if defined(BENCH_KERNEL) && BENCH_KERNEL
#if defined(TM_MQTT) && TM_MQTT
#error "BENCH_KERNEL=1 is radio-free; the robot idle meter is already built"
#endif
#undef TM_MQTT
#define TM_MQTT 1
#include "../platform/idle_meter.c"
#include "bench_load.h"

/* Same as IDLE_STACK_WORDS in app_task.c (96 words of 8 bytes). */
#define BK_METER_STACK_WORDS 96u
static uint64_t bk_meter_stack[BK_METER_STACK_WORDS];

void bk_load_start(void)
{
    if (idle_meter_calibrate() != (int)E_OK) {
        return;
    }
    T_CTSK c = {0};
    c.task = (FP)idle_meter_entry;
    c.itskpri = IDLE_METER_PRIORITY;
    c.tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF;
    c.stksz = (SZ)sizeof(bk_meter_stack);
    c.bufptr = bk_meter_stack;
#if TK_SUPPORT_SMP
    c.tskatr |= TA_ASSPRC;
    c.assprc = app_prc_core0();
#endif
    const ID id = tk_cre_tsk(&c);
    if (id > 0 && tk_sta_tsk(id, 0) < E_OK) {
        (void)tk_del_tsk(id);
    }
}
#endif

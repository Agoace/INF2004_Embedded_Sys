/*
 * run_timing.c - P1 per-run timing latches. See run_timing.h.
 *
 * Each latch carries the epoch its owner last saw. The owner clears its own
 * words when the current run epoch differs, then updates them. The only
 * shared word is `current`, written by Vehicle alone.
 */
#include "run_timing.h"
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tk/syslib.h>
#include <tm/tmonitor.h>

#define LOAD(p)     __atomic_load_n((p), __ATOMIC_SEQ_CST)
#define STORE(p, v) __atomic_store_n((p), (v), __ATOMIC_SEQ_CST)

typedef struct {
    uint32_t epoch;
    uint32_t max_us;
} max_latch_t;

typedef struct {
    uint32_t epoch;
    rt_call_stats_t s;
} call_latch_t;

typedef struct {
    uint32_t epoch;
    rt_overrun_t r;
} overrun_latch_t;

static uint32_t current; /* odd: run epoch; even: frozen */
static uint32_t frozen;  /* last odd epoch, for readers after STOP */
static max_latch_t maxes[RT_MAX_COUNT];
static call_latch_t calls[RT_CALL_COUNT];
static overrun_latch_t first;
static uint32_t motion_stamp; /* written by Motion only */
static uint32_t load_words[4]; /* Health only: epoch, samples, peak0, peak1 */
typedef struct {
    uint32_t epoch;
    rt_bkl_t r;
} bkl_latch_t;
static bkl_latch_t bkl; /* Vehicle only */
/* P2 timeline ring: core-0 writers inside one short masked section. */
#define TL_PER_LINE        8u
#define TL_LINES_PER_PAUSE 4u
static uint32_t tl_words[RT_TL_RECORDS][2]; /* us; site | arg<<8 | gpio<<16 */
static uint32_t tl_head;   /* records written since the run started */
static uint32_t tl_frozen; /* 1 after the first overrun */
static uint32_t tl_epoch;  /* run of the ring contents */
static uint32_t tl_gpio;   /* GPIO dispatcher only */
#ifdef KLS_HOST_TEST
UINT test_smp_irq_save(void);
void test_smp_irq_restore(UINT saved);
#define TL_MASK(m)   ((m) = test_smp_irq_save())
#define TL_UNMASK(m) test_smp_irq_restore(m)
#else
#define TL_MASK(m)   DI(m)
#define TL_UNMASK(m) EI(m)
#endif

static const char *const part_name[RT_MAX_COUNT] = {
    "isr_late", "isr_adc", "isr_wup", "line_sample", "lp_update",
    "barcode", "wake", "read_line", "command", "sonar", "vsm", "log",
    "publish", "status"};

static const char *const call_name[RT_CALL_COUNT] = {
    "isr_wup", "line_wake", "line_can", "line_loc", "line_unl",
    "mot_wup", "veh_wake", "veh_can", "veh_status", "veh_send"};

/* The run epoch a writer may update, or 0 while frozen. */
static uint32_t live_epoch(void)
{
    const uint32_t e = LOAD(&current);
    return (e & 1u) != 0u ? e : 0u;
}

/* The epoch a reader reports: the live run, or the last run after STOP. */
static uint32_t report_epoch(void)
{
    const uint32_t e = LOAD(&current);
    return (e & 1u) != 0u ? e : LOAD(&frozen);
}

void rt_begin(uint32_t epoch)
{
    if ((epoch & 1u) != 0u) {
        STORE(&frozen, epoch);
        /* Writers are idle here: the previous epoch was even. */
        STORE(&tl_head, 0u);
        STORE(&tl_frozen, 0u);
        STORE(&tl_epoch, epoch);
    }
    STORE(&current, epoch);
}

uint32_t rt_epoch(void)
{
    return LOAD(&current);
}

void rt_load(uint32_t epoch, uint32_t samples, uint16_t peak0,
             uint16_t peak1)
{
    STORE(&load_words[0], epoch);
    STORE(&load_words[1], samples);
    STORE(&load_words[2], (uint32_t)peak0);
    STORE(&load_words[3], (uint32_t)peak1);
}

void rt_bkl(const rt_bkl_t *record)
{
    const uint32_t e = live_epoch();
    if (e == 0u || record == 0) {
        return;
    }
    STORE(&bkl.epoch, 0u);
    STORE(&bkl.r.valid, record->valid);
    STORE(&bkl.r.acquisitions, record->acquisitions);
    STORE(&bkl.r.contentions, record->contentions);
    STORE(&bkl.r.max_wait_us, record->max_wait_us);
    STORE(&bkl.r.max_hold_us, record->max_hold_us);
    STORE(&bkl.r.max_depth, record->max_depth);
    STORE(&bkl.r.detached, record->detached);
    STORE(&bkl.r.run_ms, record->run_ms);
    STORE(&bkl.epoch, e);
}

void rt_max(rt_max_t which, uint32_t us)
{
    const uint32_t e = live_epoch();
    if (e == 0u || (unsigned int)which >= (unsigned int)RT_MAX_COUNT) {
        return;
    }
    max_latch_t *l = &maxes[which];
    if (LOAD(&l->epoch) != e) {
        STORE(&l->max_us, 0u);
        STORE(&l->epoch, e);
    }
    if (us > LOAD(&l->max_us)) {
        STORE(&l->max_us, us);
    }
}

void rt_call(rt_call_t which, uint32_t us)
{
    const uint32_t e = live_epoch();
    if (e == 0u || (unsigned int)which >= (unsigned int)RT_CALL_COUNT) {
        return;
    }
    call_latch_t *l = &calls[which];
    if (LOAD(&l->epoch) != e) {
        STORE(&l->s.count, 0u);
        STORE(&l->s.sum_us, 0u);
        STORE(&l->s.max_us, 0u);
        STORE(&l->epoch, e);
    }
    STORE(&l->s.count, LOAD(&l->s.count) + 1u);
    STORE(&l->s.sum_us, LOAD(&l->s.sum_us) + us);
    if (us > LOAD(&l->s.max_us)) {
        STORE(&l->s.max_us, us);
    }
}

void rt_overrun(const rt_overrun_t *record)
{
    const uint32_t e = live_epoch();
    if (e == 0u || record == 0 || LOAD(&first.epoch) == e) {
        return;
    }
    STORE(&first.r.step, record->step);
    STORE(&first.r.elapsed_us, record->elapsed_us);
    STORE(&first.r.tick_delta, record->tick_delta);
    STORE(&first.r.slow_part, record->slow_part);
    STORE(&first.r.slow_us, record->slow_us);
    STORE(&first.r.wake_us, record->wake_us);
    STORE(&first.r.uptime_ms, record->uptime_ms);
    STORE(&first.epoch, e);
    STORE(&tl_frozen, 1u); /* keep the timeline of the bad step */
}

void rt_motion_stamp(uint32_t us)
{
    STORE(&motion_stamp, us);
}

uint32_t rt_motion_stamp_us(void)
{
    return LOAD(&motion_stamp);
}

/* Readers: one latch of run e, or zero when its owner has not written it. */
static uint32_t max_of(uint32_t e, unsigned int i)
{
    const bool mine = e != 0u && LOAD(&maxes[i].epoch) == e;
    return mine ? LOAD(&maxes[i].max_us) : 0u;
}

static void call_of(uint32_t e, unsigned int i, rt_call_stats_t *out)
{
    const bool mine = e != 0u && LOAD(&calls[i].epoch) == e;
    out->count = mine ? LOAD(&calls[i].s.count) : 0u;
    out->sum_us = mine ? LOAD(&calls[i].s.sum_us) : 0u;
    out->max_us = mine ? LOAD(&calls[i].s.max_us) : 0u;
}

static void overrun_of(uint32_t e, rt_overrun_t *out)
{
    if (e != 0u && LOAD(&first.epoch) == e) {
        out->step = LOAD(&first.r.step);
        out->elapsed_us = LOAD(&first.r.elapsed_us);
        out->tick_delta = LOAD(&first.r.tick_delta);
        out->slow_part = LOAD(&first.r.slow_part);
        out->slow_us = LOAD(&first.r.slow_us);
        out->wake_us = LOAD(&first.r.wake_us);
        out->uptime_ms = LOAD(&first.r.uptime_ms);
    } else {
        *out = (rt_overrun_t){0};
    }
}

static void bkl_of(uint32_t e, rt_bkl_t *out)
{
    if (e != 0u && LOAD(&bkl.epoch) == e) {
        out->valid = LOAD(&bkl.r.valid);
        out->acquisitions = LOAD(&bkl.r.acquisitions);
        out->contentions = LOAD(&bkl.r.contentions);
        out->max_wait_us = LOAD(&bkl.r.max_wait_us);
        out->max_hold_us = LOAD(&bkl.r.max_hold_us);
        out->max_depth = LOAD(&bkl.r.max_depth);
        out->detached = LOAD(&bkl.r.detached);
        out->run_ms = LOAD(&bkl.r.run_ms);
    } else {
        *out = (rt_bkl_t){0};
    }
}

void rt_snapshot(rt_snapshot_t *out)
{
    if (out == 0) {
        return;
    }
    const uint32_t e = report_epoch();
    out->epoch = e;
    out->load_epoch = LOAD(&load_words[0]);
    out->load_samples = LOAD(&load_words[1]);
    out->load_peak[0] = (uint16_t)LOAD(&load_words[2]);
    out->load_peak[1] = (uint16_t)LOAD(&load_words[3]);
    bkl_of(e, &out->bkl);
    for (unsigned int i = 0u; i < (unsigned int)RT_MAX_COUNT; ++i) {
        out->max[i] = max_of(e, i);
    }
    for (unsigned int i = 0u; i < (unsigned int)RT_CALL_COUNT; ++i) {
        call_of(e, i, &out->call[i]);
    }
    overrun_of(e, &out->first);
}

/* Small frames only: the report runs on the 1 KB init task in TEST builds. */
static void report_load_and_bkl(uint32_t e)
{
    rt_bkl_t k;
    tm_printf((UB *)"[load_run] epoch %lu samples %lu peak c0 %lu c1 %lu "
                    "permille\n", (unsigned long)LOAD(&load_words[0]),
              (unsigned long)LOAD(&load_words[1]),
              (unsigned long)(LOAD(&load_words[2]) & 0xFFFFu),
              (unsigned long)(LOAD(&load_words[3]) & 0xFFFFu));
    bkl_of(e, &k);
    if (k.valid == 0u) {
        tm_printf((UB *)"[bkl] epoch %lu unavailable\n", (unsigned long)e);
        return;
    }
    tm_printf((UB *)"[bkl] epoch %lu run %lu ms acq %lu cont %lu "
                    "max_wait %lu max_hold %lu us depth %lu "
                    "detached %lu\n", (unsigned long)e,
              (unsigned long)k.run_ms, (unsigned long)k.acquisitions,
              (unsigned long)k.contentions, (unsigned long)k.max_wait_us,
              (unsigned long)k.max_hold_us, (unsigned long)k.max_depth,
              (unsigned long)k.detached);
}

static void report_maxima(uint32_t e)
{
    const unsigned long n = (unsigned long)e;
    tm_printf((UB *)"[isr_max] epoch %lu late %lu adc %lu wup %lu us\n", n,
              (unsigned long)max_of(e, RT_ISR_LATE),
              (unsigned long)max_of(e, RT_ISR_ADC),
              (unsigned long)max_of(e, RT_ISR_WUP));
    tm_printf((UB *)"[line_max] epoch %lu sample %lu lp_update %lu "
                    "barcode %lu us\n", n,
              (unsigned long)max_of(e, RT_LINE_SAMPLE),
              (unsigned long)max_of(e, RT_LINE_LPU),
              (unsigned long)max_of(e, RT_LINE_BARCODE));
    tm_printf((UB *)"[step_max] epoch %lu wake %lu read_line %lu command %lu "
                    "sonar %lu vsm %lu log %lu publish %lu status %lu us\n", n,
              (unsigned long)max_of(e, RT_STEP_WAKE),
              (unsigned long)max_of(e, RT_STEP_READ_LINE),
              (unsigned long)max_of(e, RT_STEP_COMMAND),
              (unsigned long)max_of(e, RT_STEP_SONAR),
              (unsigned long)max_of(e, RT_STEP_VSM),
              (unsigned long)max_of(e, RT_STEP_LOG),
              (unsigned long)max_of(e, RT_STEP_PUBLISH),
              (unsigned long)max_of(e, RT_STEP_STATUS));
}

static void report_calls(uint32_t e)
{
    for (unsigned int i = 0u; i < (unsigned int)RT_CALL_COUNT; ++i) {
        rt_call_stats_t c;
        call_of(e, i, &c);
        tm_printf((UB *)"[kcall] %s n %lu sum %lu max %lu us\n",
                  call_name[i], (unsigned long)c.count,
                  (unsigned long)c.sum_us, (unsigned long)c.max_us);
    }
}

static void report_overrun(uint32_t e)
{
    rt_overrun_t f;
    overrun_of(e, &f);
    if (f.step == 0u) {
        tm_printf((UB *)"[first_overrun] epoch %lu none\n", (unsigned long)e);
        return;
    }
    const unsigned int part = f.slow_part < (uint32_t)RT_MAX_COUNT
                                  ? (unsigned int)f.slow_part : 0u;
    tm_printf((UB *)"[first_overrun] epoch %lu step %lu elapsed %lu us "
                    "ticks +%lu slowest %s %lu us wake %lu us uptime %lu ms\n",
              (unsigned long)e, (unsigned long)f.step,
              (unsigned long)f.elapsed_us, (unsigned long)f.tick_delta,
              part_name[part], (unsigned long)f.slow_us,
              (unsigned long)f.wake_us, (unsigned long)f.uptime_ms);
}

void rt_report(void)
{
    const uint32_t e = report_epoch();
    report_load_and_bkl(e);
    report_maxima(e);
    report_calls(e);
    report_overrun(e);
}

/* ---- P2 timeline -------------------------------------------------------- */

void rt_tl(rt_tl_site_t site, uint32_t us, uint32_t arg)
{
    const uint32_t e = live_epoch();
    if (e == 0u || LOAD(&tl_frozen) != 0u || LOAD(&tl_epoch) != e) {
        return;
    }
    const uint32_t a = arg > 255u ? 255u : arg;
    const uint32_t meta = ((uint32_t)site & 0xFFu) | (a << 8u) |
                          ((LOAD(&tl_gpio) & 0xFFFFu) << 16u);
    UINT mask;
    TL_MASK(mask);
    const uint32_t head = LOAD(&tl_head);
    const uint32_t slot = head % RT_TL_RECORDS;
    STORE(&tl_words[slot][0], us);
    STORE(&tl_words[slot][1], meta);
    STORE(&tl_head, head + 1u);
    TL_UNMASK(mask);
}

void rt_tl_gpio(void)
{
    STORE(&tl_gpio, LOAD(&tl_gpio) + 1u);
}

static uint32_t tl_records(void)
{
    const uint32_t head = LOAD(&tl_head);
    return head < RT_TL_RECORDS ? head : RT_TL_RECORDS;
}

void rt_tl_get(uint32_t index, rt_tl_record_t *out)
{
    const uint32_t head = LOAD(&tl_head);
    const uint32_t start = head > RT_TL_RECORDS ? head - RT_TL_RECORDS : 0u;
    const uint32_t slot = (start + index) % RT_TL_RECORDS;
    const uint32_t meta = LOAD(&tl_words[slot][1]);
    out->us = LOAD(&tl_words[slot][0]);
    out->site = (uint8_t)(meta & 0xFFu);
    out->arg = (uint8_t)((meta >> 8u) & 0xFFu);
    out->gpio = (uint16_t)((meta >> 16u) & 0xFFFFu);
}

void rt_tl_info(rt_tl_info_t *out)
{
    out->epoch = LOAD(&tl_epoch);
    out->records = tl_records();
    out->frozen = LOAD(&tl_frozen);
    out->end_us = 0u;
    if (out->records != 0u) {
        rt_tl_record_t r;
        rt_tl_get(out->records - 1u, &r);
        out->end_us = r.us;
    }
}

void rt_timeline_report(void (*pause)(void))
{
    rt_tl_info_t info;
    rt_tl_info(&info);
    tm_printf((UB *)"[timeline] epoch %lu frozen %lu records %lu "
                    "end_us %lu\n", (unsigned long)info.epoch,
              (unsigned long)info.frozen, (unsigned long)info.records,
              (unsigned long)info.end_us);
    unsigned int lines = 0u;
    for (uint32_t i = 0u; i < info.records; ++i) {
        rt_tl_record_t r;
        rt_tl_get(i, &r);
        const uint32_t col = i % TL_PER_LINE;
        tm_printf((UB *)"%s -%lu %u %u g%u",
                  col == 0u ? "[tl]" : " |",
                  (unsigned long)(info.end_us - r.us), (unsigned int)r.site,
                  (unsigned int)r.arg, (unsigned int)r.gpio);
        if (col == TL_PER_LINE - 1u || i + 1u == info.records) {
            tm_printf((UB *)"\n");
            ++lines;
            if (lines % TL_LINES_PER_PAUSE == 0u && pause != 0) {
                pause();
            }
        }
    }
}

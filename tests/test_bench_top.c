/* Bench top-20 latches behind --wrap=rt_call/rt_max (host doubles). */
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "../app_program/bench_kernel/bench_top.h"

static char out[8192];
static size_t used;
static uint32_t epoch = 1u;
static unsigned int real_calls, real_maxes, pauses;

#if defined(__clang__)
#pragma clang diagnostic ignored "-Wmissing-format-attribute"
#pragma clang diagnostic ignored "-Wformat-nonliteral"
#endif
int tm_printf(const UB *format, ...)
{
    va_list ap;
    va_start(ap, format);
    const int n = vsnprintf(out + used, sizeof(out) - used,
                            (const char *)format, ap);
    va_end(ap);
    assert(n >= 0 && (size_t)n < sizeof(out) - used);
    used += (size_t)n;
    return n;
}
uint32_t rt_epoch(void) { return epoch; }
void __real_rt_call(rt_call_t which, uint32_t us)
{
    (void)which; (void)us; ++real_calls;
}
void __real_rt_max(rt_max_t which, uint32_t us)
{
    (void)which; (void)us; ++real_maxes;
}
static void pause_stub(void) { ++pauses; }

static const char *line_of(const char *key)
{
    const char *p = strstr(out, key);
    assert(p != NULL);
    return p;
}

int main(void)
{
    /* Before the window: forwarded, not latched. */
    epoch = 0u;
    __wrap_rt_call(RT_K_LINE_WAKE, 999u);
    assert(real_calls == 1u);
    epoch = 1u;
    for (uint32_t v = 1u; v <= 30u; ++v) {
        __wrap_rt_call(RT_K_LINE_WAKE, v * 10u); /* 10..300 */
    }
    __wrap_rt_call(RT_K_LINE_WAKE, 0u);
    __wrap_rt_call(RT_CALL_COUNT, 5000u);         /* out of range */
    __wrap_rt_max(RT_STEP_READ_LINE, 187u);
    __wrap_rt_max(RT_STEP_READ_LINE, 88u);
    __wrap_rt_max(RT_MAX_COUNT, 7u);
    assert(real_calls == 33u && real_maxes == 3u);

    /* Window still open: no values printed. */
    bk_top_report(pause_stub);
    assert(strcmp(out, "[bench_top] window open\n") == 0);

    /* After stop: frozen; later calls are not latched. */
    epoch = 2u;
    __wrap_rt_call(RT_K_LINE_WAKE, 5000u);
    used = 0u; out[0] = '\0'; pauses = 0u;
    bk_top_report(pause_stub);
    (void)line_of("[bench_top] call line_wake ");
    assert(strstr(out, "[bench_top] call line_wake n_ge100 21 top 300 290 "
                       "280 270 260 250 240 230 220 210 200 190 180 170 "
                       "160 150 140 130 120 110\n"));
    assert(strstr(out, "[bench_top] max read_line n_ge100 1 top 187 88\n"));
    assert(strstr(out, "[bench_top] call isr_wup n_ge100 0 top\n"));
    /* One line per counter, one pause per line. */
    unsigned int lines = 0u;
    for (const char *p = out; (p = strstr(p, "[bench_top] ")) != NULL; ++p) {
        ++lines;
    }
    assert(lines == (unsigned int)RT_CALL_COUNT + (unsigned int)RT_MAX_COUNT);
    assert(pauses == lines);
    return 0;
}

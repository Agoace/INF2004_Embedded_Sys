#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <tk/tkernel.h>
#include "../app_program/subsystem2/motion.h"
#include "../app_program/subsystem3/barcode.h"
#include "../app_program/subsystem3/barcode_live_source.h"
static uint32_t clock_us;
static bool observing, race, raced;
static barcode_live_control_t control;
static char output[16384];
static unsigned int used;
/* RTOS ABI uses unsigned char, which cannot take printf's char-format
 * attribute. Suppress only Clang's impossible attribute suggestion here. */
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-format-attribute"
#endif
int tm_printf(const UB *format, ...)
{
    assert(!observing);
    va_list args;
    va_start(args, format);
    const int n = vsnprintf(output + used, sizeof(output) - used,
                             (const char *)format, args);
    va_end(args);
    assert(n >= 0 && (unsigned int)n < sizeof(output) - used);
    used += (unsigned int)n;
    return n;
}
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
uint32_t drv_time_us(void) { return clock_us++; }
uint32_t drv_time_ms(void) { return clock_us / 1000u; }
int motion_snapshot_try_read(motion_status_t *out, uint32_t *us)
{
    if (race && !raced && clock_us >= 3470000u) {
        ++control.eligibility_revision;
        control.revision_ms = clock_us / 1000u;
        assert(barcode_live_control_publish(&control) == E_OK);
        raced = true;
    }
    *us = clock_us - clock_us % 10000u;
    const float mm = (float)(*us - 1000000u) * 0.000146f;
    const int32_t counts = (int32_t)lroundf(mm * 12.414f);
    *out = (motion_status_t){.counts = {counts, counts}};
    return E_OK;
}
static uint16_t signal(unsigned int ms)
{
    const float mm = (float)ms * 0.146f;
    float at = 80.0f;
    const char text[3] = {'*', 'A', '*'};
    if (mm < at) {
        return 160u;
    }
    unsigned int element = 0u;
    for (unsigned int ch = 0u; ch < 3u; ++ch) {
        if (ch) {
            if (mm < at + 5.8f) {
                return 160u;
            }
            at += 5.8f;
            ++element;
        }
        const uint16_t mask = barcode_pattern(text[ch]);
        for (unsigned int i = 0u; i < 9u; ++i) {
            const float width = (mask & (0x100u >> i)) ? 17.5f : 5.8f;
            if (mm < at + width) {
                return (element & 1u) ? 160u : 1000u;
            }
            at += width;
            ++element;
        }
    }
    return ms >= 2851u && ms < 2951u ? 1200u : 160u;
}
int main(int argc, char **argv)
{
    race = argc > 1 && strcmp(argv[1], "race") == 0;
    assert(barcode_live_source_init() == E_OK);
    const barcode_cal_report_t cal = {.generation = 1u,
        .mode = BC_CAL_LEARNED, .white_samples = 1904u,
        .white_mean = 167u, .rise = 317u, .fall = 242u};
    control = (barcode_live_control_t){.epoch = 7u,
        .eligibility_revision = 3u, .revision_ms = 999u,
        .mode = BC_LIVE_SHADOW_MODE};
    barcode_live_record_t first = {0}, record;
    for (unsigned int i = 0u; i < 3100u; ++i) {
        clock_us = 1000000u + i * 1000u;
        if (i % 10u == 0u) {
            assert(barcode_live_control_publish(&control) == E_OK);
        }
        observing = true;
        barcode_live_source_sample(clock_us, signal(i), &cal, 0u, 0u);
        observing = false;
        assert(barcode_live_source_try_read(&record) == E_OK);
        if (record.sequence != 0u) {
            if (first.sequence == 0u) {
                first = record;
            }
            assert(memcmp(&first, &record, sizeof(first)) == 0);
        }
    }
    assert((first.sequence == 1u) == !race);
    if (!race) {
        assert(first.epoch == 7u && first.eligibility_revision == 3u);
        assert(first.letter == 'A' && first.command == 1u && first.valid == 0u);
        assert(first.flags == (0x3fu | BC_LIVE_SHADOW));
    }
    control.mode = BC_LIVE_DISABLED;
    assert(barcode_live_control_publish(&control) == E_OK);
    barcode_live_source_dump();
    assert(strstr(output, "[BC_CPU]") && strstr(output, "spinW 167 leadW 160"));
    if (race) {
        assert(raced && strstr(output, "reason 102"));
    } else {
        assert(strstr(output, "reason 1"));
        unsigned int count = 0u;
        for (char *line = output; (line = strstr(line, "[BC_EDGE]")); ++line) {
            unsigned long epoch, seq, index, adc, upper;
            long left, right;
            assert(sscanf(line, "[BC_EDGE] epoch %lu seq %lu index %lu "
                "adc_us %lu bracket_us %lu counts %ld/%ld", &epoch, &seq,
                &index, &adc, &upper, &left, &right) == 7);
            assert(epoch == 7u && seq == 1u && index == ++count);
            assert(upper >= adc && upper - adc <= 10000u);
            assert(left == right && left > first.frame_end_counts[0]);
        }
        assert(count == 2u);
    }
    puts(output);
    return 0;
}

#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <tk/tkernel.h>
#include "../app_program/subsystem2/motion.h"
#include "../app_program/subsystem3/barcode.h"
#include "../app_program/subsystem3/barcode_live_source.h"
#include "../app_program/subsystem3/barcode_attempt.h"
#include "../app_program/subsystem3/barcode_window.h"
#ifndef BARCODE_LIVE_ENABLE
#define BARCODE_LIVE_ENABLE 0
#endif
/* Live variant: custom map A=LEFT (id2) and a mode2 vehicle control. */
#define FEED_MODE (BARCODE_LIVE_ENABLE ? BC_LIVE_PERMITTED : BC_LIVE_SHADOW_MODE)
static uint32_t clock_us;
static bool observing, race, raced, epoch_test, cancel_test, interrupt_counts;
static bool start_test, recheck_fail, interrupt_control;
static char data_letter = 'A';
static const barcode_cal_report_t cal = {.generation = 1u,
    .mode = BC_CAL_LEARNED, .white_samples = 1904u,
    .white_mean = 167u, .rise = 317u, .fall = 242u};
static barcode_live_control_t control;
#if BARCODE_FAIL_CAPTURE
static char output[150000];
#else
static char output[16384];
#endif
static unsigned int used;
/* RTOS ABI uses unsigned char, which cannot take printf's char-format
 * attribute. Suppress only Clang's impossible attribute suggestion here. */
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-format-attribute"
#pragma clang diagnostic ignored "-Wformat-nonliteral"
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
        if (recheck_fail) {
            interrupt_control = true;
        } else {
            ++control.eligibility_revision;
            control.revision_ms = clock_us / 1000u;
        }
        assert(barcode_live_control_publish(&control) == E_OK);
        raced = true;
    }
    *us = clock_us - clock_us % 10000u;
    const float mm = (float)(*us - 1000000u) * 0.000146f;
    const int32_t counts = (int32_t)lroundf(mm * 12.414f);
    *out = (motion_status_t){.counts = {counts, counts}};
    return E_OK;
}
void barcode_live_test_read_hook(const uint32_t *sequence, unsigned int index)
{
    (void)sequence;
    if (interrupt_control && index == 0u) {
        interrupt_control = false;
        assert(barcode_live_control_publish(&control) == E_OK);
    }
    if (interrupt_counts && index == 0u) {
        interrupt_counts = false;
        ++control.epoch;
        clock_us += 1000u;
        assert(barcode_live_control_publish(&control) == E_OK);
        barcode_live_source_sample(clock_us, 160u, &cal, 0u, 0u);
    }
}
static uint16_t signal(unsigned int ms)
{
    const float mm = (float)ms * 0.146f;
    float at = 80.0f;
    const char text[3] = {start_test ? 'A' : '*', data_letter, '*'};
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
                /* 700: 5.5 mm bar level (fixed-edge mode; 450..570 over
                 * white measured). Bars >= white + 800 use midpoint edges. */
                return (element & 1u) ? 160u : 700u;
            }
            at += width;
            ++element;
        }
    }
    return ms >= 2851u && ms < 2951u ? 1200u : 160u;
}
int main(int argc, char **argv)
{
    start_test = argc > 1 && strcmp(argv[1], "start_fail") == 0;
    recheck_fail = argc > 1 && strcmp(argv[1], "recheck_fail") == 0;
    race = recheck_fail || (argc > 1 && strcmp(argv[1], "race") == 0);
    epoch_test = argc > 1 && strcmp(argv[1], "counts_epoch") == 0;
    cancel_test = argc > 1 && strcmp(argv[1], "counts_cancel") == 0;
    if (argc > 1 && strcmp(argv[1], "none") == 0) data_letter = 'F';
    barcode_live_counts_t counts = {.epoch = 99u, .frames = 77u,
                                    .attempts = 66u, .failures = 55u};
    const barcode_live_counts_t sentinel = counts;
    assert(barcode_live_counts_try_read(NULL) == E_PAR);
    assert(barcode_live_counts_try_read(&counts) == E_OBJ);
    assert(memcmp(&counts, &sentinel, sizeof(counts)) == 0);
    assert(barcode_live_source_init() == E_OK);
    assert(barcode_live_counts_try_read(&counts) == E_TMOUT);
    assert(memcmp(&counts, &sentinel, sizeof(counts)) == 0);
    control = (barcode_live_control_t){.epoch = 7u,
        .eligibility_revision = 3u, .revision_ms = 999u,
        .mode = FEED_MODE};
    barcode_live_record_t first = {0}, record;
    const unsigned int samples = start_test ? 1600u :
        ((epoch_test || cancel_test) ? 850u : 3100u);
    for (unsigned int i = 0u; i < samples; ++i) {
        clock_us = 1000000u + i * 1000u;
        if (epoch_test && i == 700u) ++control.epoch;
        if (cancel_test && i == 700u) control.mode = BC_LIVE_DISABLED;
        if (i % 10u == 0u) {
            assert(barcode_live_control_publish(&control) == E_OK);
        }
        observing = true;
        barcode_live_source_sample(clock_us, signal(i), &cal, 0u, 0u);
        observing = false;
        assert(barcode_live_source_try_read(&record) == E_OK);
        if ((epoch_test || cancel_test) && i == 700u) {
            assert(barcode_live_counts_try_read(&counts) == E_OK);
            assert(counts.epoch == (epoch_test ? 8u : 7u));
            assert(counts.frames == 0u && counts.failures == (cancel_test ? 1u : 0u));
            assert(counts.attempts == counts.failures);
        }
        if (record.sequence != 0u) {
            if (first.sequence == 0u) {
                /* Exact publication sample, not the 100 ms dump cadence. */
                assert(barcode_live_counts_try_read(&counts) == E_OK);
                assert(counts.frames == 1u && counts.attempts == 1u);
                first = record;
            }
            assert(memcmp(&first, &record, sizeof(first)) == 0);
        }
    }
    assert(barcode_live_counts_try_read(&counts) == E_OK);
    assert(counts.epoch == (epoch_test ? 8u : 7u));
    assert(counts.frames == ((!race && !epoch_test && !cancel_test && !start_test) ? 1u : 0u));
    assert(counts.failures == ((race || cancel_test || start_test) ? 1u : 0u));
    assert(counts.attempts == counts.frames + counts.failures);
    barcode_live_counts_t again;
    assert(barcode_live_counts_try_read(&again) == E_OK);
    assert(memcmp(&again, &counts, sizeof(counts)) == 0);
    assert(barcode_live_source_init() == E_OK); /* Getter/init never reset. */
    assert(barcode_live_counts_try_read(&again) == E_OK);
    assert(memcmp(&again, &counts, sizeof(counts)) == 0);
    assert((first.sequence == 1u) == (!race && !epoch_test && !cancel_test && !start_test));
    if (!race && !epoch_test && !cancel_test && !start_test) {
        assert(first.epoch == 7u && first.eligibility_revision == 3u);
        const bool live = BARCODE_LIVE_ENABLE && data_letter == 'A';
        assert(first.letter == (uint8_t)data_letter);
        assert(first.valid == (live ? 1u : 0u));
        assert(first.command == (data_letter == 'A' ? 1u : 0u));
        assert(first.flags == (live ? BC_LIVE_REQUIRED :
                                      (0x3fu | BC_LIVE_SHADOW)));
    }
    rm_barcode_t attempt;
    assert(barcode_attempt_peek(&attempt));
    assert(attempt.epoch == 7u && attempt.sequence == 1u);
    assert(attempt.uptime_ms >= attempt.start_ms);
    const bool ok_attempt = !race && !epoch_test && !cancel_test && !start_test;
    /* SHADOW clears on MQTT only for a record offered for action. */
    assert(((attempt.flags & RM_BARCODE_SHADOW) == 0u) ==
           (BARCODE_LIVE_ENABLE && ok_attempt && data_letter == 'A'));
    assert((attempt.reason == BW_OK) == ok_attempt);
    assert(attempt.letter == (ok_attempt ? (uint8_t)data_letter : 0u));
    assert(barcode_attempt_ack(attempt.sequence));
    assert(!barcode_attempt_peek(&attempt));
    control.mode = BC_LIVE_DISABLED;
    assert(barcode_live_control_publish(&control) == E_OK);
    barcode_live_source_dump();
    assert(strstr(output, "[BC_REASON_COUNTS]") && strstr(output, "terminal_us "));
    assert(strstr(output, "[BC_CPU]") && strstr(output, "spinW 167 leadW 160"));
    if (epoch_test || cancel_test) {
        /* The old epoch's late cancellation remains in lifetime diagnostics,
         * but must not be charged to the new epoch's zero terminal totals. */
        assert(strstr(output, "attempts 1 failures 1"));
        assert(strstr(output, "[BC_FRAME] epoch 7"));
    } else if (start_test) {
        assert(attempt.reason==BW_START && strstr(output,"reason 5"));
    } else if (race) {
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
#if BARCODE_FAIL_CAPTURE
    if ((race || cancel_test || start_test) && !epoch_test) {
        char *ref=strstr(output,"[BC_REF]");
        char *capture=strstr(output,"[BC_FAIL_CAPTURE]");
        assert(ref && capture && capture>ref);
        assert(strstr(capture,"index 1 ") && strstr(capture,"[BC_FAIL_EDGE]"));
        assert(strstr(capture,"[BC_FAIL_ADC]"));
        if (start_test) {
            assert(strstr(capture,"elements 9 "));
            assert(strstr(capture,"edge 10 adc_lost 0 edge_lost 0"));
        }
    } else assert(!strstr(output,"[BC_FAIL_CAPTURE]"));
#endif
    /* An epoch0 stopped control must not erase the last run's reason line. */
    control.epoch = 0u;
    clock_us += 1000u;
    assert(barcode_live_control_publish(&control) == E_OK);
    barcode_live_source_sample(clock_us, 160u, &cal, 0u, 0u);
    used = 0u; output[0] = '\0';
    barcode_live_source_dump();
    assert(strstr(output, epoch_test ? "[BC_REASON_COUNTS] epoch 8" :
                                      "[BC_REASON_COUNTS] epoch 7"));
    /* Interrupt the count copy with a real LINE-writer epoch transition.
     * Getter returns once, leaves caller bytes unchanged, then reads new data. */
    interrupt_counts = true;
    again = sentinel;
    assert(barcode_live_counts_try_read(&again) == E_TMOUT);
    assert(memcmp(&again, &sentinel, sizeof(again)) == 0);
    assert(!interrupt_counts);
    assert(barcode_live_counts_try_read(&again) == E_OK);
    assert(again.epoch == control.epoch && again.frames == 0u &&
           again.failures == 0u && again.attempts == 0u);
    puts(output);
    return 0;
}

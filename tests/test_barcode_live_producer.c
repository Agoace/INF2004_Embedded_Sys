#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../app_program/subsystem3/barcode.h"
#include "../app_program/subsystem3/barcode_live_producer.h"
#ifndef BARCODE_LIVE_ENABLE
#define BARCODE_LIVE_ENABLE 0
#endif
static barcode_live_producer_t p;
#ifdef TEST_CONTROL_CACHE
static bool grace_enabled;
static uint32_t grace_start;
bool blv_decode_grace_active(const barcode_live_control_t *control, uint32_t now_ms);
bool blv_decode_grace_active(const barcode_live_control_t *control, uint32_t now_ms)
{
    return grace_enabled && control->epoch == 7u &&
        control->eligibility_revision == 3u && now_ms - grace_start < 200u;
}
#endif

/* Expected live gate: build enable, custom map (id>=2), mode2 at bind and
 * publish, and a mapped command. Anything else stays SHADOW/valid0. */
static bool expect_valid(uint8_t letter, uint32_t mode)
{
    return BARCODE_LIVE_ENABLE && barcode_window_mapping_id() >= 2u &&
           mode == BC_LIVE_PERMITTED &&
           barcode_window_table_entry((unsigned int)(letter - 'A')) != 0u;
}

static uint16_t waveform(float mm, float start, char letter, bool reverse)
{
    const char text[3] = {'*', letter, '*'};
    float runs[29];
    unsigned int count = 0u;
    for (unsigned int ch = 0u; ch < 3u; ++ch) {
        if (ch) {
            runs[count++] = 5.8f;
        }
        const uint16_t mask = barcode_pattern(text[ch]);
        for (unsigned int i = 0u; i < 9u; ++i) {
            runs[count++] = (mask & (0x100u >> i)) ? 17.5f : 5.8f;
        }
    }
    if (mm < start) {
        return 160u;
    }
    for (unsigned int i = 0u; i < count; ++i) {
        const float width = runs[reverse ? 28u - i : i];
        if (mm < start + width) {
            return (i & 1u) == 0u ? 1000u : 160u;
        }
        start += width;
    }
    return 160u;
}

int main(int argc, char **argv)
{
    const char *kind = argc > 1 ? argv[1] : "normal";
    uint64_t origin = 1000000u;
    if (strcmp(kind, "us_wrap") == 0) {
        origin = UINT64_C(0x100000000) - 800000u;
    } else if (strcmp(kind, "ms_wrap") == 0) {
        origin = (UINT64_C(0x100000000) - 500u) * 1000u;
    }
    barcode_cal_report_t cal = {.generation = 1u, .mode = BC_CAL_LEARNED,
        .white_mean = 167u, .white_samples = 1904u, .rise = 317u, .fall = 242u};
    barcode_live_input_t in = {.calibration = &cal,
        .control = {.epoch = 7u, .eligibility_revision = 3u,
                    .revision_ms = (uint32_t)(origin / 1000u) - 1u,
                    .mode = BC_LIVE_SHADOW_MODE},
        .control_ok = true, .motor_ok = true};
    const bool search = strncmp(kind, "search", 6u) == 0;
    const bool terminal_search = strncmp(kind, "search_end", 10u) == 0;
    const unsigned int search_begin = strstr(kind, "unbound") ? 500u :
        terminal_search ? 2450u : 1200u;
    const unsigned int search_duration = (strstr(kind, "199") || strstr(kind, "unbound")) ? 199u :
        strstr(kind, "200") ? 200u : strstr(kind, "late") ? 100u :
        strstr(kind, "end50") ? 70u : strstr(kind, "end51") ? 71u : 30u;
    const bool search_positive = search && !strstr(kind, "200") &&
        !strstr(kind, "context") && !strstr(kind, "late") &&
        !strstr(kind, "end51") && !strstr(kind, "hard") &&
        !strstr(kind, "unbound") && !strstr(kind, "motorfault") &&
        !strstr(kind, "loss");
    const bool reverse = strcmp(kind, "reverse") == 0;
    const bool ack_kind = strcmp(kind, "ack") == 0 ||
                          strcmp(kind, "mode2_ack") == 0;
    const bool old_ack = strcmp(kind, "old_epoch_ack") == 0;
    const bool two = ack_kind || old_ack ||
                     strcmp(kind, "no_ack") == 0 ||
                     strcmp(kind, "old_epoch_no_ack") == 0 ||
                     strcmp(kind, "busy_ack") == 0;
    if (strcmp(kind, "sequence_wrap") == 0) {
        p.sequence = UINT32_MAX;
    }
    if (strcmp(kind, "cut_equal") == 0) {
        in.control.revision_ms = (uint32_t)(origin / 1000u) + 548u;
    }
    if (strcmp(kind, "fallback") == 0) {
        cal.mode = BC_CAL_FALLBACK;
        cal.white_mean = 0u;
        cal.white_samples = 0u;
        cal.faults = BC_CAL_NO_WHITE;
    }
    if (strcmp(kind, "mode2") == 0 || strcmp(kind, "mode2_ack") == 0) {
        in.control.mode = BC_LIVE_PERMITTED;
    }
    if (search) in.control.mode = BC_LIVE_PERMITTED;
    unsigned int publications = 0u, failures = 0u, live_records = 0u;
    barcode_live_record_t original = {0};
    for (unsigned int i = 0u; i < (two ? 6000u : 3000u); ++i) {
        if (strcmp(kind, "pause") == 0 && i >= 1200u && i < 1270u) {
            continue;
        }
        const uint64_t now = origin + (uint64_t)i * 1000u;
        in.now_us = (uint32_t)now;
        in.now_ms = (uint32_t)(now / 1000u);
        in.adc_us = in.now_us;
        in.motor_us = (uint32_t)(origin + (uint64_t)(i - i % 10u) * 1000u);
        const uint32_t counts = (uint32_t)lroundf(
            (float)(i - i % 10u) * 146.0f * 12.414f / 1000.0f);
        in.left = in.right = counts;
        if (strcmp(kind, "count_wrap") == 0) {
            in.left += UINT32_MAX - 3000u;
            in.right = in.left;
        }
        in.tag = 2u + (i / 10u) * 2u;
        if (strcmp(kind, "no_progress") == 0) {
            in.tag = 2u;
        } else if (strcmp(kind, "stale") == 0 && i >= 1000u) {
            in.tag = 202u;
        }
        const float mm = (float)i * 0.146f;
        const char letter = strcmp(kind, "nonletter") == 0 ? '3' : 'A';
        in.raw = mm < 500.0f ? waveform(mm, 80.0f, letter, reverse) :
                              waveform(mm, 500.0f, 'B', false);
        in.control_ok = !(strcmp(kind, "read_fail") == 0 && i == 1200u);
        in.motor_faults = 0u;
        if (i == 1200u) {
            if (strcmp(kind, "epoch") == 0) {
                ++in.control.epoch;
            } else if (strcmp(kind, "revision") == 0) {
                ++in.control.eligibility_revision;
            } else if (strcmp(kind, "cut_change") == 0) {
                in.control.revision_ms = in.now_ms;
            } else if (strcmp(kind, "disabled") == 0) {
                in.control.mode = BC_LIVE_DISABLED;
            } else if (strcmp(kind, "loss") == 0) {
                ++in.dropped;
            } else if (strcmp(kind, "missed") == 0) {
                ++in.missed;
            } else if (strcmp(kind, "generation") == 0) {
                ++cal.generation;
            } else if (strcmp(kind, "motor_fault") == 0) {
                in.motor_faults = 1u;
            } else if (strcmp(kind, "motor_old") == 0) {
                in.motor_us -= 31000u;
            } else if (strcmp(kind, "raw_clip") == 0) {
                in.raw = 4095u;
            } else if (strcmp(kind, "clock") == 0) {
                in.now_ms += 10u;
            }
        }
        if (i == 2600u && (old_ack ||
            strcmp(kind, "old_epoch_no_ack") == 0)) {
            ++in.control.epoch;
            in.control.revision_ms = in.now_ms;
        }
        if ((ack_kind || old_ack ||
             strcmp(kind, "busy_ack") == 0) && i == 2600u) {
            assert(p.occupied && p.slot.sequence == 1u);
            in.control.ack_sequence = p.slot.sequence;
            if (strcmp(kind, "busy_ack") == 0) {
                in.control.mode = BC_LIVE_DISABLED;
            }
        }
        if (strcmp(kind, "busy_ack") == 0 && i == 4200u) {
            in.control.mode = BC_LIVE_SHADOW_MODE;
        }
        if (search) {
#ifdef TEST_CONTROL_CACHE
            grace_start = (uint32_t)(origin / 1000u) + search_begin;
            grace_enabled = !strstr(kind, "hard");
#endif
            if (i >= search_begin && i < search_begin + search_duration) {
                in.control.mode = BC_LIVE_DISABLED;
            } else {
                in.control.mode = BC_LIVE_PERMITTED;
            }
            if (strstr(kind, "context") && i == search_begin + 10u)
                ++in.control.eligibility_revision;
            if (strstr(kind, "motorfault") && i == search_begin + 5u)
                in.motor_faults = 1u;
            if (strstr(kind, "loss") && i == search_begin + 5u) ++in.dropped;
        }
        barcode_live_producer_observe(&p, &in);
        if (search && in.control.mode == BC_LIVE_DISABLED) {
            assert(!p.record_pending);
            if (strstr(kind, "unbound")) assert(!p.bound);
            assert(!p.event_pending || p.event.reason != BW_OK);
        }
        if (p.event_pending) {
            if (strcmp(kind, "nonletter") == 0) {
                assert(p.event.reason == BW_NOT_LETTER);
                assert(p.event.record.letter == '3');
            }
            if (p.event.reason != BW_OK) {
                ++failures;
            }
            p.event_pending = false;
        }
        if (p.record_pending) {
            ++publications;
            assert(p.slot.source == 2u && p.slot.version == 1u);
            assert(p.slot.reserved == 0u);
            assert(p.slot.letter == (publications == 1u ? 'A' : 'B'));
            assert(p.slot.command ==
                   barcode_window_table_entry(p.slot.letter - 'A'));
            if (expect_valid(p.slot.letter, in.control.mode)) {
                assert(p.slot.valid == 1u &&
                       p.slot.flags == BC_LIVE_REQUIRED);
                ++live_records;
            } else {
                assert(p.slot.valid == 0u);
                assert(p.slot.flags == (0x3fu | BC_LIVE_SHADOW));
            }
            assert(p.slot.direction == (reverse ? 2u : 1u));
            assert(p.slot.read_ms == barcode_live_ms_at(in.now_ms,
                in.now_us, p.window.result.read_us));
            assert(p.event.leading_white == 160u && p.event.rise == 310u);
            assert(p.event.spin_white == cal.white_mean);
            assert(p.slot.read_counts[0] == p.slot.read_counts[1]);
            assert(p.event.record_age_ms <= (terminal_search ? 50u : 10u));
            original = p.slot;
            p.record_pending = false;
        }
        if (p.occupied && publications == 1u) {
            assert(memcmp(&p.slot, &original, sizeof(original)) == 0);
        }
    }
    const bool cached_read_fail =
#ifdef TEST_CONTROL_CACHE
        strcmp(kind, "read_fail") == 0;
#else
        false;
#endif
    const bool positive = search_positive || cached_read_fail || strcmp(kind, "normal") == 0 || reverse || two ||
        strcmp(kind, "us_wrap") == 0 || strcmp(kind, "ms_wrap") == 0 ||
        strcmp(kind, "count_wrap") == 0 || strcmp(kind, "mode2") == 0 ||
        strcmp(kind, "fallback") == 0 || strcmp(kind, "sequence_wrap") == 0;
    assert(publications == (ack_kind || old_ack ? 2u :
                            positive ? 1u : 0u));
    if (ack_kind || old_ack) {
        assert(p.occupied && p.slot.sequence == 2u);
        assert(in.control.ack_sequence == 1u); /* Old ACK cannot free X+1. */
        assert(p.slot.epoch == in.control.epoch);
    }
    if (strcmp(kind, "old_epoch_no_ack") == 0) {
        assert(p.occupied && p.slot.epoch != in.control.epoch);
        assert(p.sequence == 1u); /* Epoch does not reset the slot/serial. */
    }
    if (!positive && strcmp(kind, "no_progress") != 0 && !strstr(kind, "unbound")) {
        assert(failures > 0u);
    }
    /* The mode2 positive case is the only one that may act, and only in
     * a live build with a custom confirmed map and a mapped letter A. */
    const unsigned int live_expected = search_positive ?
        (expect_valid('A', 2u) ? 1u : 0u) : strcmp(kind, "mode2") == 0 ?
        (expect_valid('A', 2u) ? 1u : 0u) : strcmp(kind, "mode2_ack") == 0 ?
        (expect_valid('A', 2u) ? 1u : 0u) + (expect_valid('B', 2u) ? 1u : 0u)
        : 0u;
    assert(live_records == live_expected);
    printf("%s live%u pubs%u failures%u max_control%lu max_record%lu\n", kind,
           live_records, publications, failures, (unsigned long)p.max_control_age_ms,
           (unsigned long)p.max_record_age_ms);
    /* Nonzero fractional phase across independent us/ms wraps. */
    const uint64_t t = origin + 2345678u;
    for (unsigned int age = 0u; age < 60000u; age += 137u) {
        assert(barcode_live_ms_at((uint32_t)(t / 1000u), (uint32_t)t,
            (uint32_t)(t - age)) == (uint32_t)((t - age) / 1000u));
    }
    return 0;
}

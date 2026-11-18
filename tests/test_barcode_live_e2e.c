/* HOST ONLY end-to-end: real S3 source/producer/window -> real vehicle
 * junction_source/jlive -> real ja. One decode must give at most one offer,
 * and a live offer is consumed at exactly one junction. No firmware build.
 * Variants (CMake): default (LIVE0), custom (LIVE1, F=LEFT id2), builtin
 * (LIVE1, built-in table: letter A maps LEFT but id1 never acts). */
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <tk/tkernel.h>
#include "../app_program/subsystem2/motion.h"
#include "../app_program/subsystem3/barcode.h"
#include "../app_program/subsystem3/barcode_live_source.h"
#include "../app_program/vehicle/junction_source.h"
#include "../app_program/vehicle/junction_live.h"

#ifndef E2E_LETTER
#define E2E_LETTER 'F'
#endif
#ifndef E2E_EXPECT_OFFER
#define E2E_EXPECT_OFFER 0
#endif

static uint32_t clock_us;
static const barcode_cal_report_t cal = {.generation = 1u,
    .mode = BC_CAL_LEARNED, .white_samples = 1904u,
    .white_mean = 167u, .rise = 317u, .fall = 242u};

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-format-attribute"
#pragma clang diagnostic ignored "-Wformat-nonliteral"
#endif
int tm_printf(const UB *format, ...)
{
    (void)format;
    return 0;
}
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
uint32_t drv_time_us(void) { return clock_us; }
uint32_t drv_time_ms(void) { return clock_us / 1000u; }
void barcode_live_test_read_hook(const uint32_t *sequence, unsigned int index)
{
    (void)sequence;
    (void)index;
}

static float travel_mm(uint32_t us)
{
    return (float)(us - 1000000u) * 0.000146f; /* 146 mm/s forward. */
}

static int32_t counts_at(uint32_t us)
{
    return (int32_t)lroundf(travel_mm(us) * 12.414f);
}

int motion_snapshot_try_read(motion_status_t *out, uint32_t *us)
{
    *us = clock_us - clock_us % 10000u;
    const int32_t c = counts_at(*us);
    *out = (motion_status_t){.counts = {c, c}};
    return E_OK;
}

/* GP28: *LETTER* at 80 mm, 5.8/17.5 mm elements; white elsewhere. */
static uint16_t gp28(float mm)
{
    float at = 80.0f;
    const char text[3] = {'*', E2E_LETTER, '*'};
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
    return 160u;
}

/* Line pair: a centred line, with two 30 mm junctions (crossings). */
#ifndef J1_MM
#define J1_MM 600.0f
#endif
#define J2_MM 900.0f
/* Live policy: 0 none, 1 EXPIRED, 2 SEARCH, 3 normal consumption. */
#ifndef E2E_POLICY
#define E2E_POLICY 0
#endif
#if E2E_POLICY
#include "../app_program/barcode_live_vehicle/policy.h"
#define E2E_SEARCH_MM 450.0f
#endif
static bool junction_at(float mm)
{
    return (mm >= J1_MM && mm < J1_MM + 30.0f) ||
           (mm >= J2_MM && mm < J2_MM + 30.0f);
}

int main(void)
{
    static ja_t action;
    static lp_t lp;
    uint16_t ping_ms = 0u;
    const uint32_t epoch = 1u;
    clock_us = 1000000u;
    assert(barcode_live_source_init() == E_OK);
    ja_init(&action, 131, epoch);
    const int32_t start_counts[2] = {0, 0};
    junction_source_start(0u, epoch, drv_time_ms(), start_counts);
    unsigned int offers = 0u, accepted = 0u, records = 0u, claimed_ticks = 0u;
    uint32_t record_sequence = 0u, offered_sequence = 0u;
    float claim_mm = -1.0f;
    ja_command_t last = {0};
    for (unsigned int ms = 0u; ms < 7000u; ++ms) {
        clock_us = 1000000u + ms * 1000u;
        const float mm = travel_mm(clock_us);
        if (ms % 10u == 0u) {
            /* Vehicle tick (LINE module): ja first, then source/offer. */
            const bool black = junction_at(mm);
            lp.flags = LP_FLAG_CALIBRATED | LP_FLAG_LINE_SEEN |
                (black ? LP_FLAG_JUNCTION : LP_FLAG_CENTRED);
            lp.norm[0] = black ? 1000u : 600u;
            lp.norm[1] = black ? 1000u : 300u;
            lp.pair = true;
            const ja_sample_t sample = {.line_stamp = clock_us,
                                        .line_age_ms = 0u, .valid = true};
            const int32_t c = counts_at(clock_us);
            const int32_t counts[2] = {c, c};
            vsm_inputs_t in = {.line = &lp, .counts = {c, c},
                               .ping_ms = &ping_ms};
            vsm_drive_t drive = {0};
#if E2E_POLICY
            /* watched_step order: route policy before the LINE step. */
            const bool search = E2E_POLICY == 2 && mm >= E2E_SEARCH_MM &&
                                mm < E2E_SEARCH_MM + 2.0f;
            blv_pending_route(&action, search ? VSM_MOD_SEARCH :
                              VSM_MOD_LINE, counts, drv_time_ms());
#endif
            const uint32_t consumed_before = action.consumed;
            (void)ja_step(&action, &in, &sample, drv_time_ms(), &drive);
            if (action.consumed != consumed_before) {
                assert(claim_mm < 0.0f); /* Exactly one junction. */
                claim_mm = mm;
                ++claimed_ticks;
            }
            junction_source_tick(VSM_MOD_LINE, &lp, &sample, counts,
                                 drv_time_ms(), &action);
            ja_command_t command;
            if (junction_source_read(&command)) {
                ++offers;
                last = command;
                const bool ok = ja_offer(&action, &command);
                accepted += ok ? 1u : 0u;
                offered_sequence = command.sequence;
                junction_source_offer_result(command.sequence, ok);
                /* Duplicate: the same command can never be offered twice. */
                assert(!ja_offer(&action, &command));
            }
            if (action.phase == JA_PENDING) {
                barcode_live_control_t control;
                assert(barcode_live_control_try_read(&control, NULL) == E_OK);
                assert(control.mode == BC_LIVE_DISABLED); /* Reader off. */
            }
        }
        barcode_live_source_sample(clock_us, gp28(mm), &cal, 0u, 0u);
        barcode_live_record_t record;
        assert(barcode_live_source_try_read(&record) == E_OK);
        if (record.sequence != record_sequence) {
            ++records;
            record_sequence = record.sequence;
            assert(record.epoch == epoch && record.letter == E2E_LETTER);
            assert(record.valid == (E2E_EXPECT_OFFER ? 1u : 0u));
            assert(record.flags == (E2E_EXPECT_OFFER ? BC_LIVE_REQUIRED :
                                    (0x3fu | BC_LIVE_SHADOW)));
        }
    }
    printf("records %u offers %u accepted %u claim_mm %.1f phase %u\n",
           records, offers, accepted, (double)claim_mm,
           (unsigned int)action.phase);
    assert(records == 1u); /* One pass, one decode, one record. */
    assert(junction_source_acknowledged() == record_sequence);
#if E2E_POLICY == 1 || E2E_POLICY == 2
    /* Offered once, never consumed: no junction acts on a dropped command. */
    assert(offers == 1u && accepted == 1u && claimed_ticks == 0u);
    assert(action.phase == JA_UNUSED && !action.result.claimed);
    assert(action.consumed == record_sequence);
    assert(action.result.reason == (E2E_POLICY == 1 ? BLV_REASON_EXPIRED :
                                    BLV_REASON_SEARCH));
    return 0;
#endif
    if (E2E_EXPECT_OFFER) {
        assert(offers == 1u && accepted == 1u);
        assert(offered_sequence == record_sequence);
        assert(last.command == JA_LEFT && last.valid == 1u &&
               last.source == BC_LIVE_SOURCE && last.epoch == epoch);
        assert(claimed_ticks == 1u && action.consumed == record_sequence);
        assert(action.result.sequence == record_sequence &&
               action.result.command == JA_LEFT && action.result.claimed);
        /* Consumed at the FIRST junction after the barcode, not the second. */
        assert(claim_mm >= J1_MM && claim_mm < J1_MM + 30.0f);
    } else {
        assert(offers == 0u && accepted == 0u && claimed_ticks == 0u);
        assert(action.consumed == 0u && action.phase == JA_IDLE);
    }
    return 0;
}

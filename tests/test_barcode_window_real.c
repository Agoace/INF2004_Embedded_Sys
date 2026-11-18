/* Real ADC/motor rows only. Old detector BC_RUN rows never classify input. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../app_program/subsystem3/barcode.h"
#include "../app_program/subsystem3/barcode_window.h"
#ifdef BCEP_VERIFY
#include "../app_program/subsystem3/barcode_evidence_pack.h"
/* Replay each real reader result through the evidence codec. No replay input,
 * reader state or output row is changed. Compare this target's CSV with the
 * existing native target, byte for byte. */
static void verify_evidence(const barcode_window_result_t *r)
{
    const barcode_live_frame_t f={.record={.sequence=r->serial,
        .read_counts={bcep_signed(r->read_left),bcep_signed(r->read_right)},
        .frame_end_counts={bcep_signed(r->frame_left),bcep_signed(r->frame_right)},
        .letter=(uint8_t)r->letter,.direction=r->direction,
        .command=r->mapped_action,.valid=r->accepted},
        .index=r->serial,.reason=r->reason,.start_us=r->start_us,
        .edge_us=r->edge_us,.frame_us=r->frame_us,.eligible_us=r->eligible_us,
        .narrow_mm=r->narrow_mm,.wide_mm=r->wide_mm,.leading_mm=r->leading_mm,
        .rise=r->rise,.fall=r->fall};
    bcep_frame_t packed; barcode_live_frame_t out;
    assert(bcep_frame_pack(&packed,&f)); assert(bcep_frame_unpack(&out,&packed));
    assert(memcmp(&out,&f,sizeof(f))==0);
}
#endif

#ifndef BC_READER_LABEL
#define BC_READER_LABEL "checkout"
#endif
static barcode_sample_t adc[BC_MAX_SAMPLES];
static barcode_motion_t motor[BC_MAX_MOTION];
static barcode_window_t state;

static const char *reason_name(uint32_t reason)
{
    static const char *names[] = {"BW_WAIT", "BW_OK", "BW_CALIBRATION",
        "BW_LOSS", "BW_MOTION", "BW_START", "BW_WIDTH", "BW_GAP",
        "BW_EXTENT", "BW_TAIL", "BW_NOT_LETTER"};
    assert(reason < sizeof(names) / sizeof(names[0]));
    return names[reason];
}

/* The CSV does NOT contain the original spin calibration. Reconstruct an
 * ASSUMED fallback report from its initial uninterrupted 120..220 ADC white
 * prefix. The reader must still earn fresh spatial white and freezes its
 * own thresholds; neither old BC_RUN widths nor a requested F are used. */
static barcode_cal_report_t reconstruct_calibration(unsigned int n)
{
    uint32_t sum = 0u;
    unsigned int prefix = 0u;
    while (prefix < n && adc[prefix].raw >= 120u && adc[prefix].raw <= 220u) {
        sum += adc[prefix++].raw;
    }
    assert(prefix >= 16u && prefix <= UINT16_MAX);
    const uint16_t white = (uint16_t)((sum + prefix / 2u) / prefix);
    const uint16_t rise = (uint16_t)(white + 150u < 350u ? white + 150u : 350u);
    const uint16_t fall = (uint16_t)(white + 75u < rise - 75u ? white + 75u : rise - 75u);
    return (barcode_cal_report_t){.mode = BC_CAL_FALLBACK,
        .white_mean = white, .white_samples = (uint16_t)prefix,
        .rise = rise, .fall = fall, .faults = BC_CAL_NO_BLACK};
}

int main(void)
{
    const char *names[] = {"barcode_large_f_forward.csv",
                           "barcode_430_large_fwd.csv",
                           "barcode_430_large_back_cable.csv",
                           "barcode_430_small_fwd.csv",
                           "barcode_430_small_back.csv"};
    puts("fixture,reader,profile,accepted,reason,event_relative_us,letter,"
         "n_mm,w_mm,first_failure,first_failure_processing_us,failures,"
         "cal_white,cal_prefix,adc_max_gap_us,motor_max_gap_us,rise,fall");
    for (unsigned int fixture = 0u; fixture < 5u; ++fixture) {
        char path[512], line[180];
        const int length = snprintf(path, sizeof(path), "%s/%s",
                                    BC_FIXTURES, names[fixture]);
        assert(length > 0 && (unsigned int)length < sizeof(path));
        FILE *file = fopen(path, "r");
        assert(file);
        unsigned int n = 0u, m = 0u;
        uint32_t adc_gap = 0u, motor_gap = 0u;
        while (fgets(line, sizeof(line), file)) {
            unsigned long index, us, raw, flags, delta;
            long left, right;
            if (sscanf(line, "BC_ADC,%lu,%lu,%lu,%lu,%lu", &index, &us,
                       &raw, &flags, &delta) == 5) {
                assert(index == n && n < BC_MAX_SAMPLES && raw < 4095u);
                assert(us <= UINT32_MAX && flags == 0u);
                if (n != 0u) {
                    assert(us > adc[n - 1u].us);
                    const uint32_t gap = (uint32_t)us - adc[n - 1u].us;
                    if (gap > adc_gap) adc_gap = gap;
                }
                adc[n++] = (barcode_sample_t){(uint32_t)us, (uint16_t)raw, 0u};
            } else if (sscanf(line, "BC_MOTOR,%lu,%lu,%ld,%ld", &index, &us,
                              &left, &right) == 4) {
                assert(index == m && m < BC_MAX_MOTION && us <= UINT32_MAX);
                assert(left >= INT32_MIN && left <= INT32_MAX);
                assert(right >= INT32_MIN && right <= INT32_MAX);
                if (m != 0u) {
                    assert(us > motor[m - 1u].us);
                    const uint32_t gap = (uint32_t)us - motor[m - 1u].us;
                    if (gap > motor_gap) motor_gap = gap;
                }
                motor[m++] = (barcode_motion_t){
                    (uint32_t)us, (int32_t)left, (int32_t)right};
            }
        }
        assert(fclose(file) == 0 && n > 2000u && m > 150u);
        const barcode_cal_report_t cal = reconstruct_calibration(n);
        for (unsigned int profile = 0u; profile < 3u; ++profile) {
            const bool compressed = profile == 1u;
            const bool live = profile == 2u;
            const uint32_t origin =
                adc[0].us < motor[0].us ? adc[0].us : motor[0].us;
            barcode_window_config_t config = barcode_window_default_config;
            /* ASSUMED HOST retained-data profile, not weakened live defaults.
             * All measured retained gaps are <=2011/49998 us. */
            if (!live) {
                config.motor_gap_us = 50000u;
                config.sample_gap_us = 3000u;
            }
            assert(barcode_window_begin(&state, &cal, &config));
            unsigned int a = 0u, b = 0u, failures = 0u;
            uint32_t serial = 0u, first_reason = BW_WAIT;
            uint32_t first_failure_us = 0u, event_us = 0u;
            while (a < n || b < m) {
                const bool motion = b < m &&
                    (a == n || motor[b].us <= adc[a].us);
                const uint32_t us = motion ? motor[b].us : adc[a].us;
                const uint32_t relative = us - origin;
                /* Preserve native timestamp differences except in the
                 * explicitly SYNTHETIC timing-only compression profile. */
                const uint32_t stamp = compressed ?
                    (uint32_t)((uint64_t)relative * 100u / 146u) : relative;
                if (motion) {
                    barcode_window_motor(&state, stamp,
                        (uint32_t)motor[b].left, (uint32_t)motor[b].right, 0u);
                    ++b;
                } else {
                    barcode_window_adc(&state, stamp, adc[a].raw, adc[a].flags);
                    ++a;
                }
                (void)barcode_window_step(&state, BW_STEP_MAX);
                if (serial != state.result.serial) {
                    serial = state.result.serial;
#ifdef BCEP_VERIFY
                    verify_evidence(&state.result);
#endif
                    event_us = state.result.accepted ? state.result.eligible_us : stamp;
                    if (!state.result.accepted) {
                        if (failures++ == 0u) {
                            first_reason = state.result.reason;
                            first_failure_us = stamp;
                        }
                    }
                }
            }
            printf("%s,%s,%s,%u,%s,%lu,%c,%.3f,%.3f,%s,%lu,%u,"
                   "%u,%u,%lu,%lu,%u,%u\n", names[fixture], BC_READER_LABEL,
                   live ? "native_live" : compressed ? "compressed_retained" :
                   "native_retained", state.result.accepted,
                   reason_name(state.result.reason), (unsigned long)event_us,
                   state.result.accepted ? state.result.letter : '-',
                   (double)state.result.narrow_mm, (double)state.result.wide_mm,
                   reason_name(first_reason), (unsigned long)first_failure_us,
                   failures, cal.white_mean, cal.white_samples,
                   (unsigned long)adc_gap, (unsigned long)motor_gap,
                   state.rise, state.fall);
            assert(state.result.accepted == (!live && fixture < 2u));
            /* Backward motion can supersede retained-sample gap failure. */
            if (live) assert(state.result.reason == BW_LOSS ||
                             state.result.reason == BW_MOTION);
            if (!live && fixture >= 2u) {
                assert(state.result.reason == (fixture == 3u ? BW_WIDTH : BW_MOTION));
            }
            if (state.result.accepted) {
                /* 73/73 mm GP28 geometry; 80/77 gave 5.766/5.776, 17.551/17.715. */
                const float expected_n[] = {5.769f, 5.778f};
                const float expected_w[] = {17.573f, 17.727f};
                const uint32_t expected_us[] = {3686283u, 3526276u};
                assert(state.result.narrow_mm > expected_n[fixture] - 0.02f &&
                       state.result.narrow_mm < expected_n[fixture] + 0.02f);
                assert(state.result.wide_mm > expected_w[fixture] - 0.02f &&
                       state.result.wide_mm < expected_w[fixture] + 0.02f);
                if (!compressed) {
                    assert(state.result.eligible_us > expected_us[fixture] - 3000u &&
                           state.result.eligible_us < expected_us[fixture] + 3000u);
                }
                assert(state.result.reason == BW_OK && state.result.letter == 'F');
                assert(state.result.mapped_action == 0u);
                assert(state.result.narrow_mm > 4.6f && state.result.narrow_mm < 7.0f);
                assert(state.result.wide_mm > 13.5f && state.result.wide_mm < 21.5f);
            }
        }
    }
    return 0;
}

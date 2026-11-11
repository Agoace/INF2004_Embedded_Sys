/* Only the four Main-approved extracted CSVs. Ignore old BC_RUN output. */
#include "../app_program/subsystem3/barcode_peak.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static barcode_sample_t rows[BC_MAX_SAMPLES];
static barcode_packed_t packed[BC_MAX_SAMPLES];
static barcode_motion_t motors[BC_MAX_MOTION];
static barcode_workspace_t width_workspace;
static barcode_report_t width;
static barcode_peak_workspace_t workspace;
static barcode_peak_report_t report;
static unsigned int frames;
static uint32_t first_frame, last_frame;

static void frame(const barcode_frame_t *v, void *context)
{
    (void)context;
    assert(v->value.decoded && strcmp(v->value.text, "F") == 0);
    assert(v->start_um != INT32_MIN && v->end_um != INT32_MIN);
    assert(v->end_us - v->start_us < 4000000u);
    const uint32_t first = v->start_us - rows[0].us;
    const uint32_t last = v->end_us - rows[0].us;
    if (frames == 0u || first < first_frame) {
        first_frame = first;
    }
    if (frames == 0u || last > last_frame) {
        last_frame = last;
    }
    ++frames;
}

static void run(size_t n, size_t m, bool use_packed)
{
    const barcode_peak_config_t config = {
        {64u, 3500u, 12414u, 50000u}, 100u, 151u, 170u};
    for (size_t i = 0u; i < n; ++i) {
        const uint32_t dt = i == 0u ? 0u : rows[i].us - rows[i - 1u].us;
        assert(dt <= UINT16_MAX);
        packed[i] = (barcode_packed_t){rows[i].raw, (uint16_t)dt};
    }
    barcode_peak_begin(use_packed ? NULL : rows, n, use_packed ? packed : NULL,
                       rows[0].us, motors, m, &config, &workspace, &report);
    frames = 0u;
    barcode_peak_sink(&workspace, frame, NULL);
    unsigned int steps = 0u;
    while (!barcode_peak_step(&workspace)) {
        assert(++steps < 10000u);
    }
    printf("peak: %s/%s %s/%s lobes %u/%u sd %.2f/%.2f\n",
           barcode_reason_name(report.time.value.reason),
           barcode_reason_name(report.distance.value.reason),
           report.time.value.text, report.distance.value.text,
           report.time.peaks, report.distance.peaks,
           (double)report.time.noise_sd, (double)report.distance.noise_sd);
}

int main(void)
{
    const char *const names[] = {"large_fwd", "large_back_cable", "small_fwd",
                                 "small_back"};
    for (size_t fixture = 0u; fixture < 4u; ++fixture) {
        char path[512], line[160];
        const int length = snprintf(path, sizeof(path), "%s/barcode_430_%s.csv",
                                    BC_FIXTURES, names[fixture]);
        assert(length > 0 && (size_t)length < sizeof(path));
        FILE *file = fopen(path, "r");
        assert(file != NULL);
        size_t n = 0u, m = 0u;
        while (fgets(line, sizeof(line), file) != NULL) {
            unsigned long index, us, raw, flags, dt;
            long left, right;
            if (sscanf(line, "BC_ADC,%lu,%lu,%lu,%lu,%lu", &index, &us, &raw,
                       &flags, &dt) == 5) {
                assert(index == n && n < BC_MAX_SAMPLES && raw <= 4095u);
                assert(flags == 0u && us <= UINT32_MAX);
                rows[n++] = (barcode_sample_t){(uint32_t)us, (uint16_t)raw, 0u};
            } else if (sscanf(line, "BC_MOTOR,%lu,%lu,%ld,%ld", &index, &us,
                              &left, &right) == 4) {
                assert(index == m && m < BC_MAX_MOTION);
                assert(us <= UINT32_MAX && left >= INT32_MIN &&
                       left <= INT32_MAX && right >= INT32_MIN &&
                       right <= INT32_MAX);
                motors[m++] = (barcode_motion_t){(uint32_t)us, (int32_t)left,
                                                 (int32_t)right};
            }
        }
        assert(fclose(file) == 0 && n > 2200u && m > 150u);
        for (unsigned int reversed = 0u; reversed < 2u; ++reversed) {
            printf("%s reversed %u\n", names[fixture], reversed);
            for (unsigned int p = 0u; p < 2u; ++p) {
                run(n, m, p != 0u);
                assert(report.time.value.decoded);
                assert(report.distance.value.decoded);
                assert(strcmp(report.time.value.text, "F") == 0);
                assert(strcmp(report.distance.value.text, "F") == 0);
                assert(report.time.value.command == BC_CMD_NONE);
                assert(report.distance.value.command == BC_CMD_NONE);
                assert(report.time.peaks == 15u &&
                       report.distance.peaks == 15u);
                assert(frames == 2u);
                const uint8_t direction =
                    (uint8_t)(((fixture & 1u) == reversed) ? 1u : 2u);
                assert(report.time.value.directions == direction);
                assert(report.distance.value.directions == direction);
            }
            barcode_config_t config = barcode_default_config;
            config.motion_gap_us = 50000u;
            config.max_gap_us = 3500u;
            barcode_decode(rows, n, motors, m, &config, &width_workspace,
                           &width);
            assert(width.time.decoded == (fixture < 2u));
            assert(width.distance.decoded == (fixture < 2u));
            barcode_result_t selected;
            barcode_peak_select(&width, &report, &selected);
            assert(selected.decoded && strcmp(selected.text, "F") == 0);
            /* Whole quiet prefix/suffix, excluding 120 ms around the
             * outer centres (~12 mm at nominal speed). No short-input
             * shortcut: both pieces must learn a valid white baseline. */
            size_t leading = 0u, trailing = 0u;
            assert(first_frame > 120000u);
            while (leading < n &&
                   rows[leading].us - rows[0].us < first_frame - 120000u) {
                ++leading;
            }
            while (trailing < n &&
                   rows[trailing].us - rows[0].us < last_frame + 120000u) {
                ++trailing;
            }
            for (unsigned int crop = 0u; crop < 2u; ++crop) {
                const size_t start = crop == 0u ? 0u : trailing;
                const size_t quiet = crop == 0u ? leading : n - trailing;
                assert(quiet >= 200u);
                barcode_decode(rows + start, quiet, motors, m, &config,
                               &width_workspace, &width);
                assert(!width.time.decoded && !width.distance.decoded);
                barcode_peak_begin(
                    rows + start, quiet, NULL, rows[start].us, motors, m,
                    &(barcode_peak_config_t){config, 100u, 151u, 170u},
                    &workspace, &report);
                while (!barcode_peak_step(&workspace)) {
                }
                assert(!report.time.value.decoded &&
                       !report.distance.value.decoded);
                assert(report.time.baseline > 140.0f &&
                       report.distance.baseline > 140.0f);
                assert(report.time.peaks == 0u && report.distance.peaks == 0u);
            }
            if (reversed == 0u) {
                const uint32_t origin = rows[0].us, end = rows[n - 1u].us;
                for (size_t i = 0u; i < n / 2u; ++i) {
                    const barcode_sample_t temp = rows[i];
                    rows[i] = rows[n - 1u - i];
                    rows[n - 1u - i] = temp;
                }
                for (size_t i = 0u; i < n; ++i) {
                    rows[i].us = origin + end - rows[i].us;
                }
                for (size_t i = 0u; i < m / 2u; ++i) {
                    const barcode_motion_t temp = motors[i];
                    motors[i] = motors[m - 1u - i];
                    motors[m - 1u - i] = temp;
                }
                for (size_t i = 0u; i < m; ++i) {
                    motors[i].us = origin + end - motors[i].us;
                }
            }
        }
    }
    return 0;
}

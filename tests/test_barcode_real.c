/* Public-safe ADC/encoder capture supplied by Main, 2026-11-09.
 * Keep raw source data unchanged; BC_RUN rows are old decoder output and
 * deliberately not used as the expected element classification. */
#include "../app_program/subsystem3/barcode.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static barcode_sample_t rows[BC_MAX_SAMPLES];
static barcode_motion_t motors[BC_MAX_MOTION];
static barcode_workspace_t workspace;
static barcode_report_t report;

static void frame(const barcode_frame_t *value, void *context)
{
    (void)context;
    assert(strcmp(value->value.text, "F") == 0);
    assert(value->start_um != INT32_MIN && value->end_um != INT32_MIN);
    printf("real frame %s start_um %ld end_um %ld\n",
           value->distance_method ? "DISTANCE" : "TIME", (long)value->start_um,
           (long)value->end_um);
}

int main(void)
{
    FILE *file = fopen(BC_FIXTURE, "r");
    assert(file != NULL);
    char line[160];
    size_t n = 0u, m = 0u;
    while (fgets(line, sizeof(line), file) != NULL) {
        unsigned long index, us, raw, flags, delta;
        long left, right;
        if (sscanf(line, "BC_ADC,%lu,%lu,%lu,%lu,%lu", &index, &us, &raw,
                   &flags, &delta) == 5) {
            assert(index == n && n < BC_MAX_SAMPLES && raw <= 4095u);
            assert(us <= UINT32_MAX && flags == 0u);
            assert(n == 0u ? delta == 0u
                           : (uint32_t)us - rows[n - 1u].us == delta);
            rows[n++] = (barcode_sample_t){(uint32_t)us, (uint16_t)raw, 0u};
        } else if (sscanf(line, "BC_MOTOR,%lu,%lu,%ld,%ld", &index, &us, &left,
                          &right) == 4) {
            assert(index == m && m < BC_MAX_MOTION && us <= UINT32_MAX);
            assert(left >= INT32_MIN && left <= INT32_MAX);
            assert(right >= INT32_MIN && right <= INT32_MAX);
            motors[m++] =
                (barcode_motion_t){(uint32_t)us, (int32_t)left, (int32_t)right};
        }
    }
    assert(fclose(file) == 0 && n > 3000u && m > 150u);
    barcode_config_t config = barcode_default_config;
    config.motion_gap_us = 50000u; /* Source image retained about 30 ms. */
    for (unsigned int reverse = 0u; reverse < 2u; ++reverse) {
        barcode_begin(rows, n, motors, m, &config, &workspace, &report);
        barcode_frame_sink(&workspace, frame, NULL);
        while (!barcode_step(&workspace)) {
            /* Frozen host replay; no task scheduling is involved. */
        }
        printf("real F reverse %u: %s/%s runs %u text %s/%s\n", reverse,
               barcode_reason_name(report.time.reason),
               barcode_reason_name(report.distance.reason), report.runs,
               report.time.text, report.distance.text);
        assert(report.time.decoded && report.distance.decoded);
        assert(strcmp(report.time.text, "F") == 0);
        assert(strcmp(report.distance.text, "F") == 0);
        assert(report.time.command == BC_CMD_NONE);
        assert(report.distance.command == BC_CMD_NONE);
        assert(report.time.directions == (reverse == 0u ? 1u : 2u));
        assert(report.distance.directions == report.time.directions);
        assert(report.time.quality == BC_QUALITY_DECODED);
        assert(report.distance.quality == BC_QUALITY_DECODED);
        if (reverse == 0u) {
            /* Last source observation below the new 430 mm target. The
             * captured frame/quiet zones must already be complete there. */
            size_t stop = 1u;
            for (; stop < m; ++stop) {
                const int64_t sum =
                    (int64_t)motors[stop].left - motors[0].left +
                    (int64_t)motors[stop].right - motors[0].right;
                if (sum * 500 >= (int64_t)430 * config.counts_per_m) {
                    break;
                }
            }
            assert(stop < m);
            size_t prefix = 0u;
            while (prefix < n && rows[prefix].us <= motors[stop - 1u].us) {
                ++prefix;
            }
            barcode_decode(rows, prefix, motors, m, &config, &workspace,
                           &report);
            assert(report.time.decoded && report.distance.decoded);
            assert(strcmp(report.time.text, "F") == 0);
            assert(strcmp(report.distance.text, "F") == 0);
            printf("real F <=430 mm prefix: %lu ADC, both methods OK\n",
                   (unsigned long)prefix);
            const uint32_t origin = rows[0].us, end = rows[n - 1u].us;
            for (size_t i = 0u; i < n / 2u; ++i) {
                const barcode_sample_t temp = rows[i];
                rows[i] = rows[n - 1u - i];
                rows[n - 1u - i] = temp;
            }
            for (size_t i = 0u; i < n; ++i) {
                rows[i].us = origin + (end - rows[i].us);
            }
            for (size_t i = 0u; i < m / 2u; ++i) {
                const barcode_motion_t temp = motors[i];
                motors[i] = motors[m - 1u - i];
                motors[m - 1u - i] = temp;
            }
            for (size_t i = 0u; i < m; ++i) {
                motors[i].us = origin + (end - motors[i].us);
            }
        }
    }
    return 0;
}

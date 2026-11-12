/* Original approved fixtures, ADC/motor rows only. No BC_RUN classification. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../app_program/subsystem3/barcode.h"
#include "../app_program/subsystem3/barcode_window.h"

static barcode_sample_t adc[BC_MAX_SAMPLES];
static barcode_motion_t motor[BC_MAX_MOTION];
static barcode_window_t state;

int main(void)
{
    const char *names[] = {"barcode_large_f_forward.csv",
                           "barcode_430_large_fwd.csv",
                           "barcode_430_large_back_cable.csv"};
    for (unsigned int fixture = 0u; fixture < 3u; ++fixture) {
        char path[512], line[180];
        const int length = snprintf(path, sizeof(path), "%s/%s",
                                    BC_FIXTURES, names[fixture]);
        assert(length > 0 && (unsigned int)length < sizeof(path));
        FILE *file = fopen(path, "r");
        assert(file);
        unsigned int n = 0u, m = 0u;
        while (fgets(line, sizeof(line), file)) {
            unsigned long index, us, raw, flags, delta;
            long left, right;
            if (sscanf(line, "BC_ADC,%lu,%lu,%lu,%lu,%lu", &index, &us,
                       &raw, &flags, &delta) == 5) {
                assert(index == n && n < BC_MAX_SAMPLES && raw < 4095u);
                assert(us <= UINT32_MAX && flags == 0u);
                adc[n++] = (barcode_sample_t){(uint32_t)us, (uint16_t)raw, 0u};
            } else if (sscanf(line, "BC_MOTOR,%lu,%lu,%ld,%ld", &index, &us,
                              &left, &right) == 4) {
                assert(index == m && m < BC_MAX_MOTION && us <= UINT32_MAX);
                assert(left >= INT32_MIN && left <= INT32_MAX);
                assert(right >= INT32_MIN && right <= INT32_MAX);
                motor[m++] = (barcode_motion_t){
                    (uint32_t)us, (int32_t)left, (int32_t)right};
            }
        }
        assert(fclose(file) == 0 && n > 2000u && m > 150u);
        for (unsigned int compressed = 0u; compressed < 2u; ++compressed) {
            const uint32_t origin =
                adc[0].us < motor[0].us ? adc[0].us : motor[0].us;
            barcode_window_config_t config = barcode_window_default_config;
            /* Historical source retained motor points at20..50ms, unlike
             * the future10ms live feed. This is HOST REPLAY ONLY. */
            config.motor_gap_us = 50000u;
            config.sample_gap_us = 3000u; /* Original retained2ms ADC. */
            const barcode_cal_report_t cal = {
                .mode = BC_CAL_LEARNED, .white_mean = 167u,
                .white_samples = 1904u, .rise = 317u, .fall = 242u};
            assert(barcode_window_begin(&state, &cal, &config));
            unsigned int a = 0u, b = 0u;
            while (a < n || b < m) {
                const bool motion = b < m &&
                    (a == n || motor[b].us <= adc[a].us);
                const uint32_t us = motion ? motor[b].us : adc[a].us;
                /* Synthetic time compression exercises faster delivery;
                 * it does NOT reproduce optical response at146mm/s. */
                const uint32_t relative = us - origin;
                const uint32_t stamp = compressed ?
                    (uint32_t)((uint64_t)relative * 100u / 146u) : relative;
                if (motion) {
                    barcode_window_motor(&state, stamp,
                        (uint32_t)motor[b].left, (uint32_t)motor[b].right, 0u);
                    ++b;
                } else {
                    barcode_window_adc(&state, stamp, adc[a].raw, 0u);
                    ++a;
                }
                (void)barcode_window_step(&state, BW_STEP_MAX);
            }
            printf("%s compressed%u accepted%d reason%lu elements%u "
                   "n%.3f w%.3f leading%.3f\n", names[fixture], compressed,
                   state.result.accepted, (unsigned long)state.result.reason,
                   state.elements, (double)state.result.narrow_mm,
                   (double)state.result.wide_mm,
                   (double)state.result.leading_mm);
            assert(state.result.accepted == (fixture < 2u));
            if (state.result.accepted) {
                assert(state.result.letter == 'F');
                assert(state.result.mapped_action == 0u);
            }
        }
    }
    return 0;
}

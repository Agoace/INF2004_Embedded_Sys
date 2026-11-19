/* Measured retained rows; ASSUMED linear ADC/position interpolation only.
 * These 9-run vectors prove star-stage behaviour, NOT a full physical F decode.
 * Motor brackets are injected to isolate the quiet/model code from geometry. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../app_program/subsystem3/barcode_window.h"
#ifndef BC_FIXTURES
#error BC_FIXTURES required
#endif
#ifndef QUIET_READER_OLD
#define QUIET_READER_OLD 0
#endif
static barcode_window_t state;
static void begin(unsigned int rise, unsigned int fall)
{
    const barcode_cal_report_t cal = {.mode = BC_CAL_FALLBACK,
        .white_mean = (uint16_t)(rise - 150u), .white_samples = 32u,
        .rise = (uint16_t)rise, .fall = (uint16_t)fall,
        .faults = BC_CAL_NO_BLACK};
    assert(barcode_window_begin(&state, &cal, NULL));
}
static void feed(uint32_t us, unsigned int raw, float mm)
{
    state.motor_count = 2u;
    state.motors[0] = (barcode_window_motor_t){us - 1u, 0u, 0u, mm - 0.001f};
    state.motors[1] = (barcode_window_motor_t){us + 1u, 1u, 1u, mm + 0.001f};
    barcode_window_adc(&state, us, (uint16_t)raw, 0u);
    assert(barcode_window_step(&state, 1u) == 1u);
}
static FILE *fixture(const char *name)
{
    char path[512];
    const int n = snprintf(path, sizeof(path), "%s/%s", BC_FIXTURES, name);
    assert(n > 0 && (unsigned int)n < sizeof(path));
    FILE *f = fopen(path, "r");
    assert(f);
    return f;
}
static void vectors(void)
{
    FILE *f = fixture("barcode_fc_024d01a_vectors.csv");
    char line[400];
    assert(fgets(line, sizeof(line), f));
    unsigned int count = 0u;
    while (fgets(line, sizeof(line), f)) {
        uint32_t v[17];
        char *s = strtok(line, ",\r\n");
        for (unsigned int i = 0u; i < 17u; ++i) {
            assert(s);
            v[i] = (uint32_t)strtoul(s, NULL, 10);
            s = strtok(NULL, ",\r\n");
        }
        assert(!s && v[7] == BW_START);
        for (unsigned int mode = 0u; mode < 4u; ++mode) {
            begin(v[4], v[5]);
            state.phase = 2u; /* Seed the captured, already bound WINDOW. */
            state.black = true;
            state.rise = (uint16_t)v[4];
            state.fall = (uint16_t)v[5];
            state.leading_mm = mode == 1u ? 29.99f : mode == 2u ? 30.0f :
                               (float)v[6] / 100.0f;
            state.start_us = mode == 3u ? UINT32_MAX - 5000u : v[2];
            float mm = 0.0f;
            for (unsigned int i = 0u; i < 9u; ++i) {
                mm += (float)v[8u + i] / 100.0f;
                feed(state.start_us + (i + 1u) * 1000u,
                     (i & 1u) == 0u ? 170u : 650u, mm);
            }
            const bool star = v[1] != 1u;
            const bool accept_stage = star && mode != 1u && !QUIET_READER_OLD;
            assert(!state.result.accepted); /* No invented remaining 20 runs. */
            if (accept_stage) {
                assert(state.phase == 2u && state.elements == 9u);
                assert(state.result.reason == BW_WAIT && state.direction == 1u);
                const float expected = v[1] == 2u ? 5.895f : 5.738333f;
                assert(fabsf(state.narrow - expected) < 0.02f);
            } else {
                assert(state.result.reason == BW_START && state.phase == 1u);
            }
        }
        ++count;
    }
    assert(fclose(f) == 0 && count == 3u);
}
typedef struct {uint32_t us; unsigned int raw, pre;} adc_t;
static void pretrace(float speed)
{
    FILE *f = fixture("barcode_fc2_024d01a_adc.csv");
    char line[220];
    adc_t rows[204];
    unsigned int n = 0u, pre = 0u;
    uint32_t last_white = 0u;
    assert(fgets(line, sizeof(line), f));
    while (fgets(line, sizeof(line), f)) {
        unsigned long epoch, index, row, us, raw, flags, prefix;
        assert(sscanf(line, "%lu,%lu,%lu,%lu,%lu,%lu,%lu", &epoch, &index,
            &row, &us, &raw, &flags, &prefix) == 7);
        assert(n < 204u && epoch == 3u && index == 5u && row == n && flags == 0u);
        rows[n++] = (adc_t){(uint32_t)us, (unsigned int)raw, (unsigned int)prefix};
        if (prefix) {
            ++pre;
            if (raw >= 120u && raw <= 220u) last_white = (uint32_t)us;
        }
    }
    assert(fclose(f) == 0 && n == 204u && pre == 64u);
    begin(324u, 249u);
    /* Header-anchor reconstruction, NOT a measured motor trace. At high speed
     * the anchor lies inside the prefix: start there, never seed future white. */
    const float anchor = speed * (float)(last_white - rows[0].us) / 1000000.0f - 43.07f;
    bool seeded = false, candidate = false;
    for (unsigned int i = 0u; i < n; ++i) {
        const uint32_t span = i + 1u < n ? rows[i + 1u].us - rows[i].us : 1u;
        for (uint32_t dt = 0u; dt < span; dt += 1000u) {
            const uint32_t us = rows[i].us + dt;
            const float mm = speed * (float)(us - rows[0].us) / 1000000.0f;
            if (mm < anchor) continue;
            if (!seeded) {
                state.white_started = true;
                state.white_start = anchor;
                state.white_count = 16u;
                state.white_mean = 174.0f;
                seeded = true;
            }
            const float raw = (float)rows[i].raw + (i + 1u < n ?
                ((float)rows[i + 1u].raw - (float)rows[i].raw) *
                (float)dt / (float)span : 0.0f);
            feed(us, (unsigned int)lroundf(raw), mm);
            if (barcode_window_candidate(&state, NULL)) candidate = true;
        }
    }
    assert(seeded && candidate == (QUIET_READER_OLD != 0));
    printf("FC2 204 ADC/64 prefix, ASSUMED %.0f mm/s: candidate %u\n",
           (double)speed, (unsigned int)candidate);
}
int main(void)
{
    vectors();
    pretrace(100.0f);
    pretrace(146.0f);
    pretrace(220.0f);
    puts("three measured vectors + clipped/nonstar/wrap + FC2 pretrace OK");
    return 0;
}

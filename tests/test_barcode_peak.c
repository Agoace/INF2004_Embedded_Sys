/* Deterministic optical/noise model, not evidence of physical small print. */
#include "../app_program/subsystem3/barcode_peak.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    float x, width, amplitude;
} bar_t;

static bar_t bars[120];
static size_t bar_count, count, motor_count;
static barcode_sample_t rows[BC_MAX_SAMPLES];
static barcode_packed_t packed[BC_MAX_SAMPLES];
static barcode_motion_t motor[BC_MAX_MOTION];
static barcode_peak_workspace_t workspace;
static barcode_peak_report_t report;
static barcode_peak_config_t config;
static const uint32_t origin = UINT32_MAX - 100000u;
static float step_mm;

static int32_t signed_bits(uint32_t value)
{
    return value <= INT32_MAX ? (int32_t)value
                              : -(int32_t)(UINT32_MAX - value) - 1;
}

static void motors(void)
{
    motor_count = 0u;
    const uint32_t period = config.speed_mm_s == 25u ? 40000u : 20000u;
    const uint32_t duration = (uint32_t)(count - 1u) * 2000u;
    for (uint32_t t = 0u; t <= duration + period && motor_count < BC_MAX_MOTION;
         t += period) {
        const int32_t relative = (int32_t)t - 2000;
        const int32_t counts = (int32_t)lroundf(
            (float)relative * config.speed_mm_s * 12414.0f / 1.0e9f);
        const int32_t value =
            signed_bits(UINT32_C(0x7ffffff0) + (uint32_t)counts);
        motor[motor_count++] =
            (barcode_motion_t){origin + t - 2000u, value, value};
    }
}

static void render(float end, unsigned int seed, bool flat)
{
    step_mm = (float)config.speed_mm_s * 0.002f;
    count = (size_t)(end / step_mm) + 1u;
    assert(count <= BC_MAX_SAMPLES);
    uint32_t noise = seed + 1u;
    for (size_t i = 0u; i < count; ++i) {
        const float x = (float)i * step_mm;
        noise = noise * 1664525u + 1013904223u;
        float value = 164.0f + (float)(noise % 7u) - 3.0f;
        for (size_t j = 0u; j < bar_count; ++j) {
            const float u = fabsf(x - bars[j].x) / (bars[j].width * 0.45f);
            if (u < 1.0f) {
                float shape = 1.0f - u * u;
                if (flat && bars[j].width > 5.0f && u < 0.6f) {
                    shape = 0.9f - 0.03f * cosf(u * 20.0f);
                }
                value += shape * bars[j].amplitude;
            }
        }
        rows[i] = (barcode_sample_t){origin + (uint32_t)i * 2000u,
                                     (uint16_t)lroundf(value), 0u};
    }
    motors();
}

static void make(const char *payload, float narrow, unsigned int seed,
                 bool flat, uint16_t speed, float quiet)
{
    config = (barcode_peak_config_t){
        {64u, 3500u, 12414u, 50000u}, speed, 151u, 180u};
    bar_count = 0u;
    float x = quiet;
    const size_t length = strlen(payload);
    for (size_t j = 0u; j < length + 2u; ++j) {
        const char c = j == 0u || j == length + 1u ? '*' : payload[j - 1u];
        const uint16_t pattern = barcode_pattern(c);
        assert(pattern != 0u && barcode_symbol(pattern) == c);
        for (size_t i = 0u; i < 9u; ++i) {
            const bool wide = (pattern & (UINT16_C(0x0100) >> i)) != 0u;
            const float width = narrow * (wide ? 3.0f : 1.0f);
            if ((i & 1u) == 0u) {
                assert(bar_count < 120u);
                const float amplitude =
                    wide ? 1050.0f + (float)(bar_count % 4u) * 50.0f
                         : (narrow > 3.0f ? 600.0f : 48.0f) +
                               (float)(bar_count % 3u) * 4.0f;
                bars[bar_count++] = (bar_t){x + width * 0.5f, width, amplitude};
            }
            x += width;
        }
        if (j != length + 1u) {
            x += narrow;
        }
    }
    render(x + quiet, seed, flat);
}

static void run(bool missing_motion)
{
    barcode_peak_begin(rows, count, NULL, origin, missing_motion ? NULL : motor,
                       motor_count, &config, &workspace, &report);
    unsigned int steps = 0u;
    while (!barcode_peak_step(&workspace)) {
        assert(++steps < 20000u);
    }
}

static void check(const char *text)
{
    run(false);
    if (!report.time.value.decoded || !report.distance.value.decoded) {
        printf("failure %s: %s/%s peaks %u/%u\n", text,
               barcode_reason_name(report.time.value.reason),
               barcode_reason_name(report.distance.value.reason),
               report.time.peaks, report.distance.peaks);
        fflush(stdout);
    }
    assert(report.time.value.decoded && report.distance.value.decoded);
    assert(strcmp(report.time.value.text, text) == 0);
    assert(strcmp(report.distance.value.text, text) == 0);
    assert(report.distance.minimum_prominence >= report.distance.threshold);
    assert(report.distance.amplitude_margin >= report.distance.noise_sd * 6.0f);
    assert(report.distance.geometry_error <= 0.35f);
    float mean = 0.0f, variance = 0.0f;
    for (size_t i = 5u; i <= 25u; ++i) {
        mean += (float)workspace.bins[i] / 21.0f;
    }
    for (size_t i = 5u; i <= 25u; ++i) {
        const float d = (float)workspace.bins[i] - mean;
        variance += d * d;
    }
    const float expected = fmaxf(sqrtf(variance / 20.0f),
                                 (float)workspace.noise_samples[10] * 1.4826f);
    assert(fabsf(expected - report.distance.noise_sd) < 0.001f);
}

static void reject(void)
{
    run(false);
    assert(!report.time.value.decoded && !report.distance.value.decoded);
}

static void reverse(void)
{
    const uint32_t end = rows[count - 1u].us;
    for (size_t i = 0u; i < count / 2u; ++i) {
        const barcode_sample_t temp = rows[i];
        rows[i] = rows[count - 1u - i];
        rows[count - 1u - i] = temp;
    }
    for (size_t i = 0u; i < count; ++i) {
        rows[i].us = origin + end - rows[i].us;
    }
    for (size_t i = 0u; i < motor_count / 2u; ++i) {
        const barcode_motion_t temp = motor[i];
        motor[i] = motor[motor_count - 1u - i];
        motor[motor_count - 1u - i] = temp;
    }
    for (size_t i = 0u; i < motor_count; ++i) {
        motor[i].us = origin + end - motor[i].us;
    }
}

int main(void)
{
    assert(barcode_pattern('?') == 0u && barcode_symbol(0u) == '\0');
    const char *alphabet = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";
    for (size_t i = 0u; i < 43u; ++i) {
        const char text[] = {alphabet[i], '\0'};
        for (unsigned int size = 0u; size < 2u; ++size) {
            make(text, size == 0u ? 2.9f : 5.8f, (unsigned int)i, size != 0u,
                 100u, 70.0f);
            check(text);
            assert(report.time.value.directions == 1u);
            reverse();
            check(text);
            assert(report.time.value.directions == 2u);
        }
    }
    for (unsigned int seed = 0u; seed < 20u; ++seed) {
        make("F", 2.9f, seed, false, 100u, 70.0f);
        check("F");
    }
    make("F", 2.9f, 1u, false, 25u, 70.0f);
    check("F");
    make("F", 2.9f, 1u, false, 200u, 70.0f);
    check("F");
    make("0123456789ABCDEF", 2.9f, 1u, false, 100u, 70.0f);
    check("0123456789ABCDEF");
    assert(report.time.peaks == 90u);
    make("0123456789ABCDEFG", 2.9f, 1u, false, 100u, 70.0f);
    reject();
    make("0123456789ABCDEFGHIJ", 2.9f, 1u, false, 100u, 70.0f);
    reject();
    assert(report.time.value.reason == BC_RUN_LIMIT);

    /* Single 18 mm line, flat/split bright top, paper/floor drift/impulse. */
    config =
        (barcode_peak_config_t){{64u, 3500u, 12414u, 50000u}, 100u, 151u, 180u};
    bar_count = 1u;
    bars[0] = (bar_t){200.0f, 18.0f, 1400.0f};
    render(430.0f, 1u, true);
    reject();
    assert(report.time.peaks == 1u);
    bar_count = 0u;
    render(430.0f, 3u, false);
    reject();
    for (size_t i = 0u; i < count; ++i) {
        rows[i].raw += (uint16_t)(i / 150u);
    }
    reject();
    rows[500].raw = 1500u;
    reject();
    for (size_t i = 0u; i < count; ++i) {
        rows[i].raw = 1500u;
    }
    reject();
    assert(report.time.value.reason == BC_CONTRAST);
    for (size_t i = 0u; i < count; ++i) {
        rows[i].raw = (uint16_t)(1170u + i % 120u);
    }
    reject();
    assert(report.time.value.reason == BC_CONTRAST);

    make("F", 2.9f, 1u, false, 100u, 70.0f);
    bars[5].amplitude = 0.0f;
    render(280.0f, 1u, false);
    reject();
    make("F", 2.9f, 1u, false, 100u, 70.0f);
    bars[5].amplitude = 12.0f;
    render(280.0f, 1u, false);
    reject();
    make("F", 2.9f, 1u, false, 100u, 70.0f);
    bars[bar_count++] = (bar_t){bars[5].x - 3.0f, 1.5f, 70.0f};
    render(280.0f, 1u, false);
    reject();
    make("F", 2.9f, 1u, false, 100u, 26.1f);
    reject(); /* Only 9n quiet. */
    make("F", 2.9f, 1u, false, 100u, 70.0f);
    bars[0].amplitude = 1100.0f;
    render(280.0f, 1u, false);
    reject();
    make("F", 2.9f, 1u, false, 100u, 70.0f);
    bars[7].x += 6.0f;
    render(280.0f, 1u, false);
    check("6"); /* About 2n movement creates another legal symbol, not F. */
    make("F", 2.9f, 1u, false, 100u, 70.0f);
    bars[7].x += 4.0f;
    render(280.0f, 1u, false);
    reject();
    make("F", 2.9f, 1u, false, 100u, 70.0f);
    for (size_t i = 0u; i < count; ++i) {
        const float x = (float)i * step_mm;
        if (x > bars[5].x && x < bars[6].x) {
            rows[i].raw = 230u;
        }
    }
    reject();
    make("F", 2.9f, 1u, false, 100u, 70.0f);
    rows[500].raw = 4096u;
    reject();
    make("F", 2.9f, 1u, false, 100u, 70.0f);
    rows[500].flags = BC_SAMPLE_GAP;
    reject();
    assert(report.time.value.reason == BC_SAMPLE_LOSS);
    make("F", 2.9f, 1u, false, 100u, 70.0f);
    for (size_t i = 0u; i < count; ++i) {
        packed[i] = (barcode_packed_t){rows[i].raw, i == 0u ? 0u : 2000u};
    }
    for (unsigned int fault = 0u; fault < 2u; ++fault) {
        packed[500].raw_flags =
            rows[500].raw | (fault == 0u ? BC_PACKED_GAP : UINT16_C(0x1000));
        barcode_peak_begin(NULL, count, packed, origin, motor, motor_count,
                           &config, &workspace, &report);
        while (!barcode_peak_step(&workspace)) {
        }
        assert(report.time.value.reason == BC_SAMPLE_LOSS);
        assert(report.distance.value.reason == BC_SAMPLE_LOSS);
    }
    make("F", 2.9f, 1u, false, 100u, 70.0f);
    rows[500].us += 10000u;
    reject();
    make("F", 2.9f, 1u, false, 100u, 70.0f);
    run(true);
    assert(report.time.value.decoded &&
           report.distance.value.reason == BC_MOTION);
    motor[5].us += 60000u;
    run(false);
    assert(report.time.value.decoded &&
           report.distance.value.reason == BC_MOTION);
    make("F", 2.9f, 1u, false, 100u, 70.0f);
    motor[5].left = motor[0].left;
    motor[5].right = motor[0].right;
    run(false);
    assert(report.time.value.decoded &&
           report.distance.value.reason == BC_MOTION);
    make("F", 2.9f, 1u, false, 100u, 70.0f);
    config.capture.counts_per_m = 1u;
    run(false);
    assert(report.time.value.decoded && !report.distance.value.decoded);

    /* Bin and peak capacities fail closed without a buffer overwrite. */
    config.speed_mm_s = 200u;
    bar_count = 0u;
    render(2400.0f, 1u, false);
    reject();
    assert(report.time.value.reason == BC_RUN_LIMIT);

    /* Separate complete captures of different payloads must conflict. */
    static bar_t first[15];
    make("A", 2.9f, 1u, false, 100u, 70.0f);
    memcpy(first, bars, sizeof(first));
    make("B", 2.9f, 1u, false, 100u, 70.0f);
    for (size_t i = 0u; i < 15u; ++i) {
        bars[i + 15u] = bars[i];
        bars[i + 15u].x += 300.0f;
        bars[i] = first[i];
    }
    bar_count = 30u;
    render(580.0f, 1u, false);
    reject();
    assert(report.time.value.reason == BC_AMBIGUOUS);
    assert(report.distance.value.reason == BC_AMBIGUOUS);
    assert(report.time.value.frames == 2u &&
           report.distance.value.frames == 2u);

    /* Gentle baseline drift is not a missing bar. A speed discontinuity
     * may reject TIME, but must leave the encoder-axis result independent. */
    make("F", 2.9f, 1u, false, 100u, 70.0f);
    for (size_t i = 0u; i < count; ++i) {
        rows[i].raw += (uint16_t)(i / 500u);
    }
    check("F");
    for (size_t i = 0u; i < count; ++i) {
        const uint32_t t = rows[i].us - origin;
        const uint32_t duration = t > 1500000u   ? 500000u
                                  : t > 1000000u ? t - 1000000u
                                                 : 0u;
        rows[i].us += duration * 7u / 10u;
    }
    for (size_t i = 0u; i < motor_count; ++i) {
        const int32_t t = signed_bits(motor[i].us - origin);
        const uint32_t duration = t > 1500000   ? 500000u
                                  : t > 1000000 ? (uint32_t)(t - 1000000)
                                                : 0u;
        motor[i].us += duration * 7u / 10u;
    }
    run(false);
    assert(report.distance.value.decoded);
    assert(strcmp(report.distance.value.text, "F") == 0);

    /* Conflicts cannot be hidden by higher apparent signal strength. */
    barcode_report_t width = {0};
    barcode_result_t selected;
    make("F", 2.9f, 1u, false, 100u, 70.0f);
    check("F");
    barcode_peak_select(&width, &report, &selected);
    assert(selected.decoded);
    width.time = report.time.value;
    strcpy(width.time.text, "A");
    barcode_peak_select(&width, &report, &selected);
    assert(!selected.decoded && selected.reason == BC_AMBIGUOUS);
    width.time = (barcode_result_t){.reason = BC_AMBIGUOUS};
    barcode_peak_select(&width, &report, &selected);
    assert(!selected.decoded && selected.reason == BC_AMBIGUOUS);
    width.time = (barcode_result_t){0};
    report.distance.value = report.time.value;
    strcpy(report.distance.value.text, "B");
    barcode_peak_select(&width, &report, &selected);
    assert(!selected.decoded && selected.reason == BC_AMBIGUOUS);
    puts("PEAK: 43 symbols x two sizes/directions; noise, line, guards OK");
    return 0;
}

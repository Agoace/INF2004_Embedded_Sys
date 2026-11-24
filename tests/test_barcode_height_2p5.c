/* GP28 at 2.5 mm: small *F* (2.9/8.7 mm) and large *F* (5.8/17.5 mm).
 * Robot H3 capture (partial, rows 103..107 and 163..179 lost in USB, not on
 * the robot) and two ASSUMED synthetic traces (SW7 synth_bc.py, Gaussian
 * spot). Motor brackets are injected at a constant speed, so the window
 * geometry and motion gates are not under test here. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../app_program/subsystem3/barcode_window.h"
#ifndef BC_FIXTURES
#error BC_FIXTURES required
#endif

static barcode_window_t state;

static void feed(uint32_t us, unsigned int raw, float mm)
{
    state.motor_count = 2u;
    state.motors[0] = (barcode_window_motor_t){us - 1u, 0u, 0u, mm - 0.001f};
    state.motors[1] = (barcode_window_motor_t){us + 1u, 1u, 1u, mm + 0.001f};
    barcode_window_adc(&state, us, (uint16_t)raw, 0u);
    assert(barcode_window_step(&state, 1u) == 1u);
}

typedef struct {
    uint32_t us;
    unsigned int raw, row;
    float mm;
} row_t;
static row_t rows[2000];

static unsigned int load(const char *name, bool synthetic)
{
    char path[512], line[200];
    const int n = snprintf(path, sizeof(path), "%s/%s", BC_FIXTURES, name);
    assert(n > 0 && (unsigned int)n < sizeof(path));
    FILE *f = fopen(path, "r");
    assert(f);
    unsigned int count = 0u;
    while (fgets(line, sizeof(line), f)) {
        unsigned int epoch, index, row, raw, flags, pre, ideal;
        unsigned long us;
        float mm = 0.0f;
        const int got = synthetic ?
            sscanf(line, "%u,%u,%u,%lu,%u,%u,%u,%f,%u", &epoch, &index, &row,
                   &us, &raw, &flags, &pre, &mm, &ideal) :
            sscanf(line, "%u,%u,%u,%lu,%u,%u,%u", &epoch, &index, &row, &us,
                   &raw, &flags, &pre);
        if (got != (synthetic ? 9 : 7)) {
            continue;
        }
        assert(count < sizeof(rows) / sizeof(rows[0]) && flags == 0u);
        rows[count++] = (row_t){(uint32_t)us, raw, row, mm};
    }
    assert(fclose(f) == 0 && count > 100u);
    return count;
}

/* Calibration as the live spin would give it: white from the first rows. */
static void begin(unsigned int white)
{
    const unsigned int rise = white + 150u < 350u ? white + 150u : 350u;
    const unsigned int fall = white + 75u < rise - 75u ? white + 75u : rise - 75u;
    const barcode_cal_report_t cal = {.mode = BC_CAL_FALLBACK,
        .white_mean = (uint16_t)white, .white_samples = 32u,
        .rise = (uint16_t)rise, .fall = (uint16_t)fall,
        .faults = BC_CAL_NO_BLACK};
    /* ASSUMED host retained-data profile: the captures keep every 4th
     * 1 ms sample (div_us 4000; measured row gaps up to 5025 us), so allow 6 ms. */
    barcode_window_config_t config = barcode_window_default_config;
    config.sample_gap_us = 6000u;
    assert(barcode_window_begin(&state, &cal, &config));
}

static unsigned int white_of(unsigned int n)
{
    unsigned int sum = 0u, k = 0u;
    for (unsigned int i = 0u; i < n && k < 32u; ++i) {
        if (rows[i].raw >= 120u && rows[i].raw <= 220u) {
            sum += rows[i].raw;
            ++k;
        }
    }
    assert(k >= 16u);
    return (sum + k / 2u) / k;
}

/* Robot H3: replay up to the USB gap at row 103; the reader must split the
 * bars that the fixed levels merged (robot: 4 elements, BW_WIDTH). Speed
 * from the captured edges: 4.42 mm in 69 ms (rows 0..1) = 64 mm/s is a
 * local value; the encoder gives 4.73 mm / 68.999 ms. Use the encoder
 * counts of the edge rows: 12414 counts/m. */
static void h3(void)
{
    const unsigned int n = load("barcode_small_2p5mm_h3_adc.csv", false);
    const unsigned int white = white_of(n);
    begin(white);
    /* Map rows with the [BC_FAIL_EDGE] mm_cmm odometer (piecewise linear
     * between the captured edges), not an assumed speed. */
    static const double eus[] = {39364604.0, 39433603.0, 39452605.0,
                                 39677603.0, 39688603.0};
    static const double emm[] = {318.35, 322.77, 324.78, 356.72, 357.97};
    unsigned int max_elements = 0u, last_row = 0u;
    for (unsigned int i = 0u; i < n; ++i) {
        if (rows[i].row >= 103u) {
            break; /* USB gap: rows 103..107 lost; stop, do not bridge. */
        }
        unsigned int k = 0u;
        while (k < 3u && (double)rows[i].us > eus[k + 1u]) {
            ++k;
        }
        const float mm = (float)(emm[k] + ((double)rows[i].us - eus[k]) *
            (emm[k + 1u] - emm[k]) / (eus[k + 1u] - eus[k]));
        if (i > 0u && rows[i].row != rows[i - 1u].row + 1u) {
            break;
        }
        if (rows[i].row == 64u && state.white_started) {
            /* The 64 pre rows hold about 16 mm; the robot measured the
             * leading white: [BC_FAIL_CAPTURE] lead_cmm 31030. */
            state.white_start = state.white_end - 310.30f;
        }
        feed(rows[i].us, rows[i].raw, mm);
        last_row = rows[i].row;
        if (state.elements > max_elements) {
            max_elements = state.elements;
        }
        assert(state.result.reason == BW_WAIT); /* No early reject. */
    }
    printf("H3 white %u rows to %u elements %u runs", white, last_row,
           (unsigned int)state.elements);
    for (unsigned int i = 0u; i < state.elements; ++i) {
        printf(" %.2f", (double)state.runs[i]);
    }
    printf("\n");
    /* Rows 64..102 hold bar n, space n, then a wide bar (reverse *): the
     * USB gap at row 103 ends the data. Robot fixed levels: bar n merged.
     * Midpoint mode must split them and must not reject before the gap. */
    assert(state.mid && state.bars >= 2u && max_elements >= 2u);
    assert(state.runs[0] > 2.0f && state.runs[0] < 4.5f);
    assert(state.runs[1] > 2.0f && state.runs[1] < 4.5f);
}

/* Robot image 20 (large sheet, 2.5 mm, [BC_FAIL_ADC] epoch 7 index 3):
 * a faint mark peaking 1111 counts 5 mm before 2800+ bars was element 1
 * on the robot (BW_START). Rows 143..158 are lost in USB (one merged log
 * line, removed); replay to row 142. Odometer: the [BC_FAIL_EDGE] mm_cmm
 * rows, piecewise linear. Before the first edge, the first edge speed. */
static void img20(void)
{
    static double eus[16], emm[16];
    unsigned int edges = 0u;
    char path[512], line[200];
    (void)snprintf(path, sizeof(path), "%s/%s", BC_FIXTURES,
                   "barcode_large_2p5mm_img20_edges.csv");
    FILE *f = fopen(path, "r");
    assert(f);
    while (fgets(line, sizeof(line), f)) {
        unsigned long us, cmm;
        if (sscanf(line, "%lu,%lu", &us, &cmm) == 2) {
            assert(edges < 16u);
            eus[edges] = (double)us;
            emm[edges++] = (double)cmm / 100.0;
        }
    }
    assert(fclose(f) == 0 && edges >= 3u);
    const unsigned int n = load("barcode_large_2p5mm_img20_adc.csv", false);
    begin(white_of(n));
    for (unsigned int i = 0u; i < n && rows[i].row <= 142u; ++i) {
        unsigned int k = 0u;
        while (k + 2u < edges && (double)rows[i].us > eus[k + 1u]) {
            ++k;
        }
        const float mm = (float)(emm[k] + ((double)rows[i].us - eus[k]) *
            (emm[k + 1u] - emm[k]) / (eus[k + 1u] - eus[k]));
        if (rows[i].row == 64u && state.white_started) {
            /* Robot leading white: lead_cmm 16346. */
            state.white_start = state.white_end - 163.46f;
        }
        feed(rows[i].us, rows[i].raw, mm);
        assert(state.result.reason == BW_WAIT);
    }
    printf("img20 mid %d bars %u elements %u runs", (int)state.mid,
           (unsigned int)state.bars, (unsigned int)state.elements);
    for (unsigned int i = 0u; i < state.elements; ++i) {
        printf(" %.2f", (double)state.runs[i]);
    }
    printf("\n");
    /* Image 20 robot runs: 2.9 1.4 5.1 6.3 17.3 ... (mark first). The mark
     * must not be a bar of the window: it is dropped (shifts 1) and the
     * first run is a real narrow bar of the 5.8 mm sheet. */
    assert(state.mid && state.shifts == 1u && state.elements == 3u);
    /* Reverse star starts n n W: bar 4.33, space 7.08 (bar/space edge bias
     * -1.4 mm, fitted by the star), wide bar 16.48. */
    assert(state.runs[0] > 3.5f && state.runs[0] < 7.5f);
    assert(state.runs[1] > 3.5f && state.runs[1] < 8.5f);
    assert(state.runs[2] > 12.0f && state.runs[2] < 21.0f);
}

static void synthetic(const char *name, char letter, float narrow, float wide)
{
    const unsigned int n = load(name, true);
    begin(white_of(n));
    uint32_t serial = 0u;
    for (unsigned int i = 0u; i < n; ++i) {
        feed(rows[i].us, rows[i].raw, rows[i].mm);
        if (state.result.serial != serial) {
            serial = state.result.serial;
            printf("  result serial %lu reason %lu at mm %.2f elements %u\n",
                   (unsigned long)serial, (unsigned long)state.result.reason,
                   (double)rows[i].mm, (unsigned int)state.elements);
        }
        if (state.result.accepted) {
            break;
        }
    }
    printf("runs");
    for (unsigned int k = 0u; k < BW_ELEMENTS; ++k) printf(" %.2f", (double)state.runs[k]);
    printf("\n");
    printf("%s accepted %d reason %lu letter %c n %.3f w %.3f\n", name,
           state.result.accepted, (unsigned long)state.result.reason,
           state.result.accepted ? state.result.letter : '-',
           (double)state.result.narrow_mm, (double)state.result.wide_mm);
    if (letter != 'F') {
        /* *Z* trace: must not decode as F. */
        assert(!(state.result.accepted && state.result.letter == 'F'));
        return;
    }
    assert(state.result.accepted && state.result.letter == 'F');
    assert(state.result.direction == 1u);
    assert(fabsf(state.result.narrow_mm - narrow) < 0.6f);
    assert(fabsf(state.result.wide_mm - wide) < 1.5f);
}

int main(int argc, char **argv)
{
    const char *kind = argc > 1 ? argv[1] : "all";
    if (strcmp(kind, "h3") == 0 || strcmp(kind, "all") == 0) {
        h3();
    }
    if (strcmp(kind, "img20") == 0 || strcmp(kind, "all") == 0) {
        img20();
    }
    static const struct {
        const char *kind, *file;
        char letter;
        float narrow, wide;
    } cases[] = {
        {"small", "synth_small_F_2p5mm.csv", 'F', 2.9f, 8.7f},
        {"small69", "synth_small_F_2p5mm_69mms.csv", 'F', 2.9f, 8.7f},
        {"large", "synth_large_F_2p5mm.csv", 'F', 5.8f, 17.5f},
        {"large69", "synth_large_F_2p5mm_69mms.csv", 'F', 5.8f, 17.5f},
        {"small_z", "synth_small_Z_2p5mm.csv", 'Z', 2.9f, 8.7f},
        {"large_z", "synth_large_Z_2p5mm.csv", 'Z', 5.8f, 17.5f},
    };
    for (unsigned int i = 0u; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        if (strcmp(kind, cases[i].kind) == 0 || strcmp(kind, "all") == 0) {
            synthetic(cases[i].file, cases[i].letter, cases[i].narrow,
                      cases[i].wide);
        }
    }
    return 0;
}

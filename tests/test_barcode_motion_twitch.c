/* Differential-drive host check for small GP28 projection retreats.
 * Optics are ASSUMED rectangular; wheel speeds are scripted (SYNTHETIC),
 * except "kr", which replays FC1 KR measured wheel speeds (ASSUMED mm/s,
 * each held for 20 ms) inside the first star. Geometry comes from the
 * reader defaults, so the test follows the configured GP28 offsets. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../app_program/subsystem3/barcode.h"
#include "../app_program/subsystem3/barcode_window.h"

static barcode_window_t state;
static float elements[32];
static unsigned int count;
static const float lead = 80.0f;
static double x, y, heading, wheel_l, wheel_r;
static float kr[8][2];
static unsigned int kr_rows;

static void make(void)
{
    const char *text = "*F*";
    count = 0u;
    for (unsigned int ch = 0u; text[ch] != '\0'; ++ch) {
        if (ch != 0u) elements[count++] = 5.8f;
        const uint16_t mask = barcode_pattern(text[ch]);
        for (unsigned int i = 0u; i < 9u; ++i) {
            elements[count++] = (mask & (0x100u >> i)) != 0u ? 17.5f : 5.8f;
        }
    }
}

static uint16_t signal(double mm)
{
    if (mm < lead) return 160u;
    double at = lead;
    for (unsigned int i = 0u; i < count; ++i) {
        if (mm < at + elements[i]) return (i & 1u) == 0u ? 1000u : 160u;
        at += elements[i];
    }
    return mm < at + 20.0 ? 160u : 1200u;
}

static double axis(void)
{
    const barcode_window_config_t *c = &barcode_window_default_config;
    return x + c->sensor_forward_mm * cos(heading) -
           c->sensor_lateral_mm * sin(heading);
}

static void move(double left, double right)
{
    const double centre = (left + right) * 0.5;
    const double turn = (right - left) / barcode_window_default_config.track_mm;
    double lx = centre, ly = 0.0;
    if (fabs(turn) > 1e-9) {
        lx = centre / turn * sin(turn);
        ly = centre / turn * (1.0 - cos(turn));
    }
    x += cos(heading) * lx - sin(heading) * ly;
    y += sin(heading) * lx + cos(heading) * ly;
    heading += turn;
    wheel_l += left;
    wheel_r += right;
}

static void load_kr(void)
{
    char path[512], line[160];
    const int n = snprintf(path, sizeof(path), "%s/barcode_fc1_twitch_kr.csv",
                           BC_FIXTURES);
    assert(n > 0 && (unsigned int)n < sizeof(path));
    FILE *f = fopen(path, "r");
    assert(f && fgets(line, sizeof(line), f));
    while (fgets(line, sizeof(line), f)) {
        unsigned long row, ms, al, ar, ml, mr;
        assert(sscanf(line, "%lu,%lu,%lu,%lu,%lu,%lu", &row, &ms, &al, &ar,
                      &ml, &mr) == 6 && kr_rows < 8u);
        kr[kr_rows][0] = (float)ml;
        kr[kr_rows][1] = (float)mr;
        ++kr_rows;
    }
    assert(fclose(f) == 0 && kr_rows == 6u);
}

/* Returns true when any attempt ended with BW_MOTION. */
static bool run(const char *kind, double trigger)
{
    const barcode_cal_report_t cal = {.mode = BC_CAL_FALLBACK,
        .white_mean = 160u, .white_samples = 32u, .rise = 320u, .fall = 240u,
        .faults = BC_CAL_NO_BLACK};
    assert(barcode_window_begin(&state, &cal, NULL));
    const barcode_window_config_t *c = &barcode_window_default_config;
    x = -c->sensor_forward_mm;
    y = -c->sensor_lateral_mm;
    heading = 0.0;
    wheel_l = wheel_r = 0.0;
    int32_t ticks_l = 0, ticks_r = 0;
    bool motion = false;
    uint32_t serial = 0u;
    int32_t start_ms = -1;
    const double scale = c->counts_per_m / 1000.0;
    for (uint32_t ms = 0u; ms <= 3600u; ++ms) {
        double vl = 146.0, vr = 146.0;
        if (start_ms < 0 && axis() >= trigger) start_ms = (int32_t)ms;
        const int32_t t = start_ms < 0 ? -1 : (int32_t)ms - start_ms;
        if (t >= 0) {
            if (strcmp(kind, "kr") == 0) {
                if (t < 120) {
                    vl = kr[t / 20][0];
                    vr = kr[t / 20][1];
                }
            } else {
                const int32_t hold = strcmp(kind, "retreat") == 0 ? 120 : 30;
                if (t < hold) {
                    vl = 0.0; vr = 292.0;
                } else if (t < 2 * hold) {
                    vl = 292.0; vr = 0.0;
                }
            }
        }
        move(vl / 1000.0, vr / 1000.0);
        if (ms % 10u == 0u) {
            int32_t l = (int32_t)lround(wheel_l * scale);
            int32_t r = (int32_t)lround(wheel_r * scale);
            if (t >= 0 && t < 10 && strcmp(kind, "reverse") == 0) {
                l = ticks_l - 3; r = ticks_r - 3;
            } else if (t >= 0 && t < 10 && strcmp(kind, "stall") == 0) {
                l = ticks_l; r = ticks_r;
            }
            if (strcmp(kind, "reverse") == 0 || strcmp(kind, "stall") == 0) {
                if (t >= 0 && t < 10) {
                    wheel_l = l / scale; wheel_r = r / scale;
                }
            }
            ticks_l = l; ticks_r = r;
            barcode_window_motor(&state, ms * 1000u, (uint32_t)l, (uint32_t)r, 0u);
        }
        barcode_window_adc(&state, ms * 1000u, signal(axis()), 0u);
        (void)barcode_window_step(&state, BW_STEP_MAX);
        if (state.result.serial != serial) {
            serial = state.result.serial;
            if (state.result.reason == BW_MOTION) motion = true;
        }
    }
    return motion;
}

int main(int argc, char **argv)
{

    make();
    load_kr();
    const char *kind = argc < 2 ? "straight" : argv[1];
    /* Inside the first wide space and on the first bar's trailing edge. */
    const double star = lead + 10.0, edge = lead + 5.8 - 0.5;
    if (strcmp(kind, "twitch_star") == 0 || strcmp(kind, "twitch_edge") == 0 ||
        strcmp(kind, "kr") == 0) {
        const bool motion = run(strcmp(kind, "kr") == 0 ? "kr" : "twitch",
                                strcmp(kind, "twitch_edge") == 0 ? edge : star);
        printf("%s motion %u accepted %u reason %lu\n", kind, motion,
               state.result.accepted, (unsigned long)state.result.reason);
        assert(!motion);
        assert(state.result.accepted && state.result.letter == 'F');
    } else if (strcmp(kind, "geometry") == 0) {
        /* MEASURED operator ruler 2026-11-16: 73 mm forward, 73 mm left. */
        assert(barcode_window_default_config.sensor_forward_mm == 73);
        assert(barcode_window_default_config.sensor_lateral_mm == 73);
        assert(barcode_window_default_config.track_mm == 111u);
    } else if (strcmp(kind, "straight") == 0) {
        assert(!run("none", 1e9) && state.result.accepted &&
               state.result.letter == 'F');
    } else {
        assert(strcmp(kind, "retreat") == 0 || strcmp(kind, "reverse") == 0 ||
               strcmp(kind, "stall") == 0);
        assert(run(kind, star));
        assert(!state.result.accepted);
    }
    return 0;
}

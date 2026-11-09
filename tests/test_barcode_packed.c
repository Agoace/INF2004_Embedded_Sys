#include "../app_program/subsystem3/barcode.h"
#include <assert.h>
#include <string.h>

static uint16_t raw[20000];
static barcode_packed_t rows[BC_MAX_SAMPLES];
static barcode_motion_t motors[BC_MAX_MOTION];
static barcode_workspace_t workspace;
static barcode_report_t report;
static barcode_frame_t found[8];
static size_t used;
static size_t frames;
static bool invert;
static uint32_t origin;

static void segment(bool black, unsigned int milliseconds)
{
    assert(used + milliseconds <= sizeof(raw) / sizeof(raw[0]));
    for (unsigned int i = 0u; i < milliseconds; ++i) {
        raw[used++] = black != invert ? 1500u : 160u;
    }
}

static void letter(const char *pattern, unsigned int n, unsigned int w)
{
    for (unsigned int i = 0u; i < 9u; ++i) {
        segment((i & 1u) == 0u, pattern[i] == 'w' ? w : n);
    }
}

static void frame(const char *pattern, unsigned int n, unsigned int w)
{
    segment(false, n * 12u);
    letter("nwnnwnwnn", n, w);
    segment(false, n);
    letter(pattern, n, w);
    segment(false, n);
    letter("nwnnwnwnn", n, w);
    segment(false, n * 12u);
}

static void sink(const barcode_frame_t *value, void *context)
{
    assert(context == &report && frames < 8u);
    assert(value->value.decoded && value->value.quality == BC_QUALITY_DECODED);
    assert(value->value.command == BC_CMD_NONE);
    assert(value->start_us - origin < value->end_us - origin);
    assert(value->end_us - origin < used * 1000u);
    found[frames++] = *value;
}

static void decode(bool backward)
{
    size_t count = 0u;
    for (size_t i = 0u; i < used; i += 2u) {
        rows[count] = (barcode_packed_t){raw[i], count == 0u ? 0u : 2000u};
        ++count;
    }
    rows[0].delta_us = 0u;
    size_t motion_count = 0u;
    for (uint32_t ms = 0u; ms <= used + 30u; ms += 30u) {
        const int32_t position = (int32_t)((uint64_t)ms * 12414u / 10000u);
        motors[motion_count++] = (barcode_motion_t){
            origin + ms * 1000u, backward ? -position : position,
            backward ? -position : position};
    }
    barcode_config_t config = barcode_default_config;
    config.max_gap_us = 3500u;
    config.motion_gap_us = 50000u;
    frames = 0u;
    barcode_begin_packed(rows, count, origin, motors, motion_count, &config,
                         &workspace, &report);
    barcode_frame_sink(&workspace, sink, &report);
    unsigned int steps = 0u;
    while (!barcode_step(&workspace)) {
        assert(++steps < 2300u);
    }
}

int main(void)
{
    for (unsigned int polarity = 0u; polarity < 2u; ++polarity) {
        invert = polarity != 0u;
        used = 0u;
        frame("nnwnwwnnn", 58u, 175u); /* 2x F */
        frame("nwwnwnnnn", 29u, 87u);  /* smaller Z: independent host frames */
        origin = UINT32_MAX - 2000000u;
        decode(false);
        assert(frames == 4u);
        assert(report.time.reason == BC_AMBIGUOUS);
        unsigned int letters[2] = {0};
        for (size_t i = 0u; i < frames; ++i) {
            const barcode_frame_t *f = &found[i];
            if (strcmp(f->value.text, "F") == 0) {
                ++letters[0];
                assert(f->start_us - origin < 1500000u);
            } else {
                assert(strcmp(f->value.text, "Z") == 0);
                ++letters[1];
                assert(f->start_us - origin > 4000000u);
            }
            assert(f->start_um >= 0 && f->end_um > f->start_um);
        }
        assert(letters[0] == 2u && letters[1] == 2u);
        for (size_t i = 0u; i < used / 2u; ++i) {
            const uint16_t tmp = raw[i];
            raw[i] = raw[used - 1u - i];
            raw[used - 1u - i] = tmp;
        }
        decode(true);
        assert(frames == 4u);
        for (size_t i = 0u; i < frames; ++i) {
            assert(found[i].value.directions == 2u);
            assert(found[i].start_um <= 0);
            assert(found[i].end_um < found[i].start_um);
        }
    }
    invert = false;
    origin = 0u;
    used = 0u;
    segment(false, 5000u); /* Later than the old 4.1 s hand-push window. */
    frame("nnwnwwnnn", 29u, 87u);
    decode(false);
    assert(frames == 2u && report.time.decoded && report.distance.decoded);
    assert(found[0].start_us > 5000000u);
    rows[12].raw_flags |= BC_PACKED_GAP;
    barcode_begin_packed(rows, (used + 1u) / 2u, 0u, NULL, 0u,
                         &barcode_default_config, &workspace, &report);
    while (!barcode_step(&workspace)) {
    }
    assert(report.time.reason == BC_SAMPLE_LOSS);
    rows[12].raw_flags &= BC_PACKED_RAW;
    rows[0].delta_us = 1u;
    barcode_begin_packed(rows, (used + 1u) / 2u, 0u, NULL, 0u,
                         &barcode_default_config, &workspace, &report);
    while (!barcode_step(&workspace)) {
    }
    assert(report.time.reason == BC_SAMPLE_LOSS);
    barcode_begin_packed(NULL, 0u, 0u, NULL, 0u, NULL, NULL, NULL);
    return 0;
}

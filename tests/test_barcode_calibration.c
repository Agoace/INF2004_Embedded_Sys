#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "../app_program/subsystem3/barcode_calibration.h"

static uint32_t feed(barcode_calibration_t *state, uint32_t us,
                     uint32_t count, uint16_t raw)
{
    for (uint32_t i = 0u; i < count; ++i) {
        barcode_calibration_add(state, us, raw);
        us += 1000u;
    }
    return us;
}

int main(int argc, char **argv)
{
    const char *name = argc > 1 ? argv[1] : "learned";
    barcode_calibration_t state = {0};
    barcode_calibration_add(&state, 0u, 1200u);
    barcode_calibration_end(&state, true);
    assert(state.report.mode == BC_CAL_IDLE && state.report.samples == 0u);
    barcode_calibration_begin(&state);
    assert(state.report.generation == 1u);
    assert(state.report.mode == BC_CAL_COLLECTING);
    assert(state.report.rise == 320u && state.report.fall == 240u);
    uint32_t us = 1000u;
    if (strcmp(name, "learned") == 0) {
        us = feed(&state, us, 16u, 160u);
        (void)feed(&state, us, 11u, 1200u);
        barcode_calibration_end(&state, true);
        assert(state.report.mode == BC_CAL_LEARNED);
        assert(state.report.samples == 27u);
        assert(state.report.minimum == 160u && state.report.maximum == 1200u);
        assert(state.report.white_mean == 160u);
        assert(state.report.black_mean == 1200u);
        assert(state.report.black_hold_us == 10000u);
        assert(state.report.rise == 310u && state.report.fall == 235u);
        assert(state.report.faults == 0u);
    } else if (strcmp(name, "white_only") == 0) {
        (void)feed(&state, us, 64u, 160u);
        barcode_calibration_end(&state, true);
        assert(state.report.mode == BC_CAL_LEARNED);
        assert(state.report.rise == 310u && state.report.fall == 235u);
        assert(state.report.faults == BC_CAL_NO_BLACK);
    } else if (strcmp(name, "air") == 0 || strcmp(name, "dark") == 0) {
        const uint16_t raw = strcmp(name, "air") == 0 ? 1200u : 1000u;
        (void)feed(&state, us, 64u, raw);
        barcode_calibration_end(&state, true);
        assert(state.report.mode == BC_CAL_FALLBACK);
        assert(state.report.rise == 320u && state.report.fall == 240u);
        assert(state.report.white_samples == 0u);
        assert((state.report.faults & BC_CAL_NO_WHITE) != 0u);
    } else if (strcmp(name, "spike") == 0) {
        us = feed(&state, us, 16u, 160u);
        us = feed(&state, us, 1u, 1800u);
        (void)feed(&state, us, 16u, 170u);
        barcode_calibration_end(&state, true);
        assert(state.report.maximum == 1800u);
        assert(state.report.white_mean == 165u);
        assert(state.report.mode == BC_CAL_LEARNED);
        assert(state.report.rise == 315u && state.report.fall == 240u);
        assert(state.report.black_hold_us == 0u);
    } else if (strcmp(name, "white_short") == 0) {
        us = feed(&state, us, 15u, 160u);
        (void)feed(&state, us, 11u, 1200u);
        barcode_calibration_end(&state, true);
        assert(state.report.mode == BC_CAL_FALLBACK);
        assert(state.report.rise == 320u && state.report.fall == 240u);
        assert((state.report.faults & BC_CAL_NO_WHITE) != 0u);
    } else if (strcmp(name, "black_short") == 0) {
        us = feed(&state, us, 16u, 160u);
        (void)feed(&state, us, 10u, 1200u);
        barcode_calibration_end(&state, true);
        assert(state.report.mode == BC_CAL_LEARNED);
        assert(state.report.black_hold_us == 9000u);
    } else if (strcmp(name, "bounds") == 0) {
        us = feed(&state, us, 16u, 120u);
        us = feed(&state, us, 16u, 220u);
        us = feed(&state, us, 16u, 799u);
        (void)feed(&state, us, 11u, 800u);
        barcode_calibration_end(&state, true);
        assert(state.report.mode == BC_CAL_LEARNED);
        assert(state.report.white_samples == 32u);
        assert(state.report.black_samples == 11u);
    } else if (strcmp(name, "dark_line") == 0) {
        const uint16_t highs[] = {1000u, 2700u, 3178u};
        for (unsigned int i = 0u; i < 3u; ++i) {
            barcode_calibration_begin(&state);
            us = feed(&state, us, 32u, 167u);
            us = feed(&state, us, 16u, highs[i]);
            barcode_calibration_end(&state, true);
            assert(state.report.mode == BC_CAL_LEARNED);
            assert(state.report.maximum == highs[i]);
            assert(state.report.rise == 317u && state.report.fall == 242u);
        }
    } else if (strcmp(name, "cap") == 0) {
        us = feed(&state, us, 32u, 220u);
        (void)feed(&state, us, 16u, 3178u);
        barcode_calibration_end(&state, true);
        assert(state.report.mode == BC_CAL_LEARNED);
        assert(state.report.rise == 350u && state.report.fall == 275u);
    } else if (strcmp(name, "aborted") == 0) {
        us = feed(&state, us, 16u, 160u);
        (void)feed(&state, us, 11u, 1200u);
        barcode_calibration_end(&state, false);
        assert(state.report.mode == BC_CAL_ABORTED);
        assert(state.report.samples == 27u && state.report.maximum == 1200u);
    } else if (strcmp(name, "no_data") == 0) {
        barcode_calibration_end(&state, true);
        assert(state.report.mode == BC_CAL_INVALID);
        assert(state.report.minimum == 0u && state.report.maximum == 0u);
    } else if (strcmp(name, "gap") == 0 || strcmp(name, "duplicate") == 0 ||
               strcmp(name, "reverse_stamp") == 0) {
        us = feed(&state, us, 16u, 160u);
        if (strcmp(name, "gap") == 0) {
            us += 5000u;
        } else if (strcmp(name, "duplicate") == 0) {
            us -= 1000u;
        } else {
            us -= 1001u;
        }
        (void)feed(&state, us, 11u, 1200u);
        barcode_calibration_end(&state, true);
        assert(state.report.mode == BC_CAL_INVALID);
        assert((state.report.faults & BC_CAL_FAULT_TIME) != 0u);
    } else if (strcmp(name, "wrap") == 0) {
        us = feed(&state, UINT32_MAX - 8000u, 16u, 160u);
        (void)feed(&state, us, 11u, 1200u);
        barcode_calibration_end(&state, true);
        assert(state.report.mode == BC_CAL_LEARNED);
        assert(state.report.max_gap_us == 1000u);
    } else if (strcmp(name, "clipping") == 0) {
        const uint16_t raw[] = {0u, 4095u, UINT16_MAX};
        for (unsigned int i = 0u; i < 3u; ++i) {
            barcode_calibration_begin(&state);
            us = feed(&state, us, 16u, 160u);
            (void)feed(&state, us, 1u, raw[i]);
            barcode_calibration_end(&state, true);
            assert(state.report.mode == BC_CAL_INVALID);
            assert((state.report.faults & BC_CAL_FAULT_RAW) != 0u);
        }
    } else if (strcmp(name, "capacity") == 0) {
        us = feed(&state, us, 65535u, 160u);
        (void)feed(&state, us, 1u, 160u);
        barcode_calibration_end(&state, true);
        assert(state.report.mode == BC_CAL_INVALID);
        assert(state.report.samples == 65535u);
        assert((state.report.faults & BC_CAL_FAULT_FULL) != 0u);
    } else if (strcmp(name, "restart") == 0) {
        us = feed(&state, us, 16u, 160u);
        (void)feed(&state, us, 11u, 1200u);
        barcode_calibration_end(&state, true);
        assert(state.report.mode == BC_CAL_LEARNED);
        barcode_calibration_begin(&state);
        assert(state.report.generation == 2u && state.report.samples == 0u);
        assert(state.report.rise == 320u && state.report.fall == 240u);
        (void)feed(&state, us, 16u, 160u);
        barcode_calibration_end(&state, true);
        assert(state.report.mode == BC_CAL_LEARNED);
        assert(state.report.black_samples == 0u);
        state.report.generation = UINT32_MAX;
        barcode_calibration_begin(&state);
        assert(state.report.generation == 1u);
    } else if (strcmp(name, "terminal") == 0) {
        (void)feed(&state, us, 16u, 160u);
        barcode_calibration_end(&state, true);
        const barcode_cal_report_t report = state.report;
        barcode_calibration_add(&state, us, 1200u);
        barcode_calibration_end(&state, false);
        assert(memcmp(&report, &state.report, sizeof(report)) == 0);
    } else {
        assert(false);
    }
    return 0;
}

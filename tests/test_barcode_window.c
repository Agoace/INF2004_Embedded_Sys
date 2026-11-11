#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../app_program/subsystem3/barcode.h"
#include "../app_program/subsystem3/barcode_window.h"

static barcode_window_t state;
static float elements[64];
static unsigned int count;
static float lead = 80.0f;
static float tail_white = 20.0f;
static int missing = -1, merged = -1;
static bool extra;
static const barcode_cal_report_t calibration = {
    .mode = BC_CAL_FALLBACK, .white_mean = 160u, .white_samples = 32u,
    .rise = 320u, .fall = 240u, .faults = BC_CAL_NO_BLACK};

static void make(const char *text, bool reverse, float scale)
{
    count = 0u;
    for (unsigned int ch = 0u; text[ch] != '\0'; ++ch) {
        if (ch != 0u) {
            elements[count++] = 5.8f * scale;
        }
        const uint16_t mask = barcode_pattern(text[ch]);
        assert(mask != 0u);
        for (unsigned int i = 0u; i < 9u; ++i) {
            elements[count++] =
                ((mask & (0x100u >> i)) != 0u ? 17.5f : 5.8f) * scale;
        }
    }
    if (reverse) {
        for (unsigned int i = 0u; i < count / 2u; ++i) {
            const float value = elements[i];
            elements[i] = elements[count - 1u - i];
            elements[count - 1u - i] = value;
        }
    }
}

static float distance(uint32_t us, bool variable)
{
    const float seconds = (float)us / 1000000.0f;
    if (!variable || seconds < 1.0f) {
        return seconds * 146.0f;
    }
    return seconds < 2.0f ? 146.0f + (seconds - 1.0f) * 90.0f :
                           236.0f + (seconds - 2.0f) * 180.0f;
}

static uint16_t signal(float mm, const char *kind)
{
    const uint16_t white = strcmp(kind, "tracking") == 0 ? 200u : 160u;
    if (strcmp(kind, "paper") == 0) {
        return 160u;
    }
    if (strcmp(kind, "flat_dark") == 0) {
        return 1200u;
    }
    if (strcmp(kind, "line_only") == 0) {
        return mm > 150.0f && mm < 168.0f ? 1200u : 160u;
    }
    if (mm < lead) {
        return strcmp(kind, "air") == 0 ? 1200u : white;
    }
    float at = lead;
    for (unsigned int i = 0u; i < count; ++i) {
        if (mm < at + elements[i]) {
            bool black = (i & 1u) == 0u;
            if ((int)i == missing) {
                black = false;
            }
            if ((int)i == merged) {
                black = true;
            }
            if (extra && i == 15u && mm - at > 2.0f && mm - at < 2.5f) {
                black = true;
            }
            return black ? 1000u :
                   strcmp(kind, "freeze") == 0 ? 200u : white;
        }
        at += elements[i];
    }
    return mm < at + tail_white ? white : 1200u;
}

static void run(const char *kind)
{
    barcode_window_config_t config = barcode_window_default_config;
    if (strcmp(kind, "two_ms") == 0) {
        config.sample_gap_us = 3000u; /* Explicit retained-data host profile. */
    }
    assert(barcode_window_begin(&state, &calibration, &config));
    const bool variable = strcmp(kind, "variable") == 0;
    const bool wrap = strcmp(kind, "wrap") == 0;
    const uint32_t origin = wrap ? UINT32_MAX - 100000u : 0u;
    const uint32_t base = wrap ? UINT32_MAX - 500u : 0u;
    uint32_t previous = base;
    bool fault = false;
    for (uint32_t us = 0u; us <= 4000000u; us += 1000u) {
        const float mm = distance(us, variable);
        const uint32_t ticks = base + (uint32_t)(mm * 12.414f);
        const bool zone = mm > 150.0f && mm < 154.0f;
        if (us % 10000u == 0u) {
            uint32_t value = ticks, flags = 0u;
            if (!fault && zone && strcmp(kind, "reversal") == 0) {
                value = previous - 1u;
                fault = true;
            }
            if (!fault && zone && strcmp(kind, "motor_fault") == 0) {
                flags = 1u;
                fault = true;
            }
            if (!(strcmp(kind, "motor_gap") == 0 &&
                  mm > 145.0f && mm < 153.0f)) {
                barcode_window_motor(&state, us + origin, value, value, flags);
                barcode_window_motor(&state, us + origin, value, value, flags);
            }
            previous = ticks;
        }
        const bool one_missing = !fault && zone &&
                                 strcmp(kind, "one_missing") == 0;
        if (one_missing) {
            fault = true;
        }
        if (!(strcmp(kind, "sample_gap") == 0 && zone) && !one_missing) {
            const uint16_t flags =
                !fault && zone && strcmp(kind, "adc_fault") == 0 ? 1u : 0u;
            if (flags != 0u) {
                fault = true;
            }
            if (strcmp(kind, "two_ms") != 0 || us % 2000u == 0u) {
                barcode_window_adc(&state, us + origin, signal(mm, kind),
                                   flags);
            }
        }
        assert(barcode_window_step(&state, 1000u) <= BW_STEP_MAX);
        if (state.result.accepted) {
            float end = lead;
            for (unsigned int i = 0u; i < count; ++i) {
                end += elements[i];
            }
            const float valid_mm = distance(state.result.eligible_us - origin,
                                            variable);
            assert(state.elements == 29u);
            assert(valid_mm >= end + 5.5f && valid_mm < end + 8.0f);
            uint32_t first = 0u;
            assert(barcode_window_candidate(&state, &first));
            assert(first == state.result.start_us);
        }
    }
}

static void alphabet(void)
{
    const char *symbols = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";
    for (unsigned int i = 0u; symbols[i] != '\0'; ++i) {
        for (unsigned int reverse = 0u; reverse < 2u; ++reverse) {
            char text[] = {'*', symbols[i], '*', '\0'};
            make(text, reverse != 0u, 1.0f);
            run("normal");
            const bool letter = symbols[i] >= 'A' && symbols[i] <= 'Z';
            assert(state.result.accepted == letter);
            if (letter) {
                assert(state.result.letter == symbols[i]);
                assert(state.result.direction == reverse + 1u);
                assert(state.result.eligible_us - state.result.edge_us <
                       70000u); /* ~1n, not 58mm/397ms */
                const uint8_t action = barcode_window_table_confirmed() ?
                    barcode_window_table_entry((unsigned int)(symbols[i] -
                                                             'A')) : 0u;
                assert(state.result.mapped_action == action);
            } else {
                assert(state.result.reason == BW_NOT_LETTER);
            }
        }
    }
}

int main(int argc, char **argv)
{
    const char *kind = argc > 1 ? argv[1] : "alphabet";
    assert(barcode_window_table_entry(26u) == 0u);
    assert((barcode_window_mapping_id() != 0u) ==
           barcode_window_table_confirmed());
    if (strcmp(kind, "alphabet") == 0) {
        alphabet();
    } else if (strcmp(kind, "missing") == 0 || strcmp(kind, "merged") == 0) {
        for (int i = strcmp(kind, "missing") == 0 ? 0 : 1; i < 29; i += 2) {
            missing = strcmp(kind, "missing") == 0 ? i : -1;
            merged = strcmp(kind, "merged") == 0 ? i : -1;
            make("*F*", false, 1.0f);
            run("normal");
            assert(!state.result.accepted);
        }
    } else if (strcmp(kind, "calibration") == 0) {
        barcode_cal_report_t bad = calibration;
        bad.mode = BC_CAL_INVALID;
        assert(!barcode_window_begin(&state, &bad, NULL));
        barcode_window_adc(&state, 0u, 160u, 0u);
        barcode_window_motor(&state, 0u, 0u, 0u, 0u);
        assert(barcode_window_step(&state, 32u) == 0u);
        assert(state.result.reason == BW_CALIBRATION);
        bad = calibration;
        bad.white_samples = 0u;
        bad.white_mean = 0u;
        bad.faults = BC_CAL_NO_WHITE;
        assert(barcode_window_begin(&state, &bad, NULL));
        assert(!state.result.accepted); /* Fresh leading white still needed. */
        barcode_window_config_t config = barcode_window_default_config;
        config.counts_per_m = 0u;
        assert(!barcode_window_begin(&state, &calibration, &config));
    } else if (strcmp(kind, "alias") == 0) {
        assert(barcode_window_begin(&state, &calibration, NULL));
        assert(barcode_window_begin(&state, &state.calibration, &state.config));
        uint32_t first = 123u;
        assert(!barcode_window_candidate(&state, &first) && first == 123u);
    } else if (strcmp(kind, "queue") == 0) {
        assert(barcode_window_begin(&state, &calibration, NULL));
        for (uint32_t i = 0u; i < 65u; ++i) {
            barcode_window_adc(&state, i * 1000u, 160u, 0u);
        }
        assert(state.result.reason == BW_LOSS);
        assert(!state.result.accepted);
    } else if (strcmp(kind, "brackets") == 0) {
        assert(barcode_window_begin(&state, &calibration, NULL));
        barcode_window_motor(&state, 0u, 0u, 0u, 0u);
        barcode_window_adc(&state, 15000u, 160u, 0u);
        assert(barcode_window_step(&state, 32u) == 0u);
        barcode_window_motor(&state, 10000u, 18u, 18u, 0u);
        assert(barcode_window_step(&state, 32u) == 0u);
        barcode_window_motor(&state, 20000u, 36u, 36u, 0u);
        assert(barcode_window_step(&state, 0u) == 0u);
        assert(barcode_window_step(&state, 1u) == 1u);
    } else {
        make(strcmp(kind, "multi") == 0 ? "*AB*" : "*F*", false,
             strcmp(kind, "small") == 0 ? 0.5f : 1.0f);
        if (strcmp(kind, "leading") == 0) {
            lead = 40.0f;
        } else if (strcmp(kind, "short_tail") == 0) {
            tail_white = 3.0f;
        } else if (strcmp(kind, "short_quiet") == 0) {
            tail_white = 7.0f;
        } else if (strcmp(kind, "gap") == 0) {
            elements[9] = 12.0f;
        } else if (strcmp(kind, "extra") == 0) {
            extra = true;
        }
        run(kind);
        const bool positive = strcmp(kind, "normal") == 0 ||
            strcmp(kind, "variable") == 0 || strcmp(kind, "wrap") == 0 ||
            strcmp(kind, "two_ms") == 0 || strcmp(kind, "freeze") == 0 ||
            strcmp(kind, "tracking") == 0 ||
            strcmp(kind, "short_quiet") == 0 ||
            strcmp(kind, "immutable") == 0;
        if (state.result.accepted != positive) {
            printf("%s accepted %d reason %lu count %u lead %.3f\n", kind,
                   state.result.accepted, (unsigned long)state.result.reason,
                   state.elements, (double)state.leading_mm);
        }
        assert(state.result.accepted == positive);
        if (positive) {
            assert(state.result.letter == 'F');
            assert(state.result.rise ==
                   (strcmp(kind, "tracking") == 0 ? 350u : 310u));
            assert(state.result.fall ==
                   (strcmp(kind, "tracking") == 0 ? 275u : 235u));
            assert(state.result.read_us - state.result.eligible_us <= 10000u);
            const barcode_window_result_t copy = state.result;
            barcode_window_adc(&state, 1u, 4095u, 1u);
            barcode_window_motor(&state, 1u, 0u, 0u, 1u);
            assert(memcmp(&copy, &state.result, sizeof(copy)) == 0);
            const uint32_t serial = state.serial;
            assert(barcode_window_rearm(&state));
            assert(!state.result.accepted && state.serial == serial);
            assert(state.queued == 0u && state.motor_count == 0u);
        }
    }
    return 0;
}

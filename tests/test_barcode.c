/* Independent n/w spellings of standard Code 39 (also used by the old
 * bare-metal reader). Production uses numeric nine-bit masks instead. */
#include "../app_program/subsystem3/barcode.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    char value;
    const char *widths;
} fixture_t;

static const fixture_t symbols[] = {
    {'0', "nnnwwnwnn"}, {'1', "wnnwnnnnw"}, {'2', "nnwwnnnnw"},
    {'3', "wnwwnnnnn"}, {'4', "nnnwwnnnw"}, {'5', "wnnwwnnnn"},
    {'6', "nnwwwnnnn"}, {'7', "nnnwnnwnw"}, {'8', "wnnwnnwnn"},
    {'9', "nnwwnnwnn"}, {'A', "wnnnnwnnw"}, {'B', "nnwnnwnnw"},
    {'C', "wnwnnwnnn"}, {'D', "nnnnwwnnw"}, {'E', "wnnnwwnnn"},
    {'F', "nnwnwwnnn"}, {'G', "nnnnnwwnw"}, {'H', "wnnnnwwnn"},
    {'I', "nnwnnwwnn"}, {'J', "nnnnwwwnn"}, {'K', "wnnnnnnww"},
    {'L', "nnwnnnnww"}, {'M', "wnwnnnnwn"}, {'N', "nnnnwnnww"},
    {'O', "wnnnwnnwn"}, {'P', "nnwnwnnwn"}, {'Q', "nnnnnnwww"},
    {'R', "wnnnnnwwn"}, {'S', "nnwnnnwwn"}, {'T', "nnnnwnwwn"},
    {'U', "wwnnnnnnw"}, {'V', "nwwnnnnnw"}, {'W', "wwwnnnnnn"},
    {'X', "nwnnwnnnw"}, {'Y', "wwnnwnnnn"}, {'Z', "nwwnwnnnn"},
    {'-', "nwnnnnwnw"}, {'.', "wwnnnnwnn"}, {' ', "nwwnnnwnn"},
    {'$', "nwnwnwnnn"}, {'/', "nwnwnnnwn"}, {'+', "nwnnnwnwn"},
    {'%', "nnnwnwnwn"}, {'*', "nwnnwnwnn"}};
static barcode_sample_t rows[BC_MAX_SAMPLES];
static barcode_motion_t motors[BC_MAX_MOTION];
static int32_t positions[BC_MAX_SAMPLES];
static barcode_workspace_t work;
static barcode_report_t report;
static size_t used;
static size_t motor_count;
static int32_t count_position;
static uint32_t base_us;
static bool inverted;
static unsigned int fixture_narrow = 8u;
static unsigned int fixture_wide;
static unsigned int fixture_quiet = 12u;

static void segment(bool black, unsigned int length, unsigned int count_rate)
{
    for (unsigned int i = 0u; i < length; ++i) {
        assert(used < BC_MAX_SAMPLES);
        const bool high = black == inverted;
        const int noise = (int)(used % 13u) - 6;
        rows[used] =
            (barcode_sample_t){.us = base_us + (uint32_t)used * 1000u,
                               .raw = (uint16_t)((high ? 3000 : 300) + noise)};
        positions[used] = count_position;
        count_position += (int32_t)count_rate;
        ++used;
    }
}

static void letter(char c, unsigned int narrow, bool warped)
{
    const char *pattern = NULL;
    for (size_t i = 0u; i < sizeof(symbols) / sizeof(symbols[0]); ++i) {
        if (symbols[i].value == c) {
            pattern = symbols[i].widths;
        }
    }
    assert(pattern != NULL);
    for (unsigned int i = 0u; i < 9u; ++i) {
        const bool wide = pattern[i] == 'w';
        const unsigned int wide_time =
            fixture_wide != 0u ? fixture_wide : narrow * 5u / 2u;
        const unsigned int time = warped ? narrow : (wide ? wide_time : narrow);
        segment((i & 1u) == 0u, time, warped ? (wide ? 5u : 2u) : 3u);
    }
}

static void frame(const char *text, bool warped)
{
    used = 0u;
    motor_count = 0u;
    count_position = 0;
    const unsigned int narrow = warped ? 40u : fixture_narrow;
    segment(false, narrow * fixture_quiet, warped ? 2u : 3u);
    letter('*', narrow, warped);
    segment(false, narrow, warped ? 2u : 3u);
    for (size_t i = 0u; text[i] != '\0'; ++i) {
        letter(text[i], narrow, warped);
        segment(false, narrow, warped ? 2u : 3u);
    }
    letter('*', narrow, warped);
    segment(false, narrow * fixture_quiet, warped ? 2u : 3u);
    for (size_t i = 0u; i < used; i += 10u) {
        motors[motor_count++] = (barcode_motion_t){
            base_us + (uint32_t)i * 1000u, positions[i], positions[i]};
    }
    const uint32_t end_ms = (uint32_t)((used + 9u) / 10u) * 10u;
    const int32_t end_count =
        count_position + (int32_t)(end_ms - (uint32_t)used) * (warped ? 2 : 3);
    motors[motor_count++] =
        (barcode_motion_t){base_us + end_ms * 1000u, end_count, end_count};
}

static void decode(void)
{
    barcode_begin(rows, used, motors, motor_count, &barcode_default_config,
                  &work, &report);
    unsigned int steps = 0u;
    while (!barcode_step(&work)) {
        assert(++steps < 2200u);
    }
}

static void reverse_pattern(void)
{
    for (size_t i = 0u; i < used / 2u; ++i) {
        const uint16_t tmp = rows[i].raw;
        rows[i].raw = rows[used - 1u - i].raw;
        rows[used - 1u - i].raw = tmp;
    }
}

static void alphabet_roundtrip(void)
{
    for (unsigned int polarity = 0u; polarity < 2u; ++polarity) {
        inverted = polarity != 0u;
        for (size_t i = 0u; i < 43u; ++i) {
            const char text[2] = {symbols[i].value, '\0'};
            frame(text, false);
            for (unsigned int reverse = 0u; reverse < 2u; ++reverse) {
                decode();
                assert(report.time.decoded && report.distance.decoded);
                assert(strcmp(report.time.text, text) == 0);
                assert(strcmp(report.distance.text, text) == 0);
                assert(report.time.directions == (reverse ? 2u : 1u));
                const char c = text[0];
                const barcode_command_t expected =
                    c >= 'A' && c <= 'D' ? (barcode_command_t)(c - 'A' + 1)
                                         : BC_CMD_NONE;
                assert(report.time.command == expected);
                assert(report.distance.command == expected);
                assert(report.time.separation_permille > 2000u);
                reverse_pattern();
            }
        }
    }
    inverted = false;
    frame("AZ 09-.$/+%", false);
    decode();
    assert(report.time.decoded && report.distance.decoded);
    assert(strcmp(report.time.text, "AZ 09-.$/+%") == 0);
    assert(report.time.command == BC_CMD_NONE);
    reverse_pattern();
    decode();
    assert(strcmp(report.time.text, "AZ 09-.$/+%") == 0);
}

static void independent_methods(void)
{
    frame("Z", true);
    decode();
    assert(!report.time.decoded && report.distance.decoded);
    assert(strcmp(report.distance.text, "Z") == 0);
    frame("A", false);
    barcode_decode(rows, used, NULL, 0u, &barcode_default_config, &work,
                   &report);
    assert(report.time.decoded && report.distance.reason == BC_MOTION);
    motors[3].left = -10000;
    motors[3].right = -10000;
    decode();
    assert(report.time.decoded && !report.distance.decoded);
    frame("B", false);
    motors[4].us += 50000u;
    decode();
    assert(report.time.decoded && report.distance.reason == BC_MOTION);
    frame("C", false);
    for (size_t i = 0u; i < motor_count; ++i) {
        motors[i].left = -motors[i].left;
        motors[i].right = -motors[i].right;
    }
    decode();
    assert(report.distance.decoded && strcmp(report.distance.text, "C") == 0);
}

static void reject_data(void)
{
    frame("A", false);
    rows[12].flags = BC_SAMPLE_GAP;
    decode();
    assert(report.time.reason == BC_SAMPLE_LOSS && !report.time.decoded);
    frame("A", false);
    rows[10].us = rows[9].us;
    decode();
    assert(report.time.reason == BC_SAMPLE_LOSS);
    frame("A", false);
    rows[20].raw = 4096u;
    decode();
    assert(report.time.reason == BC_SAMPLE_LOSS);
    frame("A", false);
    for (size_t i = 0u; i < used; ++i) {
        rows[i].raw = (uint16_t)(178u + i % 53u);
    }
    decode();
    assert(report.contrast == 52u && report.time.reason == BC_CONTRAST);
    frame("A", false);
    used -= 150u;
    decode();
    assert(!report.time.decoded);
    frame("A", false);
    /* A full black/white glitch is not smoothed into a guessed letter. */
    rows[150].raw = rows[150].raw > 2000u ? 300u : 3000u;
    decode();
    assert(!report.time.decoded);
    frame("ABCDEFGHIJKLMNOPQ", false);
    decode();
    assert(!report.time.decoded && report.time.reason == BC_PAYLOAD_LIMIT);
    frame("A", false);
    /* Crop the leading white quiet zone. */
    memmove(rows, rows + 80u, (used - 80u) * sizeof(rows[0]));
    used -= 80u;
    decode();
    assert(!report.time.decoded);
    used = BC_MAX_SAMPLES;
    for (size_t i = 0u; i < used; ++i) {
        rows[i] = (barcode_sample_t){(uint32_t)i * 1000u,
                                     i % 2u == 0u ? 300u : 3000u, 0u};
    }
    decode();
    assert(report.time.reason == BC_RUN_LIMIT);
}

/* Main supplied approximate photo widths, then operator ruler confirmation:
 * the physical print is F, not reference Z. These are idealized HOST inputs,
 * not acquired firmware data or a claim of exact measured total length. */
static void photographed_print(void)
{
    fixture_quiet = 11u;
    for (unsigned int scale = 1u; scale <= 2u; ++scale) {
        fixture_narrow = scale == 1u ? 29u : 58u;
        fixture_wide = scale == 1u ? 87u : 175u;
        frame("F", false);
        for (size_t i = 0u; i < motor_count; ++i) {
            const uint64_t ms = (motors[i].us - base_us) / 1000u;
            motors[i].left = (int32_t)(ms * 12414u / 10000u);
            motors[i].right = motors[i].left;
        }
        for (unsigned int direction = 0u; direction < 2u; ++direction) {
            decode();
            assert(report.time.decoded && report.distance.decoded);
            assert(strcmp(report.time.text, "F") == 0);
            assert(report.time.command == BC_CMD_NONE);
            assert(report.distance.command == BC_CMD_NONE);
            reverse_pattern();
        }
        printf("photo F scale %u: ideal pattern %.1f mm; ADC rows %lu\n", scale,
               (double)(20u * fixture_narrow + 9u * fixture_wide) / 10.0,
               (unsigned long)used);
    }
    fixture_quiet = 12u;
    fixture_narrow = 8u;
    fixture_wide = 0u;
}

static void conflicting_frames(void)
{
    static barcode_sample_t saved[BC_MAX_SAMPLES];
    frame("A", false);
    const size_t first = used;
    memcpy(saved, rows, first * sizeof(rows[0]));
    frame("B", false);
    assert(first + used <= BC_MAX_SAMPLES);
    memmove(rows + first, rows, used * sizeof(rows[0]));
    memcpy(rows, saved, first * sizeof(rows[0]));
    used += first;
    for (size_t i = 0u; i < used; ++i) {
        rows[i].us = (uint32_t)i * 1000u;
    }
    decode();
    assert(report.time.reason == BC_AMBIGUOUS && !report.time.decoded);
    assert(report.time.command == BC_CMD_NONE && report.time.text[0] == '\0');
}

static void wrap_and_null(void)
{
    base_us = UINT32_MAX - 100000u;
    frame("D", false);
    decode();
    assert(report.time.decoded && report.distance.decoded);
    base_us = 0u;
    barcode_decode(NULL, 0u, NULL, 0u, NULL, NULL, &report);
    assert(report.time.reason == BC_NO_DATA);
    assert(barcode_step(NULL));
    barcode_begin(NULL, 0u, NULL, 0u, NULL, &work, NULL);
    assert(barcode_step(&work));
    frame("F", false);
    barcode_decode(rows, used, motors, motor_count, &barcode_default_config,
                   &work, &report);
    assert(report.time.quality == BC_QUALITY_DECODED);
    assert(report.time.command == BC_CMD_NONE);
    assert(strcmp(report.time.text, "F") == 0);
    used = 20u;
    decode();
    assert(!report.time.decoded && report.time.quality == BC_QUALITY_REJECTED);
    barcode_decode(NULL, 0u, NULL, 0u, NULL, NULL, NULL);
    assert(strcmp(barcode_reason_name((barcode_reason_t)999), "UNKNOWN") == 0);
}

int main(void)
{
    alphabet_roundtrip();
    independent_methods();
    reject_data();
    photographed_print();
    conflicting_frames();
    wrap_and_null();
    puts("Code 39: 43 symbols x 2 polarities x 2 directions; "
         "methods/rejects OK");
    return 0;
}

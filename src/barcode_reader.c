#include "barcode_reader.h"

#include <string.h>

typedef struct { char value; const char *pattern; } code39_symbol_t;

static const code39_symbol_t CODE39[] = {
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
    {'*', "nwnnwnwnn"},
};

static bool level_for(const barcode_reader_t *reader, uint16_t sample, bool old_level) {
    const uint16_t low = reader->config.hysteresis > reader->config.threshold
        ? 0u : (uint16_t)(reader->config.threshold - reader->config.hysteresis);
    const uint32_t upper = (uint32_t)reader->config.threshold + reader->config.hysteresis;
    const uint16_t high = upper > UINT16_MAX ? UINT16_MAX : (uint16_t)upper;
    if (sample <= low) return false;
    if (sample >= high) return true;
    return old_level;
}

void barcode_reader_init(barcode_reader_t *reader, const barcode_reader_config_t *config) {
    memset(reader, 0, sizeof(*reader));
    reader->config = *config;
}

void barcode_reader_start(barcode_reader_t *reader, uint16_t first_sample) {
    reader->run_count = 0;
    reader->current_run = 1;
    reader->level = first_sample >= reader->config.threshold;
    reader->active = true;
    reader->overflowed = false;
}

void barcode_reader_process_sample(barcode_reader_t *reader, uint16_t sample) {
    if (!reader->active) return;
    const bool next = level_for(reader, sample, reader->level);
    if (next == reader->level) { ++reader->current_run; return; }
    if (reader->current_run >= reader->config.min_run_samples) {
        if (reader->run_count == BARCODE_MAX_RUNS) {
            reader->overflowed = true;
            reader->active = false;
            return;
        }
        reader->runs[reader->run_count++] = reader->current_run;
    }
    reader->current_run = 1;
    reader->level = next;
}

static char decode_character(const uint16_t *runs) {
    uint16_t sorted[9];
    char pattern[10];
    memcpy(sorted, runs, sizeof(sorted));
    for (size_t i = 1; i < 9; ++i) {
        const uint16_t x = sorted[i]; size_t j = i;
        while (j && sorted[j - 1] > x) { sorted[j] = sorted[j - 1]; --j; }
        sorted[j] = x;
    }
    const uint16_t split = (uint16_t)((sorted[5] + sorted[6]) / 2u);
    for (size_t i = 0; i < 9; ++i) pattern[i] = runs[i] > split ? 'w' : 'n';
    pattern[9] = '\0';
    for (size_t i = 0; i < sizeof(CODE39) / sizeof(CODE39[0]); ++i)
        if (strcmp(pattern, CODE39[i].pattern) == 0) return CODE39[i].value;
    return '\0';
}

void barcode_reader_finish(barcode_reader_t *reader, barcode_result_t *result) {
    memset(result, 0, sizeof(*result));
    if (reader->overflowed) { result->status = BARCODE_TRUNCATED; return; }
    if (reader->active && reader->current_run >= reader->config.min_run_samples) {
        if (reader->run_count == BARCODE_MAX_RUNS) {
            result->status = BARCODE_TRUNCATED;
            return;
        }
        reader->runs[reader->run_count++] = reader->current_run;
    }
    result->edge_count = (uint16_t)reader->run_count;
    reader->active = false;
    if (reader->run_count < 9) { result->status = BARCODE_NO_BARCODE; return; }

    size_t output = 0;
    bool started = false;
    for (size_t offset = 0; offset + 9 <= reader->run_count; offset += 10) {
        const char symbol = decode_character(&reader->runs[offset]);
        if (!symbol) { result->status = BARCODE_DECODE_FAIL; return; }
        if (!started) {
            if (symbol != '*') { result->status = BARCODE_PARTIAL; return; }
            started = true;
        } else if (symbol == '*') {
            result->text[output] = '\0'; result->status = BARCODE_OK; return;
        } else if (output + 1u < BARCODE_MAX_TEXT) {
            result->text[output++] = symbol;
        } else { result->status = BARCODE_TRUNCATED; return; }
    }
    result->status = BARCODE_PARTIAL;
}

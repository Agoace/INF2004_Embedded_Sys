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
    {'$', "nwnwnwnnn"}, {'/', "nwnwnnnwn"},
    {'+', "nwnnnwnwn"}, {'%', "nnnwnwnwn"},
    {'*', "nwnnwnwnn"},
};

static void clear_capture(barcode_reader_t *reader) {
    reader->run_count = 0;
    reader->shortest_run_us = UINT32_MAX;
    reader->collecting = false;
}

void barcode_reader_init(barcode_reader_t *reader, const barcode_reader_config_t *config,
                         bool initial_level, uint32_t now_us) {
    memset(reader, 0, sizeof(*reader));
    reader->config = *config;
    reader->level = initial_level;
    reader->last_edge_us = now_us;
    clear_capture(reader);
}

static char decode_character(const uint32_t *runs, uint32_t *narrow_max) {
    uint32_t sorted[9];
    char pattern[10];
    memcpy(sorted, runs, sizeof(sorted));
    for (size_t i = 1; i < 9; ++i) {
        const uint32_t x = sorted[i];
        size_t j = i;
        while (j && sorted[j - 1] > x) {
            sorted[j] = sorted[j - 1];
            --j;
        }
        sorted[j] = x;
    }
    /* Code 39 has exactly three wide and six narrow elements per symbol. */
    if (sorted[5] == 0 || (uint64_t)sorted[6] * 2u < (uint64_t)sorted[5] * 3u)
        return '\0';
    const uint32_t split = sorted[5] + (sorted[6] - sorted[5]) / 2u;
    for (size_t i = 0; i < 9; ++i)
        pattern[i] = runs[i] > split ? 'w' : 'n';
    pattern[9] = '\0';
    *narrow_max = sorted[5];
    for (size_t i = 0; i < sizeof(CODE39) / sizeof(CODE39[0]); ++i)
        if (strcmp(pattern, CODE39[i].pattern) == 0) return CODE39[i].value;
    return '\0';
}

static void decode_capture(barcode_reader_t *reader, barcode_result_t *result) {
    memset(result, 0, sizeof(*result));
    result->edge_count = (uint16_t)reader->run_count;
    if (reader->run_count < 29u) {
        result->status = BARCODE_NO_BARCODE;
        return;
    }
    if ((reader->run_count - 9u) % 10u != 0u) {
        result->status = BARCODE_PARTIAL;
        return;
    }

    size_t output = 0;
    for (size_t offset = 0; offset + 9u <= reader->run_count; offset += 10u) {
        uint32_t narrow_max = 0;
        const char symbol = decode_character(&reader->runs[offset], &narrow_max);
        if (!symbol) {
            result->status = BARCODE_DECODE_FAIL;
            return;
        }
        if (offset > 0 &&
            (uint64_t)reader->runs[offset - 1u] > (uint64_t)narrow_max * 5u) {
            result->status = BARCODE_DECODE_FAIL; /* Inter-character space is narrow. */
            return;
        }
        if (offset == 0) {
            if (symbol != '*') {
                result->status = BARCODE_PARTIAL;
                return;
            }
        } else if (offset + 9u == reader->run_count) {
            if (symbol != '*') {
                result->status = BARCODE_PARTIAL;
                return;
            }
            result->text[output] = '\0';
            result->status = output ? BARCODE_OK : BARCODE_NO_BARCODE;
            return;
        } else if (symbol == '*') {
            result->status = BARCODE_DECODE_FAIL;
            return;
        } else if (output + 1u < BARCODE_MAX_TEXT) {
            result->text[output++] = symbol;
        } else {
            result->status = BARCODE_TRUNCATED;
            return;
        }
    }
    result->status = BARCODE_PARTIAL;
}

static uint32_t quiet_required(const barcode_reader_t *reader) {
    uint32_t quiet = reader->config.quiet_zone_us;
    if (reader->shortest_run_us != UINT32_MAX &&
        reader->shortest_run_us <= UINT32_MAX / 10u) {
        const uint32_t scaled = reader->shortest_run_us * 10u;
        if (scaled > quiet) quiet = scaled;
    }
    return quiet;
}

bool barcode_reader_process_edge(barcode_reader_t *reader, bool level,
                                 uint32_t timestamp_us, barcode_result_t *result) {
    if (level == reader->level) return false;
    const uint32_t elapsed = timestamp_us - reader->last_edge_us;
    bool completed = false;

    if (reader->collecting) {
        if (reader->level != reader->config.black_level &&
            elapsed >= quiet_required(reader)) {
            decode_capture(reader, result);
            clear_capture(reader);
            completed = true;
            if (level == reader->config.black_level)
                reader->collecting = true; /* The same edge begins the next scan. */
        } else if (elapsed < reader->config.min_run_us) {
            clear_capture(reader); /* Discard a pulse too short to be a bar. */
        } else if (reader->run_count == BARCODE_MAX_RUNS) {
            memset(result, 0, sizeof(*result));
            result->status = BARCODE_TRUNCATED;
            clear_capture(reader);
            completed = true;
        } else {
            reader->runs[reader->run_count++] = elapsed;
            if (elapsed < reader->shortest_run_us) reader->shortest_run_us = elapsed;
        }
    } else if (level == reader->config.black_level &&
               reader->level != reader->config.black_level &&
               elapsed >= reader->config.quiet_zone_us) {
        clear_capture(reader);
        reader->collecting = true;
    }

    reader->level = level;
    reader->last_edge_us = timestamp_us;
    return completed;
}

bool barcode_reader_poll(barcode_reader_t *reader, uint32_t now_us,
                         barcode_result_t *result) {
    if (!reader->collecting || reader->level == reader->config.black_level ||
        (uint32_t)(now_us - reader->last_edge_us) < quiet_required(reader))
        return false;
    decode_capture(reader, result);
    clear_capture(reader);
    return true;
}

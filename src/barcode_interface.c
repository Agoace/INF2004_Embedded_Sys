#include "barcode_interface.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#define BARCODE_MAX_RUNS 512u

typedef struct {
    char value;
    const char *pattern;
} code39_symbol_t;

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
    {'$', "nwnwnwnnn"}, {'/', "nwnwnnnwn"}, {'+', "nwnnnwnwn"},
    {'%', "nnnwnwnwn"}, {'*', "nwnnwnwnn"},
};

typedef struct {
    barcode_config_t config;
    uint32_t runs[BARCODE_MAX_RUNS];
    size_t run_count;
    uint32_t last_edge_us;
    uint32_t shortest_run_us;
    bool level;
    bool collecting;
} barcode_state_t;

static barcode_state_t reader;
static barcode_result_t pending_result;
static bool result_pending;

static void clear_capture(void) {
    reader.run_count = 0;
    reader.shortest_run_us = UINT32_MAX;
    reader.collecting = false;
}

static void find_shortest_run(void) {
    reader.shortest_run_us = UINT32_MAX;
    for (size_t i = 0; i < reader.run_count; ++i)
        if (reader.runs[i] < reader.shortest_run_us)
            reader.shortest_run_us = reader.runs[i];
}

static char decode_character(const uint32_t runs[9], uint32_t *narrow_max) {
    uint32_t sorted[9];
    char pattern[10];
    memcpy(sorted, runs, sizeof(sorted));
    for (size_t i = 1; i < 9; ++i) {
        const uint32_t value = sorted[i];
        size_t j = i;
        while (j > 0 && sorted[j - 1] > value) {
            sorted[j] = sorted[j - 1];
            --j;
        }
        sorted[j] = value;
    }
    if (sorted[5] == 0 || (uint64_t)sorted[6] * 2u < (uint64_t)sorted[5] * 3u)
        return '\0';
    const uint32_t split = sorted[5] + (sorted[6] - sorted[5]) / 2u;
    for (size_t i = 0; i < 9; ++i)
        pattern[i] = runs[i] > split ? 'w' : 'n';
    pattern[9] = '\0';
    *narrow_max = sorted[5];
    for (size_t i = 0; i < sizeof(CODE39) / sizeof(CODE39[0]); ++i)
        if (strcmp(pattern, CODE39[i].pattern) == 0)
            return CODE39[i].value;
    return '\0';
}

static uint32_t captured_run(size_t index, bool reverse) {
    return reader.runs[reverse ? reader.run_count - 1u - index : index];
}

static void decode_direction(bool reverse, barcode_result_t *result) {
    memset(result, 0, sizeof(*result));
    if (reader.run_count < 29u) {
        result->status = BARCODE_NO_BARCODE;
        return;
    }
    if ((reader.run_count - 9u) % 10u != 0u) {
        result->status = BARCODE_PARTIAL;
        return;
    }

    size_t output = 0;
    for (size_t offset = 0; offset + 9u <= reader.run_count; offset += 10u) {
        uint32_t symbol_runs[9];
        for (size_t i = 0; i < 9u; ++i)
            symbol_runs[i] = captured_run(offset + i, reverse);
        uint32_t narrow_max = 0;
        const char symbol = decode_character(symbol_runs, &narrow_max);
        if (symbol == '\0') {
            result->status = BARCODE_DECODE_FAIL;
            return;
        }
        if (offset > 0 && (uint64_t)captured_run(offset - 1u, reverse) >
                              (uint64_t)narrow_max * 5u) {
            result->status = BARCODE_DECODE_FAIL;
            return;
        }
        if (offset == 0) {
            if (symbol != '*') {
                result->status = BARCODE_PARTIAL;
                return;
            }
        } else if (offset + 9u == reader.run_count) {
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
        } else if (output + 1u < BARCODE_TEXT_MAX) {
            result->text[output++] = symbol;
        } else {
            result->status = BARCODE_TRUNCATED;
            return;
        }
    }
    result->status = BARCODE_PARTIAL;
}

static void decode_capture(barcode_result_t *result) {
    decode_direction(false, result);
    if (result->status == BARCODE_OK || reader.run_count < 29u)
        return;

    barcode_result_t reversed;
    decode_direction(true, &reversed);
    if (reversed.status == BARCODE_OK)
        *result = reversed;
}

static uint32_t quiet_required(void) {
    uint32_t quiet = reader.config.quiet_zone_us;
    if (reader.shortest_run_us != UINT32_MAX &&
        reader.shortest_run_us <= UINT32_MAX / 10u) {
        const uint32_t scaled = reader.shortest_run_us * 10u;
        if (scaled > quiet)
            quiet = scaled;
    }
    return quiet;
}

static void publish_result(const barcode_result_t *result) {
    if (result->status == BARCODE_NO_BARCODE || result_pending)
        return;
    pending_result = *result;
    result_pending = true;
}

static void finish_capture(void) {
    barcode_result_t result;
    decode_capture(&result);
    clear_capture();
    publish_result(&result);
}

void barcode_init(const barcode_config_t *config, bool initial_level,
                  uint32_t now_us) {
    memset(&reader, 0, sizeof(reader));
    if (config != NULL)
        reader.config = *config;
    reader.level = initial_level;
    reader.last_edge_us = now_us;
    clear_capture();
    result_pending = false;
}

void barcode_sample(bool level, uint32_t now_us) {
    if (reader.collecting && reader.level != reader.config.black_level &&
        (uint32_t)(now_us - reader.last_edge_us) >= quiet_required()) {
        finish_capture();
    }
    if (level == reader.level)
        return;

    const uint32_t elapsed = now_us - reader.last_edge_us;
    if (reader.collecting) {
        if (elapsed < reader.config.min_run_us) {
            if (reader.run_count == 0u) {
                clear_capture();
            } else {
                const uint32_t preceding_run = reader.runs[--reader.run_count];
                reader.last_edge_us -= preceding_run;
                find_shortest_run();
            }
        } else if (reader.run_count == BARCODE_MAX_RUNS) {
            barcode_result_t result = {.status = BARCODE_TRUNCATED};
            clear_capture();
            publish_result(&result);
        } else {
            reader.runs[reader.run_count++] = elapsed;
            if (elapsed < reader.shortest_run_us)
                reader.shortest_run_us = elapsed;
        }
    } else if (level == reader.config.black_level &&
               reader.level != reader.config.black_level &&
               elapsed >= reader.config.quiet_zone_us) {
        clear_capture();
        reader.collecting = true;
    }
    reader.level = level;
    reader.last_edge_us = now_us;
}

bool barcode_take_result(barcode_result_t *result) {
    if (!result_pending || result == NULL)
        return false;
    *result = pending_result;
    result_pending = false;
    return true;
}

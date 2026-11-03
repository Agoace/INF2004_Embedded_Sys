#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "barcode_interface.h"

static const barcode_config_t config = {
    .black_level = false,
    .min_run_us = 80u,
    .quiet_zone_us = 5000u,
};

static const char *pattern_for(char symbol) {
    switch (symbol) {
        case '*': return "nwnnwnwnn";
        case 'A': return "wnnnnwnnw";
        case 'F': return "nnwnwwnnn";
        case 'Z': return "nwwnwnnnn";
        case '8': return "wnnwnnwnn";
        default: assert(false); return "";
    }
}

static void feed_symbol(char symbol, uint32_t *time_us, bool *level, bool gap) {
    const char *pattern = pattern_for(symbol);
    for (unsigned i = 0; i < 9u; ++i) {
        *time_us += pattern[i] == 'w' ? 600u : 200u;
        *level = !*level;
        barcode_sample(*level, *time_us);
    }
    if (gap) {
        *time_us += 200u;
        *level = !*level;
        barcode_sample(*level, *time_us);
    }
}

static void test_barcode(char symbol, uint32_t start_us) {
    uint32_t time_us = start_us;
    bool level = true;
    barcode_init(&config, level, time_us);
    time_us += 10000u;
    level = false;
    barcode_sample(level, time_us);
    feed_symbol('*', &time_us, &level, true);
    feed_symbol(symbol, &time_us, &level, true);
    feed_symbol('*', &time_us, &level, false);
    barcode_sample(level, time_us + 10000u);

    barcode_result_t result;
    assert(barcode_take_result(&result));
    assert(result.status == BARCODE_OK);
    assert(result.text[0] == symbol && result.text[1] == '\0');
    assert(!barcode_take_result(&result));
}

int main(void) {
    for (const char *symbols = "AFZ8"; *symbols; ++symbols)
        test_barcode(*symbols, 0u);
    test_barcode('A', UINT32_MAX - 5000u);
    return 0;
}

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "barcode_reader.h"
#include "ir_line_data.h"

static const barcode_reader_config_t BARCODE_TEST_CONFIG = {
    .black_level = false,
    .min_run_us = 80,
    .quiet_zone_us = 5000,
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

static void feed_symbol(barcode_reader_t *reader, char symbol,
                        uint32_t *time_us, bool *level, bool gap) {
    const char *pattern = pattern_for(symbol);
    barcode_result_t result;
    for (unsigned i = 0; i < 9; ++i) {
        *time_us += pattern[i] == 'w' ? 600u : 200u;
        *level = !*level;
        assert(!barcode_reader_process_edge(reader, *level, *time_us, &result));
    }
    if (gap) {
        *time_us += 200u;
        *level = !*level;
        assert(!barcode_reader_process_edge(reader, *level, *time_us, &result));
    }
}

static void test_barcode(char symbol, uint32_t start_us) {
    barcode_reader_t reader;
    barcode_result_t result;
    uint32_t time_us = start_us;
    bool level = true; /* White is high in this test. */
    barcode_reader_init(&reader, &BARCODE_TEST_CONFIG, level, time_us);
    time_us += 10000u;
    level = false;
    assert(!barcode_reader_process_edge(&reader, level, time_us, &result));
    feed_symbol(&reader, '*', &time_us, &level, true);
    feed_symbol(&reader, symbol, &time_us, &level, true);
    feed_symbol(&reader, '*', &time_us, &level, false);
    assert(level); /* The final edge entered the trailing white space. */
    assert(!barcode_reader_poll(&reader, time_us + 1000u, &result));
    assert(barcode_reader_poll(&reader, time_us + 10000u, &result));
    assert(result.status == BARCODE_OK);
    assert(result.text[0] == symbol && result.text[1] == '\0');
    assert(result.edge_count == 29u);
    assert(!barcode_reader_poll(&reader, time_us + 20000u, &result));
}

static void test_incomplete_barcode(void) {
    barcode_reader_t reader;
    barcode_result_t result;
    uint32_t time_us = 0;
    bool level = true;
    barcode_reader_init(&reader, &BARCODE_TEST_CONFIG, level, time_us);
    time_us += 10000u;
    level = false;
    assert(!barcode_reader_process_edge(&reader, level, time_us, &result));
    feed_symbol(&reader, '*', &time_us, &level, true);
    feed_symbol(&reader, 'A', &time_us, &level, false);
    assert(barcode_reader_poll(&reader, time_us + 10000u, &result));
    assert(result.status != BARCODE_OK);
}

static void test_line_packet(void) {
    const uint16_t adc[IR_LINE_SENSOR_COUNT] = {0u, 4095u, 0xabcu};
    const line_class_t classes[IR_LINE_SENSOR_COUNT] = {
        LINE_UNKNOWN, LINE_BLACK, LINE_SENSOR_FAULT,
    };
    ir_line_data_t packet;
    uint16_t decoded_adc[IR_LINE_SENSOR_COUNT];
    line_class_t decoded_classes[IR_LINE_SENSOR_COUNT];
    assert(sizeof(packet) == 6u);
    assert(ir_line_data_encode(adc, classes, &packet));
    assert(ir_line_black_pattern(&packet) == 2u);
    int8_t steering = 99;
    assert(!ir_line_steering_error(&packet, &steering));
    const line_class_t centered[IR_LINE_SENSOR_COUNT] = {
        LINE_WHITE, LINE_BLACK, LINE_WHITE,
    };
    assert(ir_line_data_encode(adc, centered, &packet));
    assert(ir_line_steering_error(&packet, &steering) && steering == 0);
    assert(ir_line_data_encode(adc, classes, &packet));
    assert(ir_line_data_decode(&packet, decoded_adc, decoded_classes));
    assert(memcmp(adc, decoded_adc, sizeof(adc)) == 0);
    for (unsigned i = 0; i < IR_LINE_SENSOR_COUNT; ++i)
        assert(classes[i] == decoded_classes[i]);
    packet.bytes[5] |= 0x80u;
    assert(!ir_line_data_decode(&packet, decoded_adc, decoded_classes));
    const uint16_t out_of_range[IR_LINE_SENSOR_COUNT] = {4096u, 0u, 0u};
    assert(!ir_line_data_encode(out_of_range, classes, &packet));
}

int main(void) {
    for (const char *symbols = "AFZ8"; *symbols; ++symbols)
        test_barcode(*symbols, 0u);
    test_barcode('A', UINT32_MAX - 5000u); /* Timer wraps during the scan. */
    test_incomplete_barcode();
    test_line_packet();
    return 0;
}

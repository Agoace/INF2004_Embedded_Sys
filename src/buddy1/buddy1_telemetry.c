#include "buddy1_telemetry.h"
#include <stdbool.h>
#include <string.h>

static bool has_values(uint8_t quality)
{
    return quality == B1_TLM_VALID || quality == B1_TLM_STALE;
}

static bool valid_values(const b1_tlm_frame_t *frame)
{
    if (frame->line_quality > B1_TLM_FAULT ||
        frame->barcode_quality > B1_TLM_FAULT ||
        frame->pattern > 7 || frame->barcode > 1) {
        return false;
    }
    for (size_t i = 0; i < 3; ++i) {
        if (frame->adc[i] > 4095 ||
            (!has_values(frame->line_quality) && frame->adc[i] != 0)) {
            return false;
        }
    }
    return (has_values(frame->line_quality) || frame->pattern == 0) &&
           (has_values(frame->barcode_quality) || frame->barcode == 0);
}

b1_tlm_result_t b1_tlm_encode(const b1_tlm_frame_t *frame,
                             uint8_t *output, size_t length)
{
    if (frame == NULL || output == NULL) {
        return B1_TLM_BAD_ARGUMENT;
    }
    if (length != B1_TLM_BYTES) {
        return B1_TLM_BAD_LENGTH;
    }
    if (!valid_values(frame)) {
        return B1_TLM_BAD_VALUE;
    }
    output[0] = (uint8_t)((B1_TLM_VERSION << 6) |
                          ((unsigned int)frame->line_quality << 4) |
                          ((unsigned int)frame->barcode_quality << 2));
    output[1] = (uint8_t)(frame->session >> 24);
    output[2] = (uint8_t)(frame->session >> 16);
    output[3] = (uint8_t)(frame->session >> 8);
    output[4] = (uint8_t)frame->session;
    output[5] = (uint8_t)(frame->sequence >> 8);
    output[6] = (uint8_t)frame->sequence;
    output[7] = (uint8_t)(frame->timestamp_ms >> 24);
    output[8] = (uint8_t)(frame->timestamp_ms >> 16);
    output[9] = (uint8_t)(frame->timestamp_ms >> 8);
    output[10] = (uint8_t)frame->timestamp_ms;
    output[11] = (uint8_t)(frame->adc[0] >> 4);
    output[12] = (uint8_t)(((frame->adc[0] & 0x0fu) << 4) | (frame->adc[1] >> 8));
    output[13] = (uint8_t)frame->adc[1];
    output[14] = (uint8_t)(frame->adc[2] >> 4);
    output[15] = (uint8_t)(((frame->adc[2] & 0x0fu) << 4) |
                           ((unsigned int)frame->pattern << 1) | frame->barcode);
    return B1_TLM_OK;
}

b1_tlm_result_t b1_tlm_decode(const uint8_t *data, size_t length,
                             b1_tlm_frame_t *output)
{
    if (output != NULL) {
        memset(output, 0, sizeof(*output));
    }
    if (data == NULL || output == NULL) {
        return B1_TLM_BAD_ARGUMENT;
    }
    if (length != B1_TLM_BYTES) {
        return B1_TLM_BAD_LENGTH;
    }
    if ((data[0] >> 6) != B1_TLM_VERSION) {
        return B1_TLM_BAD_VERSION;
    }
    if ((data[0] & 0x03u) != 0) {
        return B1_TLM_BAD_PADDING;
    }
    b1_tlm_frame_t frame = {0};
    frame.line_quality = (uint8_t)((data[0] >> 4) & 0x03u);
    frame.barcode_quality = (uint8_t)((data[0] >> 2) & 0x03u);
    frame.session = ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) |
                    ((uint32_t)data[3] << 8) | data[4];
    frame.sequence = (uint16_t)(((uint16_t)data[5] << 8) | data[6]);
    frame.timestamp_ms = ((uint32_t)data[7] << 24) | ((uint32_t)data[8] << 16) |
                         ((uint32_t)data[9] << 8) | data[10];
    frame.adc[0] = (uint16_t)(((uint16_t)data[11] << 4) | (data[12] >> 4));
    frame.adc[1] = (uint16_t)(((data[12] & 0x0fu) << 8) | data[13]);
    frame.adc[2] = (uint16_t)(((uint16_t)data[14] << 4) | (data[15] >> 4));
    frame.pattern = (uint8_t)((data[15] >> 1) & 0x07u);
    frame.barcode = (uint8_t)(data[15] & 0x01u);
    if (!valid_values(&frame)) {
        return B1_TLM_BAD_VALUE;
    }
    *output = frame;
    return B1_TLM_OK;
}

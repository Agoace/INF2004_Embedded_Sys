#ifndef VEHICLE_LOG_H
#define VEHICLE_LOG_H
#include <stdint.h>
#define VR_BYTES 16u
#define VR_ROWS 4501u /* >= 90 seconds at 20 ms */
#define VR_MODULE_SCAN_REVERSE 5u /* KR1 only: SCAN is in its reverse leg */
typedef struct { uint8_t bytes[VR_BYTES]; } vr_row_t;
typedef struct {
    uint32_t elapsed_ms;
    uint16_t raw[2], echo;
    int16_t asked[2], measured[2];
    int8_t position, servo;
    uint8_t state, module, events;
} vr_values_t;
/* Bit offsets, little-endian bit order; signed fields are two's complement. */
enum { VR_TIME=0, VR_STATE=16, VR_MODULE=18, VR_L=21, VR_C=33,
       VR_POS=45, VR_ASK_L=53, VR_ASK_R=63, VR_MEAS_L=73, VR_MEAS_R=83,
       VR_ECHO=93, VR_SERVO=109, VR_EVENTS=116, VR_FLAGS=124 };
void vr_pack(vr_row_t *row, const vr_values_t *values);
#endif

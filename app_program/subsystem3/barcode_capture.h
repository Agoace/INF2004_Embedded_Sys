/* TEST-only storage; one core-0 line-task writer, one TEST reader. */
#ifndef BARCODE_CAPTURE_H
#define BARCODE_CAPTURE_H
#include "barcode.h"

typedef struct {
    uint32_t samples;
    uint16_t minimum;
    uint16_t maximum;
} barcode_range_t;

void barcode_range_begin(void);
void barcode_range_end(barcode_range_t *out);
void barcode_capture_begin(void);
void barcode_capture_freeze(void);
bool barcode_capture_active(void);
bool barcode_capture_full(void);
uint32_t barcode_capture_gaps(void);
uint32_t barcode_capture_motor_overflow(void);
/* Called once per acquired sample, after releasing the line mutex. */
void barcode_capture_observe(uint32_t us, uint16_t raw);
void barcode_capture_cost(uint32_t us);
uint32_t barcode_capture_max_us(void);
/* TEST owner appends distinct coherent motion snapshots, including two
 * post-freeze observations to bracket the final ADC sample. */
void barcode_capture_motor(const barcode_motion_t *motor);
const barcode_sample_t *barcode_capture_rows(size_t *count);
const barcode_motion_t *barcode_capture_motors(size_t *count);
#endif

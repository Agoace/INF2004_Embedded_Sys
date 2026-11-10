/* Post-stop black-HIGH peak observation; no hardware or vehicle actions. */
#ifndef BARCODE_PEAK_H
#define BARCODE_PEAK_H
#include "barcode.h"

#define BC_PEAK_BINS  2048u
#define BC_PEAK_LIMIT 96u
_Static_assert(BC_PEAK_LIMIT >= (BC_MAX_PAYLOAD + 2u) * 5u,
               "Peak capacity cannot hold the maximum framed payload");

typedef struct {
    barcode_config_t capture;
    uint16_t speed_mm_s; /* TIME axis conversion only; never uses encoders. */
    uint16_t white_min;
    uint16_t white_max;
} barcode_peak_config_t;

typedef struct {
    float centre;
    float left;
    float right;
    uint16_t height;
    uint16_t prominence;
} barcode_peak_t;

typedef struct {
    barcode_result_t value;
    float baseline;
    float noise_sd;
    float threshold;
    float minimum_prominence;
    float amplitude_margin;
    float geometry_error;
    uint16_t peaks;
} barcode_peak_method_t;

typedef struct {
    barcode_peak_method_t time;
    barcode_peak_method_t distance;
} barcode_peak_report_t;

/* Caller-owned fixed storage. No large automatic arrays or allocation.
 * Public for sizing only; fields below the arrays are private state. */
typedef struct {
    uint16_t bins[BC_PEAK_BINS];
    uint16_t noise_samples[21];
    barcode_peak_t peaks[BC_PEAK_LIMIT];
    const barcode_sample_t *raw;
    const barcode_packed_t *packed;
    const barcode_motion_t *motor;
    barcode_peak_report_t *report;
    barcode_peak_config_t config;
    barcode_frame_sink_t sink;
    void *context;
    size_t count, motor_count, index, motor_index, bin_count, scan;
    size_t first, cursor, stop, lobe_left, lobe_max;
    uint32_t origin_us, us, previous_us, sum;
    uint16_t members;
    unsigned int phase, axis, orientation;
    int direction;
    float position_origin, previous_position, extent;
    float lobe_sum, lobe_weight, threshold, narrow, wide;
    float frame_margin, frame_error, frame_separation, model_separation;
    bool motion_ok, lobe_active, scanning, preparing;
    char text[BC_MAX_PAYLOAD + 1u];
    uint8_t length;
} barcode_peak_workspace_t;

void barcode_peak_begin(const barcode_sample_t *rows, size_t count,
                        const barcode_packed_t *packed, uint32_t origin_us,
                        const barcode_motion_t *motor, size_t motor_count,
                        const barcode_peak_config_t *config,
                        barcode_peak_workspace_t *workspace,
                        barcode_peak_report_t *report);
/* Each step consumes <=32 records/bins or one character's finite table.
 * Immutable inputs throughout; caller yields between steps. */
bool barcode_peak_step(barcode_peak_workspace_t *workspace);
void barcode_peak_sink(barcode_peak_workspace_t *workspace,
                       barcode_frame_sink_t sink, void *context);
/* Accepted results must agree. Rejection is not a conflicting payload;
 * ambiguity in ANY path cannot be hidden by another accepted result. */
void barcode_peak_select(const barcode_report_t *width,
                         const barcode_peak_report_t *peak,
                         barcode_result_t *selected);
#endif

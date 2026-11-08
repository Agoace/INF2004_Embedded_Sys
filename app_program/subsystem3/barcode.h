/* Pure analog Code 39 observation. Never executes a vehicle command. */
#ifndef BARCODE_H
#define BARCODE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BC_MAX_SAMPLES 4096u
#define BC_MAX_MOTION  512u
#define BC_MAX_PAYLOAD 16u
#define BC_MAX_RUNS    192u
#define BC_SAMPLE_GAP  UINT16_C(0x0001)

typedef struct {
    uint32_t us;
    uint16_t raw;
    uint16_t flags;
} barcode_sample_t;

typedef struct {
    uint32_t us;
    int32_t left;
    int32_t right;
} barcode_motion_t;

typedef enum {
    BC_CMD_NONE,
    BC_CMD_LEFT,
    BC_CMD_RIGHT,
    BC_CMD_STRAIGHT,
    BC_CMD_UTURN
} barcode_command_t;

typedef enum {
    BC_OK,
    BC_NO_DATA,
    BC_CONTRAST,
    BC_SAMPLE_LOSS,
    BC_RUN_LIMIT,
    BC_NO_FRAME,
    BC_WIDTH,
    BC_PATTERN,
    BC_QUIET,
    BC_TRUNCATED,
    BC_PAYLOAD_LIMIT,
    BC_AMBIGUOUS,
    BC_MOTION
} barcode_reason_t;

typedef enum {
    BC_QUALITY_REJECTED,
    BC_QUALITY_DECODED
} barcode_quality_t;

typedef struct {
    char text[BC_MAX_PAYLOAD + 1u];
    uint16_t separation_permille; /* min wide / max narrow; not probability */
    uint16_t frames;
    uint8_t length;
    uint8_t directions; /* bit 0 normal print order, bit 1 reverse */
    barcode_command_t command;
    barcode_reason_t reason;
    barcode_quality_t quality;
    bool decoded;
} barcode_result_t;

typedef struct {
    barcode_result_t time;
    barcode_result_t distance;
    uint16_t minimum;
    uint16_t maximum;
    uint16_t contrast;
    uint16_t runs;
} barcode_report_t;

typedef struct {
    uint16_t min_contrast;
    uint32_t max_gap_us;
    uint32_t counts_per_m;
} barcode_config_t;

/* Incremental state is caller-owned and opaque to users. The union gives
 * the private state pointer/float alignment; access uses memcpy. */
typedef union {
    max_align_t alignment;
    unsigned char bytes[192];
} barcode_state_t;

typedef struct {
    float time_us[BC_MAX_RUNS];
    float distance_mm[BC_MAX_RUNS];
    barcode_state_t state;
} barcode_workspace_t;

_Static_assert(sizeof(barcode_workspace_t) + sizeof(barcode_report_t) <= 2048u,
               "Decoder workspace and report exceed 2 KiB");

/* Inputs must stay immutable until step returns true. Each step handles at
 * most 32 input records or one possible frame (at most 16 payload symbols).
 * Caller yields between steps. No allocation, hardware or kernel calls. */
void barcode_begin(const barcode_sample_t *samples, size_t sample_count,
                   const barcode_motion_t *motion, size_t motion_count,
                   const barcode_config_t *config,
                   barcode_workspace_t *workspace, barcode_report_t *report);
bool barcode_step(barcode_workspace_t *workspace);

extern const barcode_config_t barcode_default_config;
/* Full ordinary Code 39 alphabet (not extended/ASCII shift encoding).
 * Width methods are independent: both are always reported. Only a SINGLE
 * A/B/C/D maps to a command value; other valid strings decode with NONE.
 * A command value is data, never an instruction to the vehicle.
 * Distances interpolate bracketed, coherent 10 ms encoder observations.
 * No extrapolation, missing/gapped motion, or direction reversal is accepted
 * for the distance result. Hand-push slip remains unmeasured even on BC_OK.
 * Complete frames need 10 narrow-width white quiet zones at both ends.
 * Each call scans a bounded capture; samples and motion stay immutable.
 * Null/invalid arguments produce NO_DATA if report is nonnull. */
void barcode_decode(const barcode_sample_t *samples, size_t sample_count,
                    const barcode_motion_t *motion, size_t motion_count,
                    const barcode_config_t *config,
                    barcode_workspace_t *workspace, barcode_report_t *report);
const char *barcode_reason_name(barcode_reason_t reason);
#endif

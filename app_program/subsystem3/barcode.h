/* Pure analog Code 39 observation. Never executes a vehicle command. */
#ifndef BARCODE_H
#define BARCODE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BC_MAX_SAMPLES 6144u
#define BC_MAX_MOTION  512u
#define BC_MAX_PAYLOAD 16u
#define BC_MAX_RUNS    192u
#define BC_SAMPLE_GAP  UINT16_C(0x0001)
#define BC_PACKED_RAW  UINT16_C(0x0fff)
#define BC_PACKED_GAP  UINT16_C(0x8000)

typedef struct {
    uint16_t raw_flags; /* bits 0..11 ADC, bit 15 gap, bits 12..14 zero */
    uint16_t delta_us;  /* from previous retained sample; first is zero */
} barcode_packed_t;

_Static_assert(sizeof(barcode_packed_t) == 4u, "Packed ADC record changed");

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
    uint16_t separation_permille; /* bias-corrected ratio; not probability */
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
    barcode_result_t value;
    uint32_t start_us;
    uint32_t end_us;
    int32_t
        start_um; /* capture-relative encoder position, INT32_MIN if absent */
    int32_t end_um;
    bool distance_method;
} barcode_frame_t;

typedef void (*barcode_frame_sink_t)(const barcode_frame_t *, void *);

typedef struct {
    uint16_t min_contrast;
    uint32_t max_gap_us;
    uint32_t counts_per_m;
    uint32_t motion_gap_us;
} barcode_config_t;

/* Incremental state is caller-owned and opaque to users. The union gives
 * the private state pointer/float alignment; access uses memcpy. */
typedef union {
    max_align_t alignment;
    unsigned char bytes[224];
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
void barcode_begin_packed(const barcode_packed_t *samples, size_t count,
                          uint32_t origin_us, const barcode_motion_t *motion,
                          size_t motion_count, const barcode_config_t *config,
                          barcode_workspace_t *workspace,
                          barcode_report_t *report);
/* Set after begin and before step. Sink runs once per complete frame/method,
 * only in caller context. No input mutation or vehicle action is permitted. */
void barcode_frame_sink(barcode_workspace_t *workspace,
                        barcode_frame_sink_t sink, void *context);
bool barcode_step(barcode_workspace_t *workspace);

extern const barcode_config_t barcode_default_config;
/* Full ordinary Code 39 alphabet (not extended/ASCII shift encoding).
 * Partial-contrast levels target the measured black-HIGH optics.
 * Width methods are independent: both are always reported. Only a SINGLE
 * A/B/C/D maps to a command value; other valid strings decode with NONE.
 * A command value is data, never an instruction to the vehicle.
 * Distances interpolate bracketed, source-stamped encoder observations;
 * config.motion_gap_us bounds their permitted source-time gap.
 * No extrapolation, missing/gapped motion, or direction reversal is accepted
 * for the distance result. Hand-push slip remains unmeasured even on BC_OK.
 * Complete frames need 10 narrow-width white quiet zones at both ends.
 * Each call scans a bounded capture; samples and motion stay immutable.
 * Null/invalid arguments produce NO_DATA if report is nonnull. */
void barcode_decode(const barcode_sample_t *samples, size_t sample_count,
                    const barcode_motion_t *motion, size_t motion_count,
                    const barcode_config_t *config,
                    barcode_workspace_t *workspace, barcode_report_t *report);
/* Standard table access, shared by independent observation paths. */
char barcode_symbol(uint16_t mask);
uint16_t barcode_pattern(char symbol);
const char *barcode_reason_name(barcode_reason_t reason);
#endif

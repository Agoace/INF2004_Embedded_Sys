/* Pure one-shot junction action. No kernel or motor hardware calls. */
#ifndef JUNCTION_ACTION_H
#define JUNCTION_ACTION_H

#include <stdbool.h>
#include <stdint.h>
#include "../subsystem3/line_search.h"
#include "../subsystem5/obstacle.h"
#include "vehicle_inputs.h"

/* These values also identify the command in live event kind 7. */
typedef enum {
    JA_NONE = 0, JA_LEFT = 1, JA_RIGHT = 2, JA_STRAIGHT = 3,
    JA_UTURN = 4
} ja_command_id_t;

typedef enum {
    JA_IDLE = 0, JA_PENDING = 1, JA_APPROACH = 2, JA_SETTLE = 3,
    JA_TURN = 4, JA_TURN_SETTLE = 5, JA_REACQUIRE = 6,
    JA_DONE = 7, JA_FAILED = 8, JA_UNUSED = 9, JA_CROSS = 10
} ja_phase_t;

typedef enum {
    JA_OK = 0, JA_CANCELLED = 1, JA_STALE = 2, JA_STALL = 3,
    JA_APPROACH_LIMIT = 4, JA_TURN_LIMIT = 5, JA_NO_LINE = 6,
    JA_OBSTACLE = 7, JA_SONAR_FAULT = 8, JA_REVERSE = 9,
    JA_WHEEL_IMBALANCE = 10, JA_TOTAL_LIMIT = 11, JA_SETTLE_LIMIT = 12
} ja_reason_t;

/* Draft B2b source record. One producer, one consumer, one pending slot.
 * Read counts belong to one coherent motion observation. No bit fields. */
typedef struct {
    uint32_t epoch;
    uint32_t sequence;
    uint32_t read_ms;
    int32_t read_counts[2];
    uint8_t command;
    uint8_t valid;
    uint8_t source; /* 1 fake, 2 future live barcode; producer-owned. */
    uint8_t reserved;
} ja_command_t;

typedef struct {
    uint32_t epoch;
    uint32_t sequence;
    int32_t edge_counts[2];
    int32_t centre_counts[2];
    int32_t final_counts[2];
    int32_t turn_cdeg;
    uint32_t elapsed_ms;
    uint8_t command;
    uint8_t phase;
    uint8_t reason;
    uint8_t claimed;
    uint8_t source;
    uint8_t marks; /* Bit 0 edge, bit 1 centre, bit 2 final counts. */
    uint8_t reserved[2];
} ja_result_t;

typedef struct {
    uint32_t line_stamp;
    uint32_t line_age_ms;
    int16_t speed_mm_s[2];
    bool valid;
} ja_sample_t;

typedef struct {
    uint16_t axle_to_sensors_mm;
    uint16_t cross_width_mm;
} ja_config_t;

#define JA_AXLE_TO_SENSORS_MM 123u
#define JA_CROSS_WIDTH_MM 18u
#define JA_COUNTS_PER_M 12414u
#define JA_COUNTS_PER_DEG_X100 2405u
#define JA_CONFIRM_MM 6u
#define JA_BLACK_MIN 700u
#define JA_CROSS_EXIT_MM 160u
#define JA_FAIL_BASE 0xb20u

/* One owner; all fields below are private implementation state. */
typedef struct {
    ja_config_t cfg;
    ja_command_t command;
    ja_result_t result;
    ls_t search;
    obstacle_monitor_t obstacle;
    int32_t last_counts[2];
    int32_t clear_counts[2];
    uint32_t epoch;
    uint32_t consumed;
    uint32_t stamp;
    uint32_t start_ms;
    uint32_t phase_ms;
    uint32_t progress_ms[2];
    uint32_t stable_ms;
    uint32_t settled_ms;
    uint8_t phase;
    bool have_sample;
    bool clear;
    bool candidate;
    bool clear_started;
    bool stable;
    bool settled;
    bool searching;
} ja_t;

void ja_init(ja_t *state, const ja_config_t *cfg, uint32_t epoch);
/* False means invalid, obsolete, or busy; the pending slot is unchanged. */
bool ja_offer(ja_t *state, const ja_command_t *command);
void ja_suspend(ja_t *state);
bool ja_active(const ja_t *state);
/* True owns drive for this tick, including a terminal zero-speed tick. */
bool ja_step(ja_t *state, const vsm_inputs_t *in,
             const ja_sample_t *sample, uint32_t now_ms,
             vsm_drive_t *drive);
void ja_finish(ja_t *state, const int32_t counts[2], uint32_t now_ms);
const char *ja_reason_name(uint8_t reason);
void ja_print(const ja_result_t *result);

#endif

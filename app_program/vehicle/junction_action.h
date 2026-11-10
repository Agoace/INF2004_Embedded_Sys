/* Pure forced-corner job. LINE owns travel; SEARCH owns only the spin. */
#ifndef JUNCTION_ACTION_H
#define JUNCTION_ACTION_H

#include <stdbool.h>
#include <stdint.h>
#include "../subsystem3/line_search.h"
#include "../subsystem5/obstacle.h"
#include "vehicle_inputs.h"

typedef enum {
    JA_NONE = 0, JA_LEFT = 1, JA_RIGHT = 2, JA_STRAIGHT = 3,
    JA_UTURN = 4
} ja_command_id_t;

typedef enum {
    JA_IDLE = 0, JA_PENDING = 1, JA_BAR_CLEAR = 2,
    /* 3 and 5 are legacy settle phases, never emitted by this job. */
    JA_SEARCH = 4, JA_SECOND = 6, JA_DONE = 7, JA_FAILED = 8,
    JA_UNUSED = 9, JA_CROSS = 10
} ja_phase_t;

typedef enum {
    JA_OK = 0, JA_CANCELLED = 1, JA_STALE = 2, JA_STALL = 3,
    JA_TRAVEL_LIMIT = 4, JA_TURN_LIMIT = 5, JA_NO_LINE = 6,
    JA_OBSTACLE = 7, JA_SONAR_FAULT = 8, JA_REVERSE = 9,
    JA_WHEEL_IMBALANCE = 10, JA_TOTAL_LIMIT = 11
} ja_reason_t;

/* Draft B2b source: one producer, one slot; counts from one observation. */
typedef struct {
    uint32_t epoch;
    uint32_t sequence;
    uint32_t read_ms;
    int32_t read_counts[2];
    uint8_t command;
    uint8_t valid;
    uint8_t source; /* 1 fake, 2 future live barcode. */
    uint8_t reserved;
} ja_command_t;

#define JA_MARK_EDGE 0x01u
#define JA_MARK_SEARCH 0x02u
#define JA_MARK_FINAL 0x04u
#define JA_MARK_FIRST 0x08u
#define JA_MARK_ACCEPTED 0x10u

typedef struct {
    uint32_t epoch;
    uint32_t sequence;
    int32_t edge_counts[2];
    int32_t search_counts[2];
    int32_t final_counts[2];
    int32_t first_line_cdeg;
    int32_t accepted_line_cdeg;
    uint32_t elapsed_ms;
    uint8_t command;
    uint8_t phase;
    uint8_t reason;
    uint8_t claimed;
    uint8_t source;
    uint8_t marks;
    uint8_t reserved[2];
} ja_result_t;

typedef struct {
    uint32_t line_stamp;
    uint32_t line_age_ms;
    bool valid;
} ja_sample_t;

#define JA_COUNTS_PER_M 12414u
#define JA_COUNTS_PER_DEG_X100 2405u
#define JA_CONFIRM_MM 6u
#define JA_BLACK_MIN 700u
#define JA_BAR_CLEAR_MM 33u
#define JA_BAR_MAX_MM 70u
#define JA_CROSS_EXIT_MM 160u
#define JA_FAIL_BASE 0xb20u

/* Private owner state; no shared mutation or hardware calls. */
typedef struct {
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
    uint32_t search_ms;
    uint32_t progress_ms[2];
    uint32_t white_ms;
    int16_t search_speed_mm_s;
    uint8_t phase;
    uint8_t exit_samples;
    bool have_sample;
    bool clear;
    bool candidate;
    bool clear_started;
    bool white;
    bool departed;
} ja_t;

void ja_init(ja_t *state, int16_t search_speed_mm_s, uint32_t epoch);
bool ja_offer(ja_t *state, const ja_command_t *command);
void ja_suspend(ja_t *state);
bool ja_active(const ja_t *state);
bool ja_searching(const ja_t *state);
/* False leaves LINE's drive unchanged. True owns drive for this tick. */
bool ja_step(ja_t *state, const vsm_inputs_t *in,
             const ja_sample_t *sample, uint32_t now_ms,
             vsm_drive_t *drive);
void ja_abort(ja_t *state, const int32_t counts[2], uint32_t now_ms,
               uint8_t reason);
void ja_finish(ja_t *state, const int32_t counts[2], uint32_t now_ms);
const char *ja_reason_name(uint8_t reason);
void ja_print(const ja_result_t *result);

#endif

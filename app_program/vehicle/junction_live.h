/* Vehicle-owned live admission. No producer-board writes or motor calls. */
#ifndef JUNCTION_LIVE_H
#define JUNCTION_LIVE_H

#include "junction_source.h"
#include "../subsystem3/barcode_live_source.h"

typedef enum {
    JLIVE_NONE = 0, JLIVE_SHADOW, JLIVE_ACCEPTED, JLIVE_INVALID,
    JLIVE_OLD_RUN, JLIVE_LATE, JLIVE_ROUTE, JLIVE_CROSSED,
    JLIVE_BUSY, JLIVE_GUARD, JLIVE_UNARMED, JLIVE_STOP,
    JLIVE_EDGE, JLIVE_CONFIRMED, JLIVE_LINE_STALE, JLIVE_READ_FAILURE
} jlive_reason_t;

/* <=108 B: fits the existing telemetry value union. */
struct jlive_report {
    barcode_live_record_t frame;
    barcode_live_control_t control;
    uint32_t max_delivery_ms;
    int32_t clear_counts;
    int32_t encounter_counts[2];
    uint32_t encounter_stamp_us;
    uint8_t reason;
    uint8_t reserved[3];
};
_Static_assert(sizeof(jlive_report_t) <= 108u, "Live telemetry value size");

#define JLIVE_REASON_COUNT 16u
struct jlive_counts {
    uint32_t epoch;
    uint16_t reason[JLIVE_REASON_COUNT];
};

/* Control is stored once, not duplicated in the private raw cache. */
typedef struct {
    barcode_live_record_t frame;
    uint32_t max_delivery_ms;
    int32_t clear_counts;
    int32_t encounter_counts[2];
    uint32_t encounter_stamp_us;
    uint8_t reason;
    uint8_t reserved[3];
} jlive_cache_t;

typedef struct {
    jlive_cache_t report;
    barcode_live_control_t control;
    uint32_t run_epoch;
    uint16_t reasons[JLIVE_REASON_COUNT];
    int32_t current[2];
    int32_t previous[2];
    int32_t clear_start[2];
    int32_t edge[2];
    uint32_t now;
    uint32_t line_read_ms;
    uint32_t line_stamp;
    uint32_t edge_ms;
    uint32_t seen_sequence;
    uint8_t module;
    bool permission;
    bool on_black;
    bool have_sample;
    bool clear_started;
    bool clear;
    bool candidate;
    bool latched;
    bool waiting;
    bool report_ready;
} jlive_t;
/* Vehicle state + telemetry cache + both S3 boards + owner flags. */
_Static_assert(sizeof(jlive_t) + sizeof(jlive_report_t) +
               sizeof(jlive_counts_t) + 68u + 8u <= 384u,
               "Live admission static RAM ceiling");

void jlive_start(jlive_t *s, uint32_t epoch, uint32_t now);
void jlive_route(jlive_t *s, uint8_t module, uint32_t now);
void jlive_tick(jlive_t *s, const lp_t *line, const ja_sample_t *sample,
                  const int32_t counts[2], uint32_t now, ja_t *action,
                  bool busy);
/* Receipt time is sampled AFTER the whole coherent producer copy. */
bool jlive_read(jlive_t *s, const barcode_live_record_t *record,
                  uint32_t now, ja_command_t *out);
void jlive_disposition(jlive_t *s, uint32_t sequence, bool accepted);
void jlive_stop(jlive_t *s, uint32_t now);
bool jlive_report(jlive_t *s, jlive_report_t *out);
bool jlive_counts(const jlive_t *s, jlive_counts_t *out);
void jlive_print(const jlive_report_t *report);
void jlive_print_counts(const jlive_counts_t *counts);

#endif

/* Exclusive fake START backend or checked live-record adapter. */
#ifndef JUNCTION_SOURCE_H
#define JUNCTION_SOURCE_H

#include "junction_action.h"

/* Make maps NONE/A/B/C/D to these numeric C values. */
#ifndef JUNCTION_TEST_CMD
#define JUNCTION_TEST_CMD 0
#endif
#if JUNCTION_TEST_CMD < 0 || JUNCTION_TEST_CMD > 4
#error "JUNCTION_TEST_CMD must be 0..4 (Make: NONE/A/B/C/D)"
#endif
_Static_assert(JUNCTION_TEST_CMD >= 0 && JUNCTION_TEST_CMD <= 4,
               "JUNCTION_TEST_CMD must be numeric 0..4");
#define JUNCTION_TEST_CMD_NAME \
    (JUNCTION_TEST_CMD == 0 ? "NONE" : JUNCTION_TEST_CMD == 1 ? "A" : \
     JUNCTION_TEST_CMD == 2 ? "B" : JUNCTION_TEST_CMD == 3 ? "C" : "D")

#ifndef BARCODE_LIVE_ENABLE
#define BARCODE_LIVE_ENABLE 0
#endif
#ifndef BARCODE_MEASURED_CLEAR_MM
#define BARCODE_MEASURED_CLEAR_MM 0
#endif
#if BARCODE_LIVE_ENABLE != 0 && BARCODE_LIVE_ENABLE != 1
#error "BARCODE_LIVE_ENABLE must be 0 or 1"
#endif
#if BARCODE_MEASURED_CLEAR_MM < 0 || BARCODE_MEASURED_CLEAR_MM > 65535
#error "BARCODE_MEASURED_CLEAR_MM must be 0..65535"
#endif
#if BARCODE_LIVE_ENABLE && BARCODE_MEASURED_CLEAR_MM == 0
#error "Live actions need a measured clear bound, not shadow150"
#endif
#if BARCODE_LIVE_ENABLE && JUNCTION_TEST_CMD != 0
#error "Fake and live action backends are mutually exclusive"
#endif
_Static_assert(BARCODE_LIVE_ENABLE == 0 || BARCODE_LIVE_ENABLE == 1,
               "Live enable must be numeric 0 or 1");
_Static_assert(BARCODE_MEASURED_CLEAR_MM >= 0 &&
               BARCODE_MEASURED_CLEAR_MM <= 65535,
               "Measured clear bound must be numeric 0..65535");

#define JUNCTION_SHADOW_CLEAR_MM 150u

/* START resets vehicle-owned adapter state; never resets the S3 board. */
void junction_source_start(uint8_t command, uint32_t epoch, uint32_t now_ms,
                            const int32_t counts[2]);
/* One read attempt. False leaves output untouched. Never waits. */
bool junction_source_read(ja_command_t *out);
/* Vehicle-owned status. Fake ACK/busy access stays unchanged. Live ACK
 * occurs at offer/discard, not later motor consumption. S3 reads only the
 * coherent control board; it does not call these owner-private helpers. */
void junction_source_acknowledge(uint32_t sequence);
uint32_t junction_source_acknowledged(void);
void junction_source_set_busy(bool busy);
bool junction_source_busy(void);
const char *junction_source_name(uint8_t command);

/* Vehicle owner only. Route hooks include intermediate module transitions. */
typedef struct jlive_report jlive_report_t;
typedef struct jlive_counts jlive_counts_t;
void junction_source_route(uint8_t module, uint32_t now);
void junction_source_tick(uint8_t module, const lp_t *line,
                            const ja_sample_t *sample,
                            const int32_t counts[2], uint32_t now,
                            ja_t *action);
void junction_source_offer_result(uint32_t sequence, bool accepted);
void junction_source_stop(uint32_t now);
bool junction_source_report(jlive_report_t *out);
/* Stopped owner may resend counters before a USB DUMP after queue loss. */
bool junction_source_counts(jlive_counts_t *out);

#endif

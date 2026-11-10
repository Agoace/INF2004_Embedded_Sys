/* B2a fake backend. B2b replaces this producer, not the action consumer. */
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

/* Sole writer in B2a: vehicle owner at START. No live barcode input. */
void junction_source_start(uint8_t command, uint32_t epoch, uint32_t now_ms,
                            const int32_t counts[2]);
/* One read attempt. False leaves output untouched. Never waits. */
bool junction_source_read(ja_command_t *out);
/* Consumer-owned status. B2b must check busy again AFTER reading ACK,
 * and reject an unacknowledged slot; it must never overwrite a command.
 * This is not permission for a second concurrent board writer. */
void junction_source_acknowledge(uint32_t sequence);
uint32_t junction_source_acknowledged(void);
void junction_source_set_busy(bool busy);
bool junction_source_busy(void);
const char *junction_source_name(uint8_t command);

#endif

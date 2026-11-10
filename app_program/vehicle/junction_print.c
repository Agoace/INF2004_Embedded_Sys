/* Telemetry task only. Never print from the active action control path. */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "../app.h"
#include "junction_source.h"

void ja_print(const ja_result_t *r)
{
    if (!r || r->command < JA_LEFT || r->command > JA_UTURN) {
        return;
    }
    tm_printf((UB *)"[JUNCTION] source %s cmd %s epoch %lu seq %lu "
                    "phase %u reason %u (%s) claimed %u marks %u "
                    "edge %ld,%ld centre %ld,%ld end %ld,%ld "
                    "turn_cdeg %ld elapsed_ms %lu\n",
              r->source == 1u ? "FAKE" : "BARCODE",
              junction_source_name(r->command), (unsigned long)r->epoch,
              (unsigned long)r->sequence, (unsigned int)r->phase,
              (unsigned int)r->reason, ja_reason_name(r->reason),
              (unsigned int)r->claimed, (unsigned int)r->marks,
              (long)r->edge_counts[0],
              (long)r->edge_counts[1], (long)r->centre_counts[0],
              (long)r->centre_counts[1], (long)r->final_counts[0],
              (long)r->final_counts[1], (long)r->turn_cdeg,
              (unsigned long)r->elapsed_ms);
}

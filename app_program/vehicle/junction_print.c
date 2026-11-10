/* Telemetry task only. Never print from the active action control path. */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "../app.h"
#include "junction_source.h"
#include "junction_live.h"

void ja_print(const ja_result_t *r)
{
    if (!r || r->command < JA_LEFT || r->command > JA_UTURN) {
        return;
    }
    tm_printf((UB *)"[JUNCTION] source %s cmd %s epoch %lu seq %lu "
                    "phase %u reason %u (%s) claimed %u marks %u "
                    "edge %ld,%ld search_from %ld,%ld end %ld,%ld "
                    "first_line_cdeg %ld accepted_line_cdeg %ld "
                    "elapsed_ms %lu\n",
              r->source == 1u ? "FAKE" : "BARCODE",
              junction_source_name(r->command), (unsigned long)r->epoch,
              (unsigned long)r->sequence, (unsigned int)r->phase,
              (unsigned int)r->reason, ja_reason_name(r->reason),
              (unsigned int)r->claimed, (unsigned int)r->marks,
              (long)r->edge_counts[0],
              (long)r->edge_counts[1], (long)r->search_counts[0],
              (long)r->search_counts[1], (long)r->final_counts[0],
              (long)r->final_counts[1], (long)r->first_line_cdeg,
              (long)r->accepted_line_cdeg,
              (unsigned long)r->elapsed_ms);
}

void jlive_print(const jlive_report_t *r)
{
    if (!r || (r->control.epoch == 0u && r->frame.epoch == 0u)) {
        return;
    }
    tm_printf((UB *)"[BC_ADMIT] epoch %lu seq %lu reason %u letter %u "
                    "cmd %u valid %u source %u version %u flags %u dir %u\n",
              (unsigned long)r->frame.epoch,
              (unsigned long)r->frame.sequence, (unsigned int)r->reason,
              (unsigned int)r->frame.letter, (unsigned int)r->frame.command,
              (unsigned int)r->frame.valid, (unsigned int)r->frame.source,
              (unsigned int)r->frame.version, (unsigned int)r->frame.flags,
              (unsigned int)r->frame.direction);
    tm_printf((UB *)"[BC_EVIDENCE] epoch %lu seq %lu read_ms %lu "
                    "read_counts %ld,%ld frame_end %ld,%ld captured_rev %lu\n",
              (unsigned long)r->frame.epoch,
              (unsigned long)r->frame.sequence,
              (unsigned long)r->frame.read_ms,
              (long)r->frame.read_counts[0], (long)r->frame.read_counts[1],
              (long)r->frame.frame_end_counts[0],
              (long)r->frame.frame_end_counts[1],
              (unsigned long)r->frame.eligibility_revision);
    tm_printf((UB *)"[BC_CONTROL] epoch %lu rev %lu cut_ms %lu ack %lu "
                    "mode %lu clear_counts %ld edge %ld,%ld edge_us %lu "
                    "max_delivery_ms %lu measured_clear_mm %u "
                    "shadow_clear_mm 150 UNVERIFIED\n",
              (unsigned long)r->control.epoch,
              (unsigned long)r->control.eligibility_revision,
              (unsigned long)r->control.revision_ms,
              (unsigned long)r->control.ack_sequence,
              (unsigned long)r->control.mode, (long)r->clear_counts,
              (long)r->encounter_counts[0], (long)r->encounter_counts[1],
              (unsigned long)r->encounter_stamp_us,
              (unsigned long)r->max_delivery_ms,
              (unsigned int)BARCODE_MEASURED_CLEAR_MM);
}

void jlive_print_counts(const jlive_counts_t *c)
{
    if (!c || c->epoch == 0u) {
        return;
    }
    tm_printf((UB *)"[JUNCTION_COUNTS] epoch %lu none %u shadow %u "
                    "accepted %u invalid %u old_run %u late %u route %u "
                    "crossed %u\n",
              (unsigned long)c->epoch,
              (unsigned int)c->reason[JLIVE_NONE],
              (unsigned int)c->reason[JLIVE_SHADOW],
              (unsigned int)c->reason[JLIVE_ACCEPTED],
              (unsigned int)c->reason[JLIVE_INVALID],
              (unsigned int)c->reason[JLIVE_OLD_RUN],
              (unsigned int)c->reason[JLIVE_LATE],
              (unsigned int)c->reason[JLIVE_ROUTE],
              (unsigned int)c->reason[JLIVE_CROSSED]);
    tm_printf((UB *)"[JUNCTION_COUNTS] epoch %lu busy %u guard %u "
                    "unarmed %u stop %u edge %u confirmed %u "
                    "line_age10 %u read_failure %u saturate 65535\n",
              (unsigned long)c->epoch,
              (unsigned int)c->reason[JLIVE_BUSY],
              (unsigned int)c->reason[JLIVE_GUARD],
              (unsigned int)c->reason[JLIVE_UNARMED],
              (unsigned int)c->reason[JLIVE_STOP],
              (unsigned int)c->reason[JLIVE_EDGE],
              (unsigned int)c->reason[JLIVE_CONFIRMED],
              (unsigned int)c->reason[JLIVE_LINE_STALE],
              (unsigned int)c->reason[JLIVE_READ_FAILURE]);
}

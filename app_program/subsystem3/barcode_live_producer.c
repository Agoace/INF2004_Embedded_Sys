#include "barcode_live_producer.h"
#include <math.h>
#include <string.h>

uint32_t barcode_live_ms_at(uint32_t now_ms, uint32_t now_us, uint32_t us)
{
    const uint32_t age = now_us - us;
    const uint32_t phase = now_us - now_ms * 1000u;
    return now_ms - age / 1000u - (age % 1000u > phase ? 1u : 0u);
}

static int32_t signed_count(uint32_t value)
{
    int32_t result;
    memcpy(&result, &value, sizeof(result));
    return result;
}

static bool after(uint32_t value, uint32_t cut)
{
    const uint32_t delta = value - cut;
    return delta != 0u && delta < UINT32_C(0x80000000);
}

/* A changed coherent tag proves publication SINCE the previous successful
 * read, not necessarily since now. Use that earlier time conservatively.
 * First read and any >50ms observation pause need NEW progress to qualify. */
static bool heartbeat(barcode_live_producer_t *p,
                      const barcode_live_input_t *in)
{
    p->control_age_ms = p->seen ? in->now_ms - p->progress_ms : 0u;
    if (p->control_age_ms > p->max_control_age_ms &&
        ((in->control_ok && in->control.mode != BC_LIVE_DISABLED) ||
         (p->bound && !p->occupied))) {
        p->max_control_age_ms = p->control_age_ms;
    }
    if (!in->control_ok) {
        return false;
    }
    if (!p->seen || in->now_ms - p->success_ms >
                    BC_LIVE_CONTROL_MAX_AGE_MS) {
        p->qualified = false;
        p->progress_ms = in->now_ms;
    } else if (in->tag != p->tag) {
        p->qualified = true;
        p->progress_ms = p->success_ms;
    }
    p->seen = true;
    p->tag = in->tag;
    p->success_ms = in->now_ms;
    p->control_age_ms = in->now_ms - p->progress_ms;
    return p->qualified && p->control_age_ms <= BC_LIVE_CONTROL_MAX_AGE_MS;
}

static bool same_context(const barcode_live_control_t *a,
                          const barcode_live_control_t *b)
{
    return a->epoch == b->epoch &&
           a->eligibility_revision == b->eligibility_revision &&
           a->revision_ms == b->revision_ms;
}

static void evidence(barcode_live_producer_t *p,
                     const barcode_live_input_t *in, uint32_t reason)
{
    const barcode_window_t *w = &p->window;
    p->event = (barcode_live_frame_t){
        .record = {.epoch = p->context.epoch,
                   .eligibility_revision = p->context.eligibility_revision,
                   .source = BC_LIVE_SOURCE, .version = BC_LIVE_VERSION,
                   .letter = reason == BW_NOT_LETTER ?
                       (uint8_t)w->result.letter : 0u,
                   .flags = BC_LIVE_SHADOW},
        .reason = reason, .start_us = w->start_us,
        .control_age_ms = p->control_age_ms,
        .spin_white = w->calibration.white_mean,
        .leading_white = (uint32_t)lroundf(w->white_mean),
        .rise = w->rise, .fall = w->fall};
    (void)in;
    p->event_pending = true;
}

static void cancel(barcode_live_producer_t *p,
                   const barcode_live_input_t *in, uint32_t reason)
{
    if (p->bound) {
        evidence(p, in, reason);
        ++p->failures;
    }
    p->bound = false;
    p->window_live = false;
}

static void publish(barcode_live_producer_t *p,
                    const barcode_live_input_t *in)
{
    const barcode_window_result_t *r = &p->window.result;
    evidence(p, in, BW_OK);
    barcode_live_frame_t *e = &p->event;
    e->record_age_ms = in->now_ms -
        barcode_live_ms_at(in->now_ms, in->now_us, r->read_us);
    if (e->record_age_ms > p->max_record_age_ms) {
        p->max_record_age_ms = e->record_age_ms;
    }
    if (e->record_age_ms > BC_LIVE_RECORD_MAX_AGE_MS) {
        e->reason = BP_RECORD_AGE;
        p->window_live = false;
        p->bound = false;
        ++p->failures;
        return;
    }
    if (++p->sequence == 0u) {
        ++p->sequence;
    }
    e->record.sequence = p->sequence;
    e->record.read_ms =
        barcode_live_ms_at(in->now_ms, in->now_us, r->read_us);
    e->record.read_counts[0] = signed_count(r->read_left);
    e->record.read_counts[1] = signed_count(r->read_right);
    e->record.frame_end_counts[0] = signed_count(r->frame_left);
    e->record.frame_end_counts[1] = signed_count(r->frame_right);
    e->record.command = r->mapped_action;
    e->record.letter = (uint8_t)r->letter;
    e->record.direction = r->direction;
    e->record.flags |= BC_LIVE_DISTANCE | BC_LIVE_WINDOW29 |
                      BC_LIVE_LEADING_WHITE | BC_LIVE_TAIL_WHITE |
                      BC_LIVE_LOSS_FREE;
    if (barcode_window_table_confirmed()) {
        e->record.flags |= BC_LIVE_MAP_CONFIRMED;
    }
    /* Deliberately no OFFSET_VERIFIED, no valid1, even for control mode2. */
    e->edge_us = r->edge_us;
    e->frame_us = r->frame_us;
    e->eligible_us = r->eligible_us;
    e->narrow_mm = r->narrow_mm;
    e->wide_mm = r->wide_mm;
    e->leading_mm = r->leading_mm;
    p->slot = e->record;
    p->occupied = true;
    p->record_pending = true;
    ++p->publications;
}

void barcode_live_producer_observe(barcode_live_producer_t *p,
                                  const barcode_live_input_t *in)
{
    ++p->samples;
    if (in->control_ok && in->control.epoch != p->run_epoch) {
        p->run_epoch = in->control.epoch;
        p->max_control_age_ms = 0u;
        p->max_record_age_ms = 0u;
        p->seen = false;
        p->qualified = false;
    }
    const bool fresh = heartbeat(p, in);
    const bool lost = p->counters_seen &&
        (in->missed != p->missed || in->dropped != p->dropped);
    p->missed = in->missed;
    p->dropped = in->dropped;
    p->counters_seen = true;
    if (in->now_us - in->now_ms * 1000u >= 1000u) {
        cancel(p, in, BP_CLOCK);
        return;
    }
    if (p->occupied) {
        if (!fresh || in->control.ack_sequence != p->slot.sequence) {
            return;
        }
        /* Retirement never restamps/reoffers the old record. A mode0 ACK
         * keeps the reader disabled; only future eligible samples can lead. */
        p->occupied = false;
        p->window_live = false;
        p->bound = false;
    }
    if (!fresh || in->control.epoch == 0u ||
        in->control.mode == BC_LIVE_DISABLED ||
        in->control.mode > BC_LIVE_PERMITTED) {
        cancel(p, in, fresh ? BP_DISABLED : BP_CONTROL);
        return;
    }
    const barcode_cal_report_t *cal = in->calibration;
    if (!cal || (cal->mode != BC_CAL_LEARNED &&
                 cal->mode != BC_CAL_FALLBACK) ||
        (cal->faults & (BC_CAL_FAULT_RAW | BC_CAL_FAULT_TIME |
                       BC_CAL_FAULT_FULL)) != 0u) {
        cancel(p, in, BP_CALIBRATION);
        return;
    }
    if (p->window_live && (!same_context(&p->context, &in->control) ||
                          p->cal_generation != cal->generation)) {
        cancel(p, in, BP_CONTEXT);
    }
    if (lost) {
        cancel(p, in, BW_LOSS);
        return;
    }
    if (!in->motor_ok || in->now_us - in->motor_us > 30000u ||
        in->motor_faults != 0u) {
        cancel(p, in, BP_MOTOR_AGE);
        return;
    }
    if (!p->window_live) {
        if (!barcode_window_begin(&p->window, cal, NULL)) {
            return;
        }
        p->context = in->control;
        p->cal_generation = cal->generation;
        p->window_live = true;
    }
    const uint32_t serial = p->window.result.serial;
    barcode_window_motor(&p->window, in->motor_us, in->left, in->right,
                          in->motor_faults);
    barcode_window_adc(&p->window, in->adc_us, in->raw, 0u);
    if (p->bound && p->window.result.serial != serial) {
        cancel(p, in, p->window.result.reason);
        return;
    }
    for (unsigned int n = 0u; n < BW_STEP_MAX; ++n) {
        if (barcode_window_step(&p->window, 1u) == 0u) {
            break;
        }
        uint32_t start;
        if (!p->bound && barcode_window_candidate(&p->window, &start)) {
            p->context = in->control;
            p->bound = true;
            const uint32_t first_ms =
                barcode_live_ms_at(in->now_ms, in->now_us, start);
            if (!after(first_ms, p->context.revision_ms)) {
                cancel(p, in, BP_CUT);
                return;
            }
        }
        if (p->bound && p->window.result.serial != serial) {
            if (p->window.result.accepted) {
                publish(p, in);
            } else {
                cancel(p, in, p->window.result.reason);
            }
            return;
        }
    }
}

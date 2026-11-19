/* Single LINE-owner cache. Never used by the source publication recheck. */
#include "control.h"

static barcode_live_producer_t *owner;
static barcode_live_control_t last_good;
static uint32_t last_tag, disabled_since;
static bool have_good, suspended, completed;
uint32_t blc_control_read_misses;

/* Fail closed until the live vehicle's coherent SEARCH-only grace exists.
 * Standalone host tests override this just as the vehicle implementation does. */
__attribute__((weak))
bool blv_decode_grace_active(const barcode_live_control_t *control,
                             uint32_t now_ms)
{
    (void)control;
    (void)now_ms;
    return false;
}

static bool same_context(const barcode_live_control_t *a,
                         const barcode_live_control_t *b)
{
    return a->epoch == b->epoch &&
        a->eligibility_revision == b->eligibility_revision &&
        a->revision_ms == b->revision_ms;
}

void barcode_live_producer_observe(barcode_live_producer_t *p,
                                  const barcode_live_input_t *in)
{
    if (owner != p || p->samples == 0u) {
        owner = p;
        have_good = suspended = completed = false;
        blc_control_read_misses = 0u;
    }
    barcode_live_input_t effective = *in;
    const uint32_t success = p->success_ms;
    bool cached_read = false;
    if (in->control_ok) {
        last_good = in->control;
        last_tag = in->tag;
        have_good = true;
    } else {
        ++blc_control_read_misses;
        if (have_good && p->seen && p->qualified &&
            (!p->occupied || completed) &&
            in->now_ms - p->success_ms <= BC_LIVE_CONTROL_MAX_AGE_MS &&
            in->now_ms - p->progress_ms <= BC_LIVE_CONTROL_MAX_AGE_MS) {
            effective.control = last_good;
            effective.tag = last_tag;
            effective.control_ok = true;
            effective.control.ack_sequence = 0u;
            cached_read = true;
        }
    }
    const bool context_ok = same_context(&p->context, &effective.control);
    const bool grace = effective.control_ok && context_ok &&
        effective.control.mode == BC_LIVE_DISABLED &&
        blv_decode_grace_active(&effective.control, in->now_ms);
    const bool timed_out = suspended && in->now_ms - disabled_since >= 200u;

    if (completed) {
        /* Only update heartbeat/counters here; the unpublished slot cannot
         * be ACKed, even by a coherent input with a coincident sequence. */
        effective.control.ack_sequence = 0u;
        blc_producer_uncached(p, &effective);
        if (cached_read) p->success_ms = success;
        if (p->event_pending && p->event.reason != BW_OK) {
            /* A raw clock rejection must not be overwritten by release. */
            p->occupied = p->record_pending = false;
            --p->publications;
            suspended = completed = false;
            return;
        }
        const bool fresh = effective.control_ok && p->qualified &&
            p->control_age_ms <= BC_LIVE_CONTROL_MAX_AGE_MS;
        const uint32_t age = in->now_ms - p->slot.read_ms;
        if (age > p->max_record_age_ms) p->max_record_age_ms = age;
        if (fresh && context_ok && !timed_out &&
            age <= BC_LIVE_RECORD_MAX_AGE_MS && grace) {
            return;
        }
        if (fresh && context_ok && !timed_out && in->control_ok &&
            in->control.mode == BC_LIVE_PERMITTED &&
            age <= BC_LIVE_RECORD_MAX_AGE_MS) {
            p->event.record_age_ms = age;
            p->event.control_age_ms = p->control_age_ms;
            p->event_pending = p->record_pending = true;
        } else {
            p->event.reason = !fresh ? BP_CONTROL : !context_ok ? BP_CONTEXT :
                age > BC_LIVE_RECORD_MAX_AGE_MS ? BP_RECORD_AGE : BP_DISABLED;
            p->occupied = p->bound = p->window_live = false;
            p->record_pending = false;
            p->event_pending = true;
            --p->publications;
            ++p->failures;
        }
        suspended = completed = false;
        return;
    }

    const bool hold = p->bound && p->window_live && grace && !timed_out;
    /* Blip grace before the first bar: keep the SEEK white run, frozen.
     * The window ignores samples; motion loss still rejects it. */
    const bool seek_hold = !p->bound && p->window_live && grace &&
        barcode_window_seeking(&p->window);
    if (seek_hold) {
        barcode_window_freeze(&p->window, in->adc_us);
        effective.control.mode = BC_LIVE_PERMITTED;
        effective.control.ack_sequence = 0u;
    } else {
        barcode_window_thaw(&p->window, in->adc_us);
    }
    if (hold) {
        if (!suspended) disabled_since = in->now_ms;
        suspended = true;
        /* Only internal geometry/decoder work sees mode2. The source's real
         * control read is never intercepted. Quarantine any completed result. */
        effective.control.mode = BC_LIVE_PERMITTED;
        effective.control.ack_sequence = 0u;
    } else if (suspended && (timed_out || !context_ok ||
               !in->control_ok || in->control.mode != BC_LIVE_PERMITTED)) {
        effective.control.mode = BC_LIVE_DISABLED;
    }
    blc_producer_uncached(p, &effective);
    if (cached_read) p->success_ms = success;
    if (hold && p->record_pending) {
        p->event_pending = p->record_pending = false;
        completed = true;
    }
    if (!p->bound || !p->window_live || (!hold && !completed)) {
        suspended = false;
    }
}

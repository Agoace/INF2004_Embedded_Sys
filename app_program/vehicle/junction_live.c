#include "junction_live.h"
#include <limits.h>
#include <string.h>

static int32_t delta(int32_t a, int32_t b)
{
    return (int32_t)((uint32_t)a - (uint32_t)b);
}

static int64_t mean(const int32_t a[2], const int32_t b[2])
{
    return ((int64_t)delta(a[0], b[0]) + delta(a[1], b[1])) / 2;
}

static int64_t distance(unsigned int mm)
{
    return ((int64_t)mm * JA_COUNTS_PER_M + 999) / 1000;
}

static void cut(jlive_t *s, uint32_t when, bool bump)
{
    if ((int32_t)(when - s->control.revision_ms) > 0) {
        s->control.revision_ms = when;
    }
    if (bump) {
        ++s->control.eligibility_revision;
        if (s->control.eligibility_revision == 0u) {
            ++s->control.eligibility_revision;
        }
    }
}

static void count(jlive_t *s, uint8_t reason)
{
    if (reason < JLIVE_REASON_COUNT && s->reasons[reason] < UINT16_MAX) {
        ++s->reasons[reason];
    }
}

static void report(jlive_t *s, uint8_t reason)
{
    count(s, reason);
    s->report.reason = reason;
    s->report_ready = true;
}

static void retire(jlive_t *s, uint8_t reason)
{
    /* Boot-lifetime serials never reset on epoch/STOP/route. Thus an old
     * epoch's X can retire only X, never a newer slot X+1. */
    s->control.ack_sequence = s->report.frame.sequence;
    s->waiting = false;
    report(s, reason);
    (void)barcode_live_control_publish(&s->control);
}

void jlive_start(jlive_t *s, uint32_t epoch, uint32_t now)
{
    *s = (jlive_t){.run_epoch = epoch, .control = {.epoch = epoch,
        .eligibility_revision = 1u, .revision_ms = now}};
    (void)barcode_live_control_publish(&s->control);
}

void jlive_route(jlive_t *s, uint8_t module, uint32_t now)
{
    if (module == s->module) {
        return;
    }
    s->module = module;
    s->permission = false;
    cut(s, now, true);
    s->control.mode = BC_LIVE_DISABLED;
    s->have_sample = false;
    s->clear = false;
    s->clear_started = false;
    s->candidate = false;
    s->latched = false;
    if (s->waiting) {
        retire(s, JLIVE_ROUTE);
    }
    (void)barcode_live_control_publish(&s->control);
}

static bool black(const lp_t *line)
{
    return lp_junction(line) && line->norm[0] >= JA_BLACK_MIN &&
           line->norm[1] >= JA_BLACK_MIN;
}

static void encounter(jlive_t *s, const lp_t *line,
                        const ja_sample_t *sample, const int32_t counts[2],
                        uint32_t now, ja_t *action)
{
    const bool fresh = !s->have_sample || s->line_stamp != sample->line_stamp;
    const int32_t dl = s->have_sample ? delta(counts[0], s->previous[0]) : 0;
    const int32_t dr = s->have_sample ? delta(counts[1], s->previous[1]) : 0;
    memcpy(s->previous, counts, sizeof(s->previous));
    s->line_stamp = sample->line_stamp;
    s->have_sample = true;
    if (!fresh || dl < 0 || dr < 0 || (dl == 0 && dr == 0)) {
        s->candidate = false;
        s->clear_started = false;
        s->clear = false;
        return;
    }
    if (!black(line)) {
        s->candidate = false;
        if (lp_junction(line) || !lp_line_seen(line)) {
            s->clear_started = false;
            if (!lp_line_seen(line)) {
                s->clear = false;
            }
            return;
        }
        if (!s->clear_started) {
            memcpy(s->clear_start, counts, sizeof(s->clear_start));
            s->clear_started = true;
        }
        if (mean(counts, s->clear_start) >= distance(JA_CONFIRM_MM)) {
            s->clear = true;
            s->latched = false;
        }
        return;
    }
    s->clear_started = false;
    /* Even without a fresh observer baseline (for example just after
     * SEARCH), an unarmed live token must not survive this black stripe
     * and silently target a later junction. Fake source1 is unchanged. */
    if (action->phase == JA_PENDING && action->command.source == 2u &&
        !action->clear) {
        ja_finish(action, counts, now);
        report(s, JLIVE_UNARMED);
    }
    if (!s->clear || s->latched) {
        return;
    }
    if (!s->candidate) {
        memcpy(s->edge, counts, sizeof(s->edge));
        memcpy(s->report.encounter_counts, counts,
               sizeof(s->report.encounter_counts));
        s->edge_ms = now - sample->line_age_ms;
        s->report.encounter_stamp_us = sample->line_stamp;
        cut(s, s->edge_ms, false);
        s->candidate = true;
        report(s, JLIVE_EDGE);
    }
    if (mean(counts, s->edge) >= distance(JA_CONFIRM_MM)) {
        cut(s, s->edge_ms, true);
        s->latched = true;
        s->candidate = false;
        s->clear = false;
        if (s->waiting) {
            retire(s, JLIVE_CROSSED);
        } else {
            report(s, JLIVE_CONFIRMED);
        }
    }
}

static void travel(jlive_t *s)
{
    const int64_t value = mean(s->current,
                               s->report.frame.frame_end_counts);
    s->report.clear_counts = value > INT32_MAX ? INT32_MAX :
        value < INT32_MIN ? INT32_MIN : (int32_t)value;
}

void jlive_tick(jlive_t *s, const lp_t *line, const ja_sample_t *sample,
                  const int32_t counts[2], uint32_t now, ja_t *action,
                  bool busy)
{
    const bool usable = (s->control.epoch & 1u) != 0u &&
        s->module == VSM_MOD_LINE && !ja_searching(action) && sample->valid &&
        sample->line_age_ms <= 10u && lp_calibrated(line);
    if (usable) {
        encounter(s, line, sample, counts, now, action);
    } else {
        s->have_sample = false;
        s->clear_started = false;
        s->clear = false;
        s->candidate = false;
    }
    s->on_black = usable && black(line);
    const bool permission = usable && !busy;
    if (permission != s->permission) {
        cut(s, now, true);
        s->permission = permission;
    }
    s->now = now;
    s->line_read_ms = now - sample->line_age_ms;
    memcpy(s->current, counts, sizeof(s->current));
    travel(s);
    const uint32_t old_mode = s->control.mode;
    s->control.mode = permission && !s->on_black && !s->candidate &&
                      !s->latched && !s->waiting
        ? (BARCODE_LIVE_ENABLE ? BC_LIVE_PERMITTED : BC_LIVE_SHADOW_MODE)
        : BC_LIVE_DISABLED;
    if (old_mode == BC_LIVE_DISABLED && s->control.mode != BC_LIVE_DISABLED) {
        /* A delayed first bar from a whole missed disabled interval must
         * not acquire the newer context after eligibility returns. */
        cut(s, now, false);
    }
    /* Publication must advance even if all five words are unchanged. */
    (void)barcode_live_control_publish(&s->control);
}

static bool qualified(const barcode_live_record_t *r)
{
    return r->version == BC_LIVE_VERSION && r->source == BC_LIVE_SOURCE &&
        r->reserved == 0u && r->valid <= 1u && r->command <= JA_UTURN &&
        r->letter >= 'A' && r->letter <= 'Z' &&
        (r->direction == 1u || r->direction == 2u);
}

bool jlive_read(jlive_t *s, const barcode_live_record_t *record,
                  uint32_t now, ja_command_t *out)
{
    if (!out) {
        return false;
    }
    s->now = now;
    if (!s->waiting) {
        if (!record) {
            count(s, JLIVE_READ_FAILURE);
            return false;
        }
        if (record->sequence == 0u) {
            return false;
        }
        const barcode_live_record_t r = *record;
        if (r.epoch != s->control.epoch) {
            if (s->report.frame.epoch != r.epoch ||
                s->report.frame.sequence != r.sequence) {
                s->report.frame = r;
                retire(s, JLIVE_OLD_RUN);
            }
            return false;
        }
        if (s->seen_sequence != 0u &&
            (int32_t)(r.sequence - s->seen_sequence) <= 0) {
            return false;
        }
        s->seen_sequence = r.sequence;
        s->report.frame = r;
        travel(s);
        if (!qualified(&r)) {
            retire(s, JLIVE_INVALID);
            return false;
        }
        const uint32_t age = s->now - r.read_ms;
        if ((int32_t)age < 0) {
            retire(s, JLIVE_INVALID);
            return false;
        }
        if (age > s->report.max_delivery_ms) {
            s->report.max_delivery_ms = age;
        }
        if (age > BC_LIVE_RECORD_MAX_AGE_MS) {
            retire(s, JLIVE_LATE);
            return false;
        }
    }
    const barcode_live_record_t *r = &s->report.frame;
    if (s->module != VSM_MOD_LINE) {
        retire(s, JLIVE_ROUTE);
        return false;
    }
    if (s->now - s->line_read_ms > 10u) {
        retire(s, JLIVE_LINE_STALE);
        return false;
    }
    if (r->eligibility_revision != s->control.eligibility_revision ||
        (int32_t)(r->read_ms - s->control.revision_ms) <= 0 ||
        s->on_black || s->candidate || s->latched) {
        retire(s, JLIVE_CROSSED);
        return false;
    }
    if (!s->permission) {
        retire(s, JLIVE_BUSY);
        return false;
    }
    if (!BARCODE_LIVE_ENABLE || r->command == JA_NONE || r->valid == 0u) {
        retire(s, !BARCODE_LIVE_ENABLE ? JLIVE_SHADOW : JLIVE_NONE);
        return false;
    }
    if (r->flags != BC_LIVE_REQUIRED) {
        retire(s, JLIVE_INVALID);
        return false;
    }
    /* Delivery age is checked ONCE at receipt. A held clear-guard token
     * has no lifetime/course-gap expiry, but every invalidation retires it. */
    if (delta(s->current[0], r->frame_end_counts[0]) < 0 ||
        delta(s->current[1], r->frame_end_counts[1]) < 0 ||
        s->report.clear_counts < distance(BARCODE_MEASURED_CLEAR_MM)) {
        if (!s->waiting) {
            s->waiting = true;
            s->control.mode = BC_LIVE_DISABLED;
            report(s, JLIVE_GUARD);
            (void)barcode_live_control_publish(&s->control);
        }
        return false;
    }
    *out = (ja_command_t){.epoch = r->epoch, .sequence = r->sequence,
        .read_ms = r->read_ms,
        .read_counts = {r->read_counts[0], r->read_counts[1]},
        .command = r->command, .valid = 1u, .source = r->source,
        .reserved = r->reserved};
    return true;
}

void jlive_disposition(jlive_t *s, uint32_t sequence, bool accepted)
{
    if (sequence != s->report.frame.sequence ||
        sequence == s->control.ack_sequence) {
        return;
    }
    if (accepted) {
        s->control.mode = BC_LIVE_DISABLED;
        s->permission = false;
        cut(s, s->now, true);
    }
    /* retire publishes mode0 and the accepting ACK as one value. */
    retire(s, accepted ? JLIVE_ACCEPTED : JLIVE_BUSY);
}

void jlive_stop(jlive_t *s, uint32_t now)
{
    jlive_route(s, VSM_MOD_NONE, now);
    s->control.mode = BC_LIVE_DISABLED;
    report(s, JLIVE_STOP);
    s->control.epoch = 0u;
    (void)barcode_live_control_publish(&s->control);
}

bool jlive_report(jlive_t *s, jlive_report_t *out)
{
    if (!out || !s->report_ready) {
        return false;
    }
    *out = (jlive_report_t){.frame = s->report.frame,
        .control = s->control, .max_delivery_ms = s->report.max_delivery_ms,
        .clear_counts = s->report.clear_counts,
        .encounter_counts = {s->report.encounter_counts[0],
                             s->report.encounter_counts[1]},
        .encounter_stamp_us = s->report.encounter_stamp_us,
        .reason = s->report.reason};
    if (out->reason == JLIVE_STOP) {
        out->control.epoch = s->run_epoch;
    }
    s->report_ready = false;
    return true;
}

bool jlive_counts(const jlive_t *s, jlive_counts_t *out)
{
    if (!out || s->run_epoch == 0u || s->control.epoch != 0u) {
        return false;
    }
    out->epoch = s->run_epoch;
    memcpy(out->reason, s->reasons, sizeof(out->reason));
    return true;
}

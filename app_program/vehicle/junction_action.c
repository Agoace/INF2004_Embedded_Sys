/* Pure forced-corner action. No blind travel or encoder target turn. */
#include "junction_action.h"
#include <string.h>

#define JA_LINE_MAX_AGE_MS 50u
#define JA_CLEAR_MM 6u
#define JA_BAR_MS 1500u
#define JA_CROSS_MS 3000u
#define JA_STALL_MS 500u
#define JA_WHITE_MS 20u
#define JA_SPIN_TRANSLATION_MM 15u

static int32_t delta(int32_t value, int32_t origin)
{
    return (int32_t)((uint32_t)value - (uint32_t)origin);
}

static int64_t magnitude(int64_t value)
{
    return value < 0 ? -value : value;
}

static int64_t mean_from(const int32_t counts[2], const int32_t origin[2])
{
    return ((int64_t)delta(counts[0], origin[0]) +
            delta(counts[1], origin[1])) / 2;
}

static int64_t mm_counts(unsigned int mm)
{
    return ((int64_t)mm * JA_COUNTS_PER_M + 999) / 1000;
}

static int32_t angle_from(const int32_t counts[2], const int32_t origin[2])
{
    const int64_t diff = (int64_t)delta(counts[1], origin[1]) -
                        delta(counts[0], origin[0]);
    const int64_t angle = diff * 10000 / JA_COUNTS_PER_DEG_X100;
    if (angle > INT32_MAX) {
        return INT32_MAX;
    }
    if (angle < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)angle;
}

static void phase(ja_t *s, uint8_t next)
{
    s->phase = next;
    s->result.phase = next;
}

bool ja_searching(const ja_t *s)
{
    return s->phase == JA_SEARCH || s->phase == JA_SECOND;
}

bool ja_active(const ja_t *s)
{
    return s->phase == JA_BAR_CLEAR || s->phase == JA_CROSS ||
           ja_searching(s);
}

void ja_abort(ja_t *s, const int32_t counts[2], uint32_t now,
               uint8_t reason)
{
    s->result.reason = reason;
    s->result.marks |= JA_MARK_FINAL;
    memcpy(s->result.final_counts, counts, sizeof(s->result.final_counts));
    s->result.elapsed_ms = now - s->start_ms;
    phase(s, reason == JA_OK ? JA_DONE : JA_FAILED);
}

void ja_init(ja_t *s, int16_t speed, uint32_t epoch)
{
    *s = (ja_t){.search_speed_mm_s = speed, .epoch = epoch};
}

bool ja_offer(ja_t *s, const ja_command_t *command)
{
    if (!command || command->valid != 1u || command->command < JA_LEFT ||
        command->command > JA_UTURN || command->epoch != s->epoch ||
        (s->epoch & 1u) == 0u || command->sequence == 0u ||
        command->reserved != 0u || command->source < 1u ||
        command->source > 2u || s->command.valid != 0u || ja_active(s) ||
        (s->consumed != 0u &&
         (int32_t)(command->sequence - s->consumed) <= 0)) {
        return false;
    }
    s->command = *command;
    s->result = (ja_result_t){.epoch = command->epoch,
                             .sequence = command->sequence,
                             .command = command->command,
                             .phase = JA_PENDING,
                             .source = command->source};
    s->phase = JA_PENDING;
    ja_suspend(s);
    return true;
}

void ja_suspend(ja_t *s)
{
    s->have_sample = false;
    s->clear = false;
    s->candidate = false;
    s->clear_started = false;
}

static bool black(const vsm_inputs_t *in)
{
    return lp_junction(in->line) && in->line->norm[0] >= JA_BLACK_MIN &&
           in->line->norm[1] >= JA_BLACK_MIN;
}

/* Unchanged six-mm detector, including the weak leading-edge ramp. */
static void detect(ja_t *s, const vsm_inputs_t *in, bool fresh,
                   const int32_t movement[2], uint32_t now)
{
    if (!fresh || movement[0] < 0 || movement[1] < 0 ||
        (movement[0] == 0 && movement[1] == 0)) {
        s->candidate = false;
        s->clear_started = false;
        s->clear = false;
        return;
    }
    if (!black(in)) {
        s->candidate = false;
        if (lp_junction(in->line) || !lp_line_seen(in->line)) {
            s->clear_started = false;
            if (!lp_line_seen(in->line)) {
                s->clear = false;
            }
            return;
        }
        if (!s->clear_started) {
            memcpy(s->clear_counts, in->counts, sizeof(s->clear_counts));
            s->clear_started = true;
        }
        if (mean_from(in->counts, s->clear_counts) >=
            mm_counts(JA_CLEAR_MM)) {
            s->clear = true;
        }
        return;
    }
    s->clear_started = false;
    if (!s->clear) {
        return;
    }
    if (!s->candidate) {
        memcpy(s->result.edge_counts, in->counts,
               sizeof(s->result.edge_counts));
        s->candidate = true;
    }
    if (mean_from(in->counts, s->result.edge_counts) <
        mm_counts(JA_CONFIRM_MM)) {
        return;
    }
    s->consumed = s->command.sequence;
    s->command.valid = 0u;
    s->result.claimed = 1u;
    s->result.marks |= JA_MARK_EDGE;
    s->start_ms = now;
    s->exit_samples = 0u;
    obstacle_init(&s->obstacle);
    phase(s, s->result.command == JA_STRAIGHT ? JA_CROSS : JA_BAR_CLEAR);
}

static void search_begin(ja_t *s, const vsm_inputs_t *in, uint32_t now)
{
    const bool uturn = s->result.command == JA_UTURN;
    /* These helper extents are BEYOND the hard safety window. The job
     * stops first, so unchanged ls_* never starts its opposite sweep. */
    const ls_config_t cfg = {.speed_mm_s = s->search_speed_mm_s,
        .turn_counts = ((uturn ? 280 : 160) *
                       (int)JA_COUNTS_PER_DEG_X100) / 200};
    ls_init(&s->search, &cfg);
    ls_start(&s->search, s->result.command == JA_RIGHT
        ? VC_LINE_SIDE_RIGHT : VC_LINE_SIDE_LEFT, now);
    memcpy(s->result.search_counts, in->counts,
           sizeof(s->result.search_counts));
    s->result.marks |= JA_MARK_SEARCH;
    s->search_ms = now;
    s->progress_ms[0] = now;
    s->progress_ms[1] = now;
    s->white = false;
    s->departed = false;
    phase(s, JA_SEARCH);
}

static bool new_line(ja_t *s, const vsm_inputs_t *in, bool fresh,
                      uint32_t now)
{
    if (!fresh) {
        return false;
    }
    const bool white = in->line->norm[0] < LP_PERMILLE(LP_LOST_PCT) &&
                       in->line->norm[1] < LP_PERMILLE(LP_LOST_PCT);
    if (white) {
        if (!s->white) {
            s->white_ms = now;
            s->white = true;
        }
        if (now - s->white_ms >= JA_WHITE_MS) {
            s->departed = true;
        }
    } else {
        s->white = false;
    }
    return s->departed &&
           (in->line->norm[0] >= LP_PERMILLE(LP_FOUND_PCT) ||
            in->line->norm[1] >= LP_PERMILLE(LP_FOUND_PCT));
}

static void search_step(ja_t *s, const vsm_inputs_t *in, bool fresh,
                         const int32_t movement[2], uint32_t now,
                         vsm_drive_t *drive)
{
    const bool uturn = s->result.command == JA_UTURN;
    const int direction = s->result.command == JA_RIGHT ? -1 : 1;
    const int32_t angle = angle_from(in->counts, s->result.search_counts);
    const int64_t forward = (int64_t)angle * direction;
    if (forward >= (uturn ? 27000 : 15000) ||
        now - s->search_ms >= (uturn ? 6000u : 4000u)) {
        ja_abort(s, in->counts, now, JA_TURN_LIMIT);
        return;
    }
    if (forward < -300) {
        ja_abort(s, in->counts, now, JA_REVERSE);
        return;
    }
    if (magnitude(mean_from(in->counts, s->result.search_counts)) >
        mm_counts(JA_SPIN_TRANSLATION_MM)) {
        ja_abort(s, in->counts, now, JA_WHEEL_IMBALANCE);
        return;
    }
    /* Safety wins even if a qualifying line arrives on this tick. */
    for (unsigned int i = 0u; i < 2u; ++i) {
        const int turn = i == 0u ? -direction : direction;
        if ((int64_t)movement[i] * turn > 0) {
            s->progress_ms[i] = now;
        }
        if (now - s->progress_ms[i] >= JA_STALL_MS) {
            ja_abort(s, in->counts, now, JA_STALL);
            return;
        }
    }
    bool accept = false;
    if (new_line(s, in, fresh, now)) {
        if ((s->result.marks & JA_MARK_FIRST) == 0u) {
            s->result.first_line_cdeg = angle;
            s->result.marks |= JA_MARK_FIRST;
            if (uturn) {
                s->white = false;
                s->departed = false;
                phase(s, JA_SECOND);
            } else {
                accept = true;
            }
        } else if (forward < 12000) {
            ja_abort(s, in->counts, now, JA_NO_LINE);
            return;
        } else {
            accept = true;
        }
    }
    vsm_inputs_t search_in = *in;
    for (unsigned int i = 0u; i < 2u; ++i) {
        /* Hard angle/translation bounds above ensure small virtual counts;
         * ls_* never subtracts absolute wrapped encoder counters. */
        search_in.counts[i] = delta(in->counts[i],
                                     s->result.search_counts[i]);
    }
    search_in.line_event = accept
        ? vc_event(VC_EVENT_LINE_FOUND, 0u) : VC_EVENT_NONE;
    const vsm_outcome_t out = ls_step(&s->search, &search_in, now, drive);
    if (vsm_outcome_type(out) == VSM_OUT_FOUND) {
        s->result.accepted_line_cdeg = angle;
        s->result.marks |= JA_MARK_ACCEPTED;
        ja_abort(s, in->counts, now, JA_OK);
        return;
    }
    if (vsm_outcome_type(out) != VSM_OUT_RUNNING || s->search.step != 0u) {
        ja_abort(s, in->counts, now, JA_NO_LINE);
        return;
    }
}

bool ja_step(ja_t *s, const vsm_inputs_t *in, const ja_sample_t *sample,
             uint32_t now, vsm_drive_t *drive)
{
    if (s->phase != JA_PENDING && !ja_active(s)) {
        return false;
    }
    const bool was_active = ja_active(s);
    const bool fresh = !s->have_sample || s->stamp != sample->line_stamp;
    int32_t movement[2] = {0, 0};
    if (s->have_sample) {
        for (unsigned int i = 0u; i < 2u; ++i) {
            movement[i] = delta(in->counts[i], s->last_counts[i]);
        }
    }
    memcpy(s->last_counts, in->counts, sizeof(s->last_counts));
    s->stamp = sample->line_stamp;
    s->have_sample = true;
    if (!sample->valid || sample->line_age_ms > JA_LINE_MAX_AGE_MS ||
        !lp_calibrated(in->line)) {
        if (was_active) {
            ja_abort(s, in->counts, now, JA_STALE);
            *drive = (vsm_drive_t){0};
            return true;
        }
        ja_suspend(s);
        return false;
    }
    if (!was_active) {
        detect(s, in, fresh, movement, now);
        if (!ja_active(s)) {
            return false;
        }
    }
    const bool uturn = s->result.command == JA_UTURN;
    if (now - s->start_ms >= (uturn ? 8000u : 6000u)) {
        ja_abort(s, in->counts, now, JA_TOTAL_LIMIT);
    } else if (s->phase == JA_CROSS || s->phase == JA_BAR_CLEAR) {
        const int64_t travel = mean_from(in->counts, s->result.edge_counts);
        if (s->phase == JA_CROSS) {
            if (now - s->start_ms >= JA_CROSS_MS) {
                ja_abort(s, in->counts, now, JA_TRAVEL_LIMIT);
            } else if (travel >= mm_counts(JA_CROSS_EXIT_MM) && fresh &&
                       lp_line_seen(in->line) &&
                       (in->line->norm[0] >= LP_PERMILLE(LP_LOST_PCT) ||
                        in->line->norm[1] >= LP_PERMILLE(LP_LOST_PCT))) {
                ja_abort(s, in->counts, now, JA_OK);
            }
        } else if (travel > mm_counts(JA_BAR_MAX_MM) ||
                   now - s->start_ms >= JA_BAR_MS) {
            ja_abort(s, in->counts, now, JA_TRAVEL_LIMIT);
        } else if (fresh) {
            if (travel >= mm_counts(JA_BAR_CLEAR_MM) &&
                !lp_junction(in->line) &&
                (in->line->norm[0] < JA_BLACK_MIN ||
                 in->line->norm[1] < JA_BLACK_MIN)) {
                ++s->exit_samples;
                if (s->exit_samples >= 2u) {
                    search_begin(s, in, now);
                }
            } else {
                s->exit_samples = 0u;
            }
        }
    }
    const bool spin = ja_searching(s);
    if (spin) {
        *drive = (vsm_drive_t){0};
        if (in->echo_new) {
            const obstacle_event_t ev =
                obstacle_event(obstacle_update(&s->obstacle, in->echo));
            if (ev == OBSTACLE_EVENT_STOP ||
                ev == OBSTACLE_EVENT_SENSOR_FAULT) {
                ja_abort(s, in->counts, now, ev == OBSTACLE_EVENT_STOP
                    ? JA_OBSTACLE : JA_SONAR_FAULT);
            }
        }
        if (ja_searching(s)) {
            search_step(s, in, fresh, movement, now, drive);
        }
    }
    if (s->phase == JA_FAILED) {
        *drive = (vsm_drive_t){0};
        return true;
    }
    return spin;
}

void ja_finish(ja_t *s, const int32_t counts[2], uint32_t now)
{
    if (ja_active(s)) {
        ja_abort(s, counts, now, JA_CANCELLED);
    } else if (s->phase == JA_PENDING) {
        s->command.valid = 0u;
        s->consumed = s->command.sequence;
        memcpy(s->result.final_counts, counts,
               sizeof(s->result.final_counts));
        s->result.marks |= JA_MARK_FINAL;
        s->result.reason = JA_CANCELLED;
        phase(s, JA_UNUSED);
    }
}

const char *ja_reason_name(uint8_t reason)
{
    static const char *const names[] = {
        "ok", "cancelled", "line stale", "wheel stalled", "travel limit",
        "search window", "no/early line", "obstacle", "sonar fault",
        "reverse", "wheel imbalance", "total limit", "legacy settle"
    };
    return reason < sizeof(names) / sizeof(names[0])
               ? names[reason] : "unknown";
}

/* Tick-driven junction travel, turn and reacquisition. */
#include "junction_action.h"
#include <string.h>

#define JA_LINE_MAX_AGE_MS 50u
#define JA_CLEAR_MM 6u
#define JA_APPROACH_MS 3000u
#define JA_SETTLE_MS 500u
#define JA_STALL_MS 500u
#define JA_REACQUIRE_MS 3000u
#define JA_STABLE_MS 30u
#define JA_STOPPED_MS 100u
#define JA_SPIN_MM_S 80
#define JA_SLOW_MM_S 62
#define JA_FORWARD_MM_S 100

/* Encoder subtraction is modulo 2^32; each bounded leg is < 2^31 counts. */
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
    /* Physical action bounds keep this conversion inside int32_t. */
    const int64_t angle = diff * 10000 / JA_COUNTS_PER_DEG_X100;
    if (angle > INT32_MAX) {
        return INT32_MAX;
    }
    if (angle < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)angle;
}

static void phase(ja_t *s, uint8_t next, uint32_t now)
{
    s->phase = next;
    s->result.phase = next;
    s->phase_ms = now;
    s->progress_ms[0] = now;
    s->progress_ms[1] = now;
    s->stable = false;
    s->settled = false;
}

bool ja_active(const ja_t *s)
{
    return (s->phase >= JA_APPROACH && s->phase <= JA_REACQUIRE) ||
           s->phase == JA_CROSS;
}

static void complete(ja_t *s, const int32_t counts[2], uint32_t now,
                     uint8_t reason)
{
    s->result.reason = reason;
    s->result.marks |= 4u;
    memcpy(s->result.final_counts, counts, sizeof(s->result.final_counts));
    s->result.elapsed_ms = now - s->start_ms;
    if (s->result.claimed != 0u && s->phase != JA_CROSS &&
        s->phase != JA_APPROACH && s->phase != JA_SETTLE) {
        s->result.turn_cdeg = angle_from(counts, s->result.centre_counts);
    }
    phase(s, reason == JA_OK ? JA_DONE : JA_FAILED, now);
}

void ja_init(ja_t *s, const ja_config_t *cfg, uint32_t epoch)
{
    *s = (ja_t){0};
    s->cfg = *cfg;
    s->epoch = epoch;
}

bool ja_offer(ja_t *s, const ja_command_t *command)
{
    if (!command || command->valid != 1u || command->command < JA_LEFT ||
        command->command > JA_UTURN || command->epoch != s->epoch ||
        (s->epoch & 1u) == 0u || command->sequence == 0u ||
        command->reserved != 0u || command->source < 1u ||
        command->source > 2u ||
        s->command.valid != 0u || ja_active(s) ||
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

static bool outgoing(const vsm_inputs_t *in)
{
    /* CENTRED alone permits two 5% readings. Do not accept weak floor
     * during the detector's 50/300 ms LOST grace period. */
    return lp_line_seen(in->line) && lp_centred(in->line) &&
           !lp_junction(in->line) &&
           in->line->norm[0] >= LP_PERMILLE(LP_LOST_PCT) &&
           in->line->norm[1] >= LP_PERMILLE(LP_LOST_PCT);
}

static bool stable_line(ja_t *s, const vsm_inputs_t *in, bool fresh,
                        uint32_t now)
{
    if (!fresh) {
        return false;
    }
    if (!outgoing(in)) {
        s->stable = false;
        return false;
    }
    if (!s->stable) {
        s->stable = true;
        s->stable_ms = now;
    }
    return now - s->stable_ms >= JA_STABLE_MS;
}

static bool stopped(ja_t *s, const ja_sample_t *sample, uint32_t now)
{
    if (magnitude(sample->speed_mm_s[0]) > 20 ||
        magnitude(sample->speed_mm_s[1]) > 20) {
        s->settled = false;
        return false;
    }
    if (!s->settled) {
        s->settled = true;
        s->settled_ms = now;
    }
    return now - s->settled_ms >= JA_STOPPED_MS;
}

static void detect(ja_t *s, const vsm_inputs_t *in, bool fresh,
                   const int32_t movement[2], uint32_t now)
{
    if (!fresh || movement[0] < 0 || movement[1] < 0 ||
        (movement[0] == 0 && movement[1] == 0)) {
        s->candidate = false;
        s->clear_started = false;
        /* A stopped robot must establish a clear forward approach again. */
        s->clear = false;
        return;
    }
    if (!black(in)) {
        s->candidate = false;
        /* A threshold ramp is not a new clear baseline, but must not
         * disarm a previously clear approach before both reach 700. */
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
    s->result.marks |= 1u;
    s->start_ms = now;
    s->searching = false;
    obstacle_init(&s->obstacle);
    phase(s, s->result.command == JA_STRAIGHT ? JA_CROSS : JA_APPROACH,
          now);
}

/* A wheel must progress in its commanded direction at least every 0.5 s.
 * Settle phases use a separate time bound and do not ask for progress. */
static bool stall(ja_t *s, const int32_t movement[2], uint32_t now,
                  const vsm_drive_t *drive)
{
    const int16_t speed[2] = {drive->left_mm_s, drive->right_mm_s};
    for (unsigned int i = 0u; i < 2u; ++i) {
        if (speed[i] == 0) {
            s->progress_ms[i] = now;
        } else {
            if ((speed[i] > 0 && movement[i] > 0) ||
                (speed[i] < 0 && movement[i] < 0)) {
                s->progress_ms[i] = now;
            }
            if (now - s->progress_ms[i] >= JA_STALL_MS) {
                return true;
            }
        }
    }
    return false;
}

static void travel(ja_t *s, const vsm_inputs_t *in, bool fresh,
                    uint32_t now, vsm_drive_t *drive)
{
    const unsigned int centre_mm = (unsigned int)s->cfg.axle_to_sensors_mm +
                                  s->cfg.cross_width_mm / 2u;
    const int64_t distance = mean_from(in->counts, s->result.edge_counts);
    const int64_t left = delta(in->counts[0], s->result.edge_counts[0]);
    const int64_t right = delta(in->counts[1], s->result.edge_counts[1]);
    const bool cross = s->phase == JA_CROSS;
    const unsigned int exit_mm = cross ? JA_CROSS_EXIT_MM : centre_mm;
    if (cross && (s->result.marks & 2u) == 0u &&
        distance >= mm_counts(centre_mm)) {
        memcpy(s->result.centre_counts, in->counts,
               sizeof(s->result.centre_counts));
        s->result.marks |= 2u;
    }
    if (now - s->phase_ms >= JA_APPROACH_MS ||
        distance > mm_counts(exit_mm + 30u)) {
        complete(s, in->counts, now, JA_APPROACH_LIMIT);
    } else if (left < -mm_counts(2u) || right < -mm_counts(2u)) {
        complete(s, in->counts, now, JA_REVERSE);
    } else if (magnitude(right - left) > mm_counts(12u)) {
        complete(s, in->counts, now, JA_WHEEL_IMBALANCE);
    } else if (!cross && distance >= mm_counts(centre_mm)) {
        memcpy(s->result.centre_counts, in->counts,
               sizeof(s->result.centre_counts));
        s->result.marks |= 2u;
        phase(s, JA_SETTLE, now);
    } else if (cross && stable_line(s, in, fresh, now) &&
               distance >= mm_counts(exit_mm)) {
        /* C does not turn. This mark is its measured exit, not a pivot. */
        complete(s, in->counts, now, JA_OK);
    } else {
        const int16_t speed = !cross &&
            distance >= mm_counts(centre_mm > 20u ? centre_mm - 20u : 0u)
                ? JA_SLOW_MM_S : JA_FORWARD_MM_S;
        drive->left_mm_s = speed;
        drive->right_mm_s = speed;
    }
}

static void turn(ja_t *s, const vsm_inputs_t *in, uint32_t now,
                  vsm_drive_t *drive)
{
    const int32_t direction = s->result.command == JA_RIGHT ? -1 : 1;
    const int32_t target = s->result.command == JA_UTURN ? 18000 : 9000;
    const int32_t angle = angle_from(in->counts, s->result.centre_counts);
    const int64_t forward = mean_from(in->counts, s->result.centre_counts);
    const uint32_t limit = target == 18000 ? 6000u : 4000u;
    if (now - s->phase_ms >= limit ||
        (int64_t)angle * direction > target + 2000 ||
        magnitude(forward) > mm_counts(15u)) {
        complete(s, in->counts, now, JA_TURN_LIMIT);
    } else if ((int64_t)angle * direction < -300) {
        complete(s, in->counts, now, JA_REVERSE);
    } else if ((int64_t)angle * direction >= target) {
        phase(s, JA_TURN_SETTLE, now);
    } else {
        const int32_t speed = (int64_t)angle * direction >= target - 1500
                                 ? JA_SLOW_MM_S : JA_SPIN_MM_S;
        drive->left_mm_s = (int16_t)(-direction * speed);
        drive->right_mm_s = (int16_t)(direction * speed);
    }
}

static void acquire(ja_t *s, const vsm_inputs_t *in, bool fresh,
                     uint32_t now, vsm_drive_t *drive)
{
    const int32_t direction = s->result.command == JA_RIGHT ? -1 : 1;
    const int32_t target = s->result.command == JA_UTURN ? 18000 : 9000;
    const int32_t angle = angle_from(in->counts, s->result.centre_counts);
    const int64_t error = (int64_t)angle * direction - target;
    if (now - s->phase_ms >= JA_REACQUIRE_MS ||
        magnitude(error) > 1500 ||
        magnitude(mean_from(in->counts, s->result.centre_counts)) >
            mm_counts(15u)) {
        complete(s, in->counts, now, JA_NO_LINE);
        return;
    }
    if (outgoing(in)) {
        /* Stop before confirming the line; do not coast across it. */
        if (stable_line(s, in, fresh, now)) {
            complete(s, in->counts, now, JA_OK);
        }
        return;
    }
    s->stable = false;
    if (!s->searching) {
        /* Command +/-12 deg, with a hard +/-15 deg coast/error cap.
         * ls uses HALF the differential count, unlike angle_from(). */
        const ls_config_t cfg = {.speed_mm_s = JA_SLOW_MM_S,
            .turn_counts = (12 * (int)JA_COUNTS_PER_DEG_X100) / 200};
        ls_init(&s->search, &cfg);
        ls_start(&s->search,
                 direction > 0 ? VC_LINE_SIDE_LEFT : VC_LINE_SIDE_RIGHT,
                 now);
        /* Centre the search on the REQUESTED heading, not the coast
         * position. Virtual bounded counts keep unchanged ls arithmetic
         * away from absolute encoder wrap. */
        const int32_t diff = target * (int32_t)JA_COUNTS_PER_DEG_X100 /
                             10000;
        s->search.start[0] = -direction * (diff / 2);
        s->search.start[1] = direction * (diff - diff / 2);
        s->search.have_start = true;
        s->searching = true;
    }
    vsm_inputs_t search_in = *in;
    for (unsigned int i = 0u; i < 2u; ++i) {
        search_in.counts[i] = delta(in->counts[i],
                                     s->result.centre_counts[i]);
    }
    /* Only fresh, qualified level observations above can finish this job. */
    search_in.line_event = VC_EVENT_NONE;
    const vsm_outcome_t out = ls_step(&s->search, &search_in, now, drive);
    if (vsm_outcome_type(out) == VSM_OUT_FAILED) {
        complete(s, in->counts, now, JA_NO_LINE);
    }
}

static void settle(ja_t *s, const vsm_inputs_t *in,
                    const ja_sample_t *sample, uint32_t now)
{
    const unsigned int max_mm = (unsigned int)s->cfg.axle_to_sensors_mm +
                               s->cfg.cross_width_mm / 2u + 30u;
    if (now - s->phase_ms >= JA_SETTLE_MS) {
        complete(s, in->counts, now, JA_SETTLE_LIMIT);
        return;
    }
    if (s->phase == JA_SETTLE) {
        const int64_t left = delta(in->counts[0], s->result.edge_counts[0]);
        const int64_t right = delta(in->counts[1], s->result.edge_counts[1]);
        if (mean_from(in->counts, s->result.edge_counts) >
            mm_counts(max_mm)) {
            complete(s, in->counts, now, JA_APPROACH_LIMIT);
            return;
        }
        if (mean_from(in->counts, s->result.centre_counts) <
            -mm_counts(2u)) {
            complete(s, in->counts, now, JA_REVERSE);
            return;
        }
        if (magnitude(right - left) > mm_counts(12u)) {
            complete(s, in->counts, now, JA_WHEEL_IMBALANCE);
            return;
        }
    } else {
        const int32_t direction = s->result.command == JA_RIGHT ? -1 : 1;
        const int32_t target = s->result.command == JA_UTURN ? 18000 : 9000;
        const int32_t angle = angle_from(in->counts,
                                           s->result.centre_counts);
        if ((int64_t)angle * direction > target + 2000 ||
            magnitude(mean_from(in->counts, s->result.centre_counts)) >
                mm_counts(15u)) {
            complete(s, in->counts, now, JA_TURN_LIMIT);
            return;
        }
    }
    if (stopped(s, sample, now)) {
        if (s->phase == JA_SETTLE) {
            memcpy(s->result.centre_counts, in->counts,
                   sizeof(s->result.centre_counts));
            phase(s, JA_TURN, now);
        } else {
            phase(s, JA_REACQUIRE, now);
        }
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
            complete(s, in->counts, now, JA_STALE);
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
    *drive = (vsm_drive_t){0};
    const uint32_t total = s->result.command == JA_UTURN ? 12000u : 10000u;
    if (now - s->start_ms >= total) {
        complete(s, in->counts, now, JA_TOTAL_LIMIT);
        return true;
    }
    if (in->echo_new) {
        const obstacle_event_t ev =
            obstacle_event(obstacle_update(&s->obstacle, in->echo));
        if (ev == OBSTACLE_EVENT_STOP ||
            ev == OBSTACLE_EVENT_SENSOR_FAULT) {
            complete(s, in->counts, now, ev == OBSTACLE_EVENT_STOP
                ? JA_OBSTACLE : JA_SONAR_FAULT);
            return true;
        }
    }
    switch (s->phase) {
    case JA_APPROACH:
    case JA_CROSS:
        travel(s, in, fresh, now, drive);
        break;
    case JA_SETTLE:
    case JA_TURN_SETTLE:
        settle(s, in, sample, now);
        break;
    case JA_TURN:
        turn(s, in, now, drive);
        break;
    case JA_REACQUIRE:
        acquire(s, in, fresh, now, drive);
        break;
    default:
        break;
    }
    if (ja_active(s) && stall(s, movement, now, drive)) {
        complete(s, in->counts, now, JA_STALL);
    }
    if (s->phase == JA_DONE || s->phase == JA_FAILED) {
        *drive = (vsm_drive_t){0};
    }
    return true;
}

void ja_finish(ja_t *s, const int32_t counts[2], uint32_t now)
{
    if (ja_active(s)) {
        complete(s, counts, now, JA_CANCELLED);
    } else if (s->phase == JA_PENDING) {
        s->command.valid = 0u;
        s->consumed = s->command.sequence;
        memcpy(s->result.final_counts, counts,
               sizeof(s->result.final_counts));
        s->result.marks |= 4u;
        s->result.phase = JA_UNUSED;
        s->phase = JA_UNUSED;
        s->result.reason = JA_CANCELLED;
    }
}

const char *ja_reason_name(uint8_t reason)
{
    static const char *const names[] = {
        "ok", "cancelled", "line stale", "wheel stalled",
        "approach limit", "turn limit", "no outgoing line", "obstacle",
        "sonar fault", "reverse", "wheel imbalance", "total limit",
        "settle limit"
    };
    return reason < sizeof(names) / sizeof(names[0])
               ? names[reason] : "unknown";
}

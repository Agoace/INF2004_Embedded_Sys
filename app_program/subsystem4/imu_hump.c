/* Pure hump events. Report only; never a driving input. */
#include "imu_hump.h"
#include "imu_settings.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

void imu_hump_init(imu_hump_t *s)
{
    if (s != NULL) {
        memset(s, 0, sizeof(*s));
    }
}

static void clear_window(imu_hump_t *s)
{
    s->samples = 0u;
    s->index = 0u;
    s->sum = 0.0f;
    s->output.smooth_ready = false;
}

static void clear_event(imu_hump_t *s, imu_hump_state_t state)
{
    s->output.state = (uint8_t)state;
    s->output.event_peak_mm = 0.0f;
    s->output.event_travel_mm = 0.0f;
    s->output.event_left = false;
    s->below_mm = 0.0f;
}

static void close_event(imu_hump_t *s, bool open)
{
    imu_hump_output_t *o = &s->output;

    if (o->count == 0u || o->event_peak_mm > o->max_peak_mm) {
        o->max_peak_mm = o->event_peak_mm;
        o->max_left = o->event_left;
    }
    if (o->count != UINT16_MAX) {
        ++o->count;
    }
    o->last_peak_mm = o->event_peak_mm;
    o->last_left = o->event_left;
    o->flags = (uint8_t)((o->flags & (uint8_t)~IMU_HUMP_FLAG_LEFT) |
                         (o->max_left ? IMU_HUMP_FLAG_LEFT : 0u) |
                         (open ? IMU_HUMP_FLAG_OPEN : 0u));
    clear_event(s, IMU_HUMP_IDLE);
}

static void end_run(imu_hump_t *s)
{
    if (s->output.state == (uint8_t)IMU_HUMP_ACTIVE) {
        close_event(s, true);
    }
    clear_event(s, IMU_HUMP_IDLE);
    s->output.running = false;
}

static void begin_run(imu_hump_t *s, uint32_t epoch)
{
    imu_hump_output_t *o = &s->output;
    const float smoothed = o->smoothed_mm;
    const bool ready = o->smooth_ready;

    memset(o, 0, sizeof(*o));
    o->smoothed_mm = smoothed;
    o->smooth_ready = ready;
    o->run_epoch = epoch;
    o->running = true;
    o->flags = IMU_HUMP_FLAG_IMPLEMENTED;
    s->below_mm = 0.0f;
    s->gate_latched = false;
}

static void count_gated(imu_hump_t *s)
{
    if (!s->gate_latched && s->output.gated_starts != UINT16_MAX) {
        ++s->output.gated_starts;
    }
    s->gate_latched = true;
}

/* Travel below OFF must be continuous; any value at/above OFF restarts. */
static bool below_done(imu_hump_t *s, float value, float travel)
{
    if (value >= IMU_HUMP_OFF_MM) {
        s->below_mm = 0.0f;
        return false;
    }
    s->below_mm += travel;
    return s->below_mm >= IMU_HUMP_OFF_TRAVEL_MM;
}

static void detect(imu_hump_t *s, const imu_hump_input_t *in)
{
    imu_hump_output_t *o = &s->output;
    const float travel = isfinite(in->travel_mm) ? fabsf(in->travel_mm)
                                                  : 0.0f;
    const bool motion = in->motion_valid && isfinite(in->speed_mm_s) &&
                        isfinite(in->yaw_rate_rad_s) &&
                        isfinite(in->lateral_mg);
    const bool curve = !motion ||
                       fabsf(in->lateral_mg) > IMU_HUMP_GATE_LATERAL_MG ||
                       fabsf(in->yaw_rate_rad_s) > IMU_HUMP_GATE_YAW_RAD_S;
    const bool blocked = curve ||
                         fabsf(in->speed_mm_s) < IMU_HUMP_MIN_SPEED_MM_S;
    const float magnitude = fabsf(o->smoothed_mm);
    const float side = o->event_left ? -o->smoothed_mm : o->smoothed_mm;

    switch ((imu_hump_state_t)o->state) {
    case IMU_HUMP_IDLE:
        if (magnitude < IMU_HUMP_ON_MM) {
            if (magnitude < IMU_HUMP_OFF_MM) {
                s->gate_latched = false;
            }
        } else if (blocked) {
            count_gated(s);
        } else {
            clear_event(s, IMU_HUMP_CANDIDATE);
            o->event_left = o->smoothed_mm < 0.0f;
            o->event_peak_mm = magnitude;
        }
        break;
    case IMU_HUMP_CANDIDATE:
        if (side < IMU_HUMP_ON_MM || blocked) {
            if (blocked && side >= IMU_HUMP_ON_MM) {
                count_gated(s);
            }
            clear_event(s, IMU_HUMP_IDLE);
            break;
        }
        o->event_travel_mm += travel;
        o->event_peak_mm = fmaxf(o->event_peak_mm, side);
        if (o->event_travel_mm >= IMU_HUMP_ON_TRAVEL_MM) {
            o->state = (uint8_t)IMU_HUMP_ACTIVE;
        }
        break;
    case IMU_HUMP_ACTIVE:
        o->event_travel_mm += travel;
        if (curve) {
            o->flags |= IMU_HUMP_FLAG_CURVE;
        } else {
            o->event_peak_mm = fmaxf(o->event_peak_mm, side);
        }
        if (o->event_travel_mm > IMU_HUMP_MAX_TRAVEL_MM) {
            if (o->long_rejects != UINT16_MAX) {
                ++o->long_rejects;
            }
            o->flags |= IMU_HUMP_FLAG_LONG;
            clear_event(s, IMU_HUMP_LONG);
        } else if (below_done(s, side, travel)) {
            close_event(s, false);
        }
        break;
    case IMU_HUMP_LONG:
    default:
        if (below_done(s, magnitude, travel)) {
            clear_event(s, IMU_HUMP_IDLE);
        }
        break;
    }
}

void imu_hump_step(imu_hump_t *s, const imu_hump_input_t *in)
{
    if (s == NULL || in == NULL) {
        return;
    }
    imu_hump_output_t *o = &s->output;
    const bool run = (in->run_epoch & 1u) != 0u;

    if (run && (!o->running || in->run_epoch != o->run_epoch)) {
        if (o->running) {
            end_run(s);
        }
        begin_run(s, in->run_epoch);
    } else if (!run && o->running) {
        end_run(s);
    }
    if (!in->valid || !isfinite(in->height_mm)) {
        clear_window(s);
        if (o->running) {
            o->flags |= IMU_HUMP_FLAG_FAULT;
        }
        return;
    }
    if (o->running && o->valid_samples != UINT32_MAX) {
        ++o->valid_samples;
    }
    if (s->samples == IMU_HUMP_MEAN_SAMPLES) {
        s->sum -= s->window[s->index];
    } else {
        ++s->samples;
    }
    s->window[s->index] = in->height_mm;
    s->sum += in->height_mm;
    s->index = (s->index + 1u) % IMU_HUMP_MEAN_SAMPLES;
    o->smooth_ready = s->samples == IMU_HUMP_MEAN_SAMPLES;
    if (!o->smooth_ready) {
        return;
    }
    o->smoothed_mm = s->sum / (float)IMU_HUMP_MEAN_SAMPLES;
    if (o->running) {
        detect(s, in);
    }
}

uint16_t imu_hump_tenths(float mm)
{
    if (!isfinite(mm) || mm <= 0.0f) {
        return 0u;
    }
    const float scaled = mm * 10.0f + 0.5f;

    return scaled >= (float)IMU_HUMP_PEAK_MAX ? (uint16_t)IMU_HUMP_PEAK_MAX
                                               : (uint16_t)scaled;
}

uint8_t imu_hump_detail(uint16_t number, bool left)
{
    const uint16_t n = number > IMU_HUMP_NUMBER_MAX
                           ? (uint16_t)IMU_HUMP_NUMBER_MAX : number;

    return (uint8_t)(n | (left ? IMU_HUMP_DETAIL_LEFT : 0u));
}

static imu_hump_summary_t unavailable(void)
{
    return (imu_hump_summary_t){
        .peak_01mm = (uint16_t)IMU_HUMP_PEAK_NONE,
        .count = 0u,
        .flags = IMU_HUMP_FLAG_IMPLEMENTED | IMU_HUMP_FLAG_FAULT,
        .gated = 0u};
}

void imu_hump_pack(imu_hump_board_t *out, const imu_hump_output_t *o,
                   uint32_t timestamp_ms)
{
    /* The float decisions of the summary, made once with full precision. */
    float peak = o->max_peak_mm;
    bool left = o->max_left;
    const uint16_t closed = o->count;
    uint16_t count = closed;
    uint8_t flags = (uint8_t)(o->flags | IMU_HUMP_FLAG_IMPLEMENTED);
    const bool open = o->running && o->state == (uint8_t)IMU_HUMP_ACTIVE;

    if (open) {
        if (count == 0u || o->event_peak_mm > peak) {
            peak = o->event_peak_mm;
            left = o->event_left;
        }
        if (count != UINT16_MAX) {
            ++count;
        }
        flags |= IMU_HUMP_FLAG_OPEN;
    }
    flags = (uint8_t)((flags & (uint8_t)~IMU_HUMP_FLAG_LEFT) |
                      (count != 0u && left ? IMU_HUMP_FLAG_LEFT : 0u));
    const uint32_t best = count == 0u ? 0u : imu_hump_tenths(peak);
    const uint32_t gated = o->gated_starts > IMU_HUMP_GATED_MAX
                               ? IMU_HUMP_GATED_MAX : o->gated_starts;

    out->word[0] = timestamp_ms;
    out->word[1] = o->run_epoch;
    out->word[2] = (best << IMU_HUMP_W2_BEST_SHIFT) |
                   ((uint32_t)imu_hump_tenths(o->last_peak_mm)
                    << IMU_HUMP_W2_LAST_SHIFT);
    out->word[3] = ((uint32_t)closed << IMU_HUMP_W3_COUNT_SHIFT) |
                   (gated << IMU_HUMP_W3_GATED_SHIFT) |
                   ((uint32_t)((flags & IMU_HUMP_FLAGS_MASK) >> 1u)
                    << IMU_HUMP_W3_FLAGS_SHIFT) |
                   ((open ? 1u : 0u) << IMU_HUMP_W3_OPEN_SHIFT) |
                   ((o->valid_samples != 0u ? 1u : 0u)
                    << IMU_HUMP_W3_VALID_SHIFT) |
                   ((o->last_left ? 1u : 0u) << IMU_HUMP_W3_LEFT_SHIFT);
}

imu_hump_summary_t imu_hump_board_summarize(const imu_hump_board_t *b,
                                            bool have, uint32_t run_epoch,
                                            uint32_t now_ms,
                                            uint32_t max_age_ms)
{
    imu_hump_summary_t out = unavailable();

    if (b == NULL || !have || (run_epoch & 1u) == 0u ||
        imu_hump_board_epoch(b) != run_epoch ||
        (uint32_t)(now_ms - imu_hump_board_time(b)) > max_age_ms) {
        return out;
    }
    const uint32_t w3 = b->word[3];
    const uint16_t closed = imu_hump_board_count(b);
    const bool open = ((w3 >> IMU_HUMP_W3_OPEN_SHIFT) & 1u) != 0u;
    const uint8_t flags = (uint8_t)(IMU_HUMP_FLAG_IMPLEMENTED |
        (((w3 >> IMU_HUMP_W3_FLAGS_SHIFT) & IMU_HUMP_W3_FLAGS_MASK) << 1u));

    out.gated = (uint8_t)((w3 >> IMU_HUMP_W3_GATED_SHIFT) & IMU_HUMP_U8_MASK);
    if (((w3 >> IMU_HUMP_W3_VALID_SHIFT) & 1u) == 0u) {
        out.flags = (uint8_t)((flags | IMU_HUMP_FLAG_FAULT) &
                              IMU_HUMP_FLAGS_MASK);
        return out;
    }
    out.count = open && closed != UINT16_MAX ? (uint16_t)(closed + 1u)
                                             : closed;
    out.flags = flags;
    out.peak_01mm = (uint16_t)((b->word[2] >> IMU_HUMP_W2_BEST_SHIFT) &
                               IMU_HUMP_U16_MASK);
    return out;
}

imu_hump_summary_t imu_hump_summarize(const imu_hump_report_t *r, bool have,
                                      uint32_t run_epoch, uint32_t now_ms,
                                      uint32_t max_age_ms)
{
    if (r == NULL) {
        return unavailable();
    }
    imu_hump_board_t b;
    imu_hump_pack(&b, &r->hump, r->timestamp_ms);
    return imu_hump_board_summarize(&b, have, run_epoch, now_ms, max_age_ms);
}

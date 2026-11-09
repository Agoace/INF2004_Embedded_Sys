/* Ordinary Code 39: nine alternating elements, three wide, MSB first.
 * The public symbol table is standard Code 39, not a trained classifier. */
#include "barcode.h"
#include <math.h>
#include <string.h>

#define BC_WIDE_MIN       1.6f
#define BC_WIDE_MAX       3.4f
#define BC_SPLIT_MIN      1.4f
#define BC_QUIET_NARROW   10.0f
#define BC_CAPTURE_MAX_US 60000000u

const barcode_config_t barcode_default_config = {64u, 3000u, 12414u, 30000u};
static const char alphabet[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*";
static const uint16_t patterns[] = {
    0x034u, 0x121u, 0x061u, 0x160u, 0x031u, 0x130u, 0x070u, 0x025u, 0x124u,
    0x064u, 0x109u, 0x049u, 0x148u, 0x019u, 0x118u, 0x058u, 0x00du, 0x10cu,
    0x04cu, 0x01cu, 0x103u, 0x043u, 0x142u, 0x013u, 0x112u, 0x052u, 0x007u,
    0x106u, 0x046u, 0x016u, 0x181u, 0x0c1u, 0x1c0u, 0x091u, 0x190u, 0x0d0u,
    0x085u, 0x184u, 0x0c4u, 0x0a8u, 0x0a2u, 0x08au, 0x02au, 0x094u};
_Static_assert(sizeof(patterns) / sizeof(patterns[0]) == sizeof(alphabet) - 1u,
               "Code 39 alphabet/table mismatch");

static int32_t delta32(uint32_t current, uint32_t previous)
{
    const uint32_t bits = current - previous;
    return bits <= INT32_MAX ? (int32_t)bits
                             : -(int32_t)(UINT32_MAX - bits) - 1;
}

static float displacement(const barcode_motion_t *m,
                          const barcode_motion_t *origin, uint32_t scale)
{
    const int32_t left = delta32((uint32_t)m->left, (uint32_t)origin->left);
    const int32_t right = delta32((uint32_t)m->right, (uint32_t)origin->right);
    return ((float)left + (float)right) * 500.0f / (float)scale;
}

typedef struct {
    const barcode_motion_t *rows;
    size_t count;
    size_t index;
    uint32_t origin;
    uint32_t scale;
    float previous;
    int direction;
    bool ready;
} motion_reader_t;

static bool position(motion_reader_t *r, uint32_t us, float *value)
{
    if (!r->ready) {
        return false;
    }
    const int32_t t = delta32(us, r->origin);
    while (r->index + 1u < r->count &&
           delta32(r->rows[r->index + 1u].us, r->origin) < t) {
        ++r->index;
    }
    if (r->index + 1u >= r->count ||
        delta32(r->rows[r->index].us, r->origin) > t) {
        r->ready = false;
        return false;
    }
    const barcode_motion_t *a = &r->rows[r->index];
    const barcode_motion_t *b = &r->rows[r->index + 1u];
    const float x = displacement(a, r->rows, r->scale);
    const float y = displacement(b, r->rows, r->scale);
    const float fraction = (float)(us - a->us) / (float)(b->us - a->us);
    const float next = x + fraction * (y - x);
    const float d = next - r->previous;
    const int sign = d > 0.0001f ? 1 : (d < -0.0001f ? -1 : 0);
    if (r->direction != 0 && sign != 0 && sign != r->direction) {
        r->ready = false;
        return false;
    }
    if (sign != 0) {
        r->direction = sign;
    }
    r->previous = next;
    *value = next;
    return true;
}

static float width(const float *runs, size_t count, size_t i, bool reverse)
{
    return fabsf(runs[reverse ? count - 1u - i : i]);
}

/* Rank three wide elements, then remove a bounded bar/space edge bias.
 * Blur subtracts width from bars and adds it to spaces. Derive that bias
 * from the two narrow means, never from a requested letter or symbol. */
static char character(const float *runs, size_t count, size_t start,
                      bool reverse, float *narrow, float *space_narrow,
                      uint16_t *separation, barcode_reason_t *reason)
{
    float sorted[9];
    for (size_t i = 0u; i < 9u; ++i) {
        const float v = width(runs, count, start + i, reverse);
        size_t j = i;
        if (!isfinite(v) || v <= 0.0f) {
            *reason = BC_WIDTH;
            return '\0';
        }
        while (j > 0u && sorted[j - 1u] > v) {
            sorted[j] = sorted[j - 1u];
            --j;
        }
        sorted[j] = v;
    }
    /* A tie at the split does not identify exactly three wide elements. */
    if (sorted[6] <= sorted[5]) {
        *reason = BC_WIDTH;
        return '\0';
    }
    const float threshold = (sorted[5] + sorted[6]) * 0.5f;
    uint16_t mask = 0u;
    float narrow_sum[2] = {0.0f, 0.0f};
    unsigned int narrow_count[2] = {0u, 0u};
    for (size_t i = 0u; i < 9u; ++i) {
        const float v = width(runs, count, start + i, reverse);
        if (v > threshold) {
            mask |= (uint16_t)(UINT16_C(0x0100) >> i);
        } else {
            narrow_sum[i & 1u] += v;
            ++narrow_count[i & 1u];
        }
    }
    if (narrow_count[0] == 0u || narrow_count[1] == 0u) {
        *reason = BC_WIDTH;
        return '\0';
    }
    const float bar = narrow_sum[0] / (float)narrow_count[0];
    const float space = narrow_sum[1] / (float)narrow_count[1];
    const float physical = (bar + space) * 0.5f;
    const float bias = (space - bar) * 0.5f;
    if (fabsf(bias) > physical * 0.8f) {
        *reason = BC_WIDTH;
        return '\0';
    }
    float sum[2] = {0.0f, 0.0f};
    float minimum[2] = {INFINITY, INFINITY};
    float maximum[2] = {0.0f, 0.0f};
    for (size_t i = 0u; i < 9u; ++i) {
        const unsigned int wide =
            (mask & (uint16_t)(UINT16_C(0x0100) >> i)) != 0u ? 1u : 0u;
        const float corrected = width(runs, count, start + i, reverse) +
                                ((i & 1u) == 0u ? bias : -bias);
        sum[wide] += corrected;
        if (corrected < minimum[wide]) {
            minimum[wide] = corrected;
        }
        if (corrected > maximum[wide]) {
            maximum[wide] = corrected;
        }
    }
    const float small = sum[0] / 6.0f;
    const float large = sum[1] / 3.0f;
    if (small <= 0.0f || large / small < BC_WIDE_MIN ||
        large / small > BC_WIDE_MAX || minimum[1] / maximum[0] < BC_SPLIT_MIN ||
        minimum[0] < small * 0.65f || maximum[0] > small * 1.5f ||
        minimum[1] < large * 0.65f || maximum[1] > large * 1.5f) {
        *reason = BC_WIDTH;
        return '\0';
    }
    *narrow = physical;
    *space_narrow = space;
    const float ratio = minimum[1] / maximum[0] * 1000.0f;
    *separation = (uint16_t)(ratio < 65535.0f ? ratio : 65535.0f);
    for (size_t i = 0u; i < sizeof(alphabet) - 1u; ++i) {
        if (patterns[i] == mask) {
            return alphabet[i];
        }
    }
    *reason = BC_PATTERN;
    return '\0';
}

static barcode_command_t command(const barcode_result_t *r)
{
    if (r->length != 1u) {
        return BC_CMD_NONE;
    }
    switch (r->text[0]) {
    case 'A':
        return BC_CMD_LEFT;
    case 'B':
        return BC_CMD_RIGHT;
    case 'C':
        return BC_CMD_STRAIGHT;
    case 'D':
        return BC_CMD_UTURN;
    default:
        return BC_CMD_NONE;
    }
}

static void accept(barcode_result_t *out, const barcode_result_t *candidate,
                   bool reverse)
{
    if (out->reason == BC_AMBIGUOUS) {
        return;
    }
    if (out->decoded && strcmp(out->text, candidate->text) != 0) {
        out->decoded = false;
        out->quality = BC_QUALITY_REJECTED;
        out->command = BC_CMD_NONE;
        out->reason = BC_AMBIGUOUS;
        out->text[0] = '\0';
        out->length = 0u;
        return;
    }
    if (!out->decoded) {
        *out = *candidate;
        out->decoded = true;
        out->quality = BC_QUALITY_DECODED;
        out->reason = BC_OK;
        out->command = command(out);
    }
    out->directions |= reverse ? UINT8_C(0x02) : UINT8_C(0x01);
    ++out->frames;
    if (candidate->separation_permille < out->separation_permille) {
        out->separation_permille = candidate->separation_permille;
    }
}

static void scan(const float *runs, size_t count, bool first_high,
                 bool white_high, bool reverse, size_t start,
                 barcode_result_t *out, barcode_result_t *frame, size_t *stop)
{
    const size_t real = reverse ? count - 1u - start : start;
    const bool high = first_high != ((real & 1u) != 0u);
    if (high == white_high) {
        return;
    }
    float narrow;
    float space_narrow;
    uint16_t separation;
    barcode_reason_t why = BC_NO_FRAME;
    char c = character(runs, count, start, reverse, &narrow, &space_narrow,
                       &separation, &why);
    if (c != '*') {
        return;
    }
    if (width(runs, count, start - 1u, reverse) < BC_QUIET_NARROW * narrow) {
        if (!out->decoded && out->reason != BC_AMBIGUOUS &&
            out->reason != BC_PAYLOAD_LIMIT) {
            out->reason = BC_QUIET;
        }
        return;
    }
    barcode_result_t candidate = {.separation_permille = separation};
    size_t at = start + 9u;
    bool complete = false;
    while (at + 10u < count) {
        const float gap = width(runs, count, at, reverse);
        if (gap < space_narrow * 0.5f || gap > space_narrow * 1.8f) {
            why = BC_WIDTH;
            break;
        }
        ++at;
        c = character(runs, count, at, reverse, &narrow, &space_narrow,
                      &separation, &why);
        if (c == '\0') {
            break;
        }
        if (separation < candidate.separation_permille) {
            candidate.separation_permille = separation;
        }
        at += 9u;
        if (c == '*') {
            if (candidate.length == 0u ||
                width(runs, count, at, reverse) < BC_QUIET_NARROW * narrow) {
                why = BC_QUIET;
                break;
            }
            complete = true;
            break;
        }
        if (candidate.length >= BC_MAX_PAYLOAD) {
            why = BC_PAYLOAD_LIMIT;
            break;
        }
        candidate.text[candidate.length++] = c;
    }
    if (complete) {
        accept(out, &candidate, reverse);
        candidate.decoded = true;
        candidate.quality = BC_QUALITY_DECODED;
        candidate.reason = BC_OK;
        candidate.command = command(&candidate);
        candidate.frames = 1u;
        candidate.directions = reverse ? 2u : 1u;
        *frame = candidate;
        *stop = at;
    } else if (!out->decoded && out->reason != BC_AMBIGUOUS &&
               out->reason != BC_PAYLOAD_LIMIT) {
        out->reason = why == BC_NO_FRAME ? BC_TRUNCATED : why;
    }
}

static void reject(barcode_report_t *r, barcode_reason_t why)
{
    r->time.reason = why;
    r->distance.reason = why;
}

typedef enum {
    VALIDATE,
    MOTION,
    RUNS,
    SCAN,
    DONE
} phase_t;

typedef struct {
    const barcode_sample_t *samples;
    const barcode_packed_t *packed;
    barcode_frame_sink_t sink;
    void *sink_context;
    barcode_report_t *report;
    barcode_config_t config;
    motion_reader_t reader;
    size_t count;
    size_t index;
    size_t runs;
    uint32_t run_us;
    uint32_t current_us;
    float low;
    float high;
    float previous_position;
    float run_position;
    phase_t phase;
    unsigned int mode;
    bool first_high;
    bool level;
    bool distance_ok;
} decoder_t;

_Static_assert(sizeof(decoder_t) <= sizeof(barcode_state_t),
               "Private incremental state exceeds budget");

void barcode_begin(const barcode_sample_t *samples, size_t sample_count,
                   const barcode_motion_t *motion, size_t motion_count,
                   const barcode_config_t *config,
                   barcode_workspace_t *workspace, barcode_report_t *report)
{
    decoder_t d = {.phase = DONE};
    if (report != NULL) {
        memset(report, 0, sizeof(*report));
        reject(report, BC_NO_DATA);
    }
    if (workspace == NULL) {
        return;
    }
    if (report != NULL && samples != NULL && config != NULL &&
        sample_count >= 2u && sample_count <= BC_MAX_SAMPLES &&
        config->min_contrast != 0u && config->max_gap_us != 0u &&
        config->motion_gap_us != 0u) {
        d.samples = samples;
        d.count = sample_count;
        d.config = *config;
        d.report = report;
        d.reader = (motion_reader_t){.rows = motion,
                                     .count = motion_count,
                                     .origin = samples[0].us,
                                     .scale = config->counts_per_m};
        d.reader.ready = motion != NULL && motion_count >= 2u &&
                         motion_count <= BC_MAX_MOTION &&
                         config->counts_per_m != 0u;
        d.phase = VALIDATE;
        report->minimum = 4095u;
    }
    memcpy(workspace->state.bytes, &d, sizeof(d));
}

void barcode_begin_packed(const barcode_packed_t *samples, size_t count,
                          uint32_t origin_us, const barcode_motion_t *motion,
                          size_t motion_count, const barcode_config_t *config,
                          barcode_workspace_t *workspace,
                          barcode_report_t *report)
{
    /* Reuse common argument/configuration validation, then replace the seed
     * view with the immutable packed source before any step can run. */
    const barcode_sample_t seed[2] = {{origin_us, 0u, 0u}, {origin_us, 0u, 0u}};
    barcode_begin(
        samples != NULL && count >= 2u && count <= BC_MAX_SAMPLES ? seed : NULL,
        2u, motion, motion_count, config, workspace, report);
    if (workspace != NULL) {
        decoder_t d;
        memcpy(&d, workspace->state.bytes, sizeof(d));
        if (d.phase != DONE) {
            d.samples = NULL;
            d.packed = samples;
            d.count = count;
            d.current_us = origin_us;
            memcpy(workspace->state.bytes, &d, sizeof(d));
        }
    }
}

void barcode_frame_sink(barcode_workspace_t *workspace,
                        barcode_frame_sink_t sink, void *context)
{
    if (workspace != NULL) {
        decoder_t d;
        memcpy(&d, workspace->state.bytes, sizeof(d));
        d.sink = sink;
        d.sink_context = context;
        memcpy(workspace->state.bytes, &d, sizeof(d));
    }
}

static barcode_sample_t next_sample(decoder_t *d, size_t index)
{
    if (d->packed == NULL) {
        d->current_us = d->samples[index].us;
        return d->samples[index];
    }
    const barcode_packed_t row = d->packed[index];
    d->current_us =
        index == 0u ? d->reader.origin : d->current_us + row.delta_us;
    return (barcode_sample_t){
        d->current_us, (uint16_t)(row.raw_flags & BC_PACKED_RAW),
        (uint16_t)(((row.raw_flags & UINT16_C(0xf000)) != 0u ||
                    (index == 0u && row.delta_us != 0u))
                       ? BC_SAMPLE_GAP
                       : 0u)};
}

static int32_t micrometres(float mm, bool valid)
{
    const float um = mm * 1000.0f;
    return valid && um > -2147483648.0f && um < 2147483648.0f ? (int32_t)um
                                                              : INT32_MIN;
}

static void emit_frame(decoder_t *d, const barcode_workspace_t *workspace,
                       const barcode_result_t *value, size_t start, size_t stop,
                       bool distance)
{
    const bool reverse = (d->mode & 1u) != 0u;
    const size_t first = reverse ? d->runs - stop : start;
    const size_t end = reverse ? d->runs - start : stop;
    uint32_t us = d->reader.origin;
    float mm = 0.0f;
    barcode_frame_t frame = {.value = *value, .distance_method = distance};
    for (size_t i = 0u; i <= end; ++i) {
        if (i == first) {
            frame.start_us = us;
            frame.start_um = micrometres(mm, d->distance_ok);
        }
        if (i == end) {
            frame.end_us = us;
            frame.end_um = micrometres(mm, d->distance_ok);
            break;
        }
        us += (uint32_t)workspace->time_us[i];
        mm += workspace->distance_mm[i];
    }
    d->sink(&frame, d->sink_context);
}

/* Returns after at most 32 records, or one candidate frame. All loops have
 * fixed limits; the TEST caller sleeps between steps. */
bool barcode_step(barcode_workspace_t *workspace)
{
    if (workspace == NULL) {
        return true;
    }
    decoder_t d;
    memcpy(&d, workspace->state.bytes, sizeof(d));
    barcode_report_t *r = d.report;
    for (unsigned int budget = 0u; budget < 32u && d.phase != DONE; ++budget) {
        const size_t i = d.index;
        const uint32_t previous_us = d.current_us;
        barcode_sample_t v = {0};
        if (d.phase == VALIDATE || d.phase == RUNS) {
            v = next_sample(&d, i);
        }
        if (d.phase == VALIDATE) {
            const uint32_t dt = i == 0u ? 1u : v.us - previous_us;
            if (v.flags != 0u || v.raw > 4095u || dt == 0u ||
                dt > d.config.max_gap_us ||
                v.us - d.reader.origin > BC_CAPTURE_MAX_US) {
                reject(r, BC_SAMPLE_LOSS);
                d.phase = DONE;
                break;
            }
            if (v.raw < r->minimum) {
                r->minimum = v.raw;
            }
            if (v.raw > r->maximum) {
                r->maximum = v.raw;
            }
            if (++d.index == d.count) {
                r->contrast = r->maximum - r->minimum;
                if (r->contrast < d.config.min_contrast) {
                    reject(r, BC_CONTRAST);
                    d.phase = DONE;
                    break;
                }
                /* Narrow bars have only partial contrast. Keep a 32-count
                 * rise margin (above observed white jitter), 16-count fall,
                 * and 10%/6% full-span levels for high-contrast captures. */
                d.low = (float)r->minimum +
                        fmaxf(16.0f, (float)r->contrast * 0.06f);
                d.high = (float)r->minimum +
                         fmaxf(32.0f, (float)r->contrast * 0.10f);
                d.current_us = d.reader.origin;
                const barcode_sample_t first = next_sample(&d, 0u);
                d.first_high = (float)first.raw >= (d.low + d.high) * 0.5f;
                d.level = d.first_high;
                d.phase = MOTION;
                d.index = 1u;
            }
        } else if (d.phase == MOTION) {
            if (d.reader.ready && i < d.reader.count) {
                const uint32_t dt =
                    d.reader.rows[i].us - d.reader.rows[i - 1u].us;
                if (dt == 0u || dt > d.config.motion_gap_us) {
                    d.reader.ready = false;
                }
                ++d.index;
            } else {
                d.distance_ok =
                    position(&d.reader, d.reader.origin, &d.previous_position);
                d.run_position = d.previous_position;
                d.run_us = d.reader.origin;
                d.index = 1u;
                d.phase = RUNS;
            }
        } else if (d.phase == RUNS) {
            float x = d.previous_position;
            const bool positioned = position(&d.reader, v.us, &x);
            d.distance_ok = d.distance_ok && positioned;
            const bool next =
                d.level ? (float)v.raw > d.low : (float)v.raw >= d.high;
            if (next != d.level) {
                if (d.runs >= BC_MAX_RUNS - 1u) {
                    reject(r, BC_RUN_LIMIT);
                    d.phase = DONE;
                    break;
                }
                workspace->time_us[d.runs] = (float)(v.us - d.run_us);
                workspace->distance_mm[d.runs] = x - d.run_position;
                ++d.runs;
                d.run_us = v.us;
                d.run_position = x;
                d.level = next;
            }
            d.previous_position = x;
            if (++d.index == d.count) {
                workspace->time_us[d.runs] = (float)(v.us - d.run_us);
                workspace->distance_mm[d.runs] = x - d.run_position;
                r->runs = (uint16_t)++d.runs;
                reject(r, BC_NO_FRAME);
                r->distance.reason = d.distance_ok ? BC_NO_FRAME : BC_MOTION;
                d.index = 1u;
                d.phase = d.runs > 10u ? SCAN : DONE;
            }
        } else if (d.phase == SCAN) {
            /* Modes: two methods x two polarities x two directions. */
            const bool distance = (d.mode & 4u) != 0u;
            if (!distance || d.distance_ok) {
                barcode_result_t frame = {0};
                size_t stop = 0u;
                scan(distance ? workspace->distance_mm : workspace->time_us,
                     d.runs, d.first_high, (d.mode & 2u) != 0u,
                     (d.mode & 1u) != 0u, i, distance ? &r->distance : &r->time,
                     &frame, &stop);
                if (frame.decoded && d.sink != NULL) {
                    emit_frame(&d, workspace, &frame, i, stop, distance);
                }
            }
            if (++d.index + 9u >= d.runs) {
                d.index = 1u;
                if (++d.mode == 8u) {
                    d.phase = DONE;
                }
            }
            break;
        }
    }
    memcpy(workspace->state.bytes, &d, sizeof(d));
    return d.phase == DONE;
}

void barcode_decode(const barcode_sample_t *samples, size_t sample_count,
                    const barcode_motion_t *motion, size_t motion_count,
                    const barcode_config_t *config,
                    barcode_workspace_t *workspace, barcode_report_t *report)
{
    barcode_begin(samples, sample_count, motion, motion_count, config,
                  workspace, report);
    while (!barcode_step(workspace)) {
        /* Synchronous convenience for host tools, not the TEST runner. */
    }
}

const char *barcode_reason_name(barcode_reason_t reason)
{
    static const char *const names[] = {
        "OK",        "NO_DATA",   "CONTRAST",      "SAMPLE_LOSS",
        "RUN_LIMIT", "NO_FRAME",  "WIDTH",         "PATTERN",
        "QUIET",     "TRUNCATED", "PAYLOAD_LIMIT", "AMBIGUOUS",
        "MOTION"};
    const unsigned int i = (unsigned int)reason;
    return i < sizeof(names) / sizeof(names[0]) ? names[i] : "UNKNOWN";
}

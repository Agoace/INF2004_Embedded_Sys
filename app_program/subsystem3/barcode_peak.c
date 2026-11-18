/* Independent black-HIGH peak path. WIDTH is deliberately not changed. */
#include "barcode_peak.h"
#include <float.h>
#include <math.h>
#include <string.h>

enum {
    VALIDATE,
    BUILD,
    BASELINE,
    LOBES,
    SCAN,
    DONE
};

static const char symbols[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*";

static int32_t delta(uint32_t a, uint32_t b)
{
    const uint32_t d = a - b;
    return d <= INT32_MAX ? (int32_t)d : -(int32_t)(UINT32_MAX - d) - 1;
}

static barcode_peak_method_t *method(barcode_peak_workspace_t *w)
{
    return w->axis == 0u ? &w->report->time : &w->report->distance;
}

static float displacement(const barcode_peak_workspace_t *w, size_t i)
{
    const barcode_motion_t *m = &w->motor[i];
    return ((float)delta((uint32_t)m->left, (uint32_t)w->motor[0].left) +
            (float)delta((uint32_t)m->right, (uint32_t)w->motor[0].right)) *
           500.0f / (float)w->config.capture.counts_per_m;
}

static bool position(const barcode_peak_workspace_t *w, uint32_t us,
                     float *value)
{
    if (!w->motion_ok) {
        return false;
    }
    size_t lo = 0u, hi = w->motor_count - 1u;
    const int32_t t = delta(us, w->motor[0].us);
    if (t < 0 || t > delta(w->motor[hi].us, w->motor[0].us)) {
        return false;
    }
    while (hi - lo > 1u) {
        const size_t mid = lo + (hi - lo) / 2u;
        if (delta(w->motor[mid].us, w->motor[0].us) <= t) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    const float f = (float)(us - w->motor[lo].us) /
                    (float)(w->motor[hi].us - w->motor[lo].us);
    *value =
        displacement(w, lo) + f * (displacement(w, hi) - displacement(w, lo));
    return true;
}

static void next_axis(barcode_peak_workspace_t *w)
{
    if (w->axis == 0u && w->motion_ok) {
        w->axis = 1u;
        w->phase = BUILD;
        w->index = 0u;
        w->bin_count = 0u;
        w->members = 0u;
        w->sum = 0u;
        w->us = w->origin_us;
        w->extent = 0.0f;
    } else {
        if (!w->motion_ok) {
            w->report->distance.value.reason = BC_MOTION;
        }
        w->phase = DONE;
    }
}

static void reject(barcode_peak_workspace_t *w, barcode_reason_t reason)
{
    method(w)->value = (barcode_result_t){.reason = reason};
    next_axis(w);
}

void barcode_peak_begin(const barcode_sample_t *rows, size_t count,
                        const barcode_packed_t *packed, uint32_t origin_us,
                        const barcode_motion_t *motor, size_t motor_count,
                        const barcode_peak_config_t *config,
                        barcode_peak_workspace_t *w,
                        barcode_peak_report_t *report)
{
    if (w == NULL || report == NULL) {
        return;
    }
    memset(w, 0, sizeof(*w));
    memset(report, 0, sizeof(*report));
    w->phase = DONE;
    report->time.value.reason = BC_NO_DATA;
    report->distance.value.reason = BC_NO_DATA;
    if ((rows == NULL) == (packed == NULL) || count < 2u ||
        count > BC_MAX_SAMPLES || config == NULL || config->speed_mm_s < 25u ||
        config->speed_mm_s > 200u || config->capture.max_gap_us == 0u ||
        config->white_max < config->white_min || config->white_max > 4095u ||
        (unsigned int)config->white_max - config->white_min > 128u) {
        return;
    }
    w->raw = rows;
    w->packed = packed;
    w->count = count;
    w->motor = motor;
    w->motor_count = motor_count;
    w->config = *config;
    w->report = report;
    w->origin_us = rows != NULL ? rows[0].us : origin_us;
    w->us = w->origin_us;
    w->motion_ok = motor != NULL && motor_count >= 2u &&
                   motor_count <= BC_MAX_MOTION &&
                   config->capture.counts_per_m != 0u;
    w->phase = w->motion_ok ? VALIDATE : BUILD;
    report->time.value.reason = BC_NO_FRAME;
    report->distance.value.reason = BC_NO_FRAME;
}

void barcode_peak_sink(barcode_peak_workspace_t *w, barcode_frame_sink_t sink,
                       void *context)
{
    if (w != NULL) {
        w->sink = sink;
        w->context = context;
    }
}

static bool validate_motion(barcode_peak_workspace_t *w)
{
    for (unsigned int n = 0u; n < 32u && w->index < w->motor_count; ++n) {
        const size_t i = w->index++;
        if (i == 0u) {
            continue;
        }
        const uint32_t dt = w->motor[i].us - w->motor[i - 1u].us;
        const float movement = displacement(w, i) - displacement(w, i - 1u);
        const int sign = movement > 0.0f ? 1 : movement < 0.0f ? -1 : 0;
        if (dt == 0u || dt > w->config.capture.motion_gap_us ||
            w->motor[i].us - w->motor[0].us > 60000000u ||
            (sign != 0 && w->direction != 0 && sign != w->direction)) {
            w->motion_ok = false;
            break;
        }
        if (sign != 0) {
            w->direction = sign;
        }
    }
    if (!w->motion_ok || w->index == w->motor_count) {
        w->motion_ok = w->motion_ok && w->direction != 0 &&
                       position(w, w->origin_us, &w->position_origin);
        w->index = 0u;
        w->phase = BUILD;
    }
    return false;
}

static void flush_bin(barcode_peak_workspace_t *w)
{
    w->bins[w->bin_count++] =
        (uint16_t)((w->sum + w->members / 2u) / w->members);
    w->sum = 0u;
    w->members = 0u;
}

static void build(barcode_peak_workspace_t *w)
{
    for (unsigned int n = 0u; n < 32u && w->index < w->count; ++n) {
        const size_t i = w->index++;
        uint16_t raw, flags;
        if (w->raw != NULL) {
            w->us = w->raw[i].us;
            raw = w->raw[i].raw;
            flags = w->raw[i].flags;
        } else {
            w->us += w->packed[i].delta_us;
            raw = w->packed[i].raw_flags & BC_PACKED_RAW;
            flags = w->packed[i].raw_flags & (uint16_t)~BC_PACKED_RAW;
            if (i == 0u && w->packed[i].delta_us != 0u) {
                flags = BC_PACKED_GAP;
            }
        }
        const uint32_t dt = w->us - w->previous_us;
        if (raw > 4095u || flags != 0u ||
            (i != 0u && (dt == 0u || dt > w->config.capture.max_gap_us)) ||
            w->us - w->origin_us > 60000000u) {
            w->report->time.value =
                (barcode_result_t){.reason = BC_SAMPLE_LOSS};
            w->report->distance.value = w->report->time.value;
            w->phase = DONE;
            return;
        }
        w->previous_us = w->us;
        float x = (float)(w->us - w->origin_us) * (float)w->config.speed_mm_s /
                  1000000.0f;
        if (w->axis != 0u) {
            if (!position(w, w->us, &x)) {
                reject(w, BC_MOTION);
                return;
            }
            x = (x - w->position_origin) * (float)w->direction;
        }
        if (!isfinite(x) || x < w->extent - 0.001f) {
            reject(w, BC_MOTION);
            return;
        }
        w->extent = x;
        if (x >= (float)BC_PEAK_BINS) {
            reject(w, BC_RUN_LIMIT);
            return;
        }
        const size_t bin = (size_t)x;
        if (bin > w->bin_count) {
            if (bin != w->bin_count + 1u || w->members == 0u) {
                reject(w, BC_SAMPLE_LOSS);
                return;
            }
            flush_bin(w);
        }
        w->sum += raw;
        ++w->members;
    }
    if (w->index == w->count) {
        if (w->members != 0u) {
            flush_bin(w);
        }
        w->phase = BASELINE;
    }
}

static void sort_noise(uint16_t *samples)
{
    for (size_t i = 1u; i < 21u; ++i) {
        const uint16_t value = samples[i];
        size_t j = i;
        while (j != 0u && samples[j - 1u] > value) {
            samples[j] = samples[j - 1u];
            --j;
        }
        samples[j] = value;
    }
}

static void baseline(barcode_peak_workspace_t *w)
{
    if (w->bin_count < 32u) {
        reject(w, BC_NO_FRAME);
        return;
    }
    float mean = 0.0f, variance = 0.0f;
    for (size_t i = 5u; i <= 25u; ++i) {
        w->noise_samples[i - 5u] = w->bins[i];
        const float d = (float)w->bins[i] - mean;
        mean += d / (float)(i - 4u);
        variance += d * ((float)w->bins[i] - mean);
    }
    barcode_peak_method_t *r = method(w);
    r->baseline = mean;
    /* 21 integer bins bound variance to <=4095^2. Sixteen Newton steps
     * converge at this range without pulling libc errno state into RAM. */
    const float v = variance > 0.0f ? variance / 20.0f : 0.0f;
    float root = v > 1.0f ? v : 1.0f;
    for (unsigned int i = 0u; i < 16u; ++i) {
        root = (root + v / root) * 0.5f;
    }
    sort_noise(w->noise_samples);
    const uint16_t median = w->noise_samples[10];
    for (size_t i = 0u; i < 21u; ++i) {
        const uint16_t value = w->bins[i + 5u];
        w->noise_samples[i] = value >= median ? (uint16_t)(value - median)
                                              : (uint16_t)(median - value);
    }
    sort_noise(w->noise_samples);
    /* Do not trim away disturbances: use the larger SD estimate. */
    r->noise_sd =
        fmaxf(v == 0.0f ? 0.0f : root, (float)w->noise_samples[10] * 1.4826f);
    r->threshold = fmaxf(20.0f, r->noise_sd * 6.0f);
    if (mean < (float)w->config.white_min - 32.0f ||
        mean > (float)w->config.white_max + 32.0f || r->noise_sd > 10.0f) {
        reject(w, BC_CONTRAST);
        return;
    }
    w->index = 0u;
    w->lobe_active = false;
    w->phase = LOBES;
}

static bool close_lobe(barcode_peak_workspace_t *w, size_t end)
{
    barcode_peak_method_t *r = method(w);
    if (r->peaks >= BC_PEAK_LIMIT) {
        reject(w, BC_RUN_LIMIT);
        return false;
    }
    uint16_t minimum = w->bins[w->lobe_max];
    const size_t left = w->lobe_max > 6u ? w->lobe_max - 6u : 0u;
    const size_t right =
        w->lobe_max + 6u < w->bin_count ? w->lobe_max + 6u : w->bin_count - 1u;
    for (size_t i = left; i <= right; ++i) {
        if (w->bins[i] < minimum) {
            minimum = w->bins[i];
        }
    }
    /* Include white flanks: a wide flat top must not hide its own bar. */
    if (w->lobe_left != 0u && w->bins[w->lobe_left - 1u] < minimum) {
        minimum = w->bins[w->lobe_left - 1u];
    }
    if (end < w->bin_count && w->bins[end] < minimum) {
        minimum = w->bins[end];
    }
    w->peaks[r->peaks++] = (barcode_peak_t){
        w->lobe_sum / w->lobe_weight, (float)w->lobe_left, (float)end,
        w->bins[w->lobe_max], (uint16_t)(w->bins[w->lobe_max] - minimum)};
    w->lobe_active = false;
    return true;
}

static void lobes(barcode_peak_workspace_t *w)
{
    const float white = method(w)->baseline + method(w)->threshold;
    for (unsigned int n = 0u; n < 32u && w->index < w->bin_count; ++n) {
        const size_t i = w->index++;
        if ((float)w->bins[i] > white) {
            if (!w->lobe_active) {
                w->lobe_active = true;
                w->lobe_left = i;
                w->lobe_max = i;
                w->lobe_sum = 0.0f;
                w->lobe_weight = 0.0f;
            }
            if (w->bins[i] > w->bins[w->lobe_max]) {
                w->lobe_max = i;
            }
            const float weight = (float)w->bins[i] - method(w)->baseline;
            w->lobe_sum += ((float)i + 0.5f) * weight;
            w->lobe_weight += weight;
        } else if (w->lobe_active && !close_lobe(w, i)) {
            return;
        }
    }
    if (w->index == w->bin_count) {
        if (w->lobe_active && !close_lobe(w, w->bin_count)) {
            return;
        }
        w->phase = SCAN;
        w->orientation = 0u;
        w->scan = 0u;
        w->scanning = false;
    }
}

static const barcode_peak_t *peak(const barcode_peak_workspace_t *w, size_t i)
{
    const size_t count =
        w->axis == 0u ? w->report->time.peaks : w->report->distance.peaks;
    return &w->peaks[w->orientation == 0u ? i : count - 1u - i];
}

static float centre(const barcode_peak_workspace_t *w, size_t i)
{
    const float c = peak(w, i)->centre;
    return w->orientation == 0u ? c : w->extent - c;
}

static bool is_wide(uint16_t pattern, size_t i)
{
    return (pattern & (UINT16_C(0x0100) >> i)) != 0u;
}

/* Constrained least squares: d = a*narrow + b*wide for four centre gaps.
 * The fit uses the standard symbol, never a requested payload or lobe width. */
static bool model(barcode_peak_workspace_t *w, size_t start, uint16_t mask,
                  float *narrow, float *wide, float *error)
{
    float aa = 0.0f, ab = 0.0f, bb = 0.0f, ad = 0.0f, bd = 0.0f;
    for (size_t i = 0u; i < 4u; ++i) {
        const float b = (is_wide(mask, i * 2u) ? 0.5f : 0.0f) +
                        (is_wide(mask, i * 2u + 2u) ? 0.5f : 0.0f) +
                        (is_wide(mask, i * 2u + 1u) ? 1.0f : 0.0f);
        const float a = 2.0f - b;
        const float d = centre(w, start + i + 1u) - centre(w, start + i);
        aa += a * a;
        ab += a * b;
        bb += b * b;
        ad += a * d;
        bd += b * d;
    }
    const float determinant = aa * bb - ab * ab;
    if (determinant <= 0.0f) {
        return false;
    }
    float n = (ad * bb - bd * ab) / determinant;
    float v = (bd * aa - ad * ab) / determinant;
    if (n <= 0.0f || v <= 0.0f || v / n < 1.5f || v / n > 3.6f) {
        return false;
    }
    if (v / n < 1.6f || v / n > 3.4f) {
        const float ratio = v / n < 1.6f ? 1.6f : 3.4f;
        n = (ad + ratio * bd) / (aa + 2.0f * ratio * ab + ratio * ratio * bb);
        v = ratio * n;
    }
    float minimum_wide = v, maximum_narrow = n, residual = 0.0f;
    for (size_t i = 0u; i < 4u; ++i) {
        const float bars = (is_wide(mask, i * 2u) ? v : n) * 0.5f +
                           (is_wide(mask, i * 2u + 2u) ? v : n) * 0.5f;
        const float space =
            centre(w, start + i + 1u) - centre(w, start + i) - bars;
        const bool high = is_wide(mask, i * 2u + 1u);
        const float expected = high ? v : n;
        const float e = fabsf(space - expected) / n;
        if (e > residual) {
            residual = e;
        }
        if (e > 0.35f || space < expected * 0.65f || space > expected * 1.5f ||
            (space > (n + v) * 0.5f) != high) {
            return false;
        }
        if (high && space < minimum_wide) {
            minimum_wide = space;
        }
        if (!high && space > maximum_narrow) {
            maximum_narrow = space;
        }
    }
    if (minimum_wide / maximum_narrow < 1.4f) {
        return false;
    }
    *narrow = n;
    *wide = v;
    *error = residual;
    w->model_separation = minimum_wide / maximum_narrow;
    return true;
}

static bool star(barcode_peak_workspace_t *w, size_t start, float *threshold,
                 float *narrow, float *wide, float *error)
{
    float low = 0.0f, high = FLT_MAX;
    for (size_t i = 0u; i < 5u; ++i) {
        const barcode_peak_t *p = peak(w, start + i);
        if ((float)p->prominence < method(w)->threshold) {
            return false;
        }
        if (i == 2u || i == 3u) {
            if ((float)p->height < high) {
                high = p->height;
            }
        } else if ((float)p->height > low) {
            low = p->height;
        }
    }
    if (high - low < fmaxf(method(w)->threshold, method(w)->noise_sd * 12.0f)) {
        return false;
    }
    *threshold = (high + low) * 0.5f;
    return model(w, start, barcode_pattern('*'), narrow, wide, error);
}

static char character(barcode_peak_workspace_t *w, size_t start, float *narrow,
                      float *wide, float *error)
{
    uint16_t bars = 0u;
    for (size_t i = 0u; i < 5u; ++i) {
        const barcode_peak_t *p = peak(w, start + i);
        const float margin = fabsf((float)p->height - w->threshold);
        if ((float)p->prominence < method(w)->threshold ||
            margin < fmaxf(method(w)->noise_sd * 6.0f,
                           method(w)->threshold * 0.5f)) {
            return '\0';
        }
        if ((float)p->height > w->threshold) {
            bars |= (uint16_t)(UINT16_C(0x0100) >> (i * 2u));
        }
        if (margin < w->frame_margin) {
            w->frame_margin = margin;
        }
    }
    char found = '\0';
    for (size_t i = 0u; i < sizeof(symbols) - 1u; ++i) {
        const uint16_t mask = barcode_pattern(symbols[i]);
        float n, v, e;
        if ((mask & UINT16_C(0x0155)) == bars &&
            model(w, start, mask, &n, &v, &e)) {
            if (found != '\0') {
                return '\0'; /* More than one admissible character. */
            }
            found = symbols[i];
            *narrow = n;
            *wide = v;
            *error = e;
        }
    }
    return found;
}

static barcode_command_t command(const char *text, size_t length)
{
    if (length != 1u) {
        return BC_CMD_NONE;
    }
    switch (text[0]) {
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

static uint32_t source_time(const barcode_peak_workspace_t *w, float x)
{
    if (w->axis == 0u) {
        return w->origin_us +
               (uint32_t)(x * 1000000.0f / (float)w->config.speed_mm_s);
    }
    const float target = w->position_origin + (float)w->direction * x;
    for (size_t i = 1u; i < w->motor_count; ++i) {
        const float a = displacement(w, i - 1u), b = displacement(w, i);
        if ((target >= a && target <= b) || (target <= a && target >= b)) {
            const float f = a == b ? 0.0f : (target - a) / (b - a);
            return w->motor[i - 1u].us +
                   (uint32_t)(f *
                              (float)(w->motor[i].us - w->motor[i - 1u].us));
        }
    }
    return w->origin_us; /* Never reached for bracketed accepted centres. */
}

static int32_t micrometres(float mm)
{
    const float um = mm * 1000.0f;
    return isfinite(um) && um > -2147483648.0f && um < 2147483648.0f
               ? (int32_t)um
               : INT32_MIN;
}

static void accept(barcode_peak_workspace_t *w, size_t last)
{
    barcode_peak_method_t *r = method(w);
    barcode_result_t value = {.reason = BC_OK,
                              .quality = BC_QUALITY_DECODED,
                              .decoded = true,
                              .length = w->length,
                              .frames = 1u,
                              .directions = (uint8_t)(1u << w->orientation),
                              .separation_permille =
                                  (uint16_t)(w->frame_separation * 1000.0f)};
    memcpy(value.text, w->text, sizeof(value.text));
    value.command = command(value.text, value.length);
    if (r->value.reason == BC_AMBIGUOUS ||
        (r->value.decoded && strcmp(r->value.text, value.text) != 0)) {
        r->value = (barcode_result_t){
            .reason = BC_AMBIGUOUS,
            .frames = (uint16_t)(r->value.frames + 1u),
            .directions = (uint8_t)(r->value.directions | value.directions)};
    } else if (r->value.decoded) {
        ++r->value.frames;
        r->value.directions |= value.directions;
        if (value.separation_permille < r->value.separation_permille) {
            r->value.separation_permille = value.separation_permille;
        }
    } else {
        r->value = value;
    }
    if (r->minimum_prominence == 0.0f) {
        r->minimum_prominence = FLT_MAX;
    }
    for (size_t i = w->first; i < last + 5u; ++i) {
        if ((float)peak(w, i)->prominence < r->minimum_prominence) {
            r->minimum_prominence = peak(w, i)->prominence;
        }
    }
    if (r->amplitude_margin == 0.0f || w->frame_margin < r->amplitude_margin) {
        r->amplitude_margin = w->frame_margin;
    }
    if (w->frame_error > r->geometry_error) {
        r->geometry_error = w->frame_error;
    }
    if (w->sink != NULL) {
        float a = peak(w, w->first)->centre, b = peak(w, last + 4u)->centre;
        if (a > b) {
            const float swap = a;
            a = b;
            b = swap;
        }
        barcode_frame_t frame = {.value = value,
                                 .start_us = source_time(w, a),
                                 .end_us = source_time(w, b),
                                 .start_um = INT32_MIN,
                                 .end_um = INT32_MIN,
                                 .distance_method = w->axis != 0u};
        float p;
        if (position(w, frame.start_us, &p)) {
            frame.start_um = micrometres(p - w->position_origin);
        }
        if (position(w, frame.end_us, &p)) {
            frame.end_um = micrometres(p - w->position_origin);
        }
        w->sink(&frame, w->context);
    }
}

static void scan(barcode_peak_workspace_t *w)
{
    const size_t count = method(w)->peaks;
    if (!w->scanning) {
        if (w->scan + 15u > count) {
            if (++w->orientation >= 2u) {
                next_axis(w);
                return;
            }
            w->scan = 0u;
        }
        if (w->scan + 15u > count) {
            next_axis(w);
            return;
        }
        w->first = w->scan++;
        float e;
        if (!star(w, w->first, &w->threshold, &w->narrow, &w->wide, &e)) {
            return;
        }
        float before = 0.0f;
        if (w->first != 0u) {
            const barcode_peak_t *p = peak(w, w->first - 1u);
            before = w->orientation == 0u ? p->right : w->extent - p->left;
        }
        if (centre(w, w->first) - w->narrow * 0.5f - before + 1.0f <
            w->narrow * 10.0f) {
            return;
        }
        w->cursor = w->first + 5u;
        w->stop = w->first + 10u;
        w->preparing = true;
        w->length = 0u;
        memset(w->text, 0, sizeof(w->text));
        w->frame_error = e;
        w->frame_separation = w->model_separation;
        w->frame_margin = FLT_MAX;
        w->scanning = true;
        return;
    }
    if (w->preparing) {
        if (w->stop + 5u > count ||
            w->stop > w->first + (BC_MAX_PAYLOAD + 1u) * 5u) {
            w->scanning = false;
            return;
        }
        float threshold, n, v, e;
        if (!star(w, w->stop, &threshold, &n, &v, &e)) {
            w->stop += 5u;
            return;
        }
        float after = w->extent;
        if (w->stop + 5u < count) {
            const barcode_peak_t *p = peak(w, w->stop + 5u);
            after = w->orientation == 0u ? p->left : w->extent - p->right;
        }
        if (after - centre(w, w->stop + 4u) - n * 0.5f + 1.0f < n * 10.0f) {
            w->stop += 5u;
            return;
        }
        float low = 0.0f, high = FLT_MAX;
        for (size_t j = 0u; j < 2u; ++j) {
            const size_t start = j == 0u ? w->first : w->stop;
            for (size_t i = 0u; i < 5u; ++i) {
                const float height = peak(w, start + i)->height;
                if (i == 2u || i == 3u) {
                    if (height < high) {
                        high = height;
                    }
                } else if (height > low) {
                    low = height;
                }
            }
        }
        if (high - low <
            fmaxf(method(w)->threshold, method(w)->noise_sd * 12.0f)) {
            w->scanning = false;
            return;
        }
        w->threshold = (high + low) * 0.5f;
        w->frame_margin = (high - low) * 0.5f;
        w->preparing = false;
        return;
    }
    if (w->cursor + 5u > count) {
        w->scanning = false;
        return;
    }
    float n = 0.0f, v = 0.0f, e = 0.0f;
    const char c = character(w, w->cursor, &n, &v, &e);
    /* The last bar of any legal preceding character may itself be wide. */
    const float previous_bar =
        (float)peak(w, w->cursor - 1u)->height > w->threshold ? w->wide
                                                              : w->narrow;
    const float next_bar =
        (float)peak(w, w->cursor)->height > w->threshold ? v : n;
    const float gap = centre(w, w->cursor) - centre(w, w->cursor - 1u) -
                      (previous_bar + next_bar) * 0.5f;
    if (c == '\0' || n / w->narrow < 0.65f || n / w->narrow > 1.5f ||
        gap < w->narrow * 0.5f || gap > w->narrow * 1.8f) {
        w->scanning = false;
        return;
    }
    if (e > w->frame_error) {
        w->frame_error = e;
    }
    if (w->model_separation < w->frame_separation) {
        w->frame_separation = w->model_separation;
    }
    if (c == '*') {
        float end_threshold, end_n, end_w, end_e;
        float after = w->extent;
        if (w->cursor + 5u < count) {
            const barcode_peak_t *p = peak(w, w->cursor + 5u);
            after = w->orientation == 0u ? p->left : w->extent - p->right;
        }
        if (w->length != 0u && w->cursor == w->stop &&
            star(w, w->cursor, &end_threshold, &end_n, &end_w, &end_e) &&
            after - centre(w, w->cursor + 4u) - n * 0.5f + 1.0f >= n * 10.0f) {
            accept(w, w->cursor);
        }
        w->scanning = false;
    } else if (w->length >= BC_MAX_PAYLOAD) {
        w->scanning = false;
    } else {
        w->text[w->length++] = c;
        w->cursor += 5u;
        w->narrow = n;
        w->wide = v;
    }
}

bool barcode_peak_step(barcode_peak_workspace_t *w)
{
    if (w == NULL || w->phase == DONE) {
        return true;
    }
    switch (w->phase) {
    case VALIDATE:
        return validate_motion(w);
    case BUILD:
        build(w);
        break;
    case BASELINE:
        baseline(w);
        break;
    case LOBES:
        lobes(w);
        break;
    case SCAN:
        scan(w);
        break;
    default:
        w->phase = DONE;
        break;
    }
    return w->phase == DONE;
}

void barcode_peak_select(const barcode_report_t *width,
                         const barcode_peak_report_t *peak_report,
                         barcode_result_t *selected)
{
    if (width == NULL || peak_report == NULL || selected == NULL) {
        return;
    }
    *selected = (barcode_result_t){.reason = BC_NO_FRAME};
    const barcode_result_t *const results[] = {&width->time, &width->distance,
                                               &peak_report->time.value,
                                               &peak_report->distance.value};
    for (size_t i = 0u; i < 4u; ++i) {
        const barcode_result_t *r = results[i];
        if (r->reason == BC_AMBIGUOUS ||
            (r->decoded && selected->decoded &&
             strcmp(r->text, selected->text) != 0)) {
            *selected = (barcode_result_t){.reason = BC_AMBIGUOUS};
            return;
        }
        if (r->decoded && !selected->decoded) {
            *selected = *r;
        }
    }
}

/* Course-only window policy; frozen TEST framing is deliberately unchanged. */
#include "barcode_window.h"
#include "barcode.h"
#if BARCODE_FAIL_CAPTURE
#include "barcode_fail_capture.h"
#endif
#include <math.h>
#include <string.h>

#define DISABLED 0u
#define SEEK 1u
#define WINDOW 2u
#define TAIL 3u
#define COMPLETE 4u
#define WHITE_MIN 120u
#define WHITE_MAX 220u
/* ASSUMED scope, not an optical resolution measurement. Fit the scale from
 * the start star; smaller print needs measured along-travel GP28 evidence. */
#define N_MIN 2.5f /* 2026-11-19: small sheet (2.9 mm narrow), GP28 2.5 mm. */
/* Local-midpoint edges inside WINDOW (2026-11-19, GP28 2.5 mm): narrow
 * small-sheet spaces stay far above the fixed fall level (H3 robot capture:
 * 1100..1600 counts between 3800 bars). An edge is detected when the signal
 * moves EDGE_DETECT of the local swing (and EDGE_MIN_SWING counts) away from
 * the run extreme. Its position is then set at the crossing of the midpoint
 * of the two extremes next to that edge (bar peak, space valley). */
#define EDGE_DETECT 0.4f
#define EDGE_MIN_SWING 300.0f /* H3 noise: white 25, bar 45 counts (1 sd) */
#define LEVEL_STEP 256u
/* First bar excess over floor white that selects the midpoint mode. Real
 * captures: 5.5 mm narrow bars 450..570 counts over white (fixture
 * barcode_large_f_forward.csv); 2.5 mm bars 2300..3700 (H3, image 20) and
 * the image-20 faint mark 925 (dropped by faint_first). */
#define STRONG_EXCESS 800.0f
/* ASSUMED bound: nominal L0/R292 for 30 ms retreats GP28 about 1.6..1.9 mm. */
#define RETREAT_MAX_MM 2.5f
/* SEEK re-anchors the projection axis when the heading has turned more than
 * this since the last anchor (bend exit, operator 2026-11-20). */
#define AXIS_REBASE_RAD 0.0524f /* 3 degrees */
#define N_MAX 7.0f
#define MAX_WINDOW_US 5000000u
#ifndef BW_MAP_VALUES
#define BW_MAP_VALUES 1,2,3,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
#endif
#ifndef BW_MAP_CONFIRMED
#define BW_MAP_CONFIRMED 1
#endif
#ifndef BW_MAP_POLICY_ID
#define BW_MAP_POLICY_ID 1u
#endif
static const uint8_t actions[26] = {BW_MAP_VALUES};
const barcode_window_config_t barcode_window_default_config = {
    .counts_per_m = 12414u,
    .sample_gap_us = 1500u,
    .motor_gap_us = 30000u,
    /* GP28: measured from the drive axle; verify on the robot. */
    .sensor_forward_mm = 73, /* MEASURED operator ruler 2026-11-16 */
    .sensor_lateral_mm = 73,
    .track_mm = 111u,
};
_Static_assert(sizeof(barcode_window_t) <= 2048u, "Window RAM budget");

uint8_t barcode_window_table_entry(unsigned int index)
{
    return index < 26u ? actions[index] : 0u;
}

bool barcode_window_table_confirmed(void)
{
    if (BW_MAP_CONFIRMED != 1 || BW_MAP_POLICY_ID == 0u) {
        return false;
    }
    for (unsigned int i = 0u; i < 26u; ++i) {
        if (actions[i] > 4u) {
            return false;
        }
    }
    return true;
}

uint32_t barcode_window_mapping_id(void)
{
    return barcode_window_table_confirmed() ? BW_MAP_POLICY_ID : 0u;
}

static uint32_t next_serial(barcode_window_t *state)
{
    if (++state->serial == 0u) {
        state->serial = 1u;
    }
    return state->serial;
}

/* Held-letter accept (operator 2026-11-20): once the start star, the first
 * gap and the letter (parts 1..19) passed their checks, a later failure in
 * the end star, gap 2, extent or tail keeps the letter. The edge and frame
 * are those of the letter's last edge; read is the newest motor sample. */
static void accept_held(barcode_window_t *state)
{
    const barcode_window_motor_t *read = state->motor_count > 0u ?
        &state->motors[state->motor_count - 1u] : NULL;
    const unsigned int index = (unsigned int)(state->held_letter - 'A');
    state->result = (barcode_window_result_t){
        .serial = next_serial(state), .reason = BW_OK,
        .start_us = state->start_us, .edge_us = state->held_edge_us,
        .frame_us = state->held_frame_us,
        .frame_left = state->held_frame_left,
        .frame_right = state->held_frame_right,
        .eligible_us = state->previous_adc_us,
        .read_us = read ? read->us : state->held_frame_us,
        .read_left = read ? read->left : state->held_frame_left,
        .read_right = read ? read->right : state->held_frame_right,
        .narrow_mm = state->narrow, .wide_mm = state->wide,
        .leading_mm = state->leading_mm,
        .rise = state->rise, .fall = state->fall,
        .letter = state->held_letter, .direction = state->direction,
        .mapped_action = barcode_window_table_confirmed() ? actions[index] : 0u,
        .accepted = true, .held = true};
    state->held = false;
    state->phase = COMPLETE;
}

/* Failure abandons the whole candidate. Never repair or slide its runs. */
static void reject(barcode_window_t *state, barcode_window_reason_t reason)
{
    if (state->held && (state->phase == WINDOW || state->phase == TAIL)) {
        accept_held(state);
        return;
    }
    state->held = false;
#if BARCODE_FAIL_CAPTURE
    bcfc_reject(state, (uint32_t)reason);
#endif
    if (state->phase == SEEK && state->white_started) {
        state->white_lost |= BW_WHITE_LOST_OTHER;
    }
    state->result = (barcode_window_result_t){
        .serial = next_serial(state), .reason = (uint32_t)reason};
    state->phase = SEEK;
    state->white_started = false;
    state->transitioning = false;
    state->white_count = 0u;
    state->elements = 0u;
}

static void loss(barcode_window_t *state, barcode_window_reason_t reason)
{
    reject(state, reason);
    state->queued = 0u;
    state->head = 0u;
}

bool barcode_window_begin(barcode_window_t *state,
                          const barcode_cal_report_t *calibration,
                          const barcode_window_config_t *config)
{
    const bool supplied = calibration != NULL;
    const barcode_cal_report_t saved = calibration ? *calibration :
                                        (barcode_cal_report_t){0};
    const barcode_window_config_t settings = config ? *config :
                                             barcode_window_default_config;
    calibration = &saved; /* Also supports a caller passing state's copies. */
    memset(state, 0, sizeof(*state));
    state->config = settings;
    state->result.reason = BW_CALIBRATION;
    if (!supplied || state->config.counts_per_m == 0u ||
        state->config.counts_per_m > 1000000u ||
        state->config.sample_gap_us == 0u ||
        state->config.sample_gap_us > 10000u ||
        state->config.motor_gap_us == 0u ||
        state->config.motor_gap_us > 100000u ||
        state->config.track_mm < 50u || state->config.track_mm > 500u ||
        state->config.sensor_forward_mm < -500 ||
        state->config.sensor_forward_mm > 500 ||
        state->config.sensor_lateral_mm < -500 ||
        state->config.sensor_lateral_mm > 500 ||
        (calibration->mode != BC_CAL_LEARNED &&
         calibration->mode != BC_CAL_FALLBACK) ||
        (calibration->mode == BC_CAL_LEARNED &&
         (calibration->white_samples < 16u ||
          calibration->white_mean < WHITE_MIN ||
          calibration->white_mean > WHITE_MAX ||
          (calibration->faults & BC_CAL_NO_WHITE) != 0u)) ||
        calibration->fall <= 0u ||
        calibration->rise <= calibration->fall ||
        calibration->rise >= 4095u ||
        (calibration->faults & (BC_CAL_FAULT_RAW | BC_CAL_FAULT_TIME |
         BC_CAL_FAULT_FULL)) != 0u) {
        return false;
    }
    state->calibration = *calibration;
    state->phase = SEEK;
    state->result.reason = BW_WAIT;
    return true;
}

bool barcode_window_rearm(barcode_window_t *state)
{
    const barcode_cal_report_t calibration = state->calibration;
    const barcode_window_config_t config = state->config;
    const uint32_t serial = state->serial;
    const bool valid = barcode_window_begin(state, &calibration, &config);
    state->serial = serial;
    return valid;
}

bool barcode_window_candidate(const barcode_window_t *state,
                              uint32_t *first_bar_us)
{
    const bool active = state->phase == WINDOW || state->phase == TAIL ||
                        state->phase == COMPLETE;
    if (active && first_bar_us) {
        *first_bar_us = state->start_us;
    }
    return active;
}

void barcode_window_adc(barcode_window_t *state, uint32_t us, uint16_t raw,
                        uint16_t flags)
{
    if (state->phase == DISABLED || state->phase == COMPLETE) {
        return;
    }
    const uint32_t gap = us - state->previous_adc_us;
    const bool bad_time = state->have_adc &&
        (gap == 0u || gap > state->config.sample_gap_us);
    state->previous_adc_us = us;
    state->have_adc = true;
    if (bad_time || raw == 0u || raw >= 4095u || flags != 0u) {
#if BARCODE_FAIL_CAPTURE
        bcfc_bad_adc((barcode_window_adc_t){us, raw, flags},
                     (uint16_t)((bad_time ? 1u : 0u) |
                     ((raw == 0u || raw >= 4095u) ? 2u : 0u) |
                     (flags != 0u ? 4u : 0u)));
#endif
        loss(state, BW_LOSS);
        return;
    }
    if (state->queued == BW_ADC_CAPACITY) {
#if BARCODE_FAIL_CAPTURE
        bcfc_bad_adc((barcode_window_adc_t){us, raw, flags}, 8u);
#endif
        loss(state, BW_LOSS);
        return;
    }
    const unsigned int at =
        ((unsigned int)state->head + state->queued) % BW_ADC_CAPACITY;
    state->adc[at] = (barcode_window_adc_t){us, raw, flags};
    ++state->queued;
}

/* Portable modular signed encoder difference, including counter wrap. */
static int32_t count_delta(uint32_t value, uint32_t previous)
{
    const uint32_t delta = value - previous;
    return delta <= INT32_MAX ? (int32_t)delta :
           -(int32_t)(UINT32_MAX - delta) - 1;
}

/* Set the first sensor point to the origin of the barcode-axis projection.
 * The axis is the robot's forward direction at the first motor observation.
 * This exact differential-drive step accounts for GP28's forward and lateral
 * offsets. It cannot infer an unknown angle between that axis and the print. */
static void sensor_pose_begin(barcode_window_t *state)
{
    state->centre_x_mm = -(float)state->config.sensor_forward_mm;
    state->centre_y_mm = -(float)state->config.sensor_lateral_mm;
    state->heading_rad = 0.0f;
}

static float sensor_axis_mm(const barcode_window_t *state)
{
    const float forward = (float)state->config.sensor_forward_mm;
    const float lateral = (float)state->config.sensor_lateral_mm;
    return state->centre_x_mm + forward * cosf(state->heading_rad) -
           lateral * sinf(state->heading_rad);
}

static float sensor_pose_step(barcode_window_t *state, int32_t dl, int32_t dr)
{
    const float scale = 1000.0f / (float)state->config.counts_per_m;
    const float left = (float)dl * scale;
    const float right = (float)dr * scale;
    const float centre = (left + right) * 0.5f;
    const float turn = (right - left) / (float)state->config.track_mm;
    float local_x = centre;
    float local_y = 0.0f;
    if (fabsf(turn) > 0.000001f) {
        const float radius = centre / turn;
        local_x = radius * sinf(turn);
        local_y = radius * (1.0f - cosf(turn));
    }
    const float c = cosf(state->heading_rad);
    const float s = sinf(state->heading_rad);
    state->centre_x_mm += c * local_x - s * local_y;
    state->centre_y_mm += s * local_x + c * local_y;
    state->heading_rad += turn;
    const float mm = sensor_axis_mm(state);
    /* Before the first bar, keep the barcode axis near the current heading.
     * Without this, a SEEK that starts inside a bend projects the barcode on
     * the bend-entry heading: R7 (A after a bend) read 37.6 mm for 60.1 mm
     * of wheel travel (cos 51 deg). Position stays continuous. */
    if (state->phase == SEEK && fabsf(state->heading_rad) > AXIS_REBASE_RAD) {
        state->centre_x_mm = mm - (float)state->config.sensor_forward_mm;
        state->centre_y_mm = -(float)state->config.sensor_lateral_mm;
        state->heading_rad = 0.0f;
    }
    return mm;
}

void barcode_window_motor(barcode_window_t *state, uint32_t us,
                          uint32_t left, uint32_t right, uint32_t faults)
{
    if (state->phase == DISABLED || state->phase == COMPLETE) {
        return;
    }
    float mm = 0.0f;
    if (faults != 0u) {
        loss(state, BW_MOTION);
        state->motor_count = 0u;
        return;
    }
    if (state->motor_count > 0u) {
        const barcode_window_motor_t *last =
            &state->motors[state->motor_count - 1u];
        const uint32_t dt = us - last->us;
        if (dt == 0u && left == last->left && right == last->right) {
            return; /* Re-reading an identical latest snapshot is harmless. */
        }
        const int32_t dl = count_delta(left, last->left);
        const int32_t dr = count_delta(right, last->right);
        if (dt == 0u || dt > state->config.motor_gap_us ||
            dl < -1024 || dl > 1024 || dr < -1024 || dr > 1024 ||
            dl + dr < 0 || (dl == 0 && dr == 0)) {
            loss(state, BW_MOTION);
            state->motor_count = 0u;
        } else {
            /* Stored positions never decrease: last->mm is the furthest
             * point. A small cumulative retreat (steering twitch) holds the
             * position; step() skips ADC rows until progress resumes. */
            mm = sensor_pose_step(state, dl, dr);
            if (!isfinite(mm) || mm < last->mm - RETREAT_MAX_MM) {
                loss(state, BW_MOTION);
                state->motor_count = 0u;
            } else if (mm < last->mm) {
                mm = last->mm;
            }
        }
    }
    if (state->motor_count == 0u) {
        sensor_pose_begin(state);
        mm = 0.0f;
    }
    if (state->motor_count == BW_MOTOR_CAPACITY) {
        memmove(state->motors, state->motors + 1,
                (BW_MOTOR_CAPACITY - 1u) * sizeof(state->motors[0]));
        --state->motor_count;
    }
    state->motors[state->motor_count++] =
        (barcode_window_motor_t){us, left, right, mm};
}

typedef struct {
    float narrow;
    float wide;
    float bias;
    float space;
    char symbol;
} element_model_t;

/* Same rank/edge-bias principle as WIDTH, but local to nine live runs.
 * No requested letter participates in fitting or correction. */
static bool model(const float *runs, bool reverse, element_model_t *out)
{
    float values[9], sorted[9];
    for (unsigned int i = 0u; i < 9u; ++i) {
        values[i] = runs[reverse ? 8u - i : i];
        if (!isfinite(values[i]) || values[i] <= 0.0f) {
            return false;
        }
        unsigned int j = i;
        while (j > 0u && sorted[j - 1u] > values[i]) {
            sorted[j] = sorted[j - 1u];
            --j;
        }
        sorted[j] = values[i];
    }
    if (sorted[6] <= sorted[5]) {
        return false;
    }
    const float split = (sorted[5] + sorted[6]) * 0.5f;
    uint16_t mask = 0u;
    float narrow[2] = {0.0f, 0.0f};
    unsigned int count[2] = {0u, 0u};
    for (unsigned int i = 0u; i < 9u; ++i) {
        if (values[i] > split) {
            mask |= (uint16_t)(0x100u >> i);
        } else {
            narrow[i & 1u] += values[i];
            ++count[i & 1u];
        }
    }
    if (count[0] == 0u || count[1] == 0u) {
        return false;
    }
    const float bar = narrow[0] / (float)count[0];
    const float space = narrow[1] / (float)count[1];
    const float physical = (bar + space) * 0.5f;
    const float bias = (space - bar) * 0.5f;
    if (fabsf(bias) > physical * 0.8f) {
        return false;
    }
    float sum[2] = {0.0f, 0.0f};
    float low[2] = {INFINITY, INFINITY}, high[2] = {0.0f, 0.0f};
    for (unsigned int i = 0u; i < 9u; ++i) {
        const unsigned int wide = (mask & (0x100u >> i)) != 0u ? 1u : 0u;
        const float corrected = values[i] + (i % 2u == 0u ? bias : -bias);
        sum[wide] += corrected;
        low[wide] = fminf(low[wide], corrected);
        high[wide] = fmaxf(high[wide], corrected);
    }
    const float n = sum[0] / 6.0f, w = sum[1] / 3.0f;
    if (n < N_MIN || n > N_MAX ||
        w / n < 1.6f || w / n > 3.55f || low[1] / high[0] < 1.4f ||
        low[0] < n * 0.65f || high[0] > n * 1.5f ||
        low[1] < w * 0.65f || high[1] > w * 1.5f) {
        return false;
    }
    *out = (element_model_t){physical, w, bias, space,
                              barcode_symbol(mask)};
    return out->symbol != '\0';
}

static bool start_star(barcode_window_t *state)
{
    element_model_t first;
    for (unsigned int reverse = 0u; reverse < 2u; ++reverse) {
        if (model(state->runs, reverse != 0u, &first) &&
            first.symbol == '*') {
            if (state->leading_mm < QUIET_MIN_MM) {
                return false;
            }
            state->direction = (uint8_t)(reverse + 1u);
            state->narrow = first.narrow;
            state->wide = first.wide;
            state->bias = first.bias;
            state->space = first.space;
            return true;
        }
    }
    return false;
}

/* Late-start rescue (operator 2026-11-20, option R). On a bend exit GP28 can
 * slide onto the sheet across the bar ends: the first narrow bar of the
 * start star stays below the rise threshold, so the window starts at star
 * element 2 (image 24 capture: 2.2 6.4 11.6 5.9 12.0 5.6 2.6 | 6.0 | 11.5).
 * When the normal start fails, treat runs 0..6 as star elements 2..8 in
 * travel order, rebuild the two missed runs from their own narrow/wide
 * estimates, and require the full nine-run model to give '*'. The letter,
 * gaps, extent and end star are then checked as usual. */
static bool late_start(barcode_window_t *state)
{
    for (unsigned int reverse = 0u; reverse < 2u; ++reverse) {
        /* Star classes in travel order (1 = wide). */
        bool wide[9];
        for (unsigned int i = 0u; i < 9u; ++i) {
            const unsigned int printed = reverse != 0u ? 8u - i : i;
            wide[i] = (0x094u & (0x100u >> printed)) != 0u;
        }
        float bar = 0.0f, space = 0.0f, wide_sum = 0.0f;
        unsigned int bars = 0u, spaces = 0u, wides = 0u;
        for (unsigned int i = 0u; i < 7u; ++i) {
            const unsigned int at = i + 2u;
            if (wide[at]) {
                continue;
            }
            if (at % 2u == 0u) {
                bar += state->runs[i];
                ++bars;
            } else {
                space += state->runs[i];
                ++spaces;
            }
        }
        if (bars == 0u || spaces == 0u) {
            continue;
        }
        bar /= (float)bars;
        space /= (float)spaces;
        const float bias = (space - bar) * 0.5f;
        for (unsigned int i = 0u; i < 7u; ++i) {
            const unsigned int at = i + 2u;
            if (wide[at]) {
                wide_sum += state->runs[i] + (at % 2u == 0u ? bias : -bias);
                ++wides;
            }
        }
        if (wides == 0u) {
            continue;
        }
        const float w = wide_sum / (float)wides;
        float rebuilt[9];
        rebuilt[0] = wide[0] ? w - bias : bar;
        rebuilt[1] = wide[1] ? w + bias : space;
        memcpy(rebuilt + 2, state->runs, 7u * sizeof(rebuilt[0]));
        element_model_t first;
        if (!model(rebuilt, reverse != 0u, &first) || first.symbol != '*' ||
            state->leading_mm < QUIET_MIN_MM) {
            continue;
        }
        memmove(state->runs + 2, state->runs, 9u * sizeof(state->runs[0]));
        state->runs[0] = rebuilt[0];
        state->runs[1] = rebuilt[1];
        state->elements = 11u;
        state->direction = (uint8_t)(reverse + 1u);
        state->narrow = first.narrow;
        state->wide = first.wide;
        state->bias = first.bias;
        state->space = first.space;
        return true;
    }
    return false;
}

/* End-star fallback (operator 2026-11-20, option F): classify each of the
 * nine end-star runs at the start star's split, (narrow + wide) / 2, after
 * the start star's bar/space bias. The pattern must be exactly '*' (0x094).
 * One edge moved by a SEARCH pivot keeps every run on its side of the split,
 * so the end star still confirms the read. */
static bool end_star_split(const barcode_window_t *state, bool reverse)
{
    const float split = 0.5f * (state->narrow + state->wide);
    uint16_t mask = 0u;
    for (unsigned int i = 0u; i < 9u; ++i) {
        const float raw = state->runs[20u + (reverse ? 8u - i : i)];
        const float value = raw + (i % 2u == 0u ? state->bias : -state->bias);
        if (!isfinite(value) || value <= 0.0f) {
            return false;
        }
        if (value > split) {
            mask |= (uint16_t)(0x100u >> i);
        }
    }
    return mask == 0x094u;
}

/* Letter checks on parts 10..18 against the start star: model, 20% scale
 * and gap 1. Same rules as finish_window. */
static bool letter_ok(const barcode_window_t *state, element_model_t *data)
{
    const bool reverse = state->direction == 2u;
    if (!model(state->runs + 10, reverse, data) ||
        data->symbol < 'A' || data->symbol > 'Z' ||
        fabsf(data->narrow - state->narrow) > 0.2f * state->narrow ||
        fabsf(data->wide - state->wide) > 0.2f * state->wide) {
        return false;
    }
    const float gap1 = state->runs[9] - (state->bias + data->bias) * 0.5f;
    return gap1 >= state->narrow * 0.65f && gap1 <= state->narrow * 1.5f;
}

static bool finish_window(barcode_window_t *state)
{
    element_model_t data, last;
    const bool reverse = state->direction == 2u;
    if (!model(state->runs + 10, reverse, &data)) {
        reject(state, BW_WIDTH);
        return false;
    }
    const bool last_fit = model(state->runs + 20, reverse, &last) &&
        last.symbol == '*' &&
        fabsf(last.narrow - state->narrow) <= 0.2f * state->narrow &&
        fabsf(last.wide - state->wide) <= 0.2f * state->wide;
    if (!last_fit) {
        if (!end_star_split(state, reverse)) {
            reject(state, BW_WIDTH);
            return false;
        }
        last = (element_model_t){state->narrow, state->wide, state->bias,
                                 state->space, '*'};
    }
    if (fabsf(data.narrow - state->narrow) > 0.2f * state->narrow ||
        fabsf(data.wide - state->wide) > 0.2f * state->wide) {
        reject(state, BW_WIDTH);
        return false;
    }
    const float gap1 = state->runs[9] - (state->bias + data.bias) * 0.5f;
    const float gap2 = state->runs[19] - (data.bias + last.bias) * 0.5f;
    if (gap1 < state->narrow * 0.65f || gap1 > state->narrow * 1.5f ||
        gap2 < state->narrow * 0.65f || gap2 > state->narrow * 1.5f) {
        reject(state, BW_GAP);
        return false;
    }
    float extent = (state->bias + data.bias + last.bias) / 3.0f;
    for (unsigned int i = 0u; i < BW_ELEMENTS; ++i) {
        extent += state->runs[i];
    }
    const float expected = 20.0f * state->narrow + 9.0f * state->wide;
    if (fabsf(extent - expected) > 2.0f * state->narrow) {
        reject(state, BW_EXTENT);
        return false;
    }
    if (data.symbol < 'A' || data.symbol > 'Z') {
        reject(state, BW_NOT_LETTER);
        state->result.letter = data.symbol;
        return false;
    }
    state->letter = data.symbol;
    state->tail_narrow = fmaxf(state->narrow, last.narrow);
    state->tail_start = -1.0f;
    state->phase = TAIL;
    return true;
}

static bool rise_seen(barcode_window_t *state, uint16_t raw);
static void note_bar(barcode_window_t *state);
static void window_mid(barcode_window_t *state, barcode_window_adc_t sample,
                       float mm, const barcode_window_motor_t *upper);
static void provisional_hold(barcode_window_t *state,
                             barcode_window_adc_t sample, float mm,
                             const barcode_window_motor_t *upper);

static void tail(barcode_window_t *state, barcode_window_adc_t sample,
                 float mm, const barcode_window_motor_t *upper)
{
    if ((state->mid ? rise_seen(state, sample.raw) : sample.raw >= state->rise) ||
        mm - state->edge_mm > 2.5f * state->tail_narrow) {
        reject(state, BW_TAIL);
        return;
    }
    /* Tail white. Midpoint mode: floor white up to the fixed fall level
     * (white + 75); the 2.5 mm trailing white has 25-count noise. */
    if (sample.raw < WHITE_MIN || sample.raw > state->fall ||
        (!state->mid && sample.raw > WHITE_MAX)) {
        state->tail_start = -1.0f;
        return;
    }
    if (state->tail_start < 0.0f) {
        state->tail_start = mm;
    }
    if (mm - state->tail_start < state->tail_narrow) {
        return;
    }
    const unsigned int index = (unsigned int)(state->letter - 'A');
    state->result = (barcode_window_result_t){
        .serial = next_serial(state), .reason = BW_OK,
        .start_us = state->start_us, .edge_us = state->edge_us,
        .frame_us = state->frame_us,
        .frame_left = state->frame_left, .frame_right = state->frame_right,
        .eligible_us = sample.us, .read_us = upper->us,
        .read_left = upper->left, .read_right = upper->right,
        .narrow_mm = state->narrow, .wide_mm = state->wide,
        .leading_mm = state->leading_mm,
        .rise = state->rise, .fall = state->fall,
        .letter = state->letter, .direction = state->direction,
        .mapped_action = barcode_window_table_confirmed() ? actions[index] : 0u,
        .accepted = true};
    state->phase = COMPLETE;
}

void barcode_window_freeze(barcode_window_t *state, uint32_t us)
{
    if (!state->frozen) {
        state->frozen = true;
        state->thaw = false;
        state->freeze_us = us;
    }
}

void barcode_window_thaw(barcode_window_t *state, uint32_t us)
{
    if (state->frozen) {
        state->frozen = false;
        state->thaw = true;
        state->thaw_us = us;
    }
}

static bool edge(barcode_window_t *state, barcode_window_adc_t sample,
                 float t, const barcode_window_motor_t *upper);

/* First crossing of each 256-count level since the last run extreme. */
static void record(barcode_window_t *state, uint16_t raw, float mm,
                   uint16_t prev_raw, float prev_mm)
{
    if (raw == prev_raw) {
        return;
    }
    const bool falling = raw < prev_raw;
    const unsigned int lo = falling ? raw : prev_raw;
    const unsigned int hi = falling ? prev_raw : raw;
    float *table = falling ? state->fall_mm : state->rise_mm;
    uint32_t *set = falling ? &state->fall_set : &state->rise_set;
    for (unsigned int b = lo / LEVEL_STEP + 1u;
         b * LEVEL_STEP <= hi && b < BW_LEVELS; ++b) {
        const uint32_t bit = UINT32_C(1) << b;
        if ((*set & bit) == 0u) {
            const float f = ((float)(b * LEVEL_STEP) - (float)prev_raw) /
                            ((float)raw - (float)prev_raw);
            table[b] = prev_mm + f * (mm - prev_mm);
            *set |= bit;
        }
    }
}

/* Position where the edge crossed level; fallback when not recorded. */
static float lookup(const float *table, uint32_t set, float level,
                    float fallback)
{
    const unsigned int b = (unsigned int)(level / (float)LEVEL_STEP);
    if (b + 1u >= BW_LEVELS) {
        return fallback;
    }
    const bool a = (set & (UINT32_C(1) << b)) != 0u;
    const bool c = (set & (UINT32_C(1) << (b + 1u))) != 0u;
    if (a && c) {
        const float f = (level - (float)(b * LEVEL_STEP)) / (float)LEVEL_STEP;
        return table[b] + f * (table[b + 1u] - table[b]);
    }
    return a ? table[b] : c ? table[b + 1u] : fallback;
}

/* A new bar after a white run: the envelope detection rule. */
static bool rise_seen(barcode_window_t *state, uint16_t raw)
{
    if (raw < state->run_valley) {
        state->run_valley = raw;
    }
    const float lift = (float)raw - (float)state->run_valley;
    return lift >= EDGE_MIN_SWING && lift >= EDGE_DETECT *
        ((float)state->ref_peak - (float)state->run_valley);
}

static void process(barcode_window_t *state, barcode_window_adc_t sample,
                    float mm, const barcode_window_motor_t *upper)
{
    const uint16_t prev_raw = state->prev_raw;
    const float prev_mm = state->prev_mm;
    state->prev_raw = sample.raw;
    state->prev_mm = mm;
    if (state->phase == SEEK) {
        const bool after_freeze = (state->frozen || state->thaw) &&
            (int32_t)(sample.us - state->freeze_us) >= 0;
        if (after_freeze && (state->frozen ||
                             (int32_t)(sample.us - state->thaw_us) < 0)) {
            /* Blip grace: the white run, mean and count wait. Ink seen in
             * the frozen travel still ends the run: no bar is bridged. */
            if (state->white_started && (float)sample.raw >=
                fminf(state->white_mean + 150.0f, 350.0f)) {
                state->white_lost |= BW_WHITE_LOST_BLIP;
                state->white_started = false;
                state->white_count = 0u;
                state->transitioning = false;
            }
            return;
        }
        if (after_freeze) {
            state->thaw = false;
            if (state->white_started &&
                (sample.raw < WHITE_MIN || sample.raw > WHITE_MAX)) {
                /* Not white after the blip: a bar may have started in the
                 * frozen travel, so the run cannot bridge the blip. */
                state->white_lost |= BW_WHITE_LOST_BLIP;
                state->white_started = false;
                state->white_count = 0u;
                state->transitioning = false;
                state->seek_mm = mm;
                return;
            }
            /* Frozen travel is not white: move the run start forward by
             * it. The mean and the count continue. */
            state->white_start += mm - state->seek_mm;
        }
        state->seek_mm = mm;
        if (sample.raw >= WHITE_MIN && sample.raw <= WHITE_MAX) {
            if (!state->white_started) {
                state->white_start = mm;
                state->white_mean = (float)sample.raw;
                state->white_count = 0u;
                state->white_started = true;
            }
            state->white_mean +=
                ((float)sample.raw - state->white_mean) / 16.0f;
            state->white_end = mm;
            if (state->white_count < 16u) {
                ++state->white_count;
            }
            state->transitioning = false;
            return;
        }
        /* Only qualified pre-window white sets the scale. Mat-line black
         * never does; retain the operator's cap and 75-count hysteresis. */
        const float rise = fminf(state->white_mean + 150.0f, 350.0f);
        const float fall = fminf(state->white_mean + 75.0f, rise - 75.0f);
        if (state->white_started && state->white_count >= 16u &&
            state->white_end - state->white_start >= QUIET_MIN_MM &&
            (float)sample.raw >= rise) {
            state->leading_mm = state->white_end - state->white_start;
            state->rise = (uint16_t)lroundf(rise);
            state->fall = (uint16_t)lroundf(fall);
            state->first_mm = mm;
            state->edge_mm = mm;
            state->start_us = sample.us;
            state->elements = 0u;
            state->black = true;
            state->det_mm = mm;
            state->run_peak = sample.raw;
            state->ref_valley = (uint16_t)lroundf(state->white_mean);
            state->fall_set = 0u;
            state->rise_set = 0u;
            record(state, sample.raw, mm, prev_raw, prev_mm);
            state->started = false;
            state->mode_set = false;
            state->mid = false;
            state->closing = false;
            state->bars = 0u;
            state->shifts = 0u;
            state->held = false;
            state->held_tried = false;
            state->phase = WINDOW;
#if BARCODE_FAIL_CAPTURE
            bcfc_start(state, sample, mm, upper);
#endif
        } else if ((float)sample.raw >= rise) {
            /* Unadmitted ink ends white, even when its run is short. */
            if (state->white_started) {
                state->white_lost |= BW_WHITE_LOST_BLACK;
            }
            state->white_started = false;
            state->white_count = 0u;
            state->transitioning = false;
        } else if (state->white_started) {
            if (!state->transitioning) {
                state->transition_start = mm;
                state->transitioning = true;
            }
            if (mm - state->transition_start > N_MAX) {
                state->white_lost |= BW_WHITE_LOST_OTHER;
                state->white_started = false;
            }
        }
        return;
    }
    if (sample.us - state->start_us > MAX_WINDOW_US ||
        mm - state->first_mm > 360.0f) {
        reject(state, BW_EXTENT);
        return;
    }
    if (state->phase == TAIL) {
        tail(state, sample, mm, upper);
        return;
    }
    if (state->black && sample.raw > state->run_peak) {
        state->run_peak = sample.raw;
        state->fall_set = 0u;
    }
    if (!state->black && sample.raw < state->run_valley) {
        state->run_valley = sample.raw;
        state->rise_set = 0u;
    }
    record(state, sample.raw, mm, prev_raw, prev_mm);
    if (!state->mode_set) {
        /* Mode from the first bar: a strong bar (small spot) uses the local
         * midpoint; a weak bar (5.5 mm mount) keeps the fixed levels. */
        if ((float)state->run_peak - state->white_mean >= STRONG_EXCESS) {
            state->mode_set = true;
            state->mid = true;
            state->started = false;
        } else if (sample.raw <= state->fall) {
            state->mode_set = true;
            state->mid = false;
        }
    }
    if (state->mid) {
        window_mid(state, sample, mm, upper);
        return;
    }
    const bool black = state->black ? sample.raw > state->fall :
                                     sample.raw >= state->rise;
    /* Before the star, keep the absolute fail-closed ceiling. Afterwards a
     * raw run can include the fitted bar/space edge bias; never enlarge the
     * absolute ceiling when applying the scale-relative class tolerance. */
    const float run_limit = state->elements >= 9u ?
        fminf(35.0f, 1.5f * state->wide + fabsf(state->bias)) : 35.0f;
    if (mm - state->edge_mm > run_limit) {
        reject(state, BW_WIDTH);
        return;
    }
    if (black == state->black) {
        return;
    }
    if (black) {
        state->run_peak = sample.raw;
    } else {
        note_bar(state);
        state->run_valley = sample.raw;
    }
    state->black = black;
    (void)edge(state, sample, mm, upper);
}

static void note_bar(barcode_window_t *state)
{
    if (state->bars < BW_PEAKS) {
        state->bar_peak[state->bars] = state->run_peak;
    }
    if (state->bars < UINT8_MAX) {
        ++state->bars;
    }
}

/* Local-midpoint WINDOW step (strong first bar). */
static void window_mid(barcode_window_t *state, barcode_window_adc_t sample,
                       float mm, const barcode_window_motor_t *upper)
{
    if (state->closing) {
        /* Last bar: its falling edge is the midpoint to floor white. */
        if (rise_seen(state, sample.raw)) {
            reject(state, BW_TAIL);
            return;
        }
        const float level = 0.5f * ((float)state->ref_peak + state->white_mean);
        if ((float)sample.raw <= level) {
            state->closing = false;
            (void)edge(state, sample, lookup(state->fall_mm, state->fall_set,
                                             level, mm), upper);
        } else if (mm - state->det_mm > 35.0f) {
            reject(state, BW_WIDTH);
        }
        return;
    }
    const float run_limit = state->elements >= 9u ?
        fminf(35.0f, 1.5f * state->wide + fabsf(state->bias)) : 35.0f;
    if (mm - state->det_mm > run_limit) {
        reject(state, BW_WIDTH);
        return;
    }
    if (state->black) {
        const float drop = (float)state->run_peak - (float)sample.raw;
        if (drop < EDGE_MIN_SWING || drop < EDGE_DETECT *
            ((float)state->run_peak - (float)state->ref_valley)) {
            return;
        }
        /* Bar ended: place the rising edge before it at its midpoint. */
        const float level = 0.5f * ((float)state->ref_valley +
                                    (float)state->run_peak);
        const float t = lookup(state->rise_mm, state->rise_set, level,
                               state->det_mm);
        note_bar(state);
        state->ref_peak = state->run_peak;
        state->run_valley = sample.raw;
        state->rise_set = 0u;
        state->black = false;
        state->det_mm = mm;
        if (!edge(state, sample, t, upper)) {
            return;
        }
        if (state->elements == BW_ELEMENTS - 1u) {
            state->closing = true;
        }
        return;
    }
    if (state->elements == 18u) {
        provisional_hold(state, sample, mm, upper);
    }
    const float lift = (float)sample.raw - (float)state->run_valley;
    if (lift < EDGE_MIN_SWING || lift < EDGE_DETECT *
        ((float)state->ref_peak - (float)state->run_valley)) {
        return;
    }
    /* Space ended: place the falling edge before it at its midpoint. */
    const float level = 0.5f * ((float)state->ref_peak +
                                (float)state->run_valley);
    const float t = lookup(state->fall_mm, state->fall_set, level,
                           state->det_mm);
    state->ref_valley = state->run_valley;
    state->run_peak = sample.raw;
    state->fall_set = 0u;
    state->black = true;
    state->det_mm = mm;
    (void)edge(state, sample, t, upper);
}

/* The first bar is faint when its contrast over floor white is below half
 * of the weaker of the next two bars (image 20: a mark 1111 counts high,
 * 5 mm before bars of 2800+). Narrow small-sheet bars blur to about 70% of
 * the wide bars (synthetic 2769 against 3914), so 50% keeps them. */
static bool faint_first(const barcode_window_t *state)
{
    const float weaker = fminf((float)state->bar_peak[1],
                               (float)state->bar_peak[2]);
    return (float)state->bar_peak[0] - state->white_mean <
           0.5f * (weaker - state->white_mean);
}

/* Hold the letter when parts 1..19 pass the letter checks; the real part
 * 19 replaces a provisional one. */
static void hold_letter(barcode_window_t *state, barcode_window_adc_t sample,
                        const barcode_window_motor_t *upper)
{
    element_model_t data;
    state->held = letter_ok(state, &data);
    if (state->held) {
        state->held_letter = data.symbol;
        state->held_edge_us = sample.us;
        state->held_frame_us = upper->us;
        state->held_frame_left = upper->left;
        state->held_frame_right = upper->right;
    }
}

/* Midpoint mode places a bar's falling edge only when the next bar rises.
 * When the letter's last bar has fallen through the floor midpoint (the
 * same rule as the closing bar), test a provisional part 19 so that the
 * letter is held even if no further bar is seen. */
static void provisional_hold(barcode_window_t *state,
                             barcode_window_adc_t sample, float mm,
                             const barcode_window_motor_t *upper)
{
    const float level = 0.5f * ((float)state->ref_peak + state->white_mean);
    if (state->held_tried || (float)sample.raw > level) {
        return;
    }
    state->held_tried = true;
    state->runs[18] = lookup(state->fall_mm, state->fall_set, level, mm) -
                      state->edge_mm;
    hold_letter(state, sample, upper);
}

/* One edge at position t. The first edge only starts the window. False
 * when the window rejected or finished. */
static bool edge(barcode_window_t *state, barcode_window_adc_t sample,
                 float t, const barcode_window_motor_t *upper)
{
    if (state->mid && !state->started) {
        state->started = true;
        state->edge_mm = t;
        return true;
    }
    state->runs[state->elements++] = t - state->edge_mm;
    state->edge_mm = t;
    if (state->mid && state->elements == 5u && state->shifts < 2u &&
        faint_first(state)) {
        /* Three bars seen: drop the faint mark and its space. */
        memmove(state->runs, state->runs + 2, 3u * sizeof(state->runs[0]));
        memmove(state->bar_peak, state->bar_peak + 1,
                2u * sizeof(state->bar_peak[0]));
        state->elements = 3u;
        state->bars = 2u;
        ++state->shifts;
        return true;
    }
    if (state->elements == 9u && !start_star(state) && !late_start(state)) {
        reject(state, BW_START);
        return false;
    }
    if (state->elements == 19u) {
        hold_letter(state, sample, upper);
    }
    if (state->elements == BW_ELEMENTS) {
        state->edge_us = sample.us;
        state->frame_us = upper->us;
        state->frame_left = upper->left;
        state->frame_right = upper->right;
        if (finish_window(state)) {
            state->tail_start = -1.0f;
            tail(state, sample, t, upper);
        }
        return false;
    }
    return true;
}

unsigned int barcode_window_step(barcode_window_t *state, unsigned int budget)
{
    unsigned int processed = 0u;
    if (budget > BW_STEP_MAX) {
        budget = BW_STEP_MAX;
    }
    while (state->phase != DISABLED && state->phase != COMPLETE &&
           state->queued > 0u && processed < budget) {
        const barcode_window_adc_t sample = state->adc[state->head];
        if (state->motor_count < 2u) {
            break;
        }
        unsigned int at;
        for (at = 1u; at < state->motor_count; ++at) {
            const uint32_t span = state->motors[at].us -
                                  state->motors[at - 1u].us;
            if (sample.us - state->motors[at - 1u].us <= span) {
                break;
            }
        }
        if (at == state->motor_count) {
            const uint32_t future = sample.us - state->motors[at - 1u].us;
            if (future > 0u && future < UINT32_C(0x80000000)) {
                break; /* Need a future source bracket, never extrapolate. */
            }
            loss(state, BW_LOSS);
            break;
        }
        const barcode_window_motor_t *a = &state->motors[at - 1u];
        const barcode_window_motor_t *b = &state->motors[at];
        const float fraction = (float)(sample.us - a->us) /
                               (float)(b->us - a->us);
        const float mm = a->mm + (b->mm - a->mm) * fraction;
        state->head = (uint8_t)((state->head + 1u) % BW_ADC_CAPACITY);
        --state->queued;
        ++processed;
#if BARCODE_FAIL_CAPTURE
        bcfc_observe(state, sample, mm, b);
#endif
        if (b->mm == a->mm) {
            continue; /* Held retreat: this position was already read. */
        }
        process(state, sample, mm, b);
    }
    return processed;
}

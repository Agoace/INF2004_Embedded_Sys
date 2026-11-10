/* Course-only window policy; frozen TEST framing is deliberately unchanged. */
#include "barcode_window.h"
#include "barcode.h"
#include <math.h>
#include <string.h>

#define DISABLED 0u
#define SEEK 1u
#define WINDOW 2u
#define TAIL 3u
#define COMPLETE 4u
#define WHITE_MIN 120u
#define WHITE_MAX 220u
#define N_MIN 4.6f
#define N_MAX 7.0f
#define W_MIN 13.5f
#define W_MAX 21.5f
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
    12414u, 1500u, 30000u};
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

/* Failure abandons the whole candidate. Never repair or slide its runs. */
static void reject(barcode_window_t *state, barcode_window_reason_t reason)
{
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
        loss(state, BW_LOSS);
        return;
    }
    if (state->queued == BW_ADC_CAPACITY) {
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
            dl + dr < 0) {
            loss(state, BW_MOTION);
            state->motor_count = 0u;
        } else {
            mm = last->mm + (float)(dl + dr) * 500.0f /
                 (float)state->config.counts_per_m;
        }
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
    if (n < N_MIN || n > N_MAX || w < W_MIN || w > W_MAX ||
        w / n < 1.6f || w / n > 3.4f || low[1] / high[0] < 1.4f ||
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
            if (state->leading_mm < 10.0f * first.narrow) {
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

static bool finish_window(barcode_window_t *state)
{
    element_model_t data, last;
    const bool reverse = state->direction == 2u;
    if (!model(state->runs + 10, reverse, &data) ||
        !model(state->runs + 20, reverse, &last) || last.symbol != '*') {
        reject(state, BW_WIDTH);
        return false;
    }
    if (fabsf(data.narrow - state->narrow) > 0.2f * state->narrow ||
        fabsf(last.narrow - state->narrow) > 0.2f * state->narrow ||
        fabsf(data.wide - state->wide) > 0.2f * state->wide ||
        fabsf(last.wide - state->wide) > 0.2f * state->wide) {
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

static void tail(barcode_window_t *state, barcode_window_adc_t sample,
                 float mm, const barcode_window_motor_t *upper)
{
    if (sample.raw >= state->rise ||
        mm - state->edge_mm > 2.5f * state->tail_narrow) {
        reject(state, BW_TAIL);
        return;
    }
    if (sample.raw < WHITE_MIN || sample.raw > WHITE_MAX ||
        sample.raw > state->fall) {
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

static void process(barcode_window_t *state, barcode_window_adc_t sample,
                    float mm, const barcode_window_motor_t *upper)
{
    if (state->phase == SEEK) {
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
            state->white_end - state->white_start >= 10.0f * N_MIN &&
            (float)sample.raw >= rise) {
            state->leading_mm = state->white_end - state->white_start;
            state->rise = (uint16_t)lroundf(rise);
            state->fall = (uint16_t)lroundf(fall);
            state->first_mm = mm;
            state->edge_mm = mm;
            state->start_us = sample.us;
            state->elements = 0u;
            state->black = true;
            state->phase = WINDOW;
        } else if (state->white_started) {
            if (!state->transitioning) {
                state->transition_start = mm;
                state->transitioning = true;
            }
            if (mm - state->transition_start > N_MAX) {
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
    const bool black = state->black ? sample.raw > state->fall :
                                     sample.raw >= state->rise;
    if (mm - state->edge_mm > 35.0f) {
        reject(state, BW_WIDTH);
        return;
    }
    if (black == state->black) {
        return;
    }
    state->runs[state->elements++] = mm - state->edge_mm;
    state->edge_mm = mm;
    state->black = black;
    if (state->elements == 9u && !start_star(state)) {
        reject(state, BW_START);
    } else if (state->elements == BW_ELEMENTS) {
        state->edge_us = sample.us;
        state->frame_us = upper->us;
        state->frame_left = upper->left;
        state->frame_right = upper->right;
        if (finish_window(state)) {
            tail(state, sample, mm, upper);
        }
    }
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
        process(state, sample, mm, b);
    }
    return processed;
}

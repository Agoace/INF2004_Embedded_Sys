/**
 * @file line_steer.h
 * @brief Line steering policy: normalised line position to Motion's signed
 *        steer, plus the calm choice.
 *
 * Forked from `yx_line_tracing` @1ea095c with one deliberate change: YX emits
 * left and right wheel *duty percentages*, and this target emits only Motion's
 * bounded signed steer plus the cruise/calm choice. Motion's per-wheel PI keeps
 * the PWM, so `min_pct`, `max_pct` and `right_trim_pct` are dropped.
 *
 * Mapping. YX's turn value is kept in its own units:
 *
 *   turn = gain_pct * position + kd_pct * change
 *
 * where turn 10000 means "the inner wheel stops". Motion's steer is the
 * inner-wheel scale itself:
 *
 *   inner = base * (127 - |steer|) / 127
 *
 * so
 *
 *   steer = round(127 * turn / 10000), clamped to -127..+127.
 *
 * `gain_pct` and `kd_pct` therefore keep their upstream meaning, and the values
 * tuned on the bench transfer unchanged.
 *
 * Sign convention: position > 0 means the line is to the right, which must
 * produce a positive steer so the left wheel runs faster and the robot turns
 * right toward the line.
 *
 * Pure logic, no hardware: host-tested by tests/line.
 */
#ifndef LINE_STEER_H
#define LINE_STEER_H

#include <stdbool.h>
#include <stdint.h>

/** Largest position change per update used by the D part (upstream value). */
#define LINE_STEER_MAX_CHANGE 25

/** Full-scale steer, matching motion_set_steer()'s range. */
#define LINE_STEER_FULL 127

/* Calm-mode thresholds and dwell, unchanged from upstream. */
#define LINE_STEER_CALM_ENTER   50
#define LINE_STEER_CALM_NEAR    20
#define LINE_STEER_CALM_HOLD_MS 300u

typedef struct
{
    int16_t gain_pct;   /**< P: 100 stops the inner wheel at |position| 100. */
    int16_t kd_pct;     /**< D: reaction to the change since the last update. */
} line_steer_config_t;

typedef struct
{
    line_steer_config_t cfg;
    int8_t   previous;
    bool     has_previous;
    bool     calm;
    bool     near;
    uint32_t near_ms;
} line_steer_state_t;

/** One steering decision for the Manager. */
typedef struct
{
    int8_t steer;       /**< -127..+127, positive turns right.               */
    bool   straight;    /**< Junction: hold the cruise speed, ignore position. */
    bool   calm;        /**< Request Motion's slow mode.                     */
} line_steer_output_t;

void line_steer_init(line_steer_state_t * p_state,
                     const line_steer_config_t * p_cfg);

/**
 * @brief Forget the D term and re-arm calm.
 *
 * Called on stop, fault and abort, and before every run (RT-0031 R4), so the
 * first correction after a restart has no stored kick.
 */
void line_steer_reset(line_steer_state_t * p_state);

/**
 * @brief One steering decision.
 *
 * @param position Signed line position, -100..+100.
 * @param trigger  True when this update follows a junction or a line search, so
 *                 the calm settings apply.
 * @param now_ms   Monotonic milliseconds; the D and calm timers use it.
 */
line_steer_output_t line_steer_update(line_steer_state_t * p_state,
                                      int8_t position, bool trigger,
                                      uint32_t now_ms);

/**
 * @brief Drive straight across a junction.
 *
 * Zero steer, and the D term is forgotten so the next update starts clean. A
 * junction also triggers calm.
 */
line_steer_output_t line_steer_straight(line_steer_state_t * p_state,
                                        uint32_t now_ms);

#endif /* LINE_STEER_H */

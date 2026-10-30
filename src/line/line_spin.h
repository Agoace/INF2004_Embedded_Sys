/**
 * @file line_spin.h
 * @brief Bounded, non-blocking in-place sweep used for line calibration and for
 *        line SEARCH (recovery).
 *
 * The upstream `line_spin_calibrate()` owns the drive and encoder calls and
 * blocks with `sleep_us()`. That cannot run inside this firmware, where Motion
 * owns PWM and the encoders and nothing may block the 1 ms loop. This module
 * keeps only the *sequencing*: it asks for one rotation at a time and waits for
 * Motion to report completion.
 *
 * Sequence, all three segments bounded by Motion's own rotate limits:
 *
 *   1. rotate toward the side the line was last seen on,
 *   2. rotate through the start to the opposite extreme, which is where the
 *      sensors cross the line and the calibration learns its endpoints,
 *   3. rotate back toward the start, stopping early if the line comes back
 *      under the centre sensor.
 *
 * Calibration is collected only during segments 1 and 2, which is exactly when
 * `line_spin_collecting()` is true. The caller feeds `lp_calib_add()` then, and
 * evaluates the result once the machine reaches `LINE_SPIN_PHASE_EVALUATE`.
 *
 * The same sequence serves SEARCH: nothing is committed to the calibration, and
 * the caller only asks whether the line came back.
 *
 * Pure logic, no hardware: host-tested by tests/line.
 */
#ifndef LINE_SPIN_H
#define LINE_SPIN_H

#include <stdbool.h>
#include <stdint.h>

#include "line_types.h"

/**
 * Sweep amplitude for one segment, in chassis degrees.
 *
 * One eighth of a wheel turn is 2535/8 = 317 counts, which is about 25.5 mm of
 * wheel travel, and about 24 degrees of rotation on the 120 mm wheelbase. That
 * is deliberately narrower than upstream's quarter turn: the search only needs
 * to bring the line under a neighbouring sensor. MEASURE on the bench.
 */
#define LINE_SPIN_SWEEP_DEG 24

/** Overall budget for the whole sequence, from upstream's per-step limit. */
#define LINE_SPIN_TIMEOUT_MS 6000u

typedef enum
{
    LINE_SPIN_MODE_CALIBRATE = 0,   /**< sweep, then commit calibration.      */
    LINE_SPIN_MODE_SEARCH    = 1    /**< sweep only: is the line back?        */
} line_spin_mode_t;

typedef enum
{
    LINE_SPIN_PHASE_IDLE = 0,
    LINE_SPIN_PHASE_TO_FIRST,   /**< segment 1 in flight                     */
    LINE_SPIN_PHASE_TO_SECOND,  /**< segment 2 in flight                     */
    LINE_SPIN_PHASE_RETURN,     /**< segment 3 in flight                     */
    LINE_SPIN_PHASE_EVALUATE,   /**< caller decides the outcome              */
    LINE_SPIN_PHASE_DONE
} line_spin_phase_t;

typedef enum
{
    LINE_SPIN_CMD_NONE = 0,
    LINE_SPIN_CMD_ROTATE,   /**< rotate by *p_degrees, signed              */
    LINE_SPIN_CMD_STOP,     /**< stop Motion now, then evaluate            */
    LINE_SPIN_CMD_TIMEOUT   /**< budget spent: the caller faults or stops  */
} line_spin_cmd_t;

typedef struct
{
    line_spin_mode_t  mode;
    line_spin_phase_t phase;
    int16_t  first_sign;    /**< -1 sweeps left first, +1 right first      */
    bool     segment_sent;  /**< the current segment's rotation was issued */
    bool     timed_out;
    uint32_t start_ms;
} line_spin_t;

void line_spin_init(line_spin_t * p_state);

/**
 * @brief Begin a sweep.
 *
 * @param first_side LINE_SIDE_LEFT, LINE_SIDE_RIGHT, or LINE_SIDE_UNKNOWN which
 *                   sweeps left, matching the plan's "left if unknown".
 */
void line_spin_start(line_spin_t * p_state, line_spin_mode_t mode,
                     line_side_t first_side, uint32_t now_ms);

/**
 * @brief Advance one step.
 *
 * @param motion_idle True when Motion has finished the previous rotation and no
 *                    command is outstanding.
 * @param line_centred True when the line is currently under the centre sensor.
 * @param p_degrees Receives the signed rotation for LINE_SPIN_CMD_ROTATE.
 */
line_spin_cmd_t line_spin_step(line_spin_t * p_state, bool motion_idle,
                               bool line_centred, uint32_t now_ms,
                               int16_t * p_degrees);

/** True while the caller should feed calibration samples. */
bool line_spin_collecting(const line_spin_t * p_state);

/** True while a sweep is in progress and the caller must keep stepping. */
bool line_spin_running(const line_spin_t * p_state);

/** True once the machine reached EVALUATE and is waiting for the outcome. */
bool line_spin_awaiting_result(const line_spin_t * p_state);

/** Record the caller's outcome and finish. */
void line_spin_finish(line_spin_t * p_state);

#endif /* LINE_SPIN_H */

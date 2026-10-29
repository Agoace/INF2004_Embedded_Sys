/*
 * Line steering: line position (-100..+100) to left and right wheel duty.
 *
 * Steering works on wheel speed, not on raw duty. A loaded wheel only
 * starts to turn at about min_pct duty, so min_pct is speed zero and the
 * speed is the duty above it:
 *
 *   speed = base_pct - min_pct
 *   turn  = gain_pct * position + kd_pct * (position - previous position)
 *   left  = speed + speed * turn / 10000      (wheel speeds)
 *   right = speed - speed * turn / 10000
 *   duty  = min_pct + wheel speed, or 0 if the wheel speed is 0 or less
 *
 * So the inner wheel slows down smoothly to a stop. Bench run 2026-10-29:
 * with the old raw-duty rule (duty below min_pct set to 0), the inner wheel
 * dropped from 12 % to 0 at position 41, so the steering jumped between
 * "straight" (20/21) and "pivot" (0/30) about 3 times a second.
 *
 * gain_pct 100: with the line under an outer sensor (position +/-100) the
 * inner wheel stops and the outer wheel runs at twice the base speed.
 * Position > 0 (line to the right) makes the left wheel faster, so the
 * robot turns right toward the line. With min_pct 0 the rule is the plain
 * duty ratio.
 *
 * Trim: right_trim_pct is added to the duty of the right wheel when it
 * turns (a negative trim adds to the left wheel), so the robot drives
 * straight when the two motors differ. The limits below still apply.
 *
 * Limits, in this order:
 *   1. A wheel above max_pct is set to max_pct and the other wheel is
 *      lowered by the same amount, so the turn is kept.
 *   2. A wheel below min_pct (or below 0) is set to 0: it would not turn.
 *      No reverse.
 *
 * Pure logic, no hardware: host-tested by tests/test_line_steer.c.
 * See docs/LINE_STEERING.md.
 */
#ifndef LINE_STEER_H
#define LINE_STEER_H

#include <stdbool.h>
#include <stdint.h>

/* Steer word: bits 0-6 left duty %, bits 7-13 right duty %, bits 14-15 spare. */
typedef uint16_t steer_word_t;
#define STEER_DUTY_MASK 0x7Fu
#define STEER_RIGHT_SHIFT 7u

typedef struct {
    uint8_t base_pct; /* duty of both wheels when the line is centred */
    uint8_t gain_pct; /* P: 100 = inner wheel stops at position +/-100 */
    uint8_t kd_pct;   /* D: reaction to the change since the last update; 0 = off */
    uint8_t min_pct;  /* duty at which a loaded wheel starts to turn: speed zero */
    uint8_t max_pct;  /* highest duty, at most 100 */
    int8_t right_trim_pct; /* + adds to the right wheel, - to the left; 0 = none */
} steer_config_t;

typedef struct {
    steer_config_t cfg;
    int8_t previous;
    bool has_previous;
} steer_t;

void steer_init(steer_t *s, const steer_config_t *cfg);

/* Forget the previous position (call when line following starts), so the
 * first update has no D part. */
void steer_reset(steer_t *s);

/* Call at a fixed period (10 ms). */
steer_word_t steer_update(steer_t *s, int8_t position);

static inline unsigned int steer_left(steer_word_t w) {
    return w & STEER_DUTY_MASK;
}

static inline unsigned int steer_right(steer_word_t w) {
    return (unsigned int)(w >> STEER_RIGHT_SHIFT) & STEER_DUTY_MASK;
}

#endif

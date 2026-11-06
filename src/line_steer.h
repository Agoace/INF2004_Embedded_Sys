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
 * Far gain (optional; far_gain_pct 0 = off): beyond |position| far_from,
 * far_gain_pct times the part beyond is added to the turn:
 *   turn += far_gain_pct * (|position| - far_from), with the sign of position
 * Track runs 2026-11-03 with two line sensors: gain 100 was calm on the
 * straight but turned too little with the line under or just past a
 * sensor (position +46..+61) in a curve; gain 200 stopped a wheel at every
 * sensor crossing (+/-50) and swung the robot from side to side.
 *
 * D (damping): kd_pct times the change of position since the last update
 * (10 ms). While the line moves back toward the centre, D works against
 * the turn, so the robot does not swing past the line. In a steady curve
 * the position does not change, so D does not weaken the turn. The change
 * is limited to +/-STEER_MAX_CHANGE per update: a position jump between
 * sensors only gives a small kick. Bench run 6, 2026-10-29: with D off,
 * base 25 % and max 50 %, the robot swung from 0/50 to 50/0 and back for
 * about 0.8 s after each junction.
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

/* Largest position change per update used by the D part. */
#define STEER_MAX_CHANGE 25

typedef struct {
    uint8_t base_pct; /* duty of both wheels when the line is centred */
    uint8_t gain_pct; /* P: 100 = inner wheel stops at position +/-100 */
    uint8_t kd_pct;   /* D: reaction to the change since the last update; 0 = off */
    uint8_t min_pct;  /* duty at which a loaded wheel starts to turn: speed zero */
    uint8_t max_pct;  /* highest duty, at most 100 */
    int8_t right_trim_pct; /* + adds to the right wheel, - to the left; 0 = none */
    uint8_t far_from;      /* far gain starts above this |position| */
    uint16_t far_gain_pct; /* extra P beyond far_from; 0 = off */
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

/* Drive straight: both wheels at base (with the trim). Use it in place of
 * steer_update() while the line position must not be used, for example
 * across a junction, where a side arm pulls the position to one side. It
 * forgets the previous position, so the next steer_update() has no D part. */
steer_word_t steer_straight(steer_t *s);

/* Change the settings and keep the previous position, so the D part goes
 * on without a jump (used to change between fast and calm settings). */
void steer_set_config(steer_t *s, const steer_config_t *cfg);

/*
 * Calm mode: when to use slower settings.
 *
 * Bench run 9, 2026-10-29: at base 33 % the robot swung 0/55 <-> 55/0 for
 * up to 1.4 s after the T junction; at base 25 % (run 7) the same kick
 * settled in about 0.2 s. So the robot is calm (slower settings):
 *   - at the start,
 *   - after a trigger (junction, line search),
 *   - while |position| >= STEER_CALM_ENTER,
 * and goes back to fast after |position| <= STEER_CALM_NEAR for
 * STEER_CALM_HOLD_MS without a break.
 */
#define STEER_CALM_ENTER 50
#define STEER_CALM_NEAR 20
#define STEER_CALM_HOLD_MS 300u

typedef struct {
    bool calm;
    bool near;           /* |position| <= STEER_CALM_NEAR since near_ms */
    uint32_t near_ms;
} steer_calm_t;

/* Starts calm. */
void steer_calm_init(steer_calm_t *c);

/* Call at each steer update. trigger: a junction or a line search now.
 * Returns true while calm. */
bool steer_calm_update(steer_calm_t *c, int8_t position, bool trigger, uint32_t now_ms);

/*
 * Steering in wheel speed (mm/s), for the motor controller
 * (docs/ADR_MOTOR_CONTROLLER.md). The same rule as steer_update(), with
 * base, min and max in mm/s and no trim: the motor controller holds the
 * wheel speeds, so the wheel differences and the dead zone are not the
 * caller's work.
 *
 * min_mm_s keeps the shape of the duty rule: the inner wheel slows to
 * min_mm_s, then stops (no reverse). It is a steering setting here, not a
 * motor limit. Values for the locked line tracing: docs/LINE_STEER_MM.md.
 *
 * Steer mm word: bits 0-8 left mm/s (0..511), bits 9-17 right, 18-31 spare.
 */
typedef uint32_t steer_mm_word_t;
#define STEER_MM_MASK 0x1FFu
#define STEER_MM_RIGHT_SHIFT 9u
#define STEER_MM_MAX 511

typedef struct {
    int16_t base_mm_s;     /* both wheels when the line is centred */
    uint8_t gain_pct;      /* as steer_config_t */
    uint8_t kd_pct;        /* as steer_config_t */
    int16_t min_mm_s;      /* inner wheel: lowest speed before it stops */
    int16_t max_mm_s;      /* highest wheel speed, at most STEER_MM_MAX */
    uint8_t far_from;      /* as steer_config_t */
    uint16_t far_gain_pct; /* as steer_config_t */
} steer_mm_config_t;

typedef struct {
    steer_mm_config_t cfg;
    int8_t previous;
    bool has_previous;
} steer_mm_t;

void steer_mm_init(steer_mm_t *s, const steer_mm_config_t *cfg);
void steer_mm_reset(steer_mm_t *s);
steer_mm_word_t steer_mm_update(steer_mm_t *s, int8_t position);
steer_mm_word_t steer_mm_straight(steer_mm_t *s);

static inline int16_t steer_mm_left(steer_mm_word_t w) {
    return (int16_t)(w & STEER_MM_MASK);
}
static inline int16_t steer_mm_right(steer_mm_word_t w) {
    return (int16_t)((w >> STEER_MM_RIGHT_SHIFT) & STEER_MM_MASK);
}

static inline unsigned int steer_left(steer_word_t w) {
    return w & STEER_DUTY_MASK;
}

static inline unsigned int steer_right(steer_word_t w) {
    return (unsigned int)(w >> STEER_RIGHT_SHIFT) & STEER_DUTY_MASK;
}

#endif

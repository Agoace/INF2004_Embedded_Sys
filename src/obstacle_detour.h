/*
 * Obstacle detour with straight drives and turns on the spot only (no
 * arcs). Successor of the shelved src/obstacle_bypass.c.
 *
 * Version 1, the simple version. The servo is fixed 30 degrees toward the
 * obstacle. Turns go in turn_step_deg steps: turn, stop look_ms, take one
 * reading, then decide. "Seen" = a valid echo nearer than seen_mm.
 *
 * The caller stops the robot at OBST_STOP, runs one scan, and picks the
 * detour side S from the PROFILE word. The obstacle is on the other side.
 *
 *   BACK         Only when the obstacle (sensor_ahead_mm + obstacle_mm ahead
 *                of the rear axle) is inside the turn circle, sweep_mm +
 *                margin_mm: reverse straight until it is not.
 *   TURN_AWAY    Turn toward S until the obstacle was seen and then is not
 *                seen: the sensor has passed its edge (a bottle: not seen,
 *                seen, not seen; a box: seen, not seen).
 *   STRAIGHT     Straight. A seen echo nearer than every echo of this
 *                straight so far is the new nearest reading: the distance
 *                count starts again from where the robot was when that
 *                reading was triggered. The straight ends vehicle_mm after
 *                the nearest reading (from the start with none). Then
 *                TURN_TOWARD; after max_straights straights, NO_LINE.
 *   TURN_TOWARD  Turn toward the obstacle until it is seen, then back
 *                back_off_deg away from it (no look; runs 1 and 3 of the
 *                detour clipped the corner with no back-off). Then STRAIGHT.
 *   ON_LINE      The line sensors saw the line in a STRAIGHT: straight
 *                line_ahead_mm more, so the rear axle is on the line.
 *   ALIGN        Turn back to heading 0 (along the line). Then DONE.
 *
 * The line counts only after the line sensors lost it once in the detour
 * (the robot starts on the line). The heading is the sum of the turns,
 * from the wheel counts (counts_per_deg_x100), coasting included.
 * NO_RISE and STUCK_HIGH readings give no information: look again. In a
 * straight, a valid echo nearer than DT_TOO_NEAR_MM fails.
 *
 * Pure logic: no hardware or RTOS calls; host-tested by
 * tests/test_obstacle_detour.c.
 */
#ifndef OBSTACLE_DETOUR_H
#define OBSTACLE_DETOUR_H

#include <stdbool.h>
#include <stdint.h>

#include "obstacle_profile.h"
#include "ultrasonic.h"

/* Servo settle time before a reading: base + per degree of movement. The
 * base is also the gap between readings in a straight. */
#define DT_SETTLE_BASE_MS 40u
#define DT_SETTLE_PER_DEG_MS 4u
/* STRAIGHT: an echo nearer than this fails. */
#define DT_TOO_NEAR_MM 80u
/* Wheel: 2535 counts per turn, 204.204 mm circumference. */
#define DT_COUNTS_PER_WHEEL_TURN 2535u
#define DT_WHEEL_CIRCUMFERENCE_UM 204204u

typedef struct {
    uint8_t turn_step_deg;        /* turn this much between looks */
    uint8_t away_max_deg;         /* TURN_AWAY limit */
    uint8_t toward_max_deg;       /* TURN_TOWARD limit */
    uint8_t max_straights;        /* NO_LINE after this many straights */
    uint8_t back_off_deg;         /* TURN_TOWARD: turn back this much after seen */
    uint16_t look_ms;             /* stopped before a look reading */
    uint16_t vehicle_mm;          /* STRAIGHT: distance after the nearest reading */
    uint16_t sweep_mm;            /* rear axle centre to the farthest corner */
    uint16_t margin_mm;           /* extra space round the turn circle */
    uint16_t sensor_ahead_mm;     /* servo axis ahead of the rear axle */
    uint16_t line_ahead_mm;       /* line sensors ahead of the rear axle */
    uint16_t seen_mm;             /* a valid echo nearer than this is the obstacle */
    uint16_t counts_per_deg_x100; /* turn on the spot: (right - left) counts per degree x 100 */
    uint16_t total_max_ms;        /* time limit of the detour */
} dt_config_t;

/* Step: 3 bits. */
typedef enum {
    DT_BACK = 0,
    DT_TURN_AWAY = 1,
    DT_STRAIGHT = 2,
    DT_TURN_TOWARD = 3,
    DT_ON_LINE = 4,
    DT_ALIGN = 5,
    DT_DONE = 6,
    DT_FAILED = 7,
} dt_step_t;

/* Failure reason: 3 bits. */
typedef enum {
    DT_FAIL_NONE = 0,
    DT_FAIL_NO_SIDE = 1,    /* both sides blocked in the scan */
    DT_FAIL_TURN_LIMIT = 2, /* a turn reached its limit */
    DT_FAIL_TIME = 3,       /* the detour took too long */
    DT_FAIL_SENSOR = 4,     /* ultrasonic fault streak */
    DT_FAIL_TOO_NEAR = 5,   /* STRAIGHT: an echo nearer than DT_TOO_NEAR_MM */
    DT_FAIL_NO_LINE = 6,    /* max_straights with no line */
} dt_fail_t;

/*
 * Command word: one 16-bit word.
 *   bits 0-1   motion: 0 STOP, 1 TURN (on the spot), 2 STRAIGHT
 *   bit 2      direction: TURN 0 left, 1 right; STRAIGHT 0 forward, 1 back
 *   bits 3-8   servo angle + 30, 0-60 (positive = left)
 *   bits 9-15  spare, 0
 */
typedef uint16_t dt_word_t;

typedef enum {
    DT_MOVE_STOP = 0,
    DT_MOVE_TURN = 1,
    DT_MOVE_STRAIGHT = 2,
} dt_move_t;

#define DT_MOVE_MASK 0x0003u
#define DT_DIR_BIT 0x0004u
#define DT_SERVO_SHIFT 3u
#define DT_SERVO_MASK 0x3Fu
#define DT_SERVO_BIAS 30

static inline dt_move_t dt_move(dt_word_t w) {
    return (dt_move_t)(w & DT_MOVE_MASK);
}
/* TURN: +1 left, -1 right. STRAIGHT: +1 forward, -1 back. */
static inline int dt_dir(dt_word_t w) {
    return (w & DT_DIR_BIT) ? -1 : +1;
}
static inline int dt_servo_deg(dt_word_t w) {
    return (int)((w >> DT_SERVO_SHIFT) & DT_SERVO_MASK) - DT_SERVO_BIAS;
}

typedef struct {
    dt_config_t cfg;
    uint8_t step;       /* dt_step_t */
    uint8_t fail;       /* dt_fail_t */
    int8_t side;        /* +1: detour left (obstacle right); -1: detour right */
    int8_t servo_deg;   /* commanded servo angle, S frame (+ toward S) */
    bool pending;       /* a reading was triggered and is not in yet */
    bool armed;         /* the line sensors lost the line once */
    bool looking;       /* turn: stopped for a look */
    bool seen_once;     /* TURN_AWAY: the obstacle was seen */
    bool backing;       /* TURN_TOWARD: seen, turning back_off_deg away */
    int32_t back_x100;  /* backing ends when the turn x 100 reaches this */
    uint8_t straights;  /* straights done */
    uint8_t target_deg; /* turn: look when the turn reaches this */
    int16_t heading;    /* degrees, S frame (+ toward S) */
    int32_t x, y;       /* rear axle centre at the start of the step, mm */
    int32_t trig_mm;    /* distance of this step at the pending trigger */
    int32_t from_mm;    /* distance of this step at the nearest reading */
    uint16_t best_mm;   /* nearest echo of this straight (0xFFFF: none) */
    uint32_t leg_mm;    /* BACK, ON_LINE: distance to drive */
    int32_t l0, r0;     /* wheel counts at the start of the step */
    int32_t l, r;       /* wheel counts at the last update */
    uint32_t settle_ms; /* no trigger before this time */
    uint32_t start_ms;  /* start of the detour */
} dt_t;

/* Start a detour at the STOP point. side and obstacle_mm come from the
 * scan (obstacle_profile_side, obstacle_profile_distance). left and right
 * are the wheel counts now. side NONE fails at once (DT_FAIL_NO_SIDE). */
void dt_init(dt_t *dt, const dt_config_t *cfg, obstacle_side_t side, uint16_t obstacle_mm,
             int32_t left, int32_t right, uint32_t now_ms);

/* True when a reading is wanted now: in a STRAIGHT, or stopped for a look
 * in a turn, after the servo or the robot settled. */
bool dt_wants_reading(const dt_t *dt, uint32_t now_ms);

/* Call after ultrasonic_trigger() returned true. */
void dt_triggered(dt_t *dt);

/* Call with each new ultrasonic result. Results with no pending trigger
 * are ignored. */
void dt_reading(dt_t *dt, ultrasonic_result_t result, uint32_t now_ms);

/* Call often (every 10 ms is good) with the line sensor state and the
 * wheel counts. Returns the command for the motors and the servo. Turn
 * looks return STOP. */
dt_word_t dt_update(dt_t *dt, bool line_seen, int32_t left, int32_t right, uint32_t now_ms);

/* Helpers, exported for the host test. */
int dt_sin1000(int deg);                 /* sin x 1000, any angle, rounded */
int dt_cos1000(int deg);                 /* cos x 1000, any angle, rounded */
uint32_t dt_mm_to_counts(uint32_t mm);   /* rounded */
int32_t dt_counts_to_mm(int32_t counts); /* rounded toward zero */

static inline dt_step_t dt_step(const dt_t *dt) {
    return (dt_step_t)dt->step;
}
static inline dt_fail_t dt_fail(const dt_t *dt) {
    return (dt_fail_t)dt->fail;
}

#endif

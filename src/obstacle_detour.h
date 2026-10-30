/*
 * Obstacle detour with straight drives and turns on the spot only (no
 * arcs). Successor of the shelved src/obstacle_bypass.c.
 *
 * Version 1, the simple version: in each straight past the obstacle the
 * sensor looks 30 degrees toward the obstacle, and the straight ends
 * vehicle_mm after the nearest reading of that straight was taken.
 *
 * The caller stops the robot at OBST_STOP, runs one scan, and picks the
 * detour side S from the PROFILE word. The obstacle is on the other side.
 * The path:
 *
 *            B o===============o C
 *             /     +-----+     \
 *            /      | box |      \
 *   line == A o======+-----+======o D == line
 *
 *   BACK       Only when the obstacle (sensor_ahead_mm + obstacle_mm ahead
 *              of the rear axle) is inside the turn circle, sweep_mm +
 *              margin_mm: reverse straight until it is not.
 *   TURN_OUT   Turn turn_deg toward S on the spot.
 *   OUT        Straight, see below.
 *   TURN_BACK  Turn turn_deg back: parallel to the line.
 *   PASS       Straight, see below.
 *   TURN_IN    Turn turn_deg toward the line.
 *   IN         Straight until the line sensors see the line.
 *   ON_LINE    Straight line_ahead_mm more: the rear axle is on the line.
 *   ALIGN      Turn turn_deg toward S: along the line. Then DONE.
 *
 * OUT and PASS: the servo is 30 degrees toward the obstacle. A valid echo
 * nearer than seen_mm, and nearer than every echo of this straight so far,
 * is the new nearest reading: the distance count starts again from where
 * the robot was when that reading was triggered. The straight ends when
 * the count reaches vehicle_mm. With no such echo, it counts from the
 * start of the straight.
 *
 * Turns are measured by the wheel-count difference, straights by the mean
 * wheel count. In IN the servo looks ahead and an echo nearer than
 * DT_TOO_NEAR_MM fails; IN fails when it is twice the length of OUT with
 * no line.
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

/* Servo settle time before a reading: base + per degree of movement. */
#define DT_SETTLE_BASE_MS 40u
#define DT_SETTLE_PER_DEG_MS 4u
/* IN: an echo ahead nearer than this fails. */
#define DT_TOO_NEAR_MM 80u
/* Wheel: 2535 counts per turn, 204.204 mm circumference. */
#define DT_COUNTS_PER_WHEEL_TURN 2535u
#define DT_WHEEL_CIRCUMFERENCE_UM 204204u

typedef struct {
    uint8_t turn_deg;             /* theta, 10-80 */
    uint16_t vehicle_mm;          /* OUT and PASS: distance after the nearest reading */
    uint16_t sweep_mm;            /* rear axle centre to the farthest corner */
    uint16_t margin_mm;           /* extra space round the turn circle */
    uint16_t sensor_ahead_mm;     /* servo axis ahead of the rear axle */
    uint16_t line_ahead_mm;       /* line sensors ahead of the rear axle */
    uint16_t seen_mm;             /* a valid echo nearer than this is the obstacle */
    uint16_t counts_per_deg_x100; /* turn on the spot: (right - left) counts per degree x 100 */
    uint16_t turn_max_ms;         /* time limit of each turn */
    uint16_t total_max_ms;        /* time limit of the detour */
} dt_config_t;

/* Step: 4 bits. */
typedef enum {
    DT_BACK = 0,
    DT_TURN_OUT = 1,
    DT_OUT = 2,
    DT_TURN_BACK = 3,
    DT_PASS = 4,
    DT_TURN_IN = 5,
    DT_IN = 6,
    DT_ON_LINE = 7,
    DT_ALIGN = 8,
    DT_DONE = 9,
    DT_FAILED = 10,
} dt_step_t;

/* Failure reason: 3 bits. */
typedef enum {
    DT_FAIL_NONE = 0,
    DT_FAIL_NO_SIDE = 1,   /* both sides blocked in the scan */
    DT_FAIL_TURN_TIME = 2, /* a turn took too long */
    DT_FAIL_TIME = 3,      /* the detour took too long */
    DT_FAIL_SENSOR = 4,    /* ultrasonic fault streak */
    DT_FAIL_TOO_NEAR = 5,  /* IN: something ahead */
    DT_FAIL_NO_LINE = 6,   /* IN: the line was not found */
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
    int16_t heading;    /* heading of the current step, degrees, S frame */
    int32_t x, y;       /* rear axle centre at the start of the step, mm */
    uint32_t out_mm;    /* length of OUT, for the IN guard */
    int32_t trig_mm;    /* distance of this step at the pending trigger */
    int32_t from_mm;    /* distance of this step at the nearest reading */
    uint16_t best_mm;   /* nearest echo of this straight (0xFFFF: none) */
    uint32_t leg_mm;    /* BACK, IN, ON_LINE: distance to drive */
    int32_t l0, r0;     /* wheel counts at the start of the step */
    int32_t l, r;       /* wheel counts at the last update */
    uint32_t settle_ms; /* no trigger before this time */
    uint32_t step_ms;   /* start of the current step */
    uint32_t start_ms;  /* start of the detour */
} dt_t;

/* Start a detour at the STOP point. side and obstacle_mm come from the
 * scan (obstacle_profile_side, obstacle_profile_distance). left and right
 * are the wheel counts now. side NONE fails at once (DT_FAIL_NO_SIDE). */
void dt_init(dt_t *dt, const dt_config_t *cfg, obstacle_side_t side, uint16_t obstacle_mm,
             int32_t left, int32_t right, uint32_t now_ms);

/* True when the step wants readings and the servo has settled. */
bool dt_wants_reading(const dt_t *dt, uint32_t now_ms);

/* Call after ultrasonic_trigger() returned true. */
void dt_triggered(dt_t *dt);

/* Call with each new ultrasonic result. Results with no pending trigger
 * are ignored. */
void dt_reading(dt_t *dt, ultrasonic_result_t result, uint32_t now_ms);

/* Call often (every 10 ms is good) with the line sensor state and the
 * wheel counts. Returns the command for the motors and the servo. */
dt_word_t dt_update(dt_t *dt, bool line_seen, int32_t left, int32_t right, uint32_t now_ms);

/* Helpers, exported for the host test. */
int dt_sin1000(int deg);                 /* sin x 1000, -90..90, rounded */
int dt_cos1000(int deg);                 /* cos x 1000, -90..90, rounded */
uint32_t dt_mm_to_counts(uint32_t mm);   /* rounded */
int32_t dt_counts_to_mm(int32_t counts); /* rounded toward zero */

static inline dt_step_t dt_step(const dt_t *dt) {
    return (dt_step_t)dt->step;
}
static inline dt_fail_t dt_fail(const dt_t *dt) {
    return (dt_fail_t)dt->fail;
}

#endif

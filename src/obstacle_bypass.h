/*
 * Obstacle bypass by edge following, version 4 (docs/OBSTACLE_BYPASS.md).
 *
 * Assumption: one isolated obstacle on the line, inside the field of view
 * of the scan. The caller stops the robot at OBST_STOP, runs one scan, and
 * picks the bypass side S from the PROFILE word (obstacle_profile_side).
 * The obstacle is then on the other side.
 *
 *   PIVOT   Turn on the spot toward S. The servo looks limit_deg toward
 *           the obstacle. The pivot ends when the obstacle has gone
 *           through the beam and out: one reading that is not free, then
 *           BP_STRAIGHT_CLEAR free readings in a row. Free is the scan rule
 *           (obstacle_profile.h): NO_RETURN, or a valid echo at least
 *           OBSTACLE_PROFILE_GAP_MM farther than the obstacle.
 *   AROUND  Go round the obstacle in three phases. "Seen" is the pivot
 *           rule inverted: a valid echo nearer than free_mm.
 *     DRIVE Drive forward. Steering = angle + clear_angle(clearance, d)
 *           toward S, from the latest seen point (angle, d) or the edge
 *           found in EDGE. clear_angle is the smallest angle a with
 *           sin(a) >= clearance / d, so the sensor end of the robot passes
 *           that point with clearance_mm to spare. The servo moves like a
 *           rubber band: a free reading steps it step_deg toward the
 *           obstacle (the steering does not change); a seen reading sets
 *           the steering and jumps the servo bounce_deg back toward S. A
 *           free reading at the limit toward the obstacle: lost, go to TURN,
 *           but only after the leg: the robot has driven sqrt(d^2 - c^2)
 *           (mean wheel travel) since the latest seen point or edge, so the
 *           sensor end has reached its closest point to it. Before that the
 *           servo reads the limit again. (Runs 12-14: turning back after
 *           about 25 mm made the robot spiral in.)
 *     TURN  Turn on the spot toward the obstacle, the servo at the limit,
 *           until a reading is seen. Then EDGE.
 *     EDGE  Stopped. The servo steps step_deg toward S while the readings
 *           are seen. A free reading is read again; two free readings at
 *           the same angle give the edge: steer edge + clear_angle(last d),
 *           go to DRIVE. (A slanted face can miss one echo.) Seen at the
 *           limit toward S: the edge is the limit.
 *           AROUND starts in DRIVE with the edge at the limit, where the
 *           pivot's last free readings were.
 *   DONE    The line sensors see the line again (not a junction reading),
 *           after it was lost for line_arm_ms. The line follower takes over.
 *   FAILED  Time limit, sensor fault, an obstacle too near ahead, or no
 *           free side. The caller stops the robot.
 *
 * NO_RISE and STUCK_HIGH without a fault streak give no information: the
 * same angle is read again. A fault streak stops the bypass.
 *
 * Angles: degrees, positive = left, seen from behind the robot (the scan
 * convention). Each reading is used at the servo angle of its trigger.
 *
 * Pure logic: no hardware or RTOS calls; host-tested by
 * tests/test_obstacle_bypass.c.
 */
#ifndef OBSTACLE_BYPASS_H
#define OBSTACLE_BYPASS_H

#include <stdbool.h>
#include <stdint.h>

#include "obstacle_profile.h"
#include "ultrasonic.h"

/* Free readings in a row that end the pivot. */
#define BP_STRAIGHT_CLEAR 2u
/* Servo settle time before a reading: base + per degree of movement.
 * Fits the scan settle times (60 ms for 5 degrees, 150 ms for 30). */
#define BP_SETTLE_BASE_MS 40u
#define BP_SETTLE_PER_DEG_MS 4u
/* A hit nearer than this within BP_TOO_NEAR_DEG of straight ahead fails. */
#define BP_TOO_NEAR_MM 60u
#define BP_TOO_NEAR_DEG 10

/* Free readings at one angle that give the edge in EDGE. */
#define BP_EDGE_FREE 2u

/* Robot geometry, operator measurements 2026-10-30 (about):
 *   wheel contact to wheel contact 111 mm, servo axis to rear axle 65 mm,
 *   wheel diameter 65 mm (circumference 204.204 mm). */
#define BP_HALF_TRACK_TENTHS_MM 555u
#define BP_WHEELBASE_TENTHS_MM 650u
#define BP_COUNTS_PER_WHEEL_TURN 2535u
#define BP_WHEEL_CIRCUMFERENCE_UM 204204u

typedef struct {
    uint16_t clearance_mm;  /* obstacle point to the robot centre line */
    uint8_t limit_deg;      /* servo and steering limit, each side (<= 30) */
    uint8_t step_deg;       /* servo step (DRIVE: toward the obstacle; EDGE: toward S) */
    uint8_t bounce_deg;     /* DRIVE: servo step back toward S after a seen reading */
    uint16_t line_arm_ms;   /* line lost this long before it can end the bypass */
    uint16_t pivot_max_ms;  /* time limit of the pivot and of each TURN */
    uint16_t around_max_ms; /* time limit from the start of AROUND */
} bp_config_t;

/* State: 2 bits. */
typedef enum {
    BP_PIVOT = 0,
    BP_AROUND = 1,
    BP_DONE = 2,
    BP_FAILED = 3,
} bp_state_t;

/* AROUND phase: 2 bits. */
typedef enum {
    BP_PHASE_DRIVE = 0,
    BP_PHASE_TURN = 1,
    BP_PHASE_EDGE = 2,
} bp_phase_t;

/* Failure reason: 3 bits. */
typedef enum {
    BP_FAIL_NONE = 0,
    BP_FAIL_NO_SIDE = 1,      /* both sides blocked in the scan */
    BP_FAIL_PIVOT_TIME = 2,   /* pivot or TURN too long */
    BP_FAIL_AROUND_TIME = 3,  /* line not found again */
    BP_FAIL_SENSOR = 4,       /* ultrasonic fault streak */
    BP_FAIL_TOO_NEAR = 5,     /* obstacle less than 60 mm ahead */
} bp_fail_t;

/* Motion: 2 bits. */
typedef enum {
    BP_MOVE_STOP = 0,
    BP_MOVE_PIVOT = 1, /* on the spot; direction = sign of the steer field */
    BP_MOVE_STEER = 2, /* forward with the steering angle */
} bp_move_t;

/*
 * Command word: one 16-bit word.
 *   bits 0-1    motion (bp_move_t)
 *   bits 2-7    steering angle + 30, 0-60 (PIVOT: 60 = left, 0 = right)
 *   bits 8-13   servo angle + 30, 0-60
 *   bits 14-15  spare, always 0
 */
typedef uint16_t bp_word_t;

#define BP_ANGLE_BIAS 30
#define BP_MOVE_MASK 0x0003u
#define BP_STEER_SHIFT 2u
#define BP_SERVO_SHIFT 8u
#define BP_ANGLE_MASK 0x3Fu

static inline bp_move_t bp_move(bp_word_t w) {
    return (bp_move_t)(w & BP_MOVE_MASK);
}
static inline int bp_steer_deg(bp_word_t w) {
    return (int)((w >> BP_STEER_SHIFT) & BP_ANGLE_MASK) - BP_ANGLE_BIAS;
}
static inline int bp_servo_deg(bp_word_t w) {
    return (int)((w >> BP_SERVO_SHIFT) & BP_ANGLE_MASK) - BP_ANGLE_BIAS;
}

typedef struct {
    bp_config_t cfg;
    uint8_t state;      /* bp_state_t */
    uint8_t phase;      /* bp_phase_t, in AROUND */
    uint8_t fail;       /* bp_fail_t */
    int8_t side;        /* +1: bypass left (obstacle right); -1: bypass right */
    int8_t servo_deg;   /* commanded servo angle */
    int8_t steer_deg;   /* steering angle in AROUND */
    int8_t reading_deg; /* servo angle of the pending reading */
    bool pending;       /* a reading was triggered and is not in yet */
    bool armed;         /* the line was lost long enough */
    bool seen;          /* PIVOT: the obstacle was in the beam */
    bool dark;
    bool leg_started;     /* DRIVE: leg_from is set */
    uint8_t clear_count; /* PIVOT: free in a row; EDGE: free at this angle */
    uint16_t free_mm;     /* a valid echo this far or farther is free */
    uint16_t last_mm;     /* last distance to the obstacle */
    uint32_t settle_ms;   /* no trigger before this time */
    uint32_t state_ms;    /* start of the current state */
    uint32_t turn_ms;     /* start of the current TURN */
    uint32_t leg_counts;  /* DRIVE: mean wheel travel before TURN is allowed */
    int32_t leg_from;     /* DRIVE: mean wheel count at the start of the leg */
    int32_t travel;       /* mean wheel count at the last bp_update() */
    uint32_t dark_ms;     /* start of "line not seen" */
} bp_t;

/* Start a bypass. side and obstacle_mm come from the scan
 * (obstacle_profile_side, obstacle_profile_distance). side NONE fails at
 * once (BP_FAIL_NO_SIDE). */
void bp_init(bp_t *bp, const bp_config_t *cfg, obstacle_side_t side, uint16_t obstacle_mm,
             uint32_t now_ms);

/* True when the servo has settled and a reading may be triggered. */
bool bp_wants_reading(const bp_t *bp, uint32_t now_ms);

/* Call after ultrasonic_trigger() returned true. */
void bp_triggered(bp_t *bp);

/* Call with each new ultrasonic result. Results with no pending trigger
 * are ignored. */
void bp_reading(bp_t *bp, ultrasonic_result_t result, uint32_t now_ms);

/* Call often (every 10 ms is good). line_seen and junction come from the
 * line position module; travel_counts is the mean of the two wheel counts.
 * Returns the command for the motors and the servo. */
bp_word_t bp_update(bp_t *bp, bool line_seen, bool junction, int32_t travel_counts,
                    uint32_t now_ms);

/* DRIVE leg: sqrt(distance^2 - clearance^2) in mm, rounded down; 0 when the
 * distance is not more than the clearance. */
uint32_t bp_leg_mm(uint16_t clearance_mm, uint16_t distance_mm);

/* Millimetres of travel to mean wheel counts, rounded. */
uint32_t bp_mm_to_counts(uint32_t mm);

/* Smallest whole angle a (0-90 degrees) with sin(a) >= clearance / distance,
 * from a table of sin x 1000 (sin 89 rounds to 1000); 90 when clearance is
 * more than the distance. */
int bp_clear_angle(uint16_t clearance_mm, uint16_t distance_mm);

/*
 * Steering angle to a line_steer position (use gain 100 %, kd 0), by the
 * bicycle model with the servo as the front wheel: the rear axle turns on
 * radius R = L / tan(delta), so each wheel speed is
 * speed x (1 -/+ (W/2) tan(delta) / L). With gain 100, line_steer gives
 * speed x (1 +/- position / 100), so position = -100 (W/2) tan(delta) / L,
 * rounded. Negative for a left turn (a line position > 0 turns right).
 * delta is clamped to +/-30: at 30 degrees the inner wheel runs at 51 % of
 * the base speed above min_pct.
 */
int bp_steer_position(int steer_deg);

static inline bp_state_t bp_state(const bp_t *bp) {
    return (bp_state_t)bp->state;
}
static inline bp_phase_t bp_phase(const bp_t *bp) {
    return (bp_phase_t)bp->phase;
}
static inline bp_fail_t bp_fail(const bp_t *bp) {
    return (bp_fail_t)bp->fail;
}

#endif

#ifndef OBSTACLE_SCAN_H
#define OBSTACLE_SCAN_H

#include <stdbool.h>
#include <stdint.h>

#include "ultrasonic.h"

/*
 * Servo scan for the Obstacle Scan Task.
 *
 * On SCAN_REQ the task calls obstacle_scan_start(). It then calls
 * obstacle_scan_service() often (every 1 ms or less is good). The scan
 * moves the servo through 13 points, takes 2 ultrasonic readings at each
 * point, and returns the servo to the centre. Nothing blocks.
 *
 * During a scan this module owns the ultrasonic driver: it calls
 * ultrasonic_trigger() and ultrasonic_service(). Do not trigger the sensor
 * from other code until the scan ends. See docs/OBSTACLE_SCAN.md.
 */

/* Scan points: index 0-12 (4 bits); 13-15 are not used.
 * Angle: negative = right, positive = left, seen from behind the robot.
 * Index 0 = 30 degrees right, 6 = straight ahead, 12 = 30 degrees left. */
#define OBSTACLE_SCAN_POINTS 13u
#define OBSTACLE_SCAN_CENTRE_INDEX 6u
#define OBSTACLE_SCAN_STEP_DEG 5
#define OBSTACLE_SCAN_READINGS 2u

/* Servo settle time before the first reading at a point. */
#define OBSTACLE_SCAN_SETTLE_FIRST_MS 150u /* centre to 30 degrees right */
#define OBSTACLE_SCAN_SETTLE_STEP_MS 60u   /* one 5-degree step */
#define OBSTACLE_SCAN_SETTLE_RETURN_MS 150u /* 30 degrees left to centre */

static inline int obstacle_scan_angle_deg(unsigned int index) {
    return ((int)index - (int)OBSTACLE_SCAN_CENTRE_INDEX) * OBSTACLE_SCAN_STEP_DEG;
}

/* Servo pulse for each index: 1500 us + angle x 1000/90 us, rounded.
 * A higher pulse turns the sensor left. */
static inline uint16_t obstacle_scan_pulse_us(unsigned int index) {
    static const uint16_t pulse[OBSTACLE_SCAN_POINTS] = {
        1167u, 1222u, 1278u, 1333u, 1389u, 1444u, 1500u,
        1556u, 1611u, 1667u, 1722u, 1778u, 1833u,
    };
    return index < OBSTACLE_SCAN_POINTS ? pulse[index] : 1500u;
}

/*
 * Profile point: one 16-bit word, the ultrasonic result layout without the
 * streak.
 *
 *   bits 0-8    distance code (0 = TOO_NEAR when VALID)
 *   bits 9-10   status:
 *                 VALID      nearest valid echo of the readings
 *                 NO_RETURN  no echo within 511 mm in any reading
 *                 NO_RISE    no usable reading (both readings failed)
 *   bits 11-15  always 0
 *
 * Logical size: 11 bits per point, 143 bits for 13 points.
 */
typedef uint16_t obstacle_scan_point_t;

#define OBSTACLE_SCAN_POINT_EMPTY ULTRASONIC_RESULT_NONE

/*
 * Add one reading to a point. Rules, from nearest to least information:
 *   - VALID beats everything; of two VALID readings keep the nearer one.
 *     The nearer value is the safer value for a bypass decision.
 *   - NO_RETURN beats "no usable reading".
 *   - NO_RISE and STUCK_HIGH add no information.
 */
static inline obstacle_scan_point_t obstacle_scan_combine(
    obstacle_scan_point_t point, ultrasonic_result_t reading) {
    const ultrasonic_status_t new_status = ultrasonic_result_status(reading);
    const ultrasonic_status_t old_status = ultrasonic_result_status(point);
    const obstacle_scan_point_t reading_point =
        (obstacle_scan_point_t)(reading & (ULTRASONIC_DISTANCE_MASK |
                                           ULTRASONIC_STATUS_MASK));

    if (new_status == ULTRASONIC_STATUS_VALID) {
        if (old_status == ULTRASONIC_STATUS_VALID &&
            ultrasonic_result_distance(point) <=
                ultrasonic_result_distance(reading)) {
            return point;
        }
        return reading_point;
    }
    if (new_status == ULTRASONIC_STATUS_NO_RETURN &&
        old_status != ULTRASONIC_STATUS_VALID) {
        return reading_point;
    }
    return point;
}

typedef struct {
    obstacle_scan_point_t point[OBSTACLE_SCAN_POINTS];
} obstacle_profile_t;

typedef enum {
    OBSTACLE_SCAN_NONE = 0,  /* Nothing finished in this call. */
    OBSTACLE_SCAN_POINT = 1, /* One point finished; see progress. */
    OBSTACLE_SCAN_DONE = 2,  /* Servo back at centre; profile complete. */
    OBSTACLE_SCAN_FAULT = 3, /* Sensor fault streak; scan stopped,
                                servo sent to centre. */
} obstacle_scan_event_t;

/* Set up the servo. ultrasonic_init() must be called separately. */
void obstacle_scan_init(unsigned int servo_pin);

/* Start a scan. Returns false if a scan is already running. */
bool obstacle_scan_start(void);

/* Stop a running scan and send the servo to the centre. */
void obstacle_scan_abort(void);

/* Advance the scan. Never blocks. */
obstacle_scan_event_t obstacle_scan_service(void);

bool obstacle_scan_busy(void);

/* Number of finished points in the current or last scan, 0-13. */
unsigned int obstacle_scan_progress(void);

/* Profile of the current or last scan. Unfinished points are EMPTY. */
const obstacle_profile_t *obstacle_scan_profile(void);

#endif

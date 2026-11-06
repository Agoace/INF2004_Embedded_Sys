/*
 * Servo scan module for the Vehicle Controller shell (module SCAN,
 * src/vehicle_sm.h). See docs/VC_DETOUR_RUN.md.
 *
 * The scan of src/obstacle_scan.c without hardware calls: the servo angle
 * goes out in the drive command, and the readings come in through the
 * shell inputs (ping request, ping_sent, echo).
 *
 *   WAIT     wheels 0, servo centre, for stop_ms (the robot stops)
 *   POINT i  servo to obstacle_scan_angle_deg(i) (i = 0..12, 30 degrees
 *            right to 30 degrees left); settle (first point, then one
 *            step); take OBSTACLE_SCAN_READINGS readings (pings as fast
 *            as the driver allows) and combine them
 *   RETURN   servo centre; settle
 *
 * Outcomes:
 *   SIDE    payload: bits 0-8 obstacle distance (PROFILE word), bits 9-10
 *           obstacle_side_t (the clearer side; NONE when both are blocked)
 *   FAILED  ultrasonic fault streak; payload SS_FAIL_ECHO
 *
 * The start payload (the obstacle distance) is not used.
 *
 * Pure logic: host-tested by tests/test_obstacle_modules.c.
 */
#ifndef SERVO_SCAN_H
#define SERVO_SCAN_H

#include <stdbool.h>
#include <stdint.h>

#include "obstacle_profile.h"
#include "obstacle_scan.h"
#include "../vehicle/vehicle_inputs.h"
#include "../vehicle/vehicle_sm.h"

#define SS_FAIL_ECHO 1u

/* SIDE payload: 11 of the 12 payload bits. */
#define SS_DISTANCE_MASK 0x01FFu
#define SS_SIDE_SHIFT 9u
#define SS_SIDE_MASK 0x3u

static inline uint16_t ss_payload(obstacle_side_t side, uint16_t distance) {
    return (uint16_t)((distance & SS_DISTANCE_MASK) | (((unsigned)side & SS_SIDE_MASK) << SS_SIDE_SHIFT));
}
static inline uint16_t ss_payload_distance(uint16_t payload) {
    return (uint16_t)(payload & SS_DISTANCE_MASK);
}
static inline obstacle_side_t ss_payload_side(uint16_t payload) {
    return (obstacle_side_t)((payload >> SS_SIDE_SHIFT) & SS_SIDE_MASK);
}

typedef struct {
    uint16_t stop_ms; /* WAIT before the first point */
} ss_config_t;

typedef enum {
    SS_WAIT = 0,
    SS_SETTLE = 1,
    SS_MEASURE = 2,
    SS_RETURN = 3,
} ss_phase_t;

typedef struct {
    ss_config_t cfg;
    uint8_t phase; /* ss_phase_t */
    uint8_t index; /* point now, 0..13 */
    uint8_t readings;
    bool pending;  /* a reading was triggered in MEASURE and is not in yet */
    uint32_t since_ms; /* start of the phase */
    uint32_t wait_ms;  /* length of the phase */
    obstacle_profile_t profile;
    obstacle_profile_word_t word; /* after the scan */
    uint16_t scans;               /* since ss_init() */
} ss_t;

void ss_init(ss_t *ss, const ss_config_t *cfg);
void ss_start(void *ctx, uint16_t payload, uint32_t now_ms);
vsm_outcome_t ss_step(void *ctx, const vsm_inputs_t *in, uint32_t now_ms, vsm_drive_t *drive);

static inline vsm_module_t ss_module(ss_t *ss) {
    return (vsm_module_t){ss_start, ss_step, ss};
}

#endif

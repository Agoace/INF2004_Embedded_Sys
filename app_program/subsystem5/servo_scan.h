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
 *   RETURN   servo centre; settle; publish the full profile
 *   REVERSE  if both sides are blocked, drive back 100 mm at 100 mm/s,
 *            measured by the mean wheel encoder delta; at most two retries
 *   CENTRE   stop, settle, take two fresh centre readings, then a full sweep
 *
 * No obstacle decision rule changes. After three blocked profiles, SIDE NONE
 * reaches DETOUR as before. The shell still owns ESTOP and motion faults.
 *
 * Outcomes:
 *   SIDE    payload: bits 0-8 obstacle distance (PROFILE word), bits 9-10
 *           obstacle_side_t (the clearer side; NONE when both are blocked)
 *   FAILED  SS_FAIL_ECHO, or SS_FAIL_REVERSE_TIMEOUT after a 3 s reverse leg
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

#define SS_FAIL_ECHO             1u
#define SS_FAIL_REVERSE_TIMEOUT  2u
#define SS_REVERSE_MM            100u
#define SS_REVERSE_MM_S          100
#define SS_MAX_REATTEMPTS        2u
#define SS_REVERSE_TIMEOUT_MS    3000u
#define SS_COUNTS_PER_M          12414u
#define SS_REVERSE_COUNTS        ((SS_REVERSE_MM * SS_COUNTS_PER_M + 500u) / 1000u)

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
    SS_REVERSE = 4,
    SS_CENTRE_SETTLE = 5,
    SS_CENTRE_MEASURE = 6,
    SS_FAILED = 7,
} ss_phase_t;

/* Edge notices for observers only. These never change the module route. */
typedef enum {
    SS_NOTICE_NONE = 0,
    SS_NOTICE_REVERSE_BEGIN = 1,
    SS_NOTICE_REVERSE_END = 2,
    SS_NOTICE_RESCAN_BEGIN = 3,
} ss_notice_t;

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
    uint16_t scans;               /* full sweeps started since ss_init() */
    uint16_t completed_scans;     /* full profiles finished since ss_init() */
    uint8_t attempt;              /* 2 logical bits: reverses begun, 0..2 */
    uint8_t completed_attempt;    /* 2 logical bits: attempt of the finished profile */
    uint8_t notice;               /* ss_notice_t, one step only, 2 logical bits */
    uint8_t failure;              /* SS_FAIL_*; latched until ss_start() */
    int32_t reverse_origin[2];
    obstacle_scan_point_t centre; /* fresh centre check before each retry sweep */
} ss_t;

void ss_init(ss_t *ss, const ss_config_t *cfg);
void ss_start(void *ctx, uint16_t payload, uint32_t now_ms);
vsm_outcome_t ss_step(void *ctx, const vsm_inputs_t *in, uint32_t now_ms, vsm_drive_t *drive);

static inline vsm_module_t ss_module(ss_t *ss) {
    return (vsm_module_t){ss_start, ss_step, ss};
}

#endif

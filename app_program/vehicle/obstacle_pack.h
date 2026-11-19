/*
 * obstacle_pack.h - 6-word storage layout of the obstacle board (CR1 A).
 *
 * The vehicle task is the single writer (one publication per completed
 * scan). The network side reads through vehicle_obstacle_try_read(). A
 * reader copies all words under the board sequence and then unpacks its
 * local copy. The public type rm_obstacle_t and the MQTT bytes do not
 * change.
 *
 *   word 0  scan_id
 *   word 1  uptime_ms
 *   word 2  boot_id
 *   word 3  bits 0-7    flags
 *           bits 8-9    decision (0-2)
 *           bits 10-15  closest_angle_deg + 30 (0-60)
 *           bits 16-24  closest_mm (0-511, ultrasonic distance code)
 *           bits 25-29  geometry_flags (5 defined bits)
 *           bits 30-31  0
 *   word 4  bits 0-14   profile_word (bit 15 is always 0)
 *           bits 15-17  right_free_step (0-7)
 *           bits 18-20  left_free_step (0-7)
 *           bits 21-23  right_clear_deg code: deg / 5 for 0-30, 7 for 255
 *           bits 24-26  left_clear_deg code
 *           bits 27-31  0
 *   word 5  bits 0-9    width_mm      (0-1022; 1023 = GEOMETRY_NONE)
 *           bits 10-19  left_clear_mm (same code)
 *           bits 20-29  right_clear_mm (same code)
 *           bits 30-31  0
 *
 * Geometry bound: the scan distance is at most 511 mm and the scan spans
 * 60 degrees, so width and clearances are at most 511 mm (+ rounding).
 */
#ifndef OBSTACLE_PACK_H
#define OBSTACLE_PACK_H

#include <stdbool.h>
#include <stdint.h>

#include "../platform/app_board.h"
#include "../subsystem1/mqtt_wire.h"

#define OBSTACLE_PACK_WORDS 6u

#define OBSTACLE_PACK_DECISION_SHIFT 8u
#define OBSTACLE_PACK_DECISION_MAX 2u
#define OBSTACLE_PACK_ANGLE_SHIFT 10u
#define OBSTACLE_PACK_ANGLE_OFFSET 30
#define OBSTACLE_PACK_CLOSEST_SHIFT 16u
#define OBSTACLE_PACK_CLOSEST_MAX 0x1FFu
#define OBSTACLE_PACK_GEO_FLAGS_SHIFT 25u
#define OBSTACLE_PACK_GEO_FLAGS_MAX 0x1Fu
#define OBSTACLE_PACK_W3_USED 0x3FFFFFFFu

#define OBSTACLE_PACK_PROFILE_MAX 0x7FFFu
#define OBSTACLE_PACK_RIGHT_STEP_SHIFT 15u
#define OBSTACLE_PACK_LEFT_STEP_SHIFT 18u
#define OBSTACLE_PACK_RIGHT_DEG_SHIFT 21u
#define OBSTACLE_PACK_LEFT_DEG_SHIFT 24u
#define OBSTACLE_PACK_3BIT_MAX 7u
#define OBSTACLE_PACK_W4_USED 0x07FFFFFFu

#define OBSTACLE_PACK_DEG_STEP 5u
#define OBSTACLE_PACK_DEG_MAX 30u
#define OBSTACLE_PACK_DEG_BLOCKED 255u
#define OBSTACLE_PACK_DEG_BLOCKED_CODE 7u

#define OBSTACLE_PACK_LEFT_CLEAR_SHIFT 10u
#define OBSTACLE_PACK_RIGHT_CLEAR_SHIFT 20u
#define OBSTACLE_PACK_GEOMETRY_MASK 0x3FFu
#define OBSTACLE_PACK_GEOMETRY_MAX 1022u
#define OBSTACLE_PACK_GEOMETRY_NONE_CODE 1023u
#define OBSTACLE_PACK_W5_USED 0x3FFFFFFFu

/* Board storage: sequence plus the packed words (28 bytes). */
typedef struct {
    uint32_t sequence;
    uint32_t words[OBSTACLE_PACK_WORDS];
} obstacle_board_t;

/* 3-bit code of a clearance angle; false when the angle has no code. */
static inline bool obstacle_pack_deg(uint8_t deg, uint32_t *code)
{
    if (deg == OBSTACLE_PACK_DEG_BLOCKED) {
        *code = OBSTACLE_PACK_DEG_BLOCKED_CODE;
        return true;
    }
    if ((deg > OBSTACLE_PACK_DEG_MAX) ||
        ((deg % OBSTACLE_PACK_DEG_STEP) != 0u)) {
        return false;
    }
    *code = (uint32_t)deg / OBSTACLE_PACK_DEG_STEP;
    return true;
}

static inline uint8_t obstacle_unpack_deg(uint32_t code)
{
    return (code == OBSTACLE_PACK_DEG_BLOCKED_CODE)
               ? (uint8_t)OBSTACLE_PACK_DEG_BLOCKED
               : (uint8_t)(code * OBSTACLE_PACK_DEG_STEP);
}

/* 10-bit code of a geometry length; false when it has no code. */
static inline bool obstacle_pack_mm(uint16_t mm, uint32_t *code)
{
    if (mm == RM_OBSTACLE_GEOMETRY_NONE) {
        *code = OBSTACLE_PACK_GEOMETRY_NONE_CODE;
        return true;
    }
    if (mm > OBSTACLE_PACK_GEOMETRY_MAX) {
        return false;
    }
    *code = mm;
    return true;
}

static inline uint16_t obstacle_unpack_mm(uint32_t code)
{
    return (code == OBSTACLE_PACK_GEOMETRY_NONE_CODE)
               ? (uint16_t)RM_OBSTACLE_GEOMETRY_NONE
               : (uint16_t)code;
}

/*
 * Pack value into words. Returns false, and does not change words, when a
 * field is out of the range above.
 */
static inline bool obstacle_pack(const rm_obstacle_t *value,
                                 uint32_t words[OBSTACLE_PACK_WORDS])
{
    uint32_t right_deg;
    uint32_t left_deg;
    uint32_t width;
    uint32_t left_mm;
    uint32_t right_mm;

    if ((!value) ||
        (value->decision > OBSTACLE_PACK_DECISION_MAX) ||
        ((int)value->closest_angle_deg < -OBSTACLE_PACK_ANGLE_OFFSET) ||
        ((int)value->closest_angle_deg > OBSTACLE_PACK_ANGLE_OFFSET) ||
        (value->closest_mm > OBSTACLE_PACK_CLOSEST_MAX) ||
        (value->geometry_flags > OBSTACLE_PACK_GEO_FLAGS_MAX) ||
        (value->profile_word > OBSTACLE_PACK_PROFILE_MAX) ||
        (value->right_free_step > OBSTACLE_PACK_3BIT_MAX) ||
        (value->left_free_step > OBSTACLE_PACK_3BIT_MAX) ||
        !obstacle_pack_deg(value->right_clear_deg, &right_deg) ||
        !obstacle_pack_deg(value->left_clear_deg, &left_deg) ||
        !obstacle_pack_mm(value->width_mm, &width) ||
        !obstacle_pack_mm(value->left_clear_mm, &left_mm) ||
        !obstacle_pack_mm(value->right_clear_mm, &right_mm)) {
        return false;
    }
    words[0] = value->scan_id;
    words[1] = value->uptime_ms;
    words[2] = value->boot_id;
    words[3] = (uint32_t)value->flags |
               ((uint32_t)value->decision << OBSTACLE_PACK_DECISION_SHIFT) |
               ((uint32_t)((int)value->closest_angle_deg +
                           OBSTACLE_PACK_ANGLE_OFFSET)
                << OBSTACLE_PACK_ANGLE_SHIFT) |
               ((uint32_t)value->closest_mm << OBSTACLE_PACK_CLOSEST_SHIFT) |
               ((uint32_t)value->geometry_flags
                << OBSTACLE_PACK_GEO_FLAGS_SHIFT);
    words[4] = (uint32_t)value->profile_word |
               ((uint32_t)value->right_free_step
                << OBSTACLE_PACK_RIGHT_STEP_SHIFT) |
               ((uint32_t)value->left_free_step
                << OBSTACLE_PACK_LEFT_STEP_SHIFT) |
               (right_deg << OBSTACLE_PACK_RIGHT_DEG_SHIFT) |
               (left_deg << OBSTACLE_PACK_LEFT_DEG_SHIFT);
    words[5] = width | (left_mm << OBSTACLE_PACK_LEFT_CLEAR_SHIFT) |
               (right_mm << OBSTACLE_PACK_RIGHT_CLEAR_SHIFT);
    return true;
}

/* Unpack words (a local copy) into value. */
static inline void obstacle_unpack(const uint32_t words[OBSTACLE_PACK_WORDS],
                                   rm_obstacle_t *value)
{
    const uint32_t w3 = words[3];
    const uint32_t w4 = words[4];
    const uint32_t w5 = words[5];

    *value = (rm_obstacle_t){0};
    value->scan_id = words[0];
    value->uptime_ms = words[1];
    value->boot_id = words[2];
    value->flags = (uint8_t)(w3 & UINT8_MAX);
    value->decision =
        (uint8_t)((w3 >> OBSTACLE_PACK_DECISION_SHIFT) & 0x3u);
    value->closest_angle_deg =
        (int8_t)((int)((w3 >> OBSTACLE_PACK_ANGLE_SHIFT) & 0x3Fu) -
                 OBSTACLE_PACK_ANGLE_OFFSET);
    value->closest_mm = (uint16_t)((w3 >> OBSTACLE_PACK_CLOSEST_SHIFT) &
                                   OBSTACLE_PACK_CLOSEST_MAX);
    value->geometry_flags = (uint8_t)((w3 >> OBSTACLE_PACK_GEO_FLAGS_SHIFT) &
                                      OBSTACLE_PACK_GEO_FLAGS_MAX);
    value->profile_word = (uint16_t)(w4 & OBSTACLE_PACK_PROFILE_MAX);
    value->right_free_step = (uint8_t)((w4 >> OBSTACLE_PACK_RIGHT_STEP_SHIFT) &
                                       OBSTACLE_PACK_3BIT_MAX);
    value->left_free_step = (uint8_t)((w4 >> OBSTACLE_PACK_LEFT_STEP_SHIFT) &
                                      OBSTACLE_PACK_3BIT_MAX);
    value->right_clear_deg = obstacle_unpack_deg(
        (w4 >> OBSTACLE_PACK_RIGHT_DEG_SHIFT) & OBSTACLE_PACK_3BIT_MAX);
    value->left_clear_deg = obstacle_unpack_deg(
        (w4 >> OBSTACLE_PACK_LEFT_DEG_SHIFT) & OBSTACLE_PACK_3BIT_MAX);
    value->width_mm = obstacle_unpack_mm(w5 & OBSTACLE_PACK_GEOMETRY_MASK);
    value->left_clear_mm = obstacle_unpack_mm(
        (w5 >> OBSTACLE_PACK_LEFT_CLEAR_SHIFT) & OBSTACLE_PACK_GEOMETRY_MASK);
    value->right_clear_mm = obstacle_unpack_mm(
        (w5 >> OBSTACLE_PACK_RIGHT_CLEAR_SHIFT) & OBSTACLE_PACK_GEOMETRY_MASK);
}

/*
 * Publish value (single writer). Returns false, and does not change the
 * board, when value does not pack.
 */
static inline bool obstacle_board_publish(obstacle_board_t *board,
                                          const rm_obstacle_t *value)
{
    uint32_t packed[OBSTACLE_PACK_WORDS];

    if (!obstacle_pack(value, packed)) {
        return false;
    }
    app_board_publish(&board->sequence, board->words, packed,
                      (unsigned int)sizeof(packed));
    return true;
}

/*
 * Copy the newest value. Returns false, and does not change out, when
 * nothing was published yet or a write was in progress.
 */
static inline bool obstacle_board_try_read(const obstacle_board_t *board,
                                           rm_obstacle_t *out)
{
    uint32_t packed[OBSTACLE_PACK_WORDS];

    if (!app_board_try_read(&board->sequence, board->words, packed,
                            (unsigned int)sizeof(packed))) {
        return false;
    }
    obstacle_unpack(packed, out);
    return true;
}

#endif

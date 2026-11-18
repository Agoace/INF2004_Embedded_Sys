/*
 * sonar_pack.h - 10-word storage layout of one sonar snapshot (CR1 A).
 *
 * The sonar board and the sonar history store this layout. The single
 * writer is the sonar task. A reader copies all words under the guard and
 * then unpacks its local copy. The public type sonar_snapshot_t does not
 * change.
 *
 *   word 0  generation        word 5  timestamp_ms
 *   word 1  id                word 6  deadline_ms
 *   word 2  ping_sequence     word 7  ping_generation
 *   word 3  result_sequence   word 8  ping_id
 *   word 4  trigger_ms
 *   word 9  bits 0-15   echo (ultrasonic_result_t, all 16 bits)
 *           bits 16-21  angle + 30       (0-60; 61-63 not used)
 *           bits 22-27  ping_angle + 30  (0-60; 61-63 not used)
 *           bits 28-29  status (sonar_status_t, 0-3)
 *           bit  30     pending
 *           bit  31     valid
 *
 * Counters and times keep 32 bits: they wrap and are compared modulo 2^32.
 */
#ifndef SONAR_PACK_H
#define SONAR_PACK_H

#include <stdbool.h>
#include <stdint.h>

#include "sonar_service.h"

#define SONAR_PACK_WORDS 10u
#define SONAR_PACK_SMALL_WORD 9u

#define SONAR_PACK_ECHO_MASK 0xFFFFu
#define SONAR_PACK_ANGLE_SHIFT 16u
#define SONAR_PACK_PING_ANGLE_SHIFT 22u
#define SONAR_PACK_ANGLE_MASK 0x3Fu
#define SONAR_PACK_ANGLE_OFFSET 30
#define SONAR_PACK_ANGLE_CODE_MAX 60u
#define SONAR_PACK_STATUS_SHIFT 28u
#define SONAR_PACK_STATUS_MASK 0x3u
#define SONAR_PACK_PENDING_BIT 30u
#define SONAR_PACK_VALID_BIT 31u

/* True when angle is in the request range -30..30. */
static inline bool sonar_pack_angle_ok(int8_t angle)
{
    return ((int)angle >= -SONAR_PACK_ANGLE_OFFSET) &&
           ((int)angle <= SONAR_PACK_ANGLE_OFFSET);
}

/*
 * Pack value into words. Returns false, and does not change words, when a
 * field is out of range (angle outside -30..30 or status above 3).
 */
static inline bool sonar_pack(const sonar_snapshot_t *value,
                              uint32_t words[SONAR_PACK_WORDS])
{
    if ((!value) || !sonar_pack_angle_ok(value->angle) ||
        !sonar_pack_angle_ok(value->ping_angle) ||
        (value->status > SONAR_PACK_STATUS_MASK)) {
        return false;
    }
    words[0] = value->generation;
    words[1] = value->id;
    words[2] = value->ping_sequence;
    words[3] = value->result_sequence;
    words[4] = value->trigger_ms;
    words[5] = value->timestamp_ms;
    words[6] = value->deadline_ms;
    words[7] = value->ping_generation;
    words[8] = value->ping_id;
    words[SONAR_PACK_SMALL_WORD] =
        ((uint32_t)value->echo & SONAR_PACK_ECHO_MASK) |
        ((uint32_t)((int)value->angle + SONAR_PACK_ANGLE_OFFSET)
         << SONAR_PACK_ANGLE_SHIFT) |
        ((uint32_t)((int)value->ping_angle + SONAR_PACK_ANGLE_OFFSET)
         << SONAR_PACK_PING_ANGLE_SHIFT) |
        ((uint32_t)value->status << SONAR_PACK_STATUS_SHIFT) |
        ((value->pending ? 1u : 0u) << SONAR_PACK_PENDING_BIT) |
        ((value->valid ? 1u : 0u) << SONAR_PACK_VALID_BIT);
    return true;
}

/* Unpack words (a local copy) into value. */
static inline void sonar_unpack(const uint32_t words[SONAR_PACK_WORDS],
                                sonar_snapshot_t *value)
{
    const uint32_t small = words[SONAR_PACK_SMALL_WORD];

    value->generation = words[0];
    value->id = words[1];
    value->ping_sequence = words[2];
    value->result_sequence = words[3];
    value->trigger_ms = words[4];
    value->timestamp_ms = words[5];
    value->deadline_ms = words[6];
    value->ping_generation = words[7];
    value->ping_id = words[8];
    value->echo = (ultrasonic_result_t)(small & SONAR_PACK_ECHO_MASK);
    value->angle =
        (int8_t)((int)((small >> SONAR_PACK_ANGLE_SHIFT) &
                       SONAR_PACK_ANGLE_MASK) -
                 SONAR_PACK_ANGLE_OFFSET);
    value->ping_angle =
        (int8_t)((int)((small >> SONAR_PACK_PING_ANGLE_SHIFT) &
                       SONAR_PACK_ANGLE_MASK) -
                 SONAR_PACK_ANGLE_OFFSET);
    value->status =
        (uint8_t)((small >> SONAR_PACK_STATUS_SHIFT) & SONAR_PACK_STATUS_MASK);
    value->pending = ((small >> SONAR_PACK_PENDING_BIT) & 1u) != 0u;
    value->valid = ((small >> SONAR_PACK_VALID_BIT) & 1u) != 0u;
}

#endif

/**
 * @file line_types.h
 * @brief The Line component's contract with the Manager: one aligned 32-bit
 *        observation word, plus the discrete edge and calibration events.
 *
 * No Pico SDK and no RTOS include, so the Line task, the Manager, the adapters
 * and the host tests all use the same definitions.
 *
 * Ownership (approved YX abstraction plan, RT-0031/RT-0032):
 *  - The Line component writes observations. Conceptually the mailbox holds one
 *    value: a new sample overwrites the old one. It is never a 1 kHz FIFO.
 *  - The publication counter lives beside the word, so the Manager can tell a
 *    fresh observation from a stale one without a sequence field competing for
 *    bits.
 *  - Edges (line lost / line found) and calibration results are discrete and
 *    travel through the Manager's bounded event queue, not the mailbox.
 *
 * Observation word layout, little-end first:
 *
 *   bits 0..7    position, two's complement, -100 (left) .. +100 (right)
 *   bit  8       line seen            (0 = the qualified loss policy has fired)
 *   bits 9..10   last side            line_side_t
 *   bit  11      junction             all three sensors on the line
 *   bit  12      junction hold        within the post-junction holding time
 *   bit  13      sensor fault         a sensor is railed or stuck
 *   bit  14      calibrated           the learned endpoints are usable
 *   bits 15..31  reserved, always 0
 *
 * The word is deliberately wider than the YX 16-bit line word, which stays
 * available as a diagnostic view. There is no sequence field: freshness is the
 * separate publication counter.
 */
#ifndef LINE_TYPES_H
#define LINE_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/* ---- Observation ------------------------------------------------------- */

/** One packed observation. Single aligned 32-bit value, stored atomically. */
typedef uint32_t line_observation_t;

/** Where the line was last seen. Matches YX's vc_line_side_t ordering. */
typedef enum
{
    LINE_SIDE_UNKNOWN = 0,
    LINE_SIDE_LEFT    = 1,
    LINE_SIDE_RIGHT   = 2
} line_side_t;

#define LINE_OBS_POSITION_SHIFT 0u
#define LINE_OBS_POSITION_MASK  0x000000FFu
#define LINE_OBS_SEEN           0x00000100u
#define LINE_OBS_SIDE_SHIFT     9u
#define LINE_OBS_SIDE_MASK      0x00000600u
#define LINE_OBS_JUNCTION       0x00000800u
#define LINE_OBS_JUNCTION_HOLD  0x00001000u
#define LINE_OBS_SENSOR_FAULT   0x00002000u
#define LINE_OBS_CALIBRATED     0x00004000u
/**
 * Junction straight-through has used its distance budget (bit 15).
 *
 * Upstream bounds straight-through by travel as well as by the hold, so a long
 * junction cannot keep the robot driving straight indefinitely. The producer
 * measures the travel, because only it holds the odometer.
 */
#define LINE_OBS_JUNCTION_TRAVEL_DONE 0x00008000u

/** Decoded observation, used by the pack and unpack helpers. */
typedef struct
{
    int8_t  position;       /**< -100..+100, negative is left.               */
    uint8_t side;           /**< line_side_t                                 */
    bool    seen;
    bool    junction;
    bool    junction_hold;
    bool    junction_travel_done;
    bool    sensor_fault;
    bool    calibrated;
} line_obs_fields_t;

/** Pack decoded fields. Position is clamped to -100..+100. */
line_observation_t line_obs_pack(const line_obs_fields_t * p_fields);

/** Decode a packed word. Sign-extends the position. */
void line_obs_unpack(line_observation_t word, line_obs_fields_t * p_fields);

static inline int8_t
line_obs_position (line_observation_t word)
{
    return (int8_t) (uint8_t) ((word & LINE_OBS_POSITION_MASK)
                               >> LINE_OBS_POSITION_SHIFT);
}

static inline bool
line_obs_seen (line_observation_t word)
{
    return 0u != (word & LINE_OBS_SEEN);
}

static inline line_side_t
line_obs_side (line_observation_t word)
{
    return (line_side_t) ((word & LINE_OBS_SIDE_MASK) >> LINE_OBS_SIDE_SHIFT);
}

static inline bool
line_obs_junction (line_observation_t word)
{
    return 0u != (word & LINE_OBS_JUNCTION);
}

static inline bool
line_obs_junction_hold (line_observation_t word)
{
    return 0u != (word & LINE_OBS_JUNCTION_HOLD);
}

static inline bool
line_obs_junction_travel_done (line_observation_t word)
{
    return 0u != (word & LINE_OBS_JUNCTION_TRAVEL_DONE);
}

static inline bool
line_obs_sensor_fault (line_observation_t word)
{
    return 0u != (word & LINE_OBS_SENSOR_FAULT);
}

static inline bool
line_obs_calibrated (line_observation_t word)
{
    return 0u != (word & LINE_OBS_CALIBRATED);
}

/* ---- Discrete edges ---------------------------------------------------- */

/** The two qualified transitions the Manager needs. */
typedef enum
{
    LINE_EVT_LOST  = 0,
    LINE_EVT_FOUND = 1
} line_event_id_t;

/**
 * @brief One line edge.
 *
 * LOST carries the last known side, so the Manager can start the in-place
 * search toward the correct side. The 50 / 300 ms qualification dwell already
 * happened inside the Line component; the Manager must not add another.
 */
typedef struct
{
    uint8_t  id;            /* line_event_id_t                               */
    uint8_t  side;          /* line_side_t, meaningful on LINE_EVT_LOST      */
    uint16_t reserved;
} line_event_t;

/* ---- Calibration result ------------------------------------------------ */

typedef enum
{
    LINE_CALIB_OK = 0,
    LINE_CALIB_NO_DATA = 1,          /* the sweep collected nothing           */
    LINE_CALIB_LOW_CONTRAST = 2,     /* a sensor never saw enough range       */
    LINE_CALIB_CANCELLED = 3,        /* the operator stopped the sweep        */
    LINE_CALIB_TIMEOUT = 4,          /* the sweep ran past its step limit     */
    LINE_CALIB_STALLED = 5,          /* a wheel stopped turning               */
    LINE_CALIB_INVALID = 6           /* no usable calibration for the request */
} line_calib_result_t;

/**
 * @brief One calibration outcome.
 *
 * On any failure the Line component has already invalidated the usable
 * calibration, so the Manager must not follow. The learned endpoints stay out
 * of this event: read them from the diagnostic accessor.
 */
typedef struct
{
    uint8_t result;         /* line_calib_result_t                           */
    uint8_t bad_mask;       /* L=1, C=2, R=4 where contrast was too low      */
    uint8_t line_reads_high;/* polarity: 1 = the line reads above the floor  */
    uint8_t reserved;
} line_calib_event_t;

#endif /* LINE_TYPES_H */

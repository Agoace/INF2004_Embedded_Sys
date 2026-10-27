/**
 * @file manager_types.h
 * @brief Shared types for the Manager: a line observation (the payload of the
 *        latest-value mailbox), a discrete button-toggle event, and a
 *        forwarded motion_evt_t.
 *
 * No Pico SDK and no RTOS include, so the Line task, the Manager task, the
 * bare-metal adapter and the host unit tests all use the same definitions.
 *
 * Ownership (approved motor-line-manager plan):
 *  - The Line task writes line observations. Conceptually the mailbox holds
 *    one value: a new sample overwrites the old one. Never queue every 1 ms
 *    sample.
 *  - The button ISRs and the Motion task post discrete events to the
 *    Manager's bounded queue.
 *  - The Manager reads both and is the only writer of the vehicle state.
 */
#ifndef MANAGER_TYPES_H
#define MANAGER_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#include "robot_shared.h"

/* ---- Line observation -------------------------------------------------- */

/**
 * @brief One line-sensing result: the same three masks that
 *        robocar_ir_sensors.c already computes.
 *
 * black_pattern: bit 0 = left, bit 1 = centre, bit 2 = right on black.
 * unknown_mask:  sensors between the two thresholds.
 * fault_mask:    sensors stuck at a rail.
 */
typedef struct
{
    uint8_t black_pattern;
    uint8_t unknown_mask;
    uint8_t fault_mask;
    uint8_t reserved;
} line_observation_t;

/* ---- Discrete events --------------------------------------------------- */

/** The two Robo Pico buttons in this checkpoint. Both are short-press toggles. */
typedef enum
{
    MANAGER_BUTTON_GP20 = 0,
    MANAGER_BUTTON_GP21 = 1,
    MANAGER_BUTTON_COUNT = 2
} manager_button_t;

/** A debounced short press. The Manager only sees presses, never releases. */
typedef struct
{
    uint8_t  button;        /* manager_button_t                              */
    uint8_t  pressed;       /* 1 = short press accepted                      */
    uint16_t reserved;
} button_event_t;

/**
 * @brief One slot of the Manager's bounded event queue.
 *
 * kind selects the union member: a button_event_t, or the motion_evt_t that
 * the Motion task forwards. Forwarding the struct unchanged keeps the Motion
 * contract in one place (robot_shared.h).
 */
typedef enum
{
    MANAGER_EVT_BUTTON = 0,
    MANAGER_EVT_MOTION = 1
} manager_evt_kind_t;

typedef struct
{
    uint8_t kind;           /* manager_evt_kind_t                            */
    union
    {
        button_event_t button;
        motion_evt_t   motion;
    } payload;
} manager_evt_t;

/* ---- Limits (approved plan) -------------------------------------------- */

/** A line that stays LOST this long starts the recovery manoeuvre. */
#define MANAGER_LINE_LOST_TIMEOUT_MS        100u

/** Recovery reverses at this speed for at most this distance. */
#define MANAGER_RECOVERY_SPEED_MM_S         100
#define MANAGER_RECOVERY_DISTANCE_MM        100

/** Consecutive valid tracking samples that end a recovery. */
#define MANAGER_RECOVERY_TRACKING_SAMPLES   3u

/** Bounded Manager event queue: a power of two. */
#define MANAGER_EVENT_QUEUE_LEN             16u

/** Short-press window: rejects contact bounce and a long hold. */
#define MANAGER_BUTTON_DEBOUNCE_MS          20u
#define MANAGER_BUTTON_SHORT_MAX_MS         800u

#endif /* MANAGER_TYPES_H */

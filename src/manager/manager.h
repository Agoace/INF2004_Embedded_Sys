/**
 * @file manager.h
 * @brief Platform-independent Manager core for the integrated line-following
 *        checkpoint.
 *
 * The Manager owns the vehicle state and the communication between the Line
 * task and Motion Control:
 *
 *   SELFTEST -> LINE_FOLLOW_OFF <-> LINE_FOLLOW_ON <-> RECOVERY
 *                     |                  |              |
 *                     +------------------+--------------+-> FAULT
 *
 * Boundaries (approved motor-line-manager plan):
 *  - Line observations arrive through a ONE-ELEMENT overwrite mailbox. The
 *    Manager reads the latest value once per 1 ms step and never queues
 *    observations.
 *  - Button toggles and Motion events arrive through a bounded queue.
 *  - The Manager alone calls line_motion_bridge, so the bridge alone maps the
 *    L/C/R pattern to Motion's latest-value STEER.
 *  - The Manager alone issues discrete commands through motion_command() and
 *    motion_request_stop(). Motion remains the only PWM owner.
 *
 * The core has no Pico SDK and no RTOS include, so the host tests run it
 * against the simulated motion_hal.h.
 */
#ifndef MANAGER_H
#define MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "manager_types.h"

/* ---- Vehicle states ---------------------------------------------------- */

typedef enum
{
    MANAGER_SELFTEST = 0,       /* Encoder self-test requested at boot.       */
    MANAGER_LINE_FOLLOW_OFF,    /* Idle: motors stopped, follow not armed.    */
    MANAGER_LINE_FOLLOW_ON,     /* Following: STEER refreshed every sample.   */
    MANAGER_RECOVERY,           /* Bounded reverse follow after a lost line.  */
    MANAGER_FAULT,              /* Latched fault: no drive command is sent.   */
    MANAGER_STATE_COUNT
} manager_state_t;

/** The last valid forward direction, retained for the reverse manoeuvre. */
typedef enum
{
    MANAGER_DIR_STRAIGHT = 0,
    MANAGER_DIR_LEFT,
    MANAGER_DIR_RIGHT,
    MANAGER_DIR_COUNT
} manager_line_dir_t;

/* ---- Diagnostics ------------------------------------------------------- */

typedef struct
{
    uint32_t steps;                /* manager_step() calls.                  */
    uint32_t events_dropped;       /* Bounded queue full.                    */
    uint32_t line_overwrites;      /* Observations overwritten before use.   */
    uint32_t button_toggles;       /* Accepted short presses.                */
    uint32_t button_ignored;       /* Toggles refused in SELFTEST or FAULT.  */
    uint32_t recoveries;           /* RECOVERY manoeuvres started.           */
    uint32_t reacquisitions;       /* Recoveries ended on three samples.     */
    uint32_t recovery_samples;     /* Valid samples seen during recovery.    */
    uint32_t line_faults;          /* FAULT_LINE_SENSOR latches.             */
    uint32_t motion_rejections;    /* Motion refused a command.              */
    uint32_t steer_stale;          /* MOTION_EVT_STEER_STALE events.         */
} manager_stats_t;

/* ---- Lifecycle --------------------------------------------------------- */

/**
 * @brief Reset all Manager state and request Motion's encoder self-test.
 *
 * Call motion_hal_pico_init() and motion_init() first, then start the 20 ms
 * Motion tick, then call this once.
 */
void             manager_init(uint32_t now_ms);

/**
 * @brief Publish the newest line observation (allocating gate, Line task/ISR).
 *
 * One aligned 32-bit store, so it is safe against the 1 ms Manager step. A
 * second call before the Manager runs replaces the first value.
 */
void             manager_publish_line(const line_observation_t * p_obs);

/** Post a discrete event to the bounded queue. False when the queue is full. */
bool             manager_post_event(const manager_evt_t * p_evt);

/** Convenience wrapper: post a short press for one button. */
bool             manager_post_button(manager_button_t button, uint32_t now_ms);

/* ---- Periodic work ----------------------------------------------------- */

/**
 * @brief One Manager step. Call every 1 ms from the Manager loop or task.
 *
 * Reads the line mailbox, refreshes STEER, runs the loss timer and the
 * recovery sample counter, and drains the event queue.
 */
void             manager_step(uint32_t now_ms);

/* ---- Observation ------------------------------------------------------- */

manager_state_t  manager_get_state(void);
manager_line_dir_t manager_get_line_direction(void);

/** Manager latches OR Motion's fault bits (robot_shared.h FAULT_* names). */
uint32_t         manager_get_fault_bits(void);

void             manager_get_stats(manager_stats_t * p_stats);

const char *     manager_state_name(manager_state_t state);
const char *     manager_dir_name(manager_line_dir_t dir);

#endif /* MANAGER_H */

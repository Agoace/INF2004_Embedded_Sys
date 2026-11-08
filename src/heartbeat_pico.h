/*
 * Pico layer of the heartbeat (docs/architecture/ADR_HEARTBEAT.md): the 1 ms
 * hardware
 * timer and the hardware watchdog. Only this file touches them.
 *
 * heartbeat_start(): call after every hb_add_hook() and hb_add_flag().
 * It starts a repeating timer (start-to-start, 1 ms) whose interrupt calls
 * hb_tick() and hb_isr_done() and restarts the watchdog. Only the
 * heartbeat restarts the watchdog: if the timer interrupt stops, the chip
 * resets after watchdog_ms and the PWM pins go to 0. A hang of the main
 * loop does not reset the chip; the motor controller's stale stop
 * handles that (docs/architecture/ADR_HEARTBEAT.md, decision 5).
 *
 * watchdog_ms 0: no watchdog (bench use only).
 */
#ifndef HEARTBEAT_PICO_H
#define HEARTBEAT_PICO_H

#include <stdbool.h>
#include <stdint.h>

#include "heartbeat.h"

bool heartbeat_start(hb_t *hb, uint32_t watchdog_ms);

/* Stop the timer (bench use: the watchdog test). */
void heartbeat_stop(void);

/* True if the last reset came from the watchdog. */
bool heartbeat_watchdog_reset(void);

#endif

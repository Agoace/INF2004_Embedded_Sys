/*
 * idle_meter.h - CPU load meter built on a lowest-priority counting task.
 *
 * Method:
 *   1. idle_meter_calibrate() runs once from usermain, before any
 *      application task, interrupt source or watchdog starts. It runs the
 *      counting loop with interrupts disabled for a fixed TIMER window and
 *      stores the iterations per second as the peak rate.
 *   2. idle_meter_start() creates a task at the lowest priority that runs
 *      the same loop forever. It cannot delay any other task.
 *   3. idle_meter_load_permille() (health task, 1 Hz) turns the count since
 *      its previous call into a load.
 *
 * What it measures: all time that is not spent in the counting loop, so the
 * time of interrupts, every task and the Wi-Fi code on the same core counts
 * as busy. With SMP=0 (this robot) that is all work. The other core of an
 * SMP build is not covered. The loop replaces the kernel's WFI sleep, so
 * the CPU does not sleep (more power, no timing effect).
 */
#ifndef IDLE_METER_H
#define IDLE_METER_H

#include <stdint.h>

/* TIMER window of the calibration, microseconds. */
#define IDLE_METER_CALIBRATION_US 4000u

/* Run once, interrupts disabled for the window. Returns E_OK or a kernel
 * error (then the load stays unknown). */
int idle_meter_calibrate(void);

/* Create the counting task. Returns the task ID or a negative error. */
int idle_meter_start(void);

/* Load in permille since the previous call, or 65535 if unknown. */
uint16_t idle_meter_load_permille(void);

/* Peak rate from the calibration, iterations per second (0 if none). */
uint32_t idle_meter_peak_rate(void);

#endif

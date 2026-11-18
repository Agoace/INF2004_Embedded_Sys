#ifndef OBSTACLE_SCAN_DRIVER_H
#define OBSTACLE_SCAN_DRIVER_H

#include "../app_program/subsystem5/obstacle_scan.h"
#include "ultrasonic.h"

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

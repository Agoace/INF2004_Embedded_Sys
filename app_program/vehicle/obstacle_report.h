/*
 * obstacle_report.h - one obstacle profile report per completed scan.
 *
 * The report is built from the PROFILE word and the 13 scan points that the
 * scan module already produces. This module adds no scan or detour logic.
 * The conversion is pure: no hardware, RTOS or network calls.
 *
 * The profile analysis does not estimate obstacle width, so no width is
 * reported. Clearance is the angle from straight ahead to the first free
 * scan point on each side.
 */
#ifndef OBSTACLE_REPORT_H
#define OBSTACLE_REPORT_H

#include <stdbool.h>
#include <stdint.h>

#include "../subsystem1/mqtt_wire.h"
#include "../subsystem5/obstacle_profile.h"

/* Returned by obstacle_closest_index() when no scan point is VALID. */
#define OBSTACLE_REPORT_NO_POINT 255u

/* Clearance angle reported for a blocked side. */
#define OBSTACLE_REPORT_BLOCKED_DEG 255u

/*
 * Index (0-12) of the nearest VALID scan point, or OBSTACLE_REPORT_NO_POINT.
 * On equal distances the point nearest to straight ahead wins. If two such
 * points are equally far from straight ahead, the right one (lower index)
 * wins.
 */
uint8_t obstacle_closest_index(const obstacle_profile_t *profile);

/*
 * Fill out from a finished scan.
 *
 * profile    the 13 scan points.
 * word       PROFILE word built from the same points.
 * side       decision made from the word (obstacle_side_t).
 * scan_id    number of this report (1 and up).
 * uptime_ms  robot uptime when the scan completed.
 */
void obstacle_report_make(const obstacle_profile_t *profile,
                          obstacle_profile_word_t word, obstacle_side_t side,
                          uint32_t scan_id, uint32_t uptime_ms,
                          rm_obstacle_t *out);

/*
 * Copy the newest published report (vehicle task writes, network reads).
 * Returns E_OK, E_TMOUT if a write is in progress, or E_NOEXS if no scan
 * has completed yet. A failed read leaves out unchanged.
 */
int vehicle_obstacle_try_read(rm_obstacle_t *out);

#endif

/* robot_health.h - low-rate health task and its readable snapshot. */
#ifndef ROBOT_HEALTH_H
#define ROBOT_HEALTH_H
#include "mqtt_wire.h"
#include "../platform/fault_mask.h"
int robot_health_init(void);
int robot_health_start(void);
/* E_TMOUT unavailable/busy. Bounded copy; no waits or formatting. */
int robot_health_try_read(rm_heartbeat_t *out, uint32_t *timestamp_us);
/* Vehicle only: after owner STOP publication, before rt_begin(even).
 * Copy owners separately; Health alone combines them. Never waits. */
void robot_health_faults_stop(uint32_t odd_epoch);
/* Latest coherent Health diagnostic views, including sample time.
 * E_TMOUT also means Health has not sampled the current START/STOP phase.
 * Failed reads leave both outputs unchanged; no wire change. */
int robot_health_faults_try_read(fault_mask_snapshot_t *out, uint32_t *timestamp_us);
#endif

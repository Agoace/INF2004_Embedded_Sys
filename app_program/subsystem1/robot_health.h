#ifndef ROBOT_HEALTH_H
#define ROBOT_HEALTH_H
#include "mqtt_wire.h"
int robot_health_init(void);
int robot_health_start(void);
/* E_TMOUT unavailable/busy. Bounded copy; no waits or formatting. */
int robot_health_try_read(rm_heartbeat_t *out,uint32_t *timestamp_us);
#endif

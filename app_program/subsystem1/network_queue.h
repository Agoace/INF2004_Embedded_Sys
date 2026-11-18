/*
 * network_queue.h - network queue and metrics access.
 * Public functions not already declared in robot_network.h.
 */
#ifndef NETWORK_QUEUE_H
#define NETWORK_QUEUE_H
#include "robot_network.h"
int robot_event_pop(rm_event_t *out);
void robot_command_lost(void);
void robot_queue_metrics(rm_heartbeat_t *out);
#endif

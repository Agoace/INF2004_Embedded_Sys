/*
 * robot_network.h - interface between the robot application and the MQTT
 * network module.
 *
 * Producers (vehicle, sensors, health) never call lwIP. They push events and
 * read commands through bounded queues, and provide data through weak
 * functions that the network module polls.
 */
#ifndef ROBOT_NETWORK_H
#define ROBOT_NETWORK_H
#include "mqtt_wire.h"

enum {
    ROBOT_BOARD_LINE,
    ROBOT_BOARD_SONAR,
    ROBOT_BOARD_MOTION,
    ROBOT_BOARD_VEHICLE,
    ROBOT_BOARD_HEALTH,
    ROBOT_BOARD_COUNT
};

typedef struct {
    uint8_t opcode;
    uint32_t request_id;
} robot_command_t;

int robot_network_init(void);
/* Vehicle task is the ONLY event producer. Nonblocking, lost counted. */
int robot_event_push(const rm_event_t *event);
/* Wi-Fi task producer, vehicle task consumer; ESTOP has a separate slot. */
int robot_command_push(const robot_command_t *command);
int robot_command_pop(robot_command_t *out);
void robot_network_busy(unsigned int board);
void robot_network_metrics(rm_heartbeat_t *out);
/* Integration providers. Weak safe defaults allow standalone branch builds.
 * No lwIP calls in any provider; no provider may wait for a sensor task. */
void robot_network_state(rm_state_t *out);
int robot_network_sonar(uint32_t *cursor, rm_sonar_t *out, uint32_t *missed);
/* Newest obstacle report if its scan_id differs from *cursor. E_OK new,
 * E_NOEXS nothing new, other negative values: board busy. */
int robot_network_obstacle(uint32_t *cursor, rm_obstacle_t *out);
void robot_health_collect(rm_heartbeat_t *out);
/* Vehicle posts a single pending dump request. No lwIP or logging waits. */
int robot_network_dump(uint32_t request_id);
#endif

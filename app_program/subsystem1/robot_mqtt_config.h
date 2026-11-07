#ifndef ROBOT_MQTT_CONFIG_H
#define ROBOT_MQTT_CONFIG_H
/* Application-owned settings. Never use the template demo MQTT settings. */
#if defined(__has_include)
#if __has_include("robot_mqtt_secrets.h")
#include "robot_mqtt_secrets.h"
#endif
#endif
#ifndef ROBOT_MQTT_BROKER_IP
#define ROBOT_MQTT_BROKER_IP "0.0.0.0"
#endif
#ifndef ROBOT_MQTT_BROKER_PORT
#define ROBOT_MQTT_BROKER_PORT 1883
#endif
#ifndef ROBOT_MQTT_CLIENT_ID
#define ROBOT_MQTT_CLIENT_ID "inf2004-robot"
#endif
#ifndef ROBOT_MQTT_COMMAND_TOKEN_HEX
#define ROBOT_MQTT_COMMAND_TOKEN_HEX "00000000000000000000000000000000"
#endif
#endif

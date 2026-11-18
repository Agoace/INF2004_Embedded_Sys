#ifndef MQTT_TEST_IP_ADDR_H
#define MQTT_TEST_IP_ADDR_H
#include <stdint.h>
typedef struct { uint32_t addr;unsigned int type; } ip_addr_t;
#define IPADDR_TYPE_V4 0u
#define ip_2_ip4(value) (value)
#define ip4_addr_isany_val(value) ((value).addr==0u)
#define IP_SET_TYPE(value,kind) ((value)->type=(kind))
int ip4addr_aton(const char *text,ip_addr_t *address);
#endif

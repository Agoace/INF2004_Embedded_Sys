#ifndef MQTT_TEST_CLIENT_H
#define MQTT_TEST_CLIENT_H
#include <stdint.h>
#include "lwip/ip_addr.h"
typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int err_t;
#define ERR_OK 0
#define ERR_MEM (-1)
#define ERR_ABRT (-13)
#define MQTT_DATA_FLAG_LAST 1u
typedef enum { MQTT_CONNECT_ACCEPTED=0,MQTT_CONNECT_REFUSED_SERVER=3,
    MQTT_CONNECT_DISCONNECTED=256,MQTT_CONNECT_TIMEOUT=257 } mqtt_connection_status_t;
typedef struct { int connected; } mqtt_client_t;
/* Same field order as the SDK lwIP mqtt.h (without the TLS member). */
struct mqtt_connect_client_info_t {
    const char *client_id;
    const char *client_user;
    const char *client_pass;
    u16_t keep_alive;
    const char *will_topic;
    const char *will_msg;
    u8_t will_msg_len;
    u8_t will_qos;
    u8_t will_retain;
};
typedef void (*mqtt_connection_cb_t)(mqtt_client_t *,void *,mqtt_connection_status_t);
typedef void (*mqtt_request_cb_t)(void *,err_t);
typedef void (*mqtt_incoming_publish_cb_t)(void *,const char *,u32_t);
typedef void (*mqtt_incoming_data_cb_t)(void *,const u8_t *,u16_t,u8_t);
mqtt_client_t *mqtt_client_new(void);
u8_t mqtt_client_is_connected(mqtt_client_t *client);
err_t mqtt_client_connect(mqtt_client_t *client,const ip_addr_t *address,u16_t port,
    mqtt_connection_cb_t callback,void *arg,const struct mqtt_connect_client_info_t *info);
void mqtt_set_inpub_callback(mqtt_client_t *client,mqtt_incoming_publish_cb_t publish,
    mqtt_incoming_data_cb_t data,void *arg);
err_t mqtt_subscribe(mqtt_client_t *client,const char *topic,u8_t qos,mqtt_request_cb_t callback,void *arg);
err_t mqtt_publish(mqtt_client_t *client,const char *topic,const void *payload,u16_t length,
    u8_t qos,u8_t retain,mqtt_request_cb_t callback,void *arg);
#endif

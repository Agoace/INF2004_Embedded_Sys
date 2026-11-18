/* Private retained values; no change to public rm_* types or MQTT wire. */
#ifndef NETWORK_PACK_H
#define NETWORK_PACK_H
#include "mqtt_wire.h"
#include "../platform/value_bits.h"

typedef struct { uint32_t words[8]; } network_sonar_packed_t;
typedef struct { uint32_t words[26]; } network_health_packed_t;
#define NETWORK_SONAR_U32_FIELDS(X) \
    X(history_sequence) X(generation) X(request_id) X(trigger_ms) \
    X(timestamp_ms) X(ping_sequence) X(boot_id)
#define NETWORK_HEALTH_U32_FIELDS(X) \
    X(uptime_ms) X(connect_attempts) X(telemetry_lost) X(event_lost) \
    X(command_lost) X(sonar_history_missed) X(vehicle_overruns) \
    X(motion_late) X(motion_missed) X(mqtt_hook_max_us) X(publish_ok) \
    X(publish_failed) X(line_busy) X(sonar_busy) X(motion_busy) \
    X(vehicle_busy) X(health_busy) X(latest_sonar_history_sequence) \
    X(wifi_poll_gap_max_us) X(wifi_poll_gap_samples) X(boot_id) X(board_unavailable)

static inline bool network_sonar_pack(network_sonar_packed_t *out,
                                      const rm_sonar_t *v)
{
    if (out==0 || v==0 || v->status>3u || v->angle < -30 || v->angle>30 ||
        v->ping_angle < -30 || v->ping_angle>30 || v->echo>0x3fffu) return false;
    network_sonar_packed_t p={{0}};
    uint8_t *bytes=(uint8_t *)p.words;
    unsigned int bit=0u;
#define PUT32(field) value_bits_put(bytes,&bit,v->field,32u);
    NETWORK_SONAR_U32_FIELDS(PUT32)
#undef PUT32
    value_bits_put(bytes,&bit,v->status,2u);
    value_bits_put(bytes,&bit,v->valid?1u:0u,1u);
    value_bits_put(bytes,&bit,v->pending?1u:0u,1u);
    value_bits_put(bytes,&bit,(uint32_t)((int)v->angle+30),6u);
    value_bits_put(bytes,&bit,(uint32_t)((int)v->ping_angle+30),6u);
    value_bits_put(bytes,&bit,v->echo,14u);
    *out=p;return true;
}
static inline bool network_sonar_unpack(rm_sonar_t *out,
                                        const network_sonar_packed_t *p)
{
    if (out==0 || p==0) return false;
    const uint8_t *bytes=(const uint8_t *)p->words;
    if ((bytes[31]&0xc0u)!=0u) return false;
    unsigned int bit=0u;rm_sonar_t v={0};
#define GET32(field) v.field=value_bits_get(bytes,&bit,32u);
    NETWORK_SONAR_U32_FIELDS(GET32)
#undef GET32
    v.status=(uint8_t)value_bits_get(bytes,&bit,2u);
    v.valid=value_bits_get(bytes,&bit,1u)!=0u;
    v.pending=value_bits_get(bytes,&bit,1u)!=0u;
    const uint32_t angle=value_bits_get(bytes,&bit,6u);
    const uint32_t ping=value_bits_get(bytes,&bit,6u);
    if (angle>60u || ping>60u) return false;
    v.angle=(int8_t)((int)angle-30);v.ping_angle=(int8_t)((int)ping-30);
    v.echo=(uint16_t)value_bits_get(bytes,&bit,14u);
    *out=v;return true;
}

static inline bool network_health_pack(network_health_packed_t *out,
                                       const rm_heartbeat_t *v)
{
    if (out==0 || v==0 || v->flags>127u || v->reset_reason>2u ||
        v->wifi_link_status < -3 || v->wifi_link_status>3 || v->mqtt_state>3u ||
        v->wifi_requested_priority>32u || v->wifi_observed_priority>32u ||
        v->board_unavailable_mask>31u ||
        (v->cpu_load_permille>1000u && v->cpu_load_permille!=UINT16_MAX)) return false;
    network_health_packed_t p={{0}};
    uint8_t *bytes=(uint8_t *)p.words;unsigned int bit=0u;
#define PUT32(field) value_bits_put(bytes,&bit,v->field,32u);
    NETWORK_HEALTH_U32_FIELDS(PUT32)
#undef PUT32
    value_bits_put(bytes,&bit,(uint32_t)v->wifi_start_status,32u);
    value_bits_put(bytes,&bit,v->stack_min_free,16u);
    value_bits_put(bytes,&bit,v->health_age_ms,16u);
    value_bits_put(bytes,&bit,v->cpu_load_permille==UINT16_MAX?1001u:v->cpu_load_permille,10u);
    value_bits_put(bytes,&bit,v->flags,7u);
    value_bits_put(bytes,&bit,v->reset_reason,2u);
    value_bits_put(bytes,&bit,(uint32_t)((int)v->wifi_link_status+3),3u);
    value_bits_put(bytes,&bit,v->mqtt_state,2u);
    value_bits_put(bytes,&bit,v->wifi_requested_priority,6u);
    value_bits_put(bytes,&bit,v->wifi_observed_priority,6u);
    value_bits_put(bytes,&bit,v->board_unavailable_mask,5u);
    *out=p;return true;
}
static inline bool network_health_unpack(rm_heartbeat_t *out,
                                         const network_health_packed_t *p)
{
    if (out==0 || p==0) return false;
    const uint8_t *bytes=(const uint8_t *)p->words;
    if ((bytes[101]&0xfeu)!=0u || bytes[102]!=0u || bytes[103]!=0u) return false;
    unsigned int bit=0u;rm_heartbeat_t v={0};
#define GET32(field) v.field=value_bits_get(bytes,&bit,32u);
    NETWORK_HEALTH_U32_FIELDS(GET32)
#undef GET32
    v.wifi_start_status=value_bits_i32(value_bits_get(bytes,&bit,32u));
    v.stack_min_free=(uint16_t)value_bits_get(bytes,&bit,16u);
    v.health_age_ms=(uint16_t)value_bits_get(bytes,&bit,16u);
    const uint32_t cpu=value_bits_get(bytes,&bit,10u);
    v.cpu_load_permille=cpu==1001u?UINT16_MAX:(uint16_t)cpu;
    v.flags=(uint8_t)value_bits_get(bytes,&bit,7u);
    v.reset_reason=(uint8_t)value_bits_get(bytes,&bit,2u);
    const uint32_t link=value_bits_get(bytes,&bit,3u);
    v.wifi_link_status=(int8_t)((int)link-3);
    v.mqtt_state=(uint8_t)value_bits_get(bytes,&bit,2u);
    v.wifi_requested_priority=(uint8_t)value_bits_get(bytes,&bit,6u);
    v.wifi_observed_priority=(uint8_t)value_bits_get(bytes,&bit,6u);
    v.board_unavailable_mask=(uint8_t)value_bits_get(bytes,&bit,5u);
    if (cpu>1001u || link>6u || v.reset_reason>2u ||
        v.wifi_requested_priority>32u || v.wifi_observed_priority>32u) return false;
    *out=v;return true;
}
#endif

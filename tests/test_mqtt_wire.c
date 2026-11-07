#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../app_program/subsystem1/mqtt_wire.h"
static void emit(const char *topic,const uint8_t *data,size_t size) {
    printf("robot/%s ",topic);
    for (size_t i=0u;i<size;++i) printf("%02x",(unsigned int)data[i]);
    putchar('\n');
}
int main(void) {
    uint8_t bytes[RM_HEARTBEAT_SIZE];
    const rm_state_t state={.flags=0xffu,.state=0x13u,.module=0x25u,.motion_faults=0x81u,
        .uptime_ms=0x12345678u,.sequence=0xfedcba98u,.line_age_ms=10u,.motion_age_ms=20u,
        .vehicle_age_ms=65535u,.raw_bc=0xfabcu,.kr1={1u,2u,3u,4u},
        .count_l=INT32_MIN,.count_r=-1234567,.norm_l=0xffffu,.norm_c=0x555u};
    rm_pack_state(bytes,&state);
    assert(bytes[2]==0x53u && bytes[4]==0x78u && bytes[7]==0x12u && bytes[19]==0x0au && bytes[47]==0u);
    emit("state",bytes,RM_STATE_SIZE);
    const rm_sonar_t sonar={.status=0xfdu,.valid=true,.pending=true,.angle=-30,.ping_angle=30,
        .history_sequence=0xffffffffu,.generation=0x12345678u,.request_id=33u,
        .trigger_ms=1000u,.timestamp_ms=1010u,.echo=0x4567u,.ping_sequence=123u};
    rm_pack_sonar(bytes,&sonar);assert(bytes[1]==13u && bytes[2]==226u && bytes[26]==0u && bytes[27]==0u);
    emit("sonar",bytes,RM_SONAR_SIZE);
    const rm_event_t event={.kind=6u,.from=2u,.to=3u,.sequence=0x12345678u,.uptime_ms=1234u,
        .word=0xabcdu,.detail=4u,.flags=0xffu};
    uint8_t event_bytes[RM_EVENT_SIZE];rm_pack_event(event_bytes,&event);assert(event_bytes[15]==1u);
    emit("event",event_bytes,RM_EVENT_SIZE);
    const rm_heartbeat_t heartbeat={.flags=0xffu,.reset_reason=2u,.uptime_ms=1u,.connect_attempts=2u,
        .telemetry_lost=3u,.event_lost=4u,.command_lost=5u,.sonar_history_missed=6u,
        .vehicle_overruns=7u,.motion_late=8u,.motion_missed=9u,.stack_min_free=0x1234u,
        .mqtt_hook_max_us=10u,.publish_ok=11u,.publish_failed=12u,.line_busy=13u,.sonar_busy=14u,
        .motion_busy=15u,.vehicle_busy=16u,.health_busy=17u,.latest_sonar_history_sequence=18u,
        .health_age_ms=65535u,.wifi_link_status=-3,.mqtt_state=3u,.wifi_poll_gap_max_us=19u,
        .wifi_poll_gap_samples=20u,.wifi_start_status=-50,.wifi_requested_priority=20u,.wifi_observed_priority=20u};
    rm_pack_heartbeat(bytes,&heartbeat);assert(bytes[1]==63u && bytes[3]==0u && bytes[42]==0u && bytes[98]==0u && bytes[99]==0u);
    emit("heartbeat",bytes,RM_HEARTBEAT_SIZE);
    rm_log_t log={.kind=RM_LOG_EVENT,.dump_id=111u,.generation=222u,.index=333u};
    memcpy(log.body,event_bytes,16u);rm_pack_log(bytes,&log);assert(bytes[2]==0u && bytes[3]==0u);
    emit("log",bytes,RM_LOG_SIZE);
    rm_command_t command={.opcode=RM_CMD_START,.request_id=0x12345678u},decoded={.opcode=99u};
    for (size_t i=0u;i<RM_TOKEN_SIZE;++i) command.token[i]=(uint8_t)i;
    rm_pack_command(bytes,&command);emit("cmd",bytes,RM_COMMAND_SIZE);
    assert(rm_command_decode(bytes,RM_COMMAND_SIZE,command.token,true,&decoded));
    assert(decoded.opcode==command.opcode && decoded.request_id==command.request_id && memcmp(decoded.token,command.token,16u)==0);
    assert(!rm_command_decode(bytes,RM_COMMAND_SIZE,command.token,false,&decoded));
    for (size_t i=0u;i<RM_TOKEN_SIZE;++i) {
        bytes[6u+i]^=1u;assert(!rm_command_decode(bytes,RM_COMMAND_SIZE,command.token,true,&decoded));bytes[6u+i]^=1u;
    }
    for (size_t i=0u;i<RM_COMMAND_SIZE;++i) assert(!rm_command_decode(bytes,i,command.token,true,&decoded));
    assert(!rm_command_decode(bytes,RM_COMMAND_SIZE+1u,command.token,true,&decoded));
    bytes[0]=2u;assert(!rm_command_decode(bytes,RM_COMMAND_SIZE,command.token,true,&decoded));bytes[0]=1u;
    bytes[1]=0u;assert(!rm_command_decode(bytes,RM_COMMAND_SIZE,command.token,true,&decoded));
    bytes[1]=5u;assert(!rm_command_decode(bytes,RM_COMMAND_SIZE,command.token,true,&decoded));
    assert(!rm_command_decode(NULL,RM_COMMAND_SIZE,command.token,true,&decoded));
    assert(!rm_command_decode(bytes,RM_COMMAND_SIZE,NULL,true,&decoded));
    assert(!rm_command_decode(bytes,RM_COMMAND_SIZE,command.token,true,NULL));
    assert(decoded.opcode==RM_CMD_START); /* Rejections leave output unchanged. */
    for (uint8_t op=0u;op<6u;++op) {
        assert(rm_command_allowed(op,false)==(op==1u || op==2u));
        assert(rm_command_allowed(op,true)==(op>=1u && op<=4u));
    }
    return 0;
}

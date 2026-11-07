#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../app_program/subsystem1/mqtt_wire.h"
static void emit(const char *topic,const uint8_t *data,size_t size) {
    printf("robot/%s ",topic);
    for (size_t i=0u;i<size;++i) printf("%02x",(unsigned int)data[i]);
    putchar('\n');
}
static void emit_batch(const char *name,const rm_log_batch_t *header,const uint8_t *records) {
    uint8_t frame[RM_LOG_BATCH_MAX_SIZE];
    const size_t size=rm_pack_log_batch(frame,sizeof(frame),header,records);
    assert(size==RM_LOG_BATCH_HEADER_SIZE+((size_t)header->count*RM_LOG_RECORD_SIZE));
    assert(frame[0]==1u && frame[1]==header->kind && frame[2]==header->count && frame[3]==header->flags);
    printf("batch/%s ",name);
    for (size_t i=0u;i<size;++i) printf("%02x",(unsigned int)frame[i]);
    putchar('\n');
}
static void test_batch_packer(void) {
    uint8_t records[RM_LOG_BATCH_MAX_COUNT*RM_LOG_RECORD_SIZE],frame[RM_LOG_BATCH_MAX_SIZE];
    for (size_t i=0u;i<sizeof(records);++i) records[i]=(uint8_t)(i*7u+3u);
    rm_log_batch_t header={.kind=RM_LOG_ROWS,.count=10u,.flags=RM_LOG_FLAG_JUMPED,.generation=0x01020304u,
        .first_index=0x0a0b0c0du,.publish_ms=0x11121314u,.newest_ms=0x21222324u,.log_rows=0x31323334u,
        .skipped_before=0x41424344u,.stream_id=0u};
    memset(frame,0xa5,sizeof(frame));
    assert(rm_pack_log_batch(frame,sizeof(frame),&header,records)==192u);
    assert(frame[4]==0u && frame[8]==4u && frame[11]==1u && frame[12]==0x0du && frame[15]==0x0au);
    assert(frame[16]==0x14u && frame[20]==0x24u && frame[24]==0x34u && frame[28]==0x44u);
    assert(memcmp(frame+32,records,160u)==0);
    /* Invalid input returns zero and leaves the output unchanged. */
    uint8_t before[RM_LOG_BATCH_MAX_SIZE];memset(frame,0xa5,sizeof(frame));memcpy(before,frame,sizeof(frame));
    header.count=0u;assert(rm_pack_log_batch(frame,sizeof(frame),&header,records)==0u);
    header.count=RM_LOG_BATCH_MAX_COUNT+1u;assert(rm_pack_log_batch(frame,sizeof(frame),&header,records)==0u);
    header.count=2u;header.kind=RM_LOG_ROW;assert(rm_pack_log_batch(frame,sizeof(frame),&header,records)==0u);
    header.kind=RM_LOG_BEGIN;assert(rm_pack_log_batch(frame,sizeof(frame),&header,records)==0u);
    header.kind=RM_LOG_ROWS;assert(rm_pack_log_batch(frame,63u,&header,records)==0u);
    assert(rm_pack_log_batch(NULL,sizeof(frame),&header,records)==0u);
    assert(rm_pack_log_batch(frame,sizeof(frame),NULL,records)==0u);
    assert(rm_pack_log_batch(frame,sizeof(frame),&header,NULL)==0u);
    assert(memcmp(frame,before,sizeof(frame))==0);
    assert(rm_pack_log_batch(frame,64u,&header,records)==64u);
    header.count=RM_LOG_BATCH_MAX_COUNT;assert(rm_pack_log_batch(frame,224u,&header,records)==224u);
    /* Fixtures for the Python decoder: live, jump, new generation, refill, max, wrap. */
    uint8_t events[3u*RM_EVENT_SIZE];
    for (uint8_t i=0u;i<3u;++i) {
        rm_event_t event={.kind=(uint8_t)(i+1u),.from=i,.to=(uint8_t)(i+1u),.sequence=100u+i,
            .uptime_ms=5000u+i,.word=(uint16_t)(0x1200u+i),.detail=(uint8_t)(i+7u),.flags=(uint8_t)(i&1u)};
        rm_pack_event(events+((size_t)i*RM_EVENT_SIZE),&event);
    }
    header=(rm_log_batch_t){.kind=RM_LOG_ROWS,.count=10u,.generation=7u,.first_index=100u,
        .publish_ms=5000u,.newest_ms=4950u,.log_rows=125u};
    emit_batch("live",&header,records);
    header=(rm_log_batch_t){.kind=RM_LOG_ROWS,.count=1u,.flags=RM_LOG_FLAG_JUMPED,.generation=7u,
        .first_index=240u,.publish_ms=9000u,.newest_ms=8990u,.log_rows=300u,.skipped_before=115u};
    emit_batch("jump",&header,records);
    header=(rm_log_batch_t){.kind=RM_LOG_ROWS,.count=2u,.flags=RM_LOG_FLAG_NEW_GENERATION,.generation=8u,
        .first_index=0u,.publish_ms=20000u,.newest_ms=19990u,.log_rows=2u,.skipped_before=33u};
    emit_batch("newgen",&header,records);
    header=(rm_log_batch_t){.kind=RM_LOG_ROWS,.count=12u,.flags=RM_LOG_FLAG_REFILL,.stream_id=111u,
        .generation=7u,.first_index=24u,.publish_ms=7000u,.log_rows=300u};
    emit_batch("refill_rows",&header,records);
    header=(rm_log_batch_t){.kind=RM_LOG_EVENTS,.count=3u,.flags=RM_LOG_FLAG_REFILL,.stream_id=111u,
        .generation=7u,.first_index=4u,.publish_ms=7001u,.log_rows=300u};
    emit_batch("refill_events",&header,events);
    header=(rm_log_batch_t){.kind=RM_LOG_ROWS,.count=1u,.generation=1u,.first_index=5u,
        .publish_ms=5u,.newest_ms=0xfffffff0u,.log_rows=6u};
    emit_batch("wrap",&header,records);
}
static void emit_named(const char *name,const uint8_t *data,size_t size) {
    printf("dash/%s ",name);
    for (size_t i=0u;i<size;++i) printf("%02x",(unsigned int)data[i]);
    putchar('\n');
}
static void set_bits(uint8_t kr1[16],unsigned int offset,unsigned int width,uint32_t value) {
    for (unsigned int bit=0u;bit<width;++bit) {
        const unsigned int at=offset+bit;
        kr1[at/8u]=(uint8_t)((kr1[at/8u] & ~(1u<<(at%8u))) | (((value>>bit)&1u)<<(at%8u)));
    }
}
/* Dashboard fixtures: robot/state, sonar, event and heartbeat messages packed by C. */
static void test_dashboard_fixtures(void) {
    uint8_t bytes[RM_HEARTBEAT_SIZE];
    rm_state_t state={.flags=0x0fu,.state=0u,.module=0u,.uptime_ms=10000u,.sequence=1u,
        .line_age_ms=5u,.motion_age_ms=8u,.vehicle_age_ms=10u,.raw_bc=777u,
        .count_l=1000,.count_r=-2000,.norm_l=500u,.norm_c=600u};
    set_bits(state.kr1,21u,12u,1234u);set_bits(state.kr1,33u,12u,2345u);
    set_bits(state.kr1,45u,8u,(uint32_t)(int32_t)-25 & 0xffu);
    set_bits(state.kr1,53u,10u,100u);set_bits(state.kr1,63u,10u,(uint32_t)(int32_t)-100 & 0x3ffu);
    set_bits(state.kr1,73u,10u,90u);set_bits(state.kr1,83u,10u,(uint32_t)(int32_t)-95 & 0x3ffu);
    set_bits(state.kr1,109u,7u,(uint32_t)(int32_t)-15 & 0x7fu);
    rm_pack_state(bytes,&state);emit_named("state_idle",bytes,RM_STATE_SIZE);
    state.flags=0x8fu;state.state=1u;state.module=1u;state.sequence=2u;
    rm_pack_state(bytes,&state);emit_named("state_running",bytes,RM_STATE_SIZE);
    state.flags=0x07u;state.state=0u;state.module=0u;state.vehicle_age_ms=65535u;state.sequence=3u;
    rm_pack_state(bytes,&state);emit_named("state_vehicle_stale",bytes,RM_STATE_SIZE);
    rm_sonar_t sonar={.status=1u,.valid=true,.angle=-30,.ping_angle=-30,.history_sequence=5u,
        .generation=1u,.request_id=2u,.trigger_ms=100u,.timestamp_ms=110u,.echo=345u,.ping_sequence=9u};
    rm_pack_sonar(bytes,&sonar);emit_named("sonar_valid",bytes,RM_SONAR_SIZE);
    sonar.history_sequence=6u;sonar.angle=30;sonar.valid=false;sonar.echo=(uint16_t)(1u<<9);
    rm_pack_sonar(bytes,&sonar);emit_named("sonar_error",bytes,RM_SONAR_SIZE);
    rm_event_t event={.kind=6u,.from=0u,.to=0u,.sequence=10u,.uptime_ms=11000u,
        .word=RM_CMD_START,.detail=0u};
    rm_pack_event(bytes,&event);emit_named("event_start_ok",bytes,RM_EVENT_SIZE);
    event.sequence=11u;event.word=(uint16_t)(RM_CMD_START|(1u<<8));event.detail=1u;
    rm_pack_event(bytes,&event);emit_named("event_start_not_idle",bytes,RM_EVENT_SIZE);
    event.sequence=14u;event.word=RM_CMD_CALIBRATE;event.detail=0u;
    rm_pack_event(bytes,&event);emit_named("event_calibrate_ok",bytes,RM_EVENT_SIZE);
    event.sequence=15u;event.kind=5u;event.from=1u;event.to=0u;event.word=7u;event.detail=2u;
    rm_pack_event(bytes,&event);emit_named("event_run_end",bytes,RM_EVENT_SIZE);
    rm_heartbeat_t beat={.flags=0x37u,.reset_reason=1u,.uptime_ms=60000u,.connect_attempts=2u,
        .telemetry_lost=3u,.event_lost=4u,.command_lost=5u,.stack_min_free=1234u,
        .publish_ok=100u,.publish_failed=2u,.line_busy=6u,.sonar_busy=7u,.motion_busy=8u,
        .vehicle_busy=9u,.health_busy=10u,.vehicle_overruns=11u,.motion_late=12u,.motion_missed=13u,
        .health_age_ms=40u,.mqtt_state=3u,.wifi_poll_gap_max_us=25000u,.wifi_poll_gap_samples=600u};
    rm_pack_heartbeat(bytes,&beat);emit_named("heartbeat",bytes,RM_HEARTBEAT_SIZE);
    beat.flags=0x3fu;
    rm_pack_heartbeat(bytes,&beat);emit_named("heartbeat_dump",bytes,RM_HEARTBEAT_SIZE);
}
int main(void) {
    test_batch_packer();
    test_dashboard_fixtures();
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
    /* No configured token: only ESTOP, with any 16-byte token, is allowed. */
    for (uint8_t op=0u;op<6u;++op) {
        command.opcode=op;rm_pack_command(bytes,&command);bytes[6]^=0xffu;
        assert(rm_command_decode(bytes,RM_COMMAND_SIZE,command.token,false,&decoded)==(op==RM_CMD_ESTOP));
    }
    command.opcode=RM_CMD_ESTOP;rm_pack_command(bytes,&command);
    for (size_t i=0u;i<RM_TOKEN_SIZE;++i) {
        bytes[6u+i]^=1u;
        assert(!rm_command_decode(bytes,RM_COMMAND_SIZE,command.token,true,&decoded));
        assert(rm_command_decode(bytes,RM_COMMAND_SIZE,command.token,false,&decoded));
        bytes[6u+i]^=1u;
    }
    for (size_t i=0u;i<RM_COMMAND_SIZE;++i) assert(!rm_command_decode(bytes,i,command.token,false,&decoded));
    assert(!rm_command_decode(bytes,RM_COMMAND_SIZE+1u,command.token,false,&decoded));
    bytes[0]=2u;assert(!rm_command_decode(bytes,RM_COMMAND_SIZE,command.token,false,&decoded));bytes[0]=1u;
    assert(!rm_command_decode(bytes,RM_COMMAND_SIZE,NULL,false,&decoded));
    assert(!rm_command_decode(NULL,RM_COMMAND_SIZE,command.token,false,&decoded));
    assert(!rm_command_decode(bytes,RM_COMMAND_SIZE,command.token,false,NULL));
    for (uint8_t op=0u;op<6u;++op) {
        assert(rm_command_allowed(op,false)==(op==1u || op==2u));
        assert(rm_command_allowed(op,true)==(op>=1u && op<=4u));
    }
    return 0;
}

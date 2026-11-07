#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <tk/tkernel.h>
#include "lwip/apps/mqtt.h"
#include "../app_program/subsystem1/robot_network.h"
#include "../app_program/subsystem1/network_queue.h"
#include "../app_program/subsystem1/robot_health.h"
#include "../app_program/subsystem1/telemetry.h"
#include "../lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.h"
#define FIXTURE_TOKEN_HEX "00112233445566778899aabbccddeeff"
typedef struct {
    const char *topic;uint8_t data[RM_HEARTBEAT_SIZE];uint16_t length;uint8_t qos;
    err_t result;mqtt_request_cb_t callback;void *arg;
} publication_t;
static publication_t publications[256];
static unsigned int publication_count,connect_count,new_count,subscribe_count,priority_reads;
static unsigned int irq_off,di_count,ei_count,hook_calls;
static uint32_t now_ms,clock_calls;
static bool in_hook,fail_new;
static err_t connect_result,subscribe_result;
static const char *fail_topic;
static unsigned int failures_left;
static mqtt_client_t mqtt_client;
static mqtt_connection_cb_t connection_callback;
static void *connection_arg;
static mqtt_request_cb_t subscribe_callback;
static void *subscribe_arg;
static mqtt_incoming_publish_cb_t incoming_publish_callback;
static mqtt_incoming_data_cb_t incoming_data_callback;
static void *incoming_arg;
static publication_t *unacknowledged_log;
static uint32_t command_lost,event_lost,busy[5],commands_received;
static bool queue_full;
static robot_command_t last_command;
static rm_event_t events[4];static unsigned int event_head,event_tail;
static rm_sonar_t sonar[4];static unsigned int sonar_head,sonar_tail;
static uint32_t sonar_missed;
static bool sonar_busy,health_busy;
static unsigned int event_reads,sonar_reads;
static telemetry_log_index_t ram_index={.generation=7u,.rows=2u,.events=1u,.lost=9u};
static bool ram_busy;
static unsigned int row_reads,event_log_reads;
void mock_di(UINT *irq) { assert(!irq_off);*irq=33u;irq_off=1u;++di_count; }
void mock_ei(UINT irq) { assert(irq==33u && irq_off);irq_off=0u;++ei_count; }
uint32_t drv_time_ms(void) { return now_ms; }
uint32_t drv_time_us(void) { return now_ms*1000u+clock_calls++; }
ER tk_ref_tsk(ID id,T_RTSK *status) { assert(id==TSK_SELF && !irq_off);++priority_reads;status->tskpri=20;return E_OK; }
void cyw43_utk_get_status(T_CYW43_UTK_STATUS *status) { *status=(T_CYW43_UTK_STATUS){.link_status=3}; }
int32_t cyw43_utk_timing_read(T_CYW43_TIMING *value) { *value=(T_CYW43_TIMING){.gap_max_us=12345u,.gaps=100u};return E_OK; }
int32_t cyw43_utk_start_status(void) { return E_OK; }
int ip4addr_aton(const char *text,ip_addr_t *address) {
    assert(in_hook && strcmp(text,"192.0.2.10")==0);address->addr=0xc000020au;return 1;
}
mqtt_client_t *mqtt_client_new(void) { assert(in_hook && !irq_off);++new_count;return fail_new ? NULL : &mqtt_client; }
u8_t mqtt_client_is_connected(mqtt_client_t *client) { assert(in_hook && client==&mqtt_client);return (u8_t)client->connected; }
err_t mqtt_client_connect(mqtt_client_t *client,const ip_addr_t *address,u16_t port,
    mqtt_connection_cb_t callback,void *arg,const struct mqtt_connect_client_info_t *info) {
    assert(in_hook && !irq_off && client==&mqtt_client && address->addr==0xc000020au && port==1883u);
    assert(strcmp(info->client_id,"inf2004-robot")==0 && info->keep_alive==30u);
    ++connect_count;connection_callback=callback;connection_arg=arg;return connect_result;
}
void mqtt_set_inpub_callback(mqtt_client_t *client,mqtt_incoming_publish_cb_t publish,
    mqtt_incoming_data_cb_t data,void *arg) {
    assert(client==&mqtt_client);incoming_publish_callback=publish;incoming_data_callback=data;incoming_arg=arg;
}
err_t mqtt_subscribe(mqtt_client_t *client,const char *topic,u8_t qos,mqtt_request_cb_t callback,void *arg) {
    assert(in_hook && !irq_off && client==&mqtt_client && strcmp(topic,"robot/cmd")==0 && qos==1u);
    ++subscribe_count;subscribe_callback=callback;subscribe_arg=arg;return subscribe_result;
}
err_t mqtt_publish(mqtt_client_t *client,const char *topic,const void *payload,u16_t length,
    u8_t qos,u8_t retain,mqtt_request_cb_t callback,void *arg) {
    assert(in_hook && !irq_off && client==&mqtt_client && length<=RM_HEARTBEAT_SIZE && retain==0u);
    assert(publication_count<256u);
    publication_t *record=&publications[publication_count++];
    *record=(publication_t){.topic=topic,.length=length,.qos=qos,.callback=callback,.arg=arg};
    memcpy(record->data,payload,length);
    if (failures_left && fail_topic && strcmp(fail_topic,topic)==0) { --failures_left;record->result=ERR_MEM; }
    if (strcmp(topic,"robot/log")==0) {
        assert(qos==1u && length==RM_LOG_SIZE);
        if (record->result==ERR_OK) { assert(!unacknowledged_log);unacknowledged_log=record; }
    } else assert(qos==0u);
    return record->result;
}
void robot_network_state(rm_state_t *value) { assert(in_hook);*value=(rm_state_t){.flags=15u,.state=2u,.raw_bc=321u}; }
int robot_network_sonar(uint32_t *cursor,rm_sonar_t *value,uint32_t *missed) {
    assert(in_hook);++sonar_reads;*missed=sonar_missed;sonar_missed=0u;
    if (sonar_busy) return E_OBJ;
    if (sonar_tail==sonar_head) return E_TMOUT;
    *value=sonar[sonar_tail++];*cursor=value->history_sequence;return E_OK;
}
int robot_event_pop(rm_event_t *value) {
    assert(in_hook);++event_reads;
    if (event_tail==event_head) return E_TMOUT;
    *value=events[event_tail++];return (int)sizeof(*value);
}
int robot_command_push(const robot_command_t *value) {
    if (queue_full) { ++command_lost;return E_TMOUT; }
    last_command=*value;++commands_received;return E_OK;
}
void robot_command_lost(void) { ++command_lost; }
void robot_network_busy(unsigned int board) { assert(board<5u);++busy[board]; }
void robot_queue_metrics(rm_heartbeat_t *value) {
    value->command_lost=command_lost;value->event_lost=event_lost;
    value->line_busy=busy[0];value->sonar_busy=busy[1];value->motion_busy=busy[2];
    value->vehicle_busy=busy[3];value->health_busy=busy[4];
}
int robot_health_try_read(rm_heartbeat_t *value,uint32_t *timestamp_us) {
    assert(in_hook);
    if (health_busy) return E_TMOUT;
    *value=(rm_heartbeat_t){.telemetry_lost=13u,.stack_min_free=222u};
    *timestamp_us=now_ms*1000u;return E_OK;
}
int telemetry_log_open(telemetry_log_index_t *value) { assert(in_hook);if (ram_busy) return E_TMOUT;*value=ram_index;return E_OK; }
int telemetry_log_row(uint32_t generation,uint32_t index,uint8_t value[TELEMETRY_BYTES]) {
    assert(in_hook && generation==ram_index.generation && index<ram_index.rows);++row_reads;
    memset(value,(int)(index+1u),TELEMETRY_BYTES);return E_OK;
}
int telemetry_log_event(uint32_t generation,uint32_t index,telemetry_event_t *value) {
    assert(in_hook && generation==ram_index.generation && index<ram_index.events);++event_log_reads;
    *value=(telemetry_event_t){.ms=444u,.word=0x1234u,.kind=2u,.from=1u,.to=2u};return E_OK;
}
static uint32_t u32(const uint8_t *data) { return (uint32_t)data[0]|((uint32_t)data[1]<<8)|((uint32_t)data[2]<<16)|((uint32_t)data[3]<<24); }
static void poll_at(uint32_t at) {
    now_ms=at;const unsigned int before=publication_count;in_hook=true;cyw43_utk_app_poll();in_hook=false;
    assert(publication_count-before<=4u && !irq_off && di_count==ei_count);++hook_calls;
}
static void connected(mqtt_connection_status_t status) {
    assert(connection_callback);mqtt_client.connected=status==MQTT_CONNECT_ACCEPTED;
    connection_callback(&mqtt_client,connection_arg,status);
}
static void acknowledge(err_t error) {
    assert(unacknowledged_log);publication_t *record=unacknowledged_log;unacknowledged_log=NULL;record->callback(record->arg,error);
}
static publication_t *last_topic(const char *topic) {
    for (unsigned int i=publication_count;i>0u;--i) if (strcmp(publications[i-1u].topic,topic)==0) return &publications[i-1u];
    return NULL;
}
static unsigned int count_topic(const char *topic) {
    unsigned int count=0u;for (unsigned int i=0u;i<publication_count;++i) if (strcmp(publications[i].topic,topic)==0) ++count;
    return count;
}
static void start_connected(void) {
    poll_at(0u);assert(connect_count==1u);connected(MQTT_CONNECT_ACCEPTED);poll_at(0u);
    assert(subscribe_count==1u && subscribe_callback);subscribe_callback(subscribe_arg,ERR_OK);
}
static void test_connection(void) {
    fail_new=true;poll_at(0u);assert(new_count==1u && connect_count==0u);
    fail_new=false;poll_at(2999u);assert(new_count==1u);
    connect_result=ERR_MEM;poll_at(3000u);assert(connect_count==1u);
    poll_at(5999u);assert(connect_count==1u);
    connect_result=ERR_OK;poll_at(6000u);assert(connect_count==2u);
    poll_at(9000u);assert(connect_count==2u); /* Async connection still pending. */
    connected(MQTT_CONNECT_REFUSED_SERVER);poll_at(11999u);assert(connect_count==2u);
    poll_at(12000u);assert(connect_count==3u);connected(MQTT_CONNECT_ACCEPTED);
    subscribe_result=ERR_MEM;poll_at(12000u);assert(subscribe_count==1u);
    poll_at(14999u);assert(subscribe_count==1u);
    subscribe_result=ERR_OK;poll_at(15000u);assert(subscribe_count==2u);
    poll_at(18000u);assert(subscribe_count==2u); /* No second async subscription. */
    subscribe_callback(subscribe_arg,ERR_ABRT);poll_at(20999u);assert(subscribe_count==2u);
    poll_at(21000u);assert(subscribe_count==3u);subscribe_callback(subscribe_arg,ERR_OK);
    rm_heartbeat_t value={0};robot_network_metrics(&value);
    assert(value.connect_attempts==3u && value.publish_failed==5u && value.mqtt_state==3u);
    assert((value.flags&7u)==7u && value.wifi_requested_priority==20u && value.wifi_observed_priority==20u);
    assert(value.wifi_poll_gap_max_us==12345u && value.wifi_poll_gap_samples==100u && priority_reads==1u);
    connected(MQTT_CONNECT_DISCONNECTED);poll_at(23999u);assert(connect_count==3u);
    poll_at(24000u);assert(connect_count==4u);
}
static void deliver(const char *topic,const uint8_t *data,uint32_t declared,uint16_t first,uint16_t total) {
    incoming_publish_callback(incoming_arg,topic,declared);
    incoming_data_callback(incoming_arg,data,first,0u);
    incoming_data_callback(incoming_arg,data+first,(u16_t)(total-first),MQTT_DATA_FLAG_LAST);
}
static void test_commands(void) {
    start_connected();
    rm_command_t value={.opcode=RM_CMD_START,.request_id=0x12345678u};
    for (unsigned int i=0u;i<RM_TOKEN_SIZE;++i) value.token[i]=(uint8_t)(i*17u);
    uint8_t data[RM_COMMAND_SIZE+1u];rm_pack_command(data,&value);data[RM_COMMAND_SIZE]=0u;
    deliver("robot/cmd",data,RM_COMMAND_SIZE,3u,RM_COMMAND_SIZE);
    assert(commands_received==1u && last_command.opcode==RM_CMD_START && last_command.request_id==value.request_id);
    deliver("robot/other",data,RM_COMMAND_SIZE,2u,RM_COMMAND_SIZE);
    deliver("robot/cmd",data,RM_COMMAND_SIZE-1u,2u,RM_COMMAND_SIZE-1u);
    deliver("robot/cmd",data,RM_COMMAND_SIZE,2u,RM_COMMAND_SIZE-1u);
    deliver("robot/cmd",data,RM_COMMAND_SIZE,2u,RM_COMMAND_SIZE+1u);
    data[0]=2u;deliver("robot/cmd",data,RM_COMMAND_SIZE,1u,RM_COMMAND_SIZE);data[0]=1u;
    data[1]=5u;deliver("robot/cmd",data,RM_COMMAND_SIZE,1u,RM_COMMAND_SIZE);data[1]=RM_CMD_START;
    data[21]^=1u;deliver("robot/cmd",data,RM_COMMAND_SIZE,1u,RM_COMMAND_SIZE);data[21]^=1u;
    assert(commands_received==1u && command_lost==7u);
    queue_full=true;deliver("robot/cmd",data,RM_COMMAND_SIZE,11u,RM_COMMAND_SIZE);
    assert(commands_received==1u && command_lost==8u);
    queue_full=false;data[1]=RM_CMD_ESTOP;
    deliver("robot/cmd",data,RM_COMMAND_SIZE,0u,RM_COMMAND_SIZE);
    assert(commands_received==2u && last_command.opcode==RM_CMD_ESTOP);
}
static void test_disabled_commands(void) {
    start_connected();
    rm_heartbeat_t value={0};robot_network_metrics(&value);assert((value.flags&16u)==0u);
    rm_command_t command={.opcode=RM_CMD_ESTOP,.request_id=1u};
    uint8_t data[RM_COMMAND_SIZE];rm_pack_command(data,&command);
    deliver("robot/cmd",data,RM_COMMAND_SIZE,3u,RM_COMMAND_SIZE);
    assert(commands_received==0u && command_lost==1u);
}
static void test_schedule(void) {
    start_connected();
    assert(count_topic("robot/state")==1u && count_topic("robot/heartbeat")==1u);
    poll_at(199u);assert(count_topic("robot/state")==1u);
    poll_at(200u);assert(count_topic("robot/state")==2u && count_topic("robot/heartbeat")==1u);
    poll_at(999u);assert(count_topic("robot/state")==3u && count_topic("robot/heartbeat")==1u);
    health_busy=true;poll_at(1000u);
    assert(count_topic("robot/state")==3u && count_topic("robot/heartbeat")==2u);
    publication_t *heartbeat=last_topic("robot/heartbeat");
    assert((heartbeat->data[1]&32u)==0u && u32(heartbeat->data+12)==13u && busy[ROBOT_BOARD_HEALTH]==1u);
    assert(heartbeat->data[40]==222u); /* Busy retains previous health cache. */
    events[event_head++]=(rm_event_t){.kind=2u,.sequence=7u};
    for (unsigned int i=0u;i<4u;++i) sonar[sonar_head++]=(rm_sonar_t){.history_sequence=10u+i,.status=1u};
    assert(robot_network_dump(101u)==E_OK);poll_at(2000u);
    /* state+heartbeat+event+one sonar consume the entire four-attempt budget. */
    assert(strcmp(publications[publication_count-4u].topic,"robot/state")==0);
    assert(strcmp(publications[publication_count-3u].topic,"robot/heartbeat")==0);
    assert(strcmp(publications[publication_count-2u].topic,"robot/event")==0);
    assert(strcmp(publications[publication_count-1u].topic,"robot/sonar")==0);
    assert(count_topic("robot/log")==0u);
}
static void test_retained(void) {
    start_connected();events[event_head++]=(rm_event_t){.kind=2u,.sequence=55u};
    sonar[sonar_head++]=(rm_sonar_t){.status=1u,.history_sequence=42u};sonar_missed=3u;
    fail_topic="robot/event";failures_left=1u;poll_at(1u);
    publication_t *event=last_topic("robot/event");assert(event && event->result==ERR_MEM && u32(event->data+4)==55u);
    const unsigned int reads_after_failure=event_reads;
    poll_at(2u);assert(event_reads==reads_after_failure && last_topic("robot/event")->result==ERR_OK);
    assert(u32(last_topic("robot/event")->data+4)==55u);
    sonar[sonar_head++]=(rm_sonar_t){.status=1u,.history_sequence=43u};
    fail_topic="robot/sonar";failures_left=1u;poll_at(3u);
    const unsigned int sonar_reads_after_failure=sonar_reads;
    assert(last_topic("robot/sonar")->result==ERR_MEM);
    poll_at(4u);assert(last_topic("robot/sonar")->result==ERR_OK && u32(last_topic("robot/sonar")->data+4)==43u);
    assert(sonar_reads<=sonar_reads_after_failure+1u); /* Optional next history read, not a duplicate of pending. */
    sonar_busy=true;poll_at(5u);
    rm_heartbeat_t value={0};robot_network_metrics(&value);
    assert(value.publish_failed==2u && value.sonar_history_missed==3u && value.latest_sonar_history_sequence==43u);
    assert(value.sonar_busy==1u && value.mqtt_hook_max_us>0u);
}
static void test_log(bool abort_generation,bool begin_snapshot) {
    start_connected();assert(robot_network_dump(500u)==E_OK && robot_network_dump(501u)==E_OBJ);
    if (begin_snapshot) { fail_topic="robot/log";failures_left=1u; }
    poll_at(50u);publication_t *frame=last_topic("robot/log");assert(frame && frame->data[1]==RM_LOG_BEGIN);
    assert(u32(frame->data+4)==500u && u32(frame->data+8)==7u && u32(frame->data+16)==2u);
    ++ram_index.rows; /* Logging appends while dump is active; snapshot stays at two. */
    if (begin_snapshot) {
        assert(frame->result==ERR_MEM);poll_at(100u);frame=last_topic("robot/log");
        assert(frame->data[1]==RM_LOG_BEGIN && u32(frame->data+16)==2u);
        return;
    }
    const unsigned int published_before=publication_count;
    poll_at(99u);assert(publication_count==published_before && row_reads==0u);
    acknowledge(ERR_OK);poll_at(100u);frame=last_topic("robot/log");
    assert(frame->data[1]==RM_LOG_ROW && u32(frame->data+12)==0u && frame->data[16]==1u);
    if (abort_generation) {
        acknowledge(ERR_OK);++ram_index.generation;ram_index.rows=1u;
        poll_at(150u);frame=last_topic("robot/log");
        assert(frame->data[1]==RM_LOG_ABORT && u32(frame->data+8)==7u && u32(frame->data+16)==1u && u32(frame->data+20)==1u);
        acknowledge(ERR_OK);assert(robot_network_dump(502u)==E_OK);return;
    }
    acknowledge(ERR_ABRT);poll_at(150u);frame=last_topic("robot/log");
    assert(frame->data[1]==RM_LOG_ROW && u32(frame->data+12)==0u); /* Error did not advance. */
    acknowledge(ERR_OK);poll_at(200u);frame=last_topic("robot/log");
    assert(frame->data[1]==RM_LOG_ROW && u32(frame->data+12)==1u);
    acknowledge(ERR_OK);poll_at(250u);frame=last_topic("robot/log");
    assert(frame->data[1]==RM_LOG_EVENT && frame->data[16]==1u && frame->data[31]==1u && u32(frame->data+24)==444u);
    acknowledge(ERR_OK);poll_at(300u);frame=last_topic("robot/log");
    assert(frame->data[1]==RM_LOG_END && u32(frame->data+16)==2u && u32(frame->data+20)==1u && u32(frame->data+24)==9u);
    assert(row_reads==3u && event_log_reads==1u); /* Appended third row excluded. */
    acknowledge(ERR_OK);assert(robot_network_dump(502u)==E_OK);
}
static void test_disconnect_log(bool change_generation) {
    start_connected();assert(robot_network_dump(900u)==E_OK);poll_at(50u);acknowledge(ERR_OK);
    poll_at(100u);publication_t *frame=last_topic("robot/log");
    assert(frame && frame->data[1]==RM_LOG_ROW && u32(frame->data+12)==0u);
    const unsigned int before=count_topic("robot/log");
    /* Actual SDK mqtt_close frees pending requests WITHOUT their callbacks. */
    unacknowledged_log=NULL;connected(MQTT_CONNECT_DISCONNECTED);
    rm_heartbeat_t value={0};robot_network_metrics(&value);
    assert(value.publish_failed==2u); /* Connection loss plus abandoned frame. */
    if (change_generation) { ++ram_index.generation;ram_index.rows=1u; }
    poll_at(3099u);assert(connect_count==1u);
    poll_at(3100u);assert(connect_count==2u);connected(MQTT_CONNECT_ACCEPTED);poll_at(3100u);
    assert(count_topic("robot/log")==before+1u);frame=last_topic("robot/log");
    assert(u32(frame->data+8)==7u);
    if (change_generation) assert(frame->data[1]==RM_LOG_ABORT && u32(frame->data+16)==1u);
    else assert(frame->data[1]==RM_LOG_ROW && u32(frame->data+12)==0u);
}
static void test_log_retry(const char *scenario) {
    start_connected();assert(robot_network_dump(700u)==E_OK);
    if (!strcmp(scenario,"begin_generation")) { fail_topic="robot/log";failures_left=1u; }
    if (!strcmp(scenario,"open_busy")) ram_busy=true;
    poll_at(50u);
    if (!strcmp(scenario,"open_busy")) {
        assert(!last_topic("robot/log"));ram_busy=false;poll_at(100u);
        assert(last_topic("robot/log")->data[1]==RM_LOG_BEGIN);return;
    }
    publication_t *frame=last_topic("robot/log");assert(frame && frame->data[1]==RM_LOG_BEGIN);
    ++ram_index.rows;
    if (!strcmp(scenario,"begin_async")) {
        acknowledge(ERR_ABRT);poll_at(100u);frame=last_topic("robot/log");
        assert(frame->data[1]==RM_LOG_BEGIN && u32(frame->data+16)==2u);
    } else if (!strcmp(scenario,"begin_generation")) {
        ++ram_index.generation;ram_index.rows=1u;poll_at(100u);frame=last_topic("robot/log");
        assert(frame->data[1]==RM_LOG_ABORT && u32(frame->data+8)==7u && u32(frame->data+16)==1u);
    } else {
        assert(!strcmp(scenario,"row_mem"));acknowledge(ERR_OK);
        fail_topic="robot/log";failures_left=1u;poll_at(100u);frame=last_topic("robot/log");
        assert(frame->data[1]==RM_LOG_ROW && frame->result==ERR_MEM && u32(frame->data+12)==0u);
        poll_at(150u);frame=last_topic("robot/log");assert(frame->data[1]==RM_LOG_ROW && u32(frame->data+12)==0u);
        acknowledge(ERR_OK);poll_at(200u);assert(u32(last_topic("robot/log")->data+12)==1u);
    }
}
int main(int argc,char **argv) {
    assert(argc==1 || argc==2);
    if (!(argc==2 && !strcmp(argv[1],"disabled"))) assert(strcmp(ROBOT_MQTT_COMMAND_TOKEN_HEX,FIXTURE_TOKEN_HEX)==0);
    if (argc==1) test_log(false,false);
    else if (!strcmp(argv[1],"connection")) test_connection();
    else if (!strcmp(argv[1],"commands")) test_commands();
    else if (!strcmp(argv[1],"schedule")) test_schedule();
    else if (!strcmp(argv[1],"retained")) test_retained();
    else if (!strcmp(argv[1],"log")) test_log(false,false);
    else if (!strcmp(argv[1],"abort")) test_log(true,false);
    else if (!strcmp(argv[1],"begin_snapshot")) test_log(false,true);
    else if (!strcmp(argv[1],"disabled")) test_disabled_commands();
    else if (!strcmp(argv[1],"disconnect")) test_disconnect_log(false);
    else if (!strcmp(argv[1],"disconnect_abort")) test_disconnect_log(true);
    else if (!strcmp(argv[1],"begin_async") || !strcmp(argv[1],"begin_generation") ||
             !strcmp(argv[1],"row_mem") || !strcmp(argv[1],"open_busy")) test_log_retry(argv[1]);
    else assert(0);
    assert(hook_calls>0u && irq_off==0u && di_count==ei_count);return 0;
}

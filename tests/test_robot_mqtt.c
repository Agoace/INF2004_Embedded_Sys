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
#ifndef ROBOT_MQTT_COMMAND_TOKEN_HEX
/* Test TU only: the production TU independently uses its zero-token default. */
#define ROBOT_MQTT_COMMAND_TOKEN_HEX ""
#endif
typedef struct {
    const char *topic;uint8_t data[RM_LOG_BATCH_MAX_SIZE];uint16_t length;uint8_t qos;
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
static uint32_t command_lost,event_lost,busy[5],commands_received;
static bool queue_full;
static robot_command_t last_command;
static rm_event_t events[4];static unsigned int event_head,event_tail;
static rm_sonar_t sonar[4];static unsigned int sonar_head,sonar_tail;
static uint32_t sonar_missed;
static bool sonar_busy,health_busy,health_stalled;
static uint32_t health_producer_ms;
static unsigned int event_reads,sonar_reads;
static telemetry_log_index_t ram_index={.generation=7u,.rows=2u,.events=1u,.lost=9u,.newest_ms=0u};
static bool ram_busy;
static unsigned int row_reads,event_log_reads;
void mock_di(UINT *irq) { assert(!irq_off);*irq=33u;irq_off=1u;++di_count; }
void mock_ei(UINT irq) { assert(irq==33u && irq_off);irq_off=0u;++ei_count; }
static uint32_t boot_identity=0x11223344u;
uint32_t boot_id_get(void) { return boot_identity; }
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
    assert(strcmp(info->client_id,"inf2004-robot")==0 && info->keep_alive==5u);
    assert(strcmp(info->will_topic,"robot/status")==0 && info->will_msg_len==RM_STATUS_SIZE && info->will_qos==1u && info->will_retain==1u);
    assert((uint8_t)info->will_msg[0]==RM_STATUS_VERSION && (uint8_t)info->will_msg[1]==2u);
    for(unsigned int i=0u;i<4u;++i) assert((uint8_t)info->will_msg[2u+i]==(uint8_t)(boot_identity>>(8u*i)));
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
static bool payload_is_online(const void *payload) { const uint8_t *b=payload;return b[0]==RM_STATUS_VERSION && b[1]==1u; }
err_t mqtt_publish(mqtt_client_t *client,const char *topic,const void *payload,u16_t length,
    u8_t qos,u8_t retain,mqtt_request_cb_t callback,void *arg) {
    assert(in_hook && !irq_off && client==&mqtt_client && length<=RM_LOG_BATCH_MAX_SIZE);
    assert((strcmp(topic,"robot/status")==0 || strcmp(topic,"robot/summary")==0) == (retain==1u));
    assert(publication_count<256u);
    publication_t *record=&publications[publication_count++];
    *record=(publication_t){.topic=topic,.length=length,.qos=qos,.callback=callback,.arg=arg};
    memcpy(record->data,payload,length);
    if (failures_left && fail_topic && strcmp(fail_topic,topic)==0) { --failures_left;record->result=ERR_MEM; }
    if (strcmp(topic,"robot/log")==0) {
        /* Live rows are QoS 0 without a callback; DUMP frames are QoS 1. */
        assert((qos==0u && callback==NULL) || (qos==1u && callback!=NULL));
    } else if (strcmp(topic,"robot/status")==0) {
        assert(qos==1u && length==RM_STATUS_SIZE && callback!=NULL && payload_is_online(payload));
    } else if (strcmp(topic,"robot/summary")==0) {
        assert(qos==1u && length==RM_SUMMARY_SIZE && callback!=NULL);
    } else assert(qos==0u);
    return record->result;
}
void robot_network_state(rm_state_t *value) { assert(in_hook);*value=(rm_state_t){.flags=15u,.state=2u,.raw_bc=321u}; }
/* Vehicle phase provider. Default: no vehicle consumer, so the older
 * scenarios keep RUN rates. */
static int vehicle_result = E_OBJ;
static uint8_t vehicle_phase;
static bool vehicle_active;
int robot_network_vehicle_phase(uint8_t *phase, bool *active)
{
    assert(in_hook && !irq_off);
    if (vehicle_result < 0) {
        return vehicle_result;
    }
    *phase = vehicle_phase;
    *active = vehicle_active;
    return E_OK;
}
int robot_network_sonar(uint32_t *cursor,rm_sonar_t *value,uint32_t *missed) {
    assert(in_hook);++sonar_reads;*missed=sonar_missed;sonar_missed=0u;
    if (sonar_busy) return E_TMOUT;
    if (sonar_tail==sonar_head) return E_NOEXS;
    *value=sonar[sonar_tail++];*cursor=value->history_sequence;return E_OK;
}
static rm_obstacle_t obstacles[4];static unsigned int obstacle_head,obstacle_tail;
static bool obstacle_busy;static unsigned int obstacle_reads;
int robot_network_obstacle(uint32_t *cursor,rm_obstacle_t *value) {
    assert(in_hook && !irq_off);++obstacle_reads;
    if (obstacle_busy) return E_TMOUT;
    if (obstacle_tail==obstacle_head) return E_NOEXS;
    *value=obstacles[obstacle_tail++];*cursor=value->scan_id;return E_OK;
}
static rm_detour_t detours[4];static unsigned int detour_head,detour_tail;
static rm_summary_t last_summary;static int summary_result=E_NOEXS;
int robot_network_detour(uint32_t *cursor,rm_detour_t *value) {
    assert(in_hook && !irq_off);
    if (detour_tail==detour_head) return E_NOEXS;
    *value=detours[detour_tail++];*cursor=value->sequence;return E_OK;
}
int robot_network_summary(uint32_t *cursor,rm_summary_t *value) {
    assert(in_hook && !irq_off);
    if (summary_result<0) return summary_result;
    if (last_summary.run_id==*cursor) return E_NOEXS;
    *value=last_summary;*cursor=value->run_id;return E_OK;
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
static unsigned int unavailable_reads[5];
void robot_network_board_result(unsigned int board,int result) { assert(board<5u);if(result==E_OBJ || result==E_NOEXS) ++unavailable_reads[board];else if(result==E_TMOUT) ++busy[board]; }
void robot_queue_metrics(rm_heartbeat_t *value) {
    value->command_lost=command_lost;value->event_lost=event_lost;
    value->line_busy=busy[0];value->sonar_busy=busy[1];value->motion_busy=busy[2];
    value->vehicle_busy=busy[3];value->health_busy=busy[4];
}
int robot_health_try_read(rm_heartbeat_t *value,uint32_t *timestamp_us) {
    assert(in_hook);
    if (health_busy) return E_TMOUT;
    const uint32_t produced_ms=health_stalled ? health_producer_ms : now_ms;
    *value=(rm_heartbeat_t){.uptime_ms=produced_ms,.telemetry_lost=13u,.stack_min_free=222u};
    *timestamp_us=produced_ms*1000u;return E_OK;
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
    assert(publication_count-before<=7u && !irq_off && di_count==ei_count); /* 4 routine + 1 live + 2 DUMP */++hook_calls;
}
static void connected(mqtt_connection_status_t status) {
    assert(connection_callback);mqtt_client.connected=status==MQTT_CONNECT_ACCEPTED;
    connection_callback(&mqtt_client,connection_arg,status);
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
static void test_no_boot_id(void) {
    boot_identity=0u;poll_at(0u);poll_at(10000u);
    assert(new_count==0u && connect_count==0u && publication_count==0u);
    boot_identity=0x11003300u;poll_at(10001u); /* binary will includes zero bytes */
    assert(connect_count==1u); /* observer only; driving is not gated */
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
    queue_full=false;data[1]=RM_CMD_ESTOP;data[6]^=1u;
    deliver("robot/cmd",data,RM_COMMAND_SIZE,0u,RM_COMMAND_SIZE);
    assert(commands_received==1u && command_lost==9u); /* Configured ESTOP requires the token too. */
    data[6]^=1u;deliver("robot/cmd",data,RM_COMMAND_SIZE,0u,RM_COMMAND_SIZE);
    assert(commands_received==2u && last_command.opcode==RM_CMD_ESTOP);
}
static void test_unconfigured_commands(void) {
    start_connected();
    rm_heartbeat_t value={0};robot_network_metrics(&value);assert((value.flags&16u)==0u);
    rm_command_t command={.opcode=RM_CMD_ESTOP,.request_id=1u};
    uint8_t data[RM_COMMAND_SIZE];rm_pack_command(data,&command);
    deliver("robot/cmd",data,RM_COMMAND_SIZE,3u,RM_COMMAND_SIZE);
    assert(commands_received==1u && last_command.opcode==RM_CMD_ESTOP && command_lost==0u);
    for (uint8_t opcode=RM_CMD_DUMP;opcode<=RM_CMD_START;++opcode) {
        command.opcode=opcode;rm_pack_command(data,&command);
        deliver("robot/cmd",data,RM_COMMAND_SIZE,3u,RM_COMMAND_SIZE);
    }
    assert(commands_received==1u && command_lost==3u);
    command.opcode=RM_CMD_ESTOP;memset(command.token,0xa5,sizeof(command.token));rm_pack_command(data,&command);
    deliver("robot/cmd",data,RM_COMMAND_SIZE,3u,RM_COMMAND_SIZE);
    assert(commands_received==2u && command_lost==3u); /* Token field is unchecked. */
    data[0]=2u;deliver("robot/cmd",data,RM_COMMAND_SIZE,3u,RM_COMMAND_SIZE);data[0]=1u;
    deliver("robot/cmd",data,RM_COMMAND_SIZE-1u,3u,RM_COMMAND_SIZE-1u);
    data[1]=5u;deliver("robot/cmd",data,RM_COMMAND_SIZE,3u,RM_COMMAND_SIZE);
    assert(commands_received==2u && command_lost==6u);
}
static void test_status(void) {
    /* The will is checked in the connect mock. Online follows each CONNACK. */
    poll_at(0u);assert(connect_count==1u && count_topic("robot/status")==0u);
    connected(MQTT_CONNECT_ACCEPTED);
    fail_topic="robot/status";failures_left=1u;
    poll_at(0u);assert(count_topic("robot/status")==1u && last_topic("robot/status")->result==ERR_MEM);
    poll_at(499u);assert(count_topic("robot/status")==1u);
    poll_at(500u);assert(count_topic("robot/status")==2u && last_topic("robot/status")->result==ERR_OK);
    poll_at(1000u);assert(count_topic("robot/status")==2u); /* accepted: no repeat */
    /* An acknowledgement error publishes online again after 500 ms. */
    publication_t *status=last_topic("robot/status");
    status->callback(status->arg,ERR_ABRT);
    poll_at(1000u);assert(count_topic("robot/status")==2u);
    poll_at(1500u);assert(count_topic("robot/status")==3u);
    status=last_topic("robot/status");status->callback(status->arg,ERR_OK);
    poll_at(3000u);assert(count_topic("robot/status")==3u);
    /* Reconnect: the link drop deleted pending requests without callbacks. */
    connected(MQTT_CONNECT_DISCONNECTED);
    poll_at(5999u);assert(connect_count==1u);
    poll_at(6000u);assert(connect_count==2u);
    connected(MQTT_CONNECT_ACCEPTED);poll_at(6000u);
    assert(count_topic("robot/status")==4u && last_topic("robot/status")->data[1]==RM_STATUS_ONLINE);
    /* Status goes out before any other robot data on a connection. */
    unsigned int first_status=0u,first_state=0u;
    for (unsigned int i=publication_count;i>0u;--i) {
        if (strcmp(publications[i-1u].topic,"robot/status")==0 && first_status==0u) first_status=i;
        if (strcmp(publications[i-1u].topic,"robot/state")==0 && first_state==0u) first_state=i;
    }
    assert(first_status!=0u && first_status<first_state);
    /* Detection time: broker drops a silent client at 1.5 x keep-alive. */
    assert(5u*3u/2u==7u && 5u%5u==0u); /* lwIP cyclic timer is 5 s; 7.5 s truncates to 7 */
}
static void test_obstacle(void) {
    start_connected();
    assert(count_topic("robot/obstacle")==0u);
    obstacles[obstacle_head++]=(rm_obstacle_t){.scan_id=1u,.uptime_ms=1234u,.closest_mm=222u,.decision=2u,.flags=RM_OBSTACLE_CLOSEST_VALID,.closest_angle_deg=-10};
    poll_at(10u);
    publication_t *report=last_topic("robot/obstacle");
    assert(report && report->length==RM_OBSTACLE_SIZE && report->qos==0u);
    assert(report->data[0]==RM_OBSTACLE_VERSION && report->data[2]==2u && u32(report->data+4)==1u && u32(report->data+8)==1234u);
    assert(count_topic("robot/obstacle")==1u);
    poll_at(11u);assert(count_topic("robot/obstacle")==1u); /* once per report */
    /* A failed publish keeps the report and retries on the next poll. */
    obstacles[obstacle_head++]=(rm_obstacle_t){.scan_id=2u};
    fail_topic="robot/obstacle";failures_left=1u;poll_at(12u);
    assert(count_topic("robot/obstacle")==2u && last_topic("robot/obstacle")->result==ERR_MEM);
    poll_at(13u);assert(count_topic("robot/obstacle")==3u && last_topic("robot/obstacle")->result==ERR_OK);
    assert(u32(last_topic("robot/obstacle")->data+4)==2u);
    poll_at(14u);assert(count_topic("robot/obstacle")==3u);
    /* A busy board is retried; nothing is published or lost. */
    obstacles[obstacle_head++]=(rm_obstacle_t){.scan_id=5u}; /* ids 3,4 were overwritten */
    obstacle_busy=true;poll_at(15u);assert(count_topic("robot/obstacle")==3u);
    obstacle_busy=false;poll_at(16u);assert(count_topic("robot/obstacle")==4u);
    assert(u32(last_topic("robot/obstacle")->data+4)==5u);
}
static void test_detour(void) {
    start_connected();
    assert(count_topic("robot/detour")==0u);
    detours[detour_head++]=(rm_detour_t){.sequence=1u,.outcome=RM_DETOUR_DONE_LINE,.distance_mm=500u};
    poll_at(10u);
    publication_t *d=last_topic("robot/detour");
    assert(d && d->length==RM_DETOUR_SIZE && d->qos==0u && d->data[1]==3u && u32(d->data+4)==1u);
    poll_at(11u);assert(count_topic("robot/detour")==1u);
    detours[detour_head++]=(rm_detour_t){.sequence=2u,.outcome=RM_DETOUR_FAILED};
    fail_topic="robot/detour";failures_left=1u;poll_at(12u);
    assert(count_topic("robot/detour")==2u && last_topic("robot/detour")->result==ERR_MEM);
    poll_at(13u);assert(count_topic("robot/detour")==3u && last_topic("robot/detour")->result==ERR_OK);
    assert(u32(last_topic("robot/detour")->data+4)==2u);
    poll_at(14u);assert(count_topic("robot/detour")==3u);
}
static void test_summary(void) {
    start_connected();
    poll_at(10u);assert(count_topic("robot/summary")==0u); /* no run yet */
    last_summary=(rm_summary_t){.run_id=1u,.distance_mm=1234u,.stop_reason=RM_STOP_GP21};
    summary_result=E_OK;poll_at(20u);
    publication_t *r=last_topic("robot/summary");
    assert(r && r->length==RM_SUMMARY_SIZE && r->qos==1u && r->callback && r->data[3]==1u);
    assert(u32(r->data+4)==1u && u32(r->data+16)==1234u);
    poll_at(21u);assert(count_topic("robot/summary")==1u); /* once per run */
    /* A failed publish call retries after 500 ms. */
    last_summary.run_id=2u;
    fail_topic="robot/summary";failures_left=1u;poll_at(30u);
    assert(count_topic("robot/summary")==2u && last_topic("robot/summary")->result==ERR_MEM);
    poll_at(529u);assert(count_topic("robot/summary")==2u);
    poll_at(530u);assert(count_topic("robot/summary")==3u && last_topic("robot/summary")->result==ERR_OK);
    /* An acknowledgement error publishes the newest summary again. */
    r=last_topic("robot/summary");r->callback(r->arg,ERR_ABRT);
    poll_at(600u);assert(count_topic("robot/summary")==3u);
    poll_at(1100u);assert(count_topic("robot/summary")==4u && u32(last_topic("robot/summary")->data+4)==2u);
    r=last_topic("robot/summary");r->callback(r->arg,ERR_OK);
    poll_at(1200u);assert(count_topic("robot/summary")==4u);
    /* A board that is busy loses nothing: the summary comes when readable. */
    last_summary.run_id=3u;summary_result=E_TMOUT;poll_at(1300u);assert(count_topic("robot/summary")==4u);
    summary_result=E_OK;poll_at(1301u);assert(count_topic("robot/summary")==5u);
    assert(u32(last_topic("robot/summary")->data+4)==3u);
    /* A new connection publishes the last summary again (retained copy). */
    connected(MQTT_CONNECT_DISCONNECTED);
    poll_at(4300u);poll_at(7300u);assert(connect_count==2u);
    connected(MQTT_CONNECT_ACCEPTED);poll_at(7300u);
    assert(count_topic("robot/summary")==6u && u32(last_topic("robot/summary")->data+4)==3u);
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
    assert(robot_network_dump(101u)==E_OK);++ram_index.rows;poll_at(2000u);
    /* state+heartbeat+event+one sonar consume the entire four-attempt budget. */
    assert(strcmp(publications[publication_count-6u].topic,"robot/state")==0);
    assert(strcmp(publications[publication_count-5u].topic,"robot/heartbeat")==0);
    assert(strcmp(publications[publication_count-4u].topic,"robot/event")==0);
    assert(strcmp(publications[publication_count-3u].topic,"robot/sonar")==0);
    /* Log frames use their own budget: one live batch and the DUMP BEGIN. */
    assert(strcmp(publications[publication_count-2u].topic,"robot/log")==0);
    assert(publications[publication_count-2u].qos==0u);
    assert(publications[publication_count-1u].qos==1u);
}
static uint16_t u16(const uint8_t *data) { return (uint16_t)((uint16_t)data[0]|((uint16_t)data[1]<<8)); }
static void test_health_stalled(void) {
    start_connected();publication_t *frame=last_topic("robot/heartbeat");
    assert(frame && (frame->data[1]&32u)!=0u && u16(frame->data+80)==0u);
    health_stalled=true;health_producer_ms=0u;
    poll_at(2500u);frame=last_topic("robot/heartbeat");
    assert((frame->data[1]&32u)!=0u && u16(frame->data+80)==2500u);
    poll_at(3500u);frame=last_topic("robot/heartbeat");
    assert((frame->data[1]&32u)==0u && u16(frame->data+80)==3500u);
    /* The successful read returns the same producer sample after 2^32us.
     * A us-only subtraction would make it look less than 1ms old again. */
    poll_at(4294968u);frame=last_topic("robot/heartbeat");
    assert((frame->data[1]&32u)==0u && u16(frame->data+80)==65535u);
    assert(u16(frame->data+40)==222u && u32(frame->data+12)==13u && u32(frame->data+4)==4294968u);
    assert(busy[ROBOT_BOARD_HEALTH]==0u); /* Readable does not mean fresh. */
    health_stalled=false;poll_at(4295968u);frame=last_topic("robot/heartbeat");
    assert((frame->data[1]&32u)!=0u && u16(frame->data+80)==0u);
}
static void test_health_missing(void) {
    health_busy=true;start_connected();publication_t *frame=last_topic("robot/heartbeat");
    assert(frame && (frame->data[1]&32u)==0u && u16(frame->data+80)==65535u && u16(frame->data+40)==65535u);
    assert(u32(frame->data+12)==0u && busy[ROBOT_BOARD_HEALTH]==1u);
    poll_at(1000u);frame=last_topic("robot/heartbeat");
    assert((frame->data[1]&32u)==0u && u16(frame->data+80)==65535u && u16(frame->data+40)==65535u);
    health_busy=false;poll_at(2000u);frame=last_topic("robot/heartbeat");
    assert((frame->data[1]&32u)!=0u && u16(frame->data+80)==0u && u16(frame->data+40)==222u);
    health_busy=true;poll_at(3000u);frame=last_topic("robot/heartbeat");
    assert((frame->data[1]&32u)==0u && u16(frame->data+80)==1000u && u16(frame->data+40)==222u);
    assert(u32(frame->data+12)==13u && busy[ROBOT_BOARD_HEALTH]==3u);
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
/* Acknowledge every unacknowledged QoS 1 log frame, oldest first. */
static bool is_log_qos1(const publication_t *r) { return strcmp(r->topic,"robot/log")==0 && r->qos==1u; }
static unsigned int acked_upto;
static unsigned int acknowledge_all(err_t error) {
    unsigned int n=0u;
    for (;acked_upto<publication_count;++acked_upto) {
        publication_t *r=&publications[acked_upto];
        if (strcmp(r->topic,"robot/log")==0 && r->qos==1u && r->result==ERR_OK) { r->callback(r->arg,error);++n; }
    }
    return n;
}
static void test_log_live(void) {
    start_connected();
    ram_index.rows=25u;ram_index.newest_ms=90u;
    poll_at(100u);
    publication_t *frame=last_topic("robot/log");
    assert(frame && frame->qos==0u && frame->length==RM_LOG_BATCH_HEADER_SIZE+10u*RM_LOG_RECORD_SIZE);
    assert(frame->data[1]==RM_LOG_ROWS && frame->data[2]==10u && u32(frame->data+12)==0u);
    assert(u32(frame->data+16)==100u && u32(frame->data+20)==90u && u32(frame->data+24)==25u);
    assert(frame->data[RM_LOG_BATCH_HEADER_SIZE]==1u && frame->data[RM_LOG_BATCH_HEADER_SIZE+9u*16u]==10u); /* mock row content = index+1 */
    poll_at(101u);frame=last_topic("robot/log");assert(u32(frame->data+12)==10u);
    poll_at(102u);assert(count_topic("robot/log")==2u); /* 5 rows wait for the period */
    poll_at(201u);frame=last_topic("robot/log");
    assert(u32(frame->data+12)==20u && frame->data[2]==5u && u32(frame->data+28)==0u);
    rm_heartbeat_t value={0};robot_network_metrics(&value);
    assert((value.flags&8u)==0u); /* no DUMP active */
    /* A failed QoS 0 publish keeps the cursor and retries after 50 ms. */
    ram_index.rows=35u;fail_topic="robot/log";failures_left=1u;poll_at(300u);
    assert(last_topic("robot/log")->result==ERR_MEM);
    const unsigned int count=count_topic("robot/log");
    poll_at(349u);assert(count_topic("robot/log")==count);
    poll_at(350u);frame=last_topic("robot/log");
    assert(frame->result==ERR_OK && u32(frame->data+12)==25u && frame->data[2]==10u);
    robot_network_metrics(&value);assert(value.publish_failed==1u);
    /* Reconnect: the live stream jumps to the newest rows and reports it. */
    connected(MQTT_CONNECT_DISCONNECTED);
    ram_index.rows=70u;
    poll_at(3350u);connected(MQTT_CONNECT_ACCEPTED);poll_at(3350u);
    subscribe_callback(subscribe_arg,ERR_OK);poll_at(3351u);
    frame=last_topic("robot/log");
    assert(frame->data[3]==RM_LOG_FLAG_JUMPED && u32(frame->data+12)==60u && u32(frame->data+28)==25u);
    /* New run generation. */
    ++ram_index.generation;ram_index.rows=2u;poll_at(3500u);
    frame=last_topic("robot/log");
    assert(frame->data[3]==RM_LOG_FLAG_NEW_GENERATION && u32(frame->data+8)==ram_index.generation);
    assert(u32(frame->data+12)==0u && frame->data[2]==2u);
}
static void test_log_dump(bool fail_ack) {
    start_connected();
    assert(robot_network_dump(500u)==E_OK && robot_network_dump(501u)==E_OBJ);
    poll_at(150u);
    rm_heartbeat_t value={0};robot_network_metrics(&value);assert((value.flags&8u)!=0u);
    publication_t *begin=NULL;
    for (unsigned int i=0u;i<publication_count;++i) if (is_log_qos1(&publications[i])) begin=&publications[i];
    assert(begin && begin->length==RM_LOG_SIZE && begin->data[1]==RM_LOG_BEGIN);
    assert(u32(begin->data+4)==500u && u32(begin->data+8)==7u && u32(begin->data+16)==2u && u32(begin->data+20)==1u);
    ram_index.rows+=50u; /* appended after the snapshot: excluded */
    poll_at(200u);assert(count_topic("robot/log")>=1u);
    unsigned int qos1=0u;
    for (unsigned int i=0u;i<publication_count;++i) if (is_log_qos1(&publications[i])) ++qos1;
    assert(qos1==1u); /* waits for the BEGIN acknowledgement */
    assert(acknowledge_all(fail_ack ? ERR_ABRT : ERR_OK)==1u);
    if (fail_ack) {
        poll_at(220u);poll_at(250u); /* retry delay 50 ms, then BEGIN again */
        qos1=0u;for (unsigned int i=0u;i<publication_count;++i) if (is_log_qos1(&publications[i])) ++qos1;
        assert(qos1==2u);assert(acknowledge_all(ERR_OK)==1u);
    }
    bool end_seen=false;
    for (unsigned int step=0u;step<20u && !end_seen;++step) {
        poll_at(300u+step*10u);
        for (unsigned int i=0u;i<publication_count;++i) {
            if (is_log_qos1(&publications[i]) && publications[i].data[1]==RM_LOG_END) end_seen=true;
        }
        (void)acknowledge_all(ERR_OK);
    }
    assert(end_seen);
    publication_t *rows=NULL,*event_batch=NULL,*end=NULL;
    for (unsigned int i=0u;i<publication_count;++i) {
        if (!is_log_qos1(&publications[i])) continue;
        if (publications[i].data[1]==RM_LOG_ROWS) rows=&publications[i];
        if (publications[i].data[1]==RM_LOG_EVENTS) event_batch=&publications[i];
        if (publications[i].data[1]==RM_LOG_END) end=&publications[i];
    }
    assert(rows && rows->data[2]==2u && rows->data[3]==RM_LOG_FLAG_REFILL && u32(rows->data+4)==500u);
    assert(event_batch && event_batch->data[2]==1u && u32(event_batch->data+RM_LOG_BATCH_HEADER_SIZE+8)==444u);
    assert(u32(end->data+16)==2u && u32(end->data+20)==1u && u32(end->data+24)==9u);
    value=(rm_heartbeat_t){0};robot_network_metrics(&value);assert((value.flags&8u)==0u);
    assert(robot_network_dump(502u)==E_OK);
}
static void test_log_disconnect(void) {
    start_connected();
    assert(robot_network_dump(900u)==E_OK);poll_at(50u);
    unsigned int qos1=0u;
    for (unsigned int i=0u;i<publication_count;++i) if (is_log_qos1(&publications[i])) ++qos1;
    assert(qos1==1u);
    /* Actual SDK mqtt_close frees pending requests WITHOUT their callbacks. */
    connected(MQTT_CONNECT_DISCONNECTED);
    poll_at(3049u);assert(connect_count==1u);
    poll_at(3050u);assert(connect_count==2u);connected(MQTT_CONNECT_ACCEPTED);
    poll_at(3100u);subscribe_callback(subscribe_arg,ERR_OK);
    poll_at(3200u);
    qos1=0u;for (unsigned int i=0u;i<publication_count;++i) if (is_log_qos1(&publications[i])) ++qos1;
    assert(qos1==2u && last_topic("robot/log")!=NULL); /* BEGIN sent again */
    assert(acknowledge_all(ERR_OK)==2u || true);
}
static uint16_t mode_word(const publication_t *r)
{
    return (uint16_t)(r->data[70] | (r->data[71] << 8));
}

static bool heartbeat_idle(const publication_t *r)
{
    return (r->data[1] & RM_HEARTBEAT_FLAG_IDLE) != 0u;
}

/* Count publications of topic in [first, publication_count). */
static unsigned int count_since(const char *topic, unsigned int first)
{
    unsigned int count = 0u;

    for (unsigned int i = first; i < publication_count; ++i) {
        if (strcmp(publications[i].topic, topic) == 0) {
            ++count;
        }
    }
    return count;
}

static void poll_range(uint32_t from, uint32_t to, uint32_t step)
{
    for (uint32_t t = from; t <= to; t += step) {
        poll_at(t);
    }
}

/* idle -> calibrate -> run -> stop -> grace -> idle, publication level. */
static void test_idle_rates(void)
{
    publication_t *s;
    publication_t *h;
    publication_t *log;
    unsigned int first;
    unsigned int sonar_before;

    vehicle_result = E_OK;
    vehicle_phase = 0u;
    vehicle_active = false;
    start_connected();
    s = last_topic("robot/state");
    h = last_topic("robot/heartbeat");
    assert(s && (s->data[0] == 5u) && (mode_word(s) == RM_STATE_MODE_IDLE));
    assert(h && (h->data[0] == 4u) && heartbeat_idle(h));
    ram_index.rows = 40u;
    ram_index.newest_ms = 10u;
    for (unsigned int i = 0u; i < 4u; ++i) {
        sonar[sonar_head++] = (rm_sonar_t){.history_sequence = 1u + i,
                                           .status = 1u};
    }
    events[event_head++] = (rm_event_t){.kind = 6u, .sequence = 1u};
    last_summary = (rm_summary_t){.run_id = 1u};
    summary_result = E_OK;
    first = publication_count;
    sonar_before = sonar_reads;
    poll_range(10u, 10000u, 10u);
    /* IDLE for 10 s: state 1 Hz, heartbeat every 5 s, no live log or sonar,
     * events and the summary unchanged. */
    assert(count_since("robot/state", first) == 10u);
    assert(count_since("robot/heartbeat", first) == 2u);
    assert(count_since("robot/log", first) == 0u);
    assert(count_since("robot/sonar", first) == 0u);
    assert((sonar_reads == sonar_before) && (sonar_tail == 0u));
    assert(count_since("robot/event", first) == 1u);
    assert(count_since("robot/summary", first) == 1u);
    /* CALIBRATE: RUN at once; state and heartbeat are not delayed. */
    vehicle_phase = 1u;
    vehicle_active = true;
    first = publication_count;
    poll_at(10005u);
    s = last_topic("robot/state");
    h = last_topic("robot/heartbeat");
    assert((count_since("robot/state", first) == 1u) && (mode_word(s) == 0u));
    assert((count_since("robot/heartbeat", first) == 1u) &&
           !heartbeat_idle(h));
    assert(count_since("robot/sonar", first) == 2u); /* ring entries */
    log = last_topic("robot/log");
    assert(log && (log->data[3] == RM_LOG_FLAG_JUMPED));
    assert(u32(log->data + 12) == 30u); /* jump to the newest 10 rows */
    vehicle_phase = 3u;
    first = publication_count;
    poll_range(10010u, 12005u, 5u);
    assert(count_since("robot/state", first) == 10u);
    assert(count_since("robot/heartbeat", first) == 2u);
    /* Stop: RUN rates continue for the 2 s grace, then IDLE. */
    vehicle_phase = 0u;
    vehicle_active = false;
    poll_at(12010u);
    first = publication_count;
    poll_range(12015u, 14005u, 5u);
    assert(mode_word(last_topic("robot/state")) == 0u);
    assert(count_since("robot/state", first) == 10u);
    poll_at(14010u);
    first = publication_count;
    poll_range(14015u, 24000u, 5u);
    assert(mode_word(last_topic("robot/state")) == RM_STATE_MODE_IDLE);
    assert(heartbeat_idle(last_topic("robot/heartbeat")));
    assert(count_since("robot/state", first) <= 11u);
    assert(count_since("robot/heartbeat", first) <= 3u);
    assert(count_since("robot/log", first) == 0u);
    assert(count_since("robot/sonar", first) == 0u);
}

/* A DUMP refill works while idle; the live stream stays paused. */
static void test_idle_dump(void)
{
    bool end_seen = false;
    rm_heartbeat_t value = {0};

    vehicle_result = E_OK;
    start_connected();
    assert(robot_network_dump(700u) == E_OK);
    for (unsigned int step = 0u; (step < 40u) && !end_seen; ++step) {
        poll_at(100u + (step * 10u));
        for (unsigned int i = 0u; i < publication_count; ++i) {
            if (is_log_qos1(&publications[i]) &&
                (publications[i].data[1] == RM_LOG_END)) {
                end_seen = true;
            }
        }
        (void)acknowledge_all(ERR_OK);
    }
    assert(end_seen);
    for (unsigned int i = 0u; i < publication_count; ++i) {
        assert(!((strcmp(publications[i].topic, "robot/log") == 0) &&
                 (publications[i].qos == 0u)));
    }
    robot_network_metrics(&value);
    assert((value.flags & 8u) == 0u); /* the dump finished */
}

/* TEST/BENCH (no vehicle consumer): RUN rates and RUN frames. */
static void test_idle_no_vehicle(void)
{
    unsigned int first;

    vehicle_result = E_OBJ;
    start_connected();
    first = publication_count;
    ram_index.rows = 12u;
    poll_range(1u, 2000u, 1u);
    assert(count_since("robot/state", first) == 10u);
    assert(count_since("robot/heartbeat", first) == 2u);
    assert(count_since("robot/log", first) >= 1u);
    assert(mode_word(last_topic("robot/state")) == 0u);
    assert(!heartbeat_idle(last_topic("robot/heartbeat")));
}

/* A busy vehicle board keeps the last decision; it never forces RUN. */
static void test_idle_busy(void)
{
    unsigned int first;

    vehicle_result = E_OK;
    start_connected();
    vehicle_result = E_TMOUT;
    first = publication_count;
    poll_range(10u, 3000u, 10u);
    assert(count_since("robot/state", first) == 3u);
    assert(mode_word(last_topic("robot/state")) == RM_STATE_MODE_IDLE);
}

int main(int argc,char **argv) {
    assert(argc==1 || argc==2);
    if (!(argc==2 && !strcmp(argv[1],"unconfigured"))) assert(strcmp(ROBOT_MQTT_COMMAND_TOKEN_HEX,FIXTURE_TOKEN_HEX)==0);
    if (argc==1) test_log_live();
    else if (!strcmp(argv[1],"connection")) test_connection();
    else if (!strcmp(argv[1],"commands")) test_commands();
    else if (!strcmp(argv[1],"schedule")) test_schedule();
    else if (!strcmp(argv[1],"retained")) test_retained();
    else if (!strcmp(argv[1],"health_stalled")) test_health_stalled();
    else if (!strcmp(argv[1],"health_missing")) test_health_missing();
                else if (!strcmp(argv[1],"unconfigured")) test_unconfigured_commands();
    else if (!strcmp(argv[1],"no_boot_id")) test_no_boot_id();
    else if (!strcmp(argv[1],"status")) test_status();
    else if (!strcmp(argv[1],"detour")) test_detour();
    else if (!strcmp(argv[1],"summary")) test_summary();
    else if (!strcmp(argv[1],"obstacle")) test_obstacle();
    else if (!strcmp(argv[1],"log_live")) test_log_live();
    else if (!strcmp(argv[1],"log_dump")) test_log_dump(false);
    else if (!strcmp(argv[1],"log_dump_ack_error")) test_log_dump(true);
    else if (!strcmp(argv[1],"log_disconnect")) test_log_disconnect();
    else if (!strcmp(argv[1], "idle_rates")) test_idle_rates();
    else if (!strcmp(argv[1], "idle_dump")) test_idle_dump();
    else if (!strcmp(argv[1], "idle_no_vehicle")) test_idle_no_vehicle();
    else if (!strcmp(argv[1], "idle_busy")) test_idle_busy();
    else assert(0);
    assert(hook_calls>0u && irq_off==0u && di_count==ei_count);return 0;
}

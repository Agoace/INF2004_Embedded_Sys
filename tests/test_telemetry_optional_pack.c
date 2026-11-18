#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define TM_MQTT 1
#include "../app_program/subsystem1/telemetry_pack.h"
#include "../app_program/subsystem1/network_pack.h"

static void test_scan(void)
{
    assert(sizeof(telemetry_scan_packed_t)==11u);
    for (uint32_t mask=0u;mask<8192u;++mask) {
        for (uint8_t hits=0u;hits<=5u;++hits) {
            for (uint8_t attempt=1u;attempt<=2u;++attempt) {
                telemetry_scan_t v={.scan_id=UINT32_MAX,.duration_ms=UINT32_MAX,
                    .fine_mask=(uint16_t)mask,.coarse_hits=hits,.attempt=attempt},o={0};
                telemetry_scan_packed_t p;
                assert(telemetry_scan_pack(&p,&v));
                assert((p.bytes[10]&0xfeu)==0u);
                assert(telemetry_scan_unpack(&o,&p));
                assert(o.scan_id==v.scan_id && o.duration_ms==v.duration_ms);
                assert(o.fine_mask==mask && o.coarse_hits==hits && o.attempt==attempt);
            }
        }
    }
}
static void test_info(void)
{
    assert(sizeof(telemetry_info_packed_t)==104u);
    telemetry_info_t v={0},o={0};
    memset(v.reason,'x',sizeof(v.reason));
    v.wake_lost=UINT32_MAX;v.overruns=UINT32_MAX;v.response_max_us=UINT32_MAX;
    v.gap_min_us=UINT32_MAX;v.gap_max_us=UINT32_MAX;v.sampled=UINT32_MAX;
    v.missed=UINT32_MAX;v.dropped=UINT32_MAX;v.isr_max_us=UINT32_MAX;
    v.line_max_us=UINT32_MAX;v.counts[0]=INT32_MIN;v.counts[1]=INT32_MAX;
    for (uint16_t n=0u;n<4096u;++n) {
        v.floor[0]=n;v.floor[1]=(uint16_t)(4095u-n);
        v.line[0]=n;v.line[1]=n;v.raw_bc=n;v.faults=(uint8_t)(n&15u);
        telemetry_info_packed_t p;
        assert(telemetry_info_pack(&p,&v));
        assert(telemetry_info_unpack(&o,&p));
        assert(memcmp(v.reason,o.reason,sizeof(v.reason))==0);
        assert(o.floor[0]==n && o.floor[1]==4095u-n && o.raw_bc==n);
        assert(o.line[0]==n && o.line[1]==n && o.faults==(n&15u));
        assert(o.counts[0]==INT32_MIN && o.counts[1]==INT32_MAX);
        assert(o.wake_lost==UINT32_MAX && o.line_max_us==UINT32_MAX);
    }
}
static void test_sonar(void)
{
    assert(sizeof(network_sonar_packed_t)==32u);
    rm_sonar_t v={.history_sequence=UINT32_MAX,.generation=UINT32_MAX,
        .request_id=UINT32_MAX,.trigger_ms=UINT32_MAX,.timestamp_ms=UINT32_MAX,
        .ping_sequence=UINT32_MAX,.boot_id=UINT32_MAX},o={0};
    for (int angle=-30;angle<=30;++angle) {
        for (int ping=-30;ping<=30;++ping) {
            for (unsigned int state=0u;state<16u;++state) {
                v.angle=(int8_t)angle;v.ping_angle=(int8_t)ping;
                v.status=(uint8_t)(state&3u);v.valid=(state&4u)!=0u;
                v.pending=(state&8u)!=0u;v.echo=0x3fffu;
                network_sonar_packed_t p;
                assert(network_sonar_pack(&p,&v));
                assert(network_sonar_unpack(&o,&p));
                assert(o.angle==angle && o.ping_angle==ping && o.status==v.status);
                assert(o.valid==v.valid && o.pending==v.pending && o.echo==v.echo);
                assert(o.boot_id==UINT32_MAX && o.request_id==UINT32_MAX);
            }
        }
    }
    for (uint16_t echo=0u;echo<16384u;++echo) {
        v.echo=echo;network_sonar_packed_t p;
        assert(network_sonar_pack(&p,&v));
        assert(network_sonar_unpack(&o,&p) && o.echo==echo);
    }
}
static void test_health(void)
{
    assert(sizeof(network_health_packed_t)==104u);
    rm_heartbeat_t v={0},o={0};
#define SET32(field) v.field=UINT32_MAX;
    NETWORK_HEALTH_U32_FIELDS(SET32)
#undef SET32
    v.wifi_start_status=INT32_MIN;v.stack_min_free=UINT16_MAX;v.health_age_ms=UINT16_MAX;
    v.flags=127u;v.reset_reason=2u;v.mqtt_state=3u;v.board_unavailable_mask=31u;
    for (unsigned int cpu=0u;cpu<=1001u;++cpu) {
        v.cpu_load_permille=cpu==1001u?UINT16_MAX:(uint16_t)cpu;
        for (int link=-3;link<=3;++link) {
            v.wifi_link_status=(int8_t)link;
            v.wifi_requested_priority=(uint8_t)(cpu%33u);
            v.wifi_observed_priority=(uint8_t)((cpu+1u)%33u);
            network_health_packed_t p;
            memset(&p,0xff,sizeof(p));
            assert(network_health_pack(&p,&v));
            assert((p.words[25]&UINT32_C(0xfffffe00))==0u);
            assert(network_health_unpack(&o,&p));
#define CHECK32(field) assert(o.field==v.field);
            NETWORK_HEALTH_U32_FIELDS(CHECK32)
#undef CHECK32
            assert(o.wifi_start_status==INT32_MIN && o.stack_min_free==UINT16_MAX);
            assert(o.cpu_load_permille==v.cpu_load_permille && o.wifi_link_status==link);
            assert(o.wifi_requested_priority==v.wifi_requested_priority);
            assert(o.wifi_observed_priority==v.wifi_observed_priority);
            assert(o.flags==127u && o.reset_reason==2u && o.mqtt_state==3u);
        }
    }
    network_health_packed_t p={0},saved=p;
    v.cpu_load_permille=1002u;
    assert(!network_health_pack(&p,&v) && memcmp(&p,&saved,sizeof(p))==0);
    v.cpu_load_permille=UINT16_MAX;v.wifi_requested_priority=33u;
    assert(!network_health_pack(&p,&v));
}
static void test_invalid_and_boundaries(void)
{
    telemetry_scan_t scan={.attempt=1u};telemetry_scan_packed_t sp={{0}},ss=sp;
    scan.attempt=0u;assert(!telemetry_scan_pack(&sp,&scan));
    scan.attempt=3u;assert(!telemetry_scan_pack(&sp,&scan));
    scan.attempt=1u;scan.coarse_hits=6u;assert(!telemetry_scan_pack(&sp,&scan));
    scan.coarse_hits=0u;scan.fine_mask=8192u;assert(!telemetry_scan_pack(&sp,&scan));
    assert(memcmp(&sp,&ss,sizeof(sp))==0);
    scan.fine_mask=0u;assert(telemetry_scan_pack(&sp,&scan));
    for (unsigned int bit=1u;bit<8u;++bit) {
        telemetry_scan_packed_t bad=sp;bad.bytes[10]|=(uint8_t)(1u<<bit);
        assert(!telemetry_scan_unpack(&scan,&bad));
    }
    telemetry_info_t info={0};telemetry_info_packed_t ip={{0}},is=ip;
    info.raw_bc=4096u;assert(!telemetry_info_pack(&ip,&info));
    info.raw_bc=0u;info.faults=16u;assert(!telemetry_info_pack(&ip,&info));
    assert(memcmp(&ip,&is,sizeof(ip))==0);
    rm_sonar_t sonar={0};network_sonar_packed_t np={{0}},ns=np;
    sonar.angle=-31;assert(!network_sonar_pack(&np,&sonar));
    sonar.angle=31;assert(!network_sonar_pack(&np,&sonar));
    sonar.angle=0;sonar.ping_angle=31;assert(!network_sonar_pack(&np,&sonar));
    sonar.ping_angle=0;sonar.status=4u;assert(!network_sonar_pack(&np,&sonar));
    sonar.status=0u;sonar.echo=0x4000u;assert(!network_sonar_pack(&np,&sonar));
    assert(memcmp(&np,&ns,sizeof(np))==0);
    sonar.echo=0u;assert(network_sonar_pack(&np,&sonar));
    for (unsigned int bit=6u;bit<8u;++bit) {
        network_sonar_packed_t bad=np;((uint8_t *)bad.words)[31]|=(uint8_t)(1u<<bit);
        assert(!network_sonar_unpack(&sonar,&bad));
    }
    rm_heartbeat_t health={0},out={.uptime_ms=77u};
    network_health_packed_t hp;
    for (unsigned int flags=0u;flags<128u;++flags) {
        for (unsigned int reset=0u;reset<3u;++reset) {
            for (unsigned int state=0u;state<4u;++state) {
                for (unsigned int mask=0u;mask<32u;++mask) {
                    health.flags=(uint8_t)flags;health.reset_reason=(uint8_t)reset;
                    health.mqtt_state=(uint8_t)state;health.board_unavailable_mask=(uint8_t)mask;
                    assert(network_health_pack(&hp,&health));
                    assert(network_health_unpack(&out,&hp));
                    assert(out.flags==flags && out.reset_reason==reset);
                    assert(out.mqtt_state==state && out.board_unavailable_mask==mask);
                }
            }
        }
    }
    unsigned char before[sizeof(out)];memcpy(before,&out,sizeof(out));
    for (unsigned int bit=809u;bit<832u;++bit) {
        network_health_packed_t bad=hp;
        ((uint8_t *)bad.words)[bit/8u]|=(uint8_t)(1u<<(bit%8u));
        assert(!network_health_unpack(&out,&bad));
        assert(memcmp(before,&out,sizeof(out))==0);
    }
    assert(!network_health_pack(0,&health));assert(!network_health_unpack(0,&hp));
    assert(!network_sonar_pack(0,&sonar));assert(!network_sonar_unpack(0,&np));
    assert(!telemetry_scan_pack(0,&scan));assert(!telemetry_scan_unpack(0,&sp));
    assert(!telemetry_info_pack(0,&info));assert(!telemetry_info_unpack(0,&ip));
}
int main(void)
{
    test_invalid_and_boundaries();
    test_scan();test_info();test_sonar();test_health();
    puts("test_telemetry_optional_pack: PASS");return 0;
}

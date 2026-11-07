#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <tk/tkernel.h>
#include "../app_program/vehicle/vehicle_board.h"
#include "../app_program/subsystem1/robot_network.h"
#include "../app_program/subsystem2/motion.h"
#include "../app_program/subsystem3/line_service.h"
#include "../app_program/subsystem5/sonar_service.h"
static uint32_t now_ms,now_us,motion_us;
static line_snapshot_t line_value;
static motion_status_t motion_value;
static vehicle_snapshot_t vehicle_value;
static sonar_snapshot_t sonar_value;
static sonar_history_entry_t history_value;
static int line_error=E_TMOUT,motion_error=E_TMOUT,vehicle_error=E_TMOUT,sonar_error=E_TMOUT,history_error=E_TMOUT;
static unsigned int reads[5],busy[5];
static bool advance_line,advance_sonar;
uint32_t drv_time_ms(void) { return now_ms; }
uint32_t drv_time_us(void) { return now_us; }
int line_snapshot_try_read(line_snapshot_t *out) {
    ++reads[0];if (line_error<0) return line_error;
    if (advance_line) { ++now_ms;now_us+=1000u;line_value.timestamp_us=now_us;advance_line=false; }
    *out=line_value;return E_OK;
}
int motion_snapshot_try_read(motion_status_t *out,uint32_t *timestamp) { ++reads[2];if (motion_error<0) return motion_error;*out=motion_value;*timestamp=motion_us;return E_OK; }
int vehicle_snapshot_try_read(vehicle_snapshot_t *out) { ++reads[3];if (vehicle_error<0) return vehicle_error;*out=vehicle_value;return E_OK; }
int sonar_snapshot_try_read(sonar_snapshot_t *out) {
    ++reads[1];if (sonar_error<0) return sonar_error;
    if (advance_sonar) { ++now_ms;now_us+=1000u;sonar_value.timestamp_ms=now_ms;advance_sonar=false; }
    *out=sonar_value;return E_OK;
}
int sonar_history_read(uint32_t *cursor,sonar_history_entry_t *out,uint32_t *missed) {
    ++reads[4];*missed=0u;if (history_error<0) return history_error;
    *out=history_value;*cursor=history_value.sequence;*missed=3u;return E_OK;
}
static rm_detour_t detour_value;static int detour_error=E_NOEXS;
static rm_summary_t summary_value;static int summary_error=E_NOEXS;
int vehicle_detour_try_read(rm_detour_t *out) { if (detour_error<0) return detour_error;*out=detour_value;return E_OK; }
int vehicle_summary_try_read(rm_summary_t *out) { if (summary_error<0) return summary_error;*out=summary_value;return E_OK; }
static rm_obstacle_t obstacle_value;static int obstacle_error=E_NOEXS;
int vehicle_obstacle_try_read(rm_obstacle_t *out) { if (obstacle_error<0) return obstacle_error;*out=obstacle_value;return E_OK; }
void robot_network_busy(unsigned int board) { assert(board<5u);++busy[board]; }
static unsigned int unavailable_reads[5];
void robot_network_board_result(unsigned int board,int result) { assert(board<5u);if(result==E_OBJ || result==E_NOEXS) ++unavailable_reads[board];else if(result<0) ++busy[board]; }
static uint16_t row_bits(const uint8_t *row,unsigned int offset,unsigned int width) {
    uint16_t value=0u;
    for (unsigned int bit=0u;bit<width;++bit)
        value|=(uint16_t)((uint16_t)((row[(offset+bit)/8u]>>((offset+bit)%8u))&1u)<<bit);
    return value;
}
int main(void) {
    rm_state_t value;
    robot_network_state(&value);
    assert(value.line_age_ms==65535u && value.motion_age_ms==65535u && value.vehicle_age_ms==65535u);
    assert(value.flags==112u && row_bits(value.kr1,93u,16u)==ULTRASONIC_RESULT_NONE);
    now_ms=100u;now_us=100000u;motion_us=80000u;
    line_value=(line_snapshot_t){.valid=true,.sequence=1u,.timestamp_us=90000u,.raw_bc=4095u,.raw={123u,456u,0u}};
    line_value.line.norm[0]=789u;line_value.line.norm[1]=987u;line_value.line.position=-25;
    motion_value=(motion_status_t){.ticks=1u,.counts={-123456,234567},.travel_mm={-9,22},.speed_mm_s={-20,30},.duty_permille={-345,678},.heading_cdeg=-1234567,.faults=3u,.late_wakes=7u,.missed_wakes=8u};
    vehicle_value=(vehicle_snapshot_t){.sequence=1u,.timestamp_us=70000u,.active=true,.state=2u,.module=3u,.servo_deg=-15,
        .left_mm_s=-40,.right_mm_s=50,.overruns=9u};
    sonar_value=(sonar_snapshot_t){.status=SONAR_ECHO,.valid=true,.timestamp_ms=0u,.echo=123u};
    line_error=motion_error=vehicle_error=sonar_error=E_OK;
    robot_network_state(&value);
    assert(value.line_age_ms==10u && value.motion_age_ms==20u && value.vehicle_age_ms==30u);
    assert(value.flags==143u && value.raw_bc==4095u && value.norm_l==789u && value.norm_c==987u);
    assert(value.count_l==-123456 && value.count_r==234567 && value.motion_faults==3u && value.state==2u && value.module==3u);
    assert(value.duty_left_permille==-345 && value.duty_right_permille==678);
    assert(value.heading_cdeg==-1234567);
    assert(value.turn_rate_ddps==258); /* (30 - -20) mm/s * 12414 / 2405 = 258.1 */
    assert(value.dist_left_mm==-9 && value.dist_right_mm==22 && value.dist_avg_mm==6); /* (-9+22)/2 */
    assert(row_bits(value.kr1,93u,16u)==123u); /* Sonar age 100ms is included. */
    const rm_state_t retained=value;
    now_ms=101u;now_us=101000u;line_error=motion_error=vehicle_error=sonar_error=E_TMOUT;
    robot_network_state(&value);
    assert(value.flags==241u && value.line_age_ms==11u && value.motion_age_ms==21u && value.vehicle_age_ms==31u);
    assert(value.raw_bc==retained.raw_bc && value.norm_l==retained.norm_l && value.norm_c==retained.norm_c);
    assert(value.count_l==retained.count_l && value.count_r==retained.count_r && value.state==retained.state && value.module==retained.module);
    assert(row_bits(value.kr1,93u,16u)==ULTRASONIC_RESULT_NONE);
    assert(busy[0]==2u && busy[1]==2u && busy[2]==2u && busy[3]==2u);
    /* Successful but stalled sequence must not become fresh when us wraps. */
    line_error=motion_error=vehicle_error=E_OK;sonar_error=E_OK;
    now_ms=4295068u;now_us=100001u;
    robot_network_state(&value);
    assert(value.line_age_ms==65535u && value.motion_age_ms==65535u && value.vehicle_age_ms==65535u && (value.flags&14u)==0u);
    assert(row_bits(value.kr1,93u,16u)==ULTRASONIC_RESULT_NONE);
    line_value.sequence=2u;motion_value.ticks=2u;vehicle_value.sequence=2u;
    line_value.timestamp_us=now_us-1000u;motion_us=now_us-1000u;vehicle_value.timestamp_us=now_us-1000u;
    robot_network_state(&value);
    assert(value.line_age_ms==1u && value.motion_age_ms==1u && value.vehicle_age_ms==1u && (value.flags&14u)==14u);
    /* Per-source observation time also survives millisecond counter wrap. */
    now_ms=UINT32_MAX-3u;now_us=200000u;
    line_value.sequence=UINT32_MAX;motion_value.ticks=UINT32_MAX;vehicle_value.sequence=UINT32_MAX;
    line_value.timestamp_us=195000u;motion_us=195000u;vehicle_value.timestamp_us=195000u;
    robot_network_state(&value);assert(value.line_age_ms==5u);
    now_ms=2u;now_us=206000u;robot_network_state(&value);
    assert(value.line_age_ms==11u && value.motion_age_ms==11u && value.vehicle_age_ms==11u);
    assert((value.flags&2u)==0u && (value.flags&12u)==12u);
    line_value.sequence=0u;motion_value.ticks=1u;vehicle_value.sequence=0u;
    line_value.timestamp_us=now_us;motion_us=now_us;vehicle_value.timestamp_us=now_us;
    robot_network_state(&value);assert(value.line_age_ms==0u && value.motion_age_ms==0u && value.vehicle_age_ms==0u);
    /* History is independent of the current-board reader, even while busy. */
    sonar_error=E_TMOUT;history_error=E_OK;
    history_value=(sonar_history_entry_t){.sequence=42u,.result={.generation=12u,.id=34u,.status=SONAR_DEADLINE,
        .angle=-30,.ping_angle=25,.valid=false,.pending=true,.trigger_ms=50u,.timestamp_ms=60u,.echo=0x456u,.ping_sequence=99u}};
    const unsigned int board_reads=reads[1];uint32_t cursor=0u,missed=0u;rm_sonar_t sonar;
    assert(robot_network_sonar(&cursor,&sonar,&missed)==E_OK && reads[1]==board_reads);
    assert(cursor==42u && missed==3u && sonar.history_sequence==42u && sonar.generation==12u && sonar.request_id==34u);
    assert(sonar.status==SONAR_DEADLINE && sonar.angle==-30 && sonar.ping_angle==25 && !sonar.valid && sonar.pending);
    assert(sonar.trigger_ms==50u && sonar.timestamp_ms==60u && sonar.echo==0x456u && sonar.ping_sequence==99u);
    const rm_sonar_t old_sonar=sonar;history_error=E_OBJ;
    assert(robot_network_sonar(&cursor,&sonar,&missed)==E_OBJ && cursor==42u && missed==0u && memcmp(&sonar,&old_sonar,sizeof(sonar))==0);
    /* Health uses bounded boards only. No blocking API is provided by this test. */
    const unsigned int line_reads=reads[0],sonar_board_reads=reads[1],history_reads=reads[4];
    rm_heartbeat_t health={.telemetry_lost=123u};
    robot_health_collect(&health);
    assert(health.motion_late==7u && health.motion_missed==8u && health.vehicle_overruns==9u && health.telemetry_lost==123u);
    motion_error=vehicle_error=E_TMOUT;motion_value.late_wakes=100u;vehicle_value.overruns=100u;
    robot_health_collect(&health);
    assert(health.motion_late==7u && health.motion_missed==8u && health.vehicle_overruns==9u);
    assert(reads[0]==line_reads && reads[1]==sonar_board_reads && reads[4]==history_reads);
    assert(busy[2]==3u && busy[3]==3u);
    /* A higher-priority publisher can run during a bounded read. Capture the
     * age reference clock after all reads, not before their newer timestamps. */
    now_ms=1000u;now_us=1000000u;motion_us=now_us;
    line_error=motion_error=vehicle_error=sonar_error=E_OK;
    line_value.sequence=10u;line_value.timestamp_us=now_us;
    motion_value.ticks=10u;vehicle_value.sequence=10u;vehicle_value.timestamp_us=now_us;
    sonar_value=(sonar_snapshot_t){.status=SONAR_ECHO,.valid=true,.echo=321u};
    advance_line=advance_sonar=true;robot_network_state(&value);
    assert(now_ms==1002u && !advance_line && !advance_sonar);
    assert(value.line_age_ms==1u && value.motion_age_ms==2u && value.vehicle_age_ms==2u && (value.flags&14u)==14u);
    assert(row_bits(value.kr1,93u,16u)==321u);
    /* Obstacle adapter: new report only, gaps visible in scan_id, busy counted. */
    {
        uint32_t scan_cursor=0u;rm_obstacle_t out;
        const unsigned int busy_before=busy[ROBOT_BOARD_VEHICLE];
        obstacle_error=E_NOEXS;assert(robot_network_obstacle(&scan_cursor,&out)==E_NOEXS && scan_cursor==0u);
        obstacle_error=E_TMOUT;assert(robot_network_obstacle(&scan_cursor,&out)==E_TMOUT);
        assert(busy[ROBOT_BOARD_VEHICLE]==busy_before+1u);
        obstacle_error=E_OK;obstacle_value=(rm_obstacle_t){.scan_id=4u,.closest_mm=300u};
        assert(robot_network_obstacle(&scan_cursor,&out)==E_OK && scan_cursor==4u && out.closest_mm==300u);
        assert(robot_network_obstacle(&scan_cursor,&out)==E_NOEXS && scan_cursor==4u); /* same report */
        obstacle_value.scan_id=7u;
        assert(robot_network_obstacle(&scan_cursor,&out)==E_OK && scan_cursor==7u && out.scan_id==7u); /* 5,6 missed */
    }
    /* Detour and summary adapters use their own cursors (sequence, run_id). */
    {
        uint32_t detour_cursor=0u,summary_cursor=0u;rm_detour_t d;rm_summary_t r;
        detour_error=E_NOEXS;summary_error=E_NOEXS;
        assert(robot_network_detour(&detour_cursor,&d)==E_NOEXS && robot_network_summary(&summary_cursor,&r)==E_NOEXS);
        detour_error=E_TMOUT;summary_error=E_TMOUT;
        const unsigned int busy_now=busy[ROBOT_BOARD_VEHICLE];
        assert(robot_network_detour(&detour_cursor,&d)==E_TMOUT && robot_network_summary(&summary_cursor,&r)==E_TMOUT);
        assert(busy[ROBOT_BOARD_VEHICLE]==busy_now+2u);
        detour_error=E_OK;summary_error=E_OK;
        detour_value=(rm_detour_t){.sequence=3u,.outcome=RM_DETOUR_DONE_LINE};
        summary_value=(rm_summary_t){.run_id=2u,.distance_mm=77u};
        assert(robot_network_detour(&detour_cursor,&d)==E_OK && detour_cursor==3u && d.outcome==RM_DETOUR_DONE_LINE);
        assert(robot_network_detour(&detour_cursor,&d)==E_NOEXS);
        assert(robot_network_summary(&summary_cursor,&r)==E_OK && summary_cursor==2u && r.distance_mm==77u);
        assert(robot_network_summary(&summary_cursor,&r)==E_NOEXS);
        summary_cursor=0u; /* a new connection asks for the last summary again */
        assert(robot_network_summary(&summary_cursor,&r)==E_OK && r.run_id==2u);
    }
    /* An absent TEST vehicle is not a busy writer, in either reader. */
    {
        const unsigned int before_busy=busy[ROBOT_BOARD_VEHICLE];
        const unsigned int before_absent=unavailable_reads[ROBOT_BOARD_VEHICLE];
        vehicle_error=E_OBJ;
        robot_network_state(&value);
        assert((value.flags&64u)==0u && (value.flags&8u)==0u);
        robot_health_collect(&health);
        assert(busy[ROBOT_BOARD_VEHICLE]==before_busy);
        assert(unavailable_reads[ROBOT_BOARD_VEHICLE]==before_absent+2u);
    }
    return 0;
}

/* Reuse the scheduler and sensor doubles without editing the existing test.
 * Public vehicle snapshots and a sleep wrapper provide the observation hook. */
#define main original_vehicle_test
#define tk_slp_tsk base_vehicle_sleep
#define drv_motor_hw_button_down base_vehicle_button
#define telemetry_send base_telemetry_send
#include "test_vehicle_task.c"
#undef telemetry_send
#undef drv_motor_hw_button_down
#undef tk_slp_tsk
#undef main
#include "../app_program/vehicle/vehicle_board.h"
#include "../app_program/subsystem1/robot_network.h"
#include "../app_program/vehicle/run_stats.h"
#include <tk/smp.h>
#define irq_off test_smp_irq_mask
static unsigned int mode, network_events, network_lost, dump_requests;
static unsigned int command_losses;
static robot_command_t ordinary[8],estop_slot;
static unsigned int command_head,command_tail;
static bool have_estop,injected,verified_dump,verified_priority;
static unsigned int rows_at_dump,rows_after_stop;
static uint32_t stopped_at;
static bool rejected_uncalibrated;
static unsigned int hump_events;
void mock_di(UINT *irq) { *irq = test_smp_irq_save(); }
void mock_ei(UINT irq) { test_smp_irq_restore(irq); }
int robot_event_push(const rm_event_t *event) {
    assert(event);++network_events;++network_lost;
    if (event->kind==6u && event->word==(uint16_t)(RM_CMD_START|(2u<<8)) && event->detail==2u) rejected_uncalibrated=true;
    if (event->kind==RM_EVENT_HUMP) {
        /* Live only, inside the run, once per new event: 10.8 mm, #1, RIGHT. */
        assert(event->from==0u && event->to==0u && event->word==108u && event->detail==1u);
        assert((vehicle_run_epoch()&1u)!=0u);++hump_events;
    }
    return E_TMOUT; /* Network queue failure must not stop the vehicle. */
}
int robot_command_pop(robot_command_t *out) {
    if (have_estop) { *out=estop_slot;have_estop=false;return (int)sizeof(*out); }
    if (command_tail==command_head) return E_TMOUT;
    *out=ordinary[command_tail++%8u];return (int)sizeof(*out);
}
void robot_command_lost(void) { ++command_losses; }
int robot_network_dump(uint32_t request_id) { assert(request_id==100u);++dump_requests;return E_OBJ; }
int telemetry_send(const telemetry_message_t *message) { return base_telemetry_send(message); }
bool drv_motor_hw_button_down(unsigned int pin) {
    if (mode==1u) return false; /* No physical start in uncalibrated START case. */
    if (mode==2u) return pin==21u && now>=20u && now<50u;
    return base_vehicle_button(pin);
}
ER tk_slp_tsk(TMO timeout) {
    const ER result=base_vehicle_sleep(timeout);
    vehicle_snapshot_t value;assert(vehicle_snapshot_try_read(&value)==E_OK);
    if (mode==1u) {
        if (!injected && now>=20u) { ordinary[command_head++%8u]=(robot_command_t){RM_CMD_START,1u};injected=true; }
        if (now>=40u) assert(!value.active && !value.calibrated && !calibrating && line_drives==0u);
    } else if (value.active && value.phase==3u && !injected) {
        injected=true;
        if (mode==2u) {
            /* START was queued first, but the dedicated ESTOP must win. */
            ordinary[command_head++%8u]=(robot_command_t){RM_CMD_START,200u};
            estop_slot=(robot_command_t){RM_CMD_ESTOP,201u};have_estop=true;
        } else {
            rows_at_dump=rows;ordinary[command_head++%8u]=(robot_command_t){RM_CMD_DUMP,100u};
        }
    }
    if (mode==0u && dump_requests>0u && now<5000u && value.command_id==100u) {
        assert(value.command_status==3u && value.active && estops==0u);
        if (rows>rows_at_dump && line_drives>0u) verified_dump=true;
    }
    if (mode==2u && injected && estops>0u) {
        assert(estops==1u && asked[0]==0 && asked[1]==0 && command_tail==command_head && command_losses==1u);
        if (stopped_at==0u) { stopped_at=now;rows_after_stop=rows; }
        if (now-stopped_at>500u) {
            assert(!value.active && rows==rows_after_stop && !calibrating);verified_priority=true;
        }
    }
    return result;
}
int main(int argc,char **argv) {
    if (argc>1 && !strcmp(argv[1],"uncalibrated")) mode=1u;
    else if (argc>1 && !strcmp(argv[1],"estop")) mode=2u;
    lp_init(&line);lp_set_pair(&line,true,0);watchdog_ready=true;services_ready=true;
    assert(vehicle_task_start()==E_OK && starts==1u);
    if (setjmp(done)==0) ((void (*)(INT,void *))entry)(0,0);
    assert(injected && listener==7 && notifications==1u && prints==1u && !irq_off);
    assert(network_events>0u && network_lost==network_events);
    if (mode==1u) assert(rejected_uncalibrated && line_drives==0u && rows==0u && estops==0u && asked[0]==0 && asked[1]==0);
    else if (mode==2u) assert(verified_priority && command_losses==1u && estops==1u);
    else assert(verified_dump && dump_requests==1u && line_drives>0u && rows>0u && estops==1u);
    if (mode==1u) assert(hump_events==0u);
    else {
        rm_summary_t summary;assert(vehicle_summary_try_read(&summary)==E_OK);
        /* ESTOP may end the run before its first hump poll; the summary
         * still takes one final fresh board read. */
        assert(hump_events==(mode==0u ? 1u : hump_events) && hump_events<=1u);
        assert(summary.hump_peak==108u && summary.hump_count==1u);
        assert(summary.hump_flags==RM_HUMP_IMPLEMENTED && summary.hump_gated==0u);
    }
    return 0;
}

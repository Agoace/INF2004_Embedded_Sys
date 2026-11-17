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
#include "../app_program/vehicle/vehicle_sm.h"
#include "../app_program/subsystem1/robot_network.h"
#include "../app_program/vehicle/run_stats.h"
static unsigned int mode,irq_off,network_events,network_lost,dump_requests,command_losses;
static robot_command_t ordinary[8],estop_slot;
static unsigned int command_head,command_tail;
static bool have_estop,injected,verified_dump,verified_priority;
static unsigned int rows_at_dump,rows_after_stop;
static uint32_t stopped_at;
static bool rejected_uncalibrated;
static unsigned int hump_events;
static bool fault_state_seen;
/* Cause byte carried on the live IMU-unavailable event (RT-0099). */
static unsigned int unavailable_events;
static unsigned int unavailable_detail;
/* Bit N is set when a kind-3/word-3 event carried detail N. A refused start
 * reports the same word, so the mask covers both the latch and the refusal. */
static unsigned int unavailable_detail_mask;
/* IMU recovery observations: word bit 15 marks them, low byte is the gate. */
static unsigned int recovery_events;
static unsigned int recovery_pending;
static unsigned int recovery_cancelled;
static unsigned int recovery_pose;
static unsigned int recovery_invalid_zero;
static unsigned int recovery_stale;
static bool changed_recovery_board;
void mock_di(UINT *irq) { assert(!irq_off);irq_off=1u;*irq=77u; }
void mock_ei(UINT irq) { assert(irq==77u && irq_off);irq_off=0u; }
int robot_event_push(const rm_event_t *event) {
    assert(event);++network_events;++network_lost;
    if (event->kind==6u && event->word==(uint16_t)(RM_CMD_START|(2u<<8)) && event->detail==2u) rejected_uncalibrated=true;
    if (event->kind==3u && (event->word & 0x8000u) != 0u) {
        const unsigned int gate = event->word & 0xFFu;
        ++recovery_events;
        if (gate==8u) ++recovery_pending;
        if (gate==9u) ++recovery_cancelled;
        if (gate==4u) ++recovery_pose;
        if (gate==3u && event->detail==IMU_SAFETY_CAUSE_ZERO_VECTOR)
            ++recovery_invalid_zero;
        if (gate==2u && event->detail==IMU_SAFETY_CAUSE_STALE)
            ++recovery_stale;
    }
    if (event->kind==3u && event->word==3u) {
        /* IMU_SAFETY_IMU_UNAVAILABLE, latch or refused start: detail=cause. */
        if (unavailable_events==0u) unavailable_detail=event->detail;
        if (event->detail<32u) unavailable_detail_mask|=(1u<<event->detail);
        ++unavailable_events;
    }
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
    if ((mode==10u || mode==11u || mode==12u) &&
        recovery_pending>0u && !changed_recovery_board) {
        /* A new unsafe observation arrives after GP21 queued a clear. The
         * old latch is still present, so its reason alone cannot detect it. */
        changed_recovery_board=true;
        if (mode==10u) {
            safety_out.flags=IMU_SAFETY_FLAG_VALID;
            safety_out.clear_ms=0u;
        } else if (mode==11u) {
            safety_out.flags=IMU_SAFETY_FLAG_ZERO_VECTOR;
            safety_out.clear_ms=0u;
        } else {
            safety_busy=true;
        }
    }
    if (mode==1u) {
        if (!injected && now>=20u) { ordinary[command_head++%8u]=(robot_command_t){RM_CMD_START,1u};injected=true; }
        if (now>=40u) assert(!value.active && !value.calibrated && !calibrating && line_drives==0u);
    } else if (value.active && value.phase==3u && !injected) {
        injected=true;
        if (mode==2u) {
            /* START was queued first, but the dedicated ESTOP must win. */
            ordinary[command_head++%8u]=(robot_command_t){RM_CMD_START,200u};
            estop_slot=(robot_command_t){RM_CMD_ESTOP,201u};have_estop=true;
        } else if (mode==3u) {
            /* A confirmed trigger while the run is active. The all-clear
             * window is already complete, so a later GP21 press reaches the
             * writer; the writer refuses the clear, which must keep the
             * persistent FAULT. */
            safety_out.latched_trigger=(uint8_t)IMU_SAFETY_LIFT;
            safety_out.clear_ms=IMU_SAFETY_CLEAR_MS;safety_refuse_clear=true;
        } else if (mode==4u) {
            /* The IMU goes unavailable while the robot is moving. */
            safety_busy=true;
        } else if (mode==5u) {
            /* Reads succeed, but the published sample is not valid. */
            safety_out.flags=0u;
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
    if ((mode==3u || mode==4u || mode==5u) && estops>0u) {
        /* F2: an IMU stop must publish a persistent FAULT, not IDLE. It must
         * also stop issuing wheel commands. */
        assert(estops==1u && asked[0]==0 && asked[1]==0);
        if (stopped_at==0u) stopped_at=now;
        if (now-stopped_at>300u) {
            assert(!value.active && value.state==(uint8_t)VSM_FAULT);
            fault_state_seen=true;
        }
    }
    return result;
}
int main(int argc,char **argv) {
    if (argc>1 && !strcmp(argv[1],"uncalibrated")) mode=1u;
    else if (argc>1 && !strcmp(argv[1],"estop")) mode=2u;
    else if (argc>1 && !strcmp(argv[1],"imufault")) mode=3u;
    else if (argc>1 && !strcmp(argv[1],"imuunavailable")) mode=4u;
    else if (argc>1 && !strcmp(argv[1],"imuinvalid")) mode=5u;
    else if (argc>1 && !strcmp(argv[1],"imuabsent")) mode=6u;
    else if (argc>1 && !strcmp(argv[1],"imupose")) mode=7u;
    else if (argc>1 && !strcmp(argv[1],"imuclearretry")) mode=8u;
    else if (argc>1 && !strcmp(argv[1],"imuclearintent")) mode=9u;
    else if (argc>1 && !strcmp(argv[1],"imuclearpose")) mode=10u;
    else if (argc>1 && !strcmp(argv[1],"imuclearinvalid")) mode=11u;
    else if (argc>1 && !strcmp(argv[1],"imuclearstale")) mode=12u;
    if (mode==8u) {
        /* The writer applies the clear 250 ms after the request: longer than
         * the old single 100 ms attempt, so only a retry can release it. */
        safety_out.latched_trigger=(uint8_t)IMU_SAFETY_LIFT;
        safety_out.clear_ms=IMU_SAFETY_CLEAR_MS;
        safety_clear_delay_ms=250u;
    }
    if (mode>=9u && mode<=12u) {
        /* The writer never applies the clear: the bounded intent must expire,
         * record the outcome and keep the latch. */
        safety_out.latched_trigger=(uint8_t)IMU_SAFETY_LIFT;
        safety_out.clear_ms=IMU_SAFETY_CLEAR_MS;
        safety_refuse_clear=true;
    }
    if (mode==6u) safety_busy=true; /* no board at all: every read times out */
    if (mode==7u) safety_out.flags=IMU_SAFETY_FLAG_VALID; /* fresh, not upright */
    lp_init(&line);lp_set_pair(&line,true,0);watchdog_ready=true;services_ready=true;
    assert(vehicle_task_start()==E_OK && starts==1u);
    if (setjmp(done)==0) ((void (*)(INT,void *))entry)(0,0);
    /* Modes 6 and 7 must refuse the start, so no run is ever active. */
    assert((injected || mode==6u || mode==7u ||
            (mode>=9u && mode<=12u)) && listener==7 &&
           notifications==1u && !irq_off);
    /* The IMU scenarios add a refused-restart line and one latched-reason
     * line, so they have more than the single boot line. */
    if (mode<=2u) assert(prints==1u);
    else assert(prints>=1u);
    assert(network_events>0u && network_lost==network_events);
    if (mode==1u) {
        assert(rejected_uncalibrated && line_drives==0u && rows==0u && estops==0u && asked[0]==0 && asked[1]==0);
        assert(hump_events==0u);
        return 0;
    }
    if (mode==6u || mode==7u) {
        /* The gate must refuse, and the event must say why: an absent board
         * is not a pose refusal (RT-0101/F2). */
        const unsigned int want =
            (mode==6u) ? (unsigned int)IMU_SAFETY_CAUSE_ABSENT
                       : (unsigned int)IMU_SAFETY_CAUSE_POSE;
        assert(line_drives==0u && estops==0u && asked[0]==0 && asked[1]==0);
        assert(unavailable_events>=1u);
        assert(unavailable_detail==want);
        assert(unavailable_detail_mask==(1u<<want));
        return 0;
    }
    if (mode==8u) {
        /* The retried request released the latch and a later press started a
         * run. A single 100 ms attempt cannot do this. */
        assert(safety_clear_requests>=2u);
        assert(recovery_pending==1u);
        assert(run_seen!=0u && line_drives>0u);
        assert(first_drive_ms>=5000u);
        assert(estops>=1u);
        return 0;
    }
    if (mode==9u) {
        /* The refused clear never releases: the bounded intent expires, the
         * outcome is recorded, and no wheel ever moves. */
        assert(safety_clear_requests>=2u);
        /* The harness presses GP21 at 20 ms and again at 5000 ms. Each
         * accepted press emits exactly one pending event. */
        assert(recovery_pending==2u);
        assert(recovery_cancelled==2u);
        assert(run_seen==0u && line_drives==0u && spins==0u);
        assert(asked[0]==0 && asked[1]==0 && first_drive_ms==0u);
        assert(estops>=1u);
        return 0;
    }
    if (mode>=10u && mode<=12u) {
        assert(changed_recovery_board && recovery_pending==1u);
        assert((mode==10u && recovery_pose>=1u) ||
               (mode==11u && recovery_invalid_zero>=1u) ||
               (mode==12u && recovery_stale>=1u));
        assert(run_seen==0u && line_drives==0u && spins==0u);
        assert(asked[0]==0 && asked[1]==0 && first_drive_ms==0u);
        return 0;
    }
    if (mode==3u || mode==4u || mode==5u) {
        /* F2 and the approved recovery rule. */
        rm_summary_t summary;
        assert(fault_state_seen);
        assert(fault_state_seen && estops==1u);
        assert(vehicle_summary_try_read(&summary)==E_OK);
        assert(summary.shell_fault==(uint8_t)VSM_FAULT_IMU);
        assert(summary.shell_fault_detail==
               (uint16_t)(mode==3u ? IMU_SAFETY_LIFT
                                   : IMU_SAFETY_IMU_UNAVAILABLE));
        if (mode==5u) {
            /* The published sample is invalid, so the cause is INVALID. */
            assert(unavailable_events>=1u);
            assert(unavailable_detail==(unsigned int)IMU_SAFETY_CAUSE_INVALID);
            assert(unavailable_detail_mask==
                   (1u<<(unsigned int)IMU_SAFETY_CAUSE_INVALID));
            return 0;
        }
        if (mode==3u) assert(safety_clear_requests>=1u);
        else {
            assert(safety_clear_requests==0u);
            /* Mode 4 makes every board read busy after an earlier successful
             * read, so the cached copy ages out: the cause is STALE, not an
             * absent board or an invalid sample. */
            assert(unavailable_events>=1u);
            assert(unavailable_detail==(unsigned int)IMU_SAFETY_CAUSE_STALE);
            /* The later GP21 press while the latch is in force must keep the
             * saved cause, so no kind-3 event may carry detail 0. */
            assert((unavailable_detail_mask &
                    (1u<<(unsigned int)IMU_SAFETY_CAUSE_NONE))==0u);
        }
        return 0;
    }
    if (mode==2u) assert(verified_priority && command_losses==1u && estops==1u);
    else assert(verified_dump && dump_requests==1u && line_drives>0u && rows>0u && estops==1u);
    {
        rm_summary_t summary;assert(vehicle_summary_try_read(&summary)==E_OK);
        /* ESTOP may end the run before its first hump poll; the summary
         * still takes one final fresh board read. */
        assert(hump_events==(mode==0u ? 1u : hump_events) && hump_events<=1u);
        assert(summary.hump_peak==108u && summary.hump_count==1u);
        assert(summary.hump_flags==RM_HUMP_IMPLEMENTED && summary.hump_gated==0u);
    }
    return 0;
}

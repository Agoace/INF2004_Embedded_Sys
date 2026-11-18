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
#include "../app_program/platform/fault_mask.h"
#include "../app_program/subsystem1/robot_network.h"
#include "../app_program/vehicle/run_stats.h"
#include "../app_program/subsystem3/barcode_live_source.h"
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
static unsigned int barcode_case, barcode_reads;
/* IMU events by kind: kind 3 is motion/ESTOP only, kind 10 IMU safety. */
static unsigned int imu_stops, imu_pending, imu_cleared, imu_other;
static uint16_t imu_stop_word, imu_last_word;
static uint8_t imu_stop_detail;
static unsigned int estop_events; /* kind 3 word ESTOP, once per stop */
static uint32_t first_barcode_epoch;
static unsigned int shell_fault_case;
static bool shell_fault_seen;
static bool observed_imu_latch;
static bool capture_time_limit;
static unsigned int stop_captures;
static uint32_t captured_epoch, captured_vehicle_faults;

/* Health API seam: copy the real owner word before either epoch goes even.
 * The real Health tests separately check its capture board and collection. */
void robot_health_faults_stop(uint32_t epoch)
{
    assert((epoch & 1u) != 0u && epoch != captured_epoch);
    assert(vehicle_run_epoch() == epoch && rt_epoch() == epoch);
    assert(vehicle_fault_mask_try_read(&captured_vehicle_faults));
    const uint32_t expected = shell_fault_case == 1u ? FAULT_SHELL_OTHER :
        (mode == 3u || mode == 4u || mode == 6u) ? FAULT_IMU_SAFETY : 0u;
    assert(captured_vehicle_faults == expected);
    captured_epoch = epoch;
    ++stop_captures;
}

vsm_drive_t __real_vsm_step(vsm_t *state, const vsm_inputs_t *in,
                           uint8_t faults, uint32_t ms);
vsm_drive_t __wrap_vsm_step(vsm_t *state, const vsm_inputs_t *in,
                           uint8_t faults, uint32_t ms)
{
    /* Keep navigation running so this case reaches the real 90 s limit. */
    if (capture_time_limit) {
        return (vsm_drive_t){.left_mm_s = 100, .right_mm_s = 100,
                             .action = VSM_ACT_SPEED};
    }
    vsm_drive_t drive = __real_vsm_step(state, in, faults, ms);
    if (shell_fault_case != 0u && state->state == VSM_RUNNING) {
        const vsm_fault_t cause = shell_fault_case == 1u ? VSM_FAULT_FAILED :
            shell_fault_case == 2u ? VSM_FAULT_SAFETY : VSM_FAULT_IMU;
        vsm_set_fault(state, cause, 123u);
        drive = (vsm_drive_t){.action = VSM_ACT_STOP};
    }
    return drive;
}


int __wrap_barcode_live_counts_try_read(barcode_live_counts_t *out)
{
    ++barcode_reads;
    const uint32_t epoch = vehicle_run_epoch();
    assert((epoch & 1u) != 0u); /* Poll before STOP clears run context. */
    if (first_barcode_epoch == 0u) { first_barcode_epoch = epoch; }
    if (barcode_case == 2u ||
        (barcode_case == 9u && epoch != first_barcode_epoch)) {
        return E_NOSPT;
    }
    if (barcode_case == 3u) { return E_TMOUT; }
    *out = (barcode_live_counts_t){.epoch = epoch,
        .frames = 7u, .failures = 3u, .attempts = 10u};
    if (barcode_case == 4u) { out->epoch = epoch - 1u; }
    if (barcode_case == 5u) {
        out->frames = UINT32_MAX;
        out->failures = UINT32_MAX;
    }
    if (barcode_case == 6u) {
        out->frames = 0u; out->failures = 0u; out->attempts = 0u;
    }
    if (barcode_case == 7u && epoch != first_barcode_epoch) {
        out->frames = 4u; out->failures = 2u; out->attempts = 6u;
    }
    if (barcode_case == 8u && now >= 4990u) { return E_TMOUT; }
    return E_OK;
}
void mock_di(UINT *irq) { *irq = test_smp_irq_save(); }
void mock_ei(UINT irq) { test_smp_irq_restore(irq); }
int robot_event_push(const rm_event_t *event) {
    assert(event);++network_events;++network_lost;
    if (event->kind==6u && event->word==(uint16_t)(RM_CMD_START|(2u<<8)) && event->detail==2u) rejected_uncalibrated=true;
    if (event->kind==3u) {
        /* Motion fault bits only; IMU words never share kind 3. */
        assert((event->word&0x8000u)==0u && (event->word&~0x0fu)==0u);
        assert(mode<3u || mode==5u || event->word==MC_FAULT_ESTOP);
        if (event->word==MC_FAULT_ESTOP) ++estop_events;
    }
    if (event->kind==RM_EVENT_IMU_SAFETY) {
        assert(event->detail<=7u && event->from<=VSM_FAULT && event->to<=4u);
        imu_last_word=event->word;
        if ((event->word&0x8000u)==0u) {
            assert(event->word>=1u && event->word<=3u && event->from==VSM_FAULT);
            ++imu_stops;imu_stop_word=event->word;imu_stop_detail=event->detail;
        } else if ((event->word&0xffu)==8u) {
            assert(event->word==(0x8000u|(IMU_SAFETY_LIFT<<8)|8u));++imu_pending;
        } else if ((event->word&0xffu)==0u) {
            assert(event->word==(0x8000u|(IMU_SAFETY_LIFT<<8)));
            assert(imu_pending==1u);++imu_cleared;
        } else { ++imu_other; }
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
int telemetry_send(const telemetry_message_t *message) {
    if (shell_fault_case != 0u && message->kind == TELEMETRY_EVENT &&
        message->data.event.kind == 4u) {
        uint32_t faults = UINT32_MAX;
        assert(vehicle_fault_mask_try_read(&faults));
        assert(faults == (shell_fault_case == 1u ? FAULT_SHELL_OTHER : 0u));
        assert(message->data.event.word == 123u); /* Detail stays separate. */
        shell_fault_seen = true;
    }
    return base_telemetry_send(message);
}
bool drv_motor_hw_button_down(unsigned int pin) {
    if (capture_time_limit) return pin == 21u && now >= 20u && now < 50u;
    if (barcode_case == 7u || barcode_case == 9u) {
        return pin == 21u && ((now >= 20u && now < 50u) ||
            (now >= 5000u && now < 5050u) ||
            (now >= 6500u && now < 6550u) ||
            (now >= 12000u && now < 12050u));
    }
    if (mode==1u || mode==5u) return false; /* No physical start in uncalibrated START case. */
    if (mode==6u) return pin==21u && ((now>=20u && now<50u) ||
                                      (now>=3000u && now<3050u));
    if (mode>=2u) return pin==21u && now>=20u && now<50u;
    return base_vehicle_button(pin);
}
ER tk_slp_tsk(TMO timeout) {
    const ER result=base_vehicle_sleep(timeout);
    vehicle_snapshot_t value;assert(vehicle_snapshot_try_read(&value)==E_OK);
    if (shell_fault_case != 0u) return result;
    if ((mode == 3u || mode == 4u || mode == 6u) &&
        value.state == VSM_FAULT) {
        uint32_t faults = 0u;
        assert(vehicle_fault_mask_try_read(&faults));
        assert(faults == FAULT_IMU_SAFETY);
        observed_imu_latch = true;
    }
    if (mode==5u) {
        if (!injected && now>=20u) {
            ordinary[command_head++%8u]=(robot_command_t){RM_CMD_DUMP,100u};
            injected=true;
        }
        if (now>=100u) assert(dump_requests==1u && !value.active);
    } else if (mode==1u) {
        if (!injected && now>=20u) { ordinary[command_head++%8u]=(robot_command_t){RM_CMD_START,1u};injected=true; }
        if (now>=40u) assert(!value.active && !value.calibrated && !calibrating && line_drives==0u);
    } else if (value.active && value.phase==3u && !injected) {
        injected=true;
        if (mode==3u || mode==6u) { imu_lift_test=true; }
        if (mode>=3u) { estop_latch_test=true; }
        if (mode==4u) { imu_stale_test=true; }
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
    static const char *const cases[] = {"", "barcode", "barcode_absent",
        "barcode_busy", "barcode_old", "barcode_saturate", "barcode_zero",
        "barcode_runs", "barcode_final_busy", "barcode_runs_absent"};
    for (unsigned int i = 1u; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        if (argc > 1 && !strcmp(argv[1], cases[i])) { barcode_case = i; }
    }
    if (barcode_case == 7u || barcode_case == 9u) {
        test_end_ms = 14000u;
    }
    if (argc>1 && !strcmp(argv[1],"uncalibrated")) mode=1u;
    else if (argc>1 && !strcmp(argv[1],"estop")) mode=2u;
    else if (argc>1 && !strcmp(argv[1],"imulift")) mode=3u;
    else if (argc>1 && !strcmp(argv[1],"imustale")) mode=4u;
    else if (argc>1 && !strcmp(argv[1],"imurecover")) {
        mode=6u;imu_recover_test=true;
    }
    else if (argc>1 && !strcmp(argv[1],"imuabsent")) {
        mode=5u;imu_absent_test=true;
    }
    if (argc > 1 && !strcmp(argv[1], "capturetime")) {
        capture_time_limit = true;
        test_end_ms = 95000u;
    }
    if (argc > 1 && !strcmp(argv[1], "shellother")) shell_fault_case = 1u;
    if (argc > 1 && !strcmp(argv[1], "shellmotion")) shell_fault_case = 2u;
    if (argc > 1 && !strcmp(argv[1], "shellimu")) shell_fault_case = 3u;
    uint32_t faults = UINT32_C(0xdeadbeef);
    assert(!vehicle_fault_mask_try_read(NULL));
    assert(!vehicle_fault_mask_try_read(&faults));
    assert(faults == UINT32_C(0xdeadbeef));
    lp_init(&line);lp_set_pair(&line,true,0);watchdog_ready=true;services_ready=true;
    assert(vehicle_task_start()==E_OK && starts==1u);
    if (setjmp(done)==0) ((void (*)(INT,void *))entry)(0,0);
    assert((vehicle_run_epoch() & 1u) == 0u);
    assert(stop_captures == vehicle_run_epoch() / 2u);
    if (capture_time_limit) {
        assert(stop_captures == 1u && captured_vehicle_faults == 0u);
        assert(!strcmp(last_reason, "90 s time limit"));
        return 0;
    }
    if (mode != 1u && mode != 5u) {
        assert(stop_captures != 0u);
    }
    assert(vehicle_fault_mask_try_read(&faults));
    assert(faults == ((mode == 3u || mode == 4u) ? FAULT_IMU_SAFETY : 0u));
    /* No reader changes storage, consumes a latch or alters the clear policy. */
    for (unsigned int i = 0u; i < 20u; ++i) {
        uint32_t again = ~faults;
        assert(vehicle_fault_mask_try_read(&again) && again == faults);
    }
    if (shell_fault_case != 0u) {
        assert(shell_fault_seen && faults == 0u);
        return 0;
    }
    if (mode==5u) {
        assert(injected && dump_requests==1u && estops==0u && line_drives==0u);
        return 0;
    }
    if (mode>=3u) {
        assert(observed_imu_latch);
        vehicle_snapshot_t snapshot;
        rm_summary_t summary;
        assert(injected && estops==1u && asked[0]==0 && asked[1]==0);
        assert(vehicle_snapshot_try_read(&snapshot)==E_OK);
        /* One kind 10 STOP per latch edge, with trigger and cause. */
        assert(imu_stops==1u && imu_other==0u && estop_events==1u);
        assert(imu_stop_word==(mode==4u ? IMU_SAFETY_IMU_UNAVAILABLE :
                                          IMU_SAFETY_LIFT));
        assert(imu_stop_detail==(mode==4u ? IMU_SAFETY_CAUSE_STALE : 0u));
        if (mode==6u) {
            /* One PENDING per press, then exactly one CLEARED. */
            assert(imu_pending==1u && imu_cleared==1u);
            assert(imu_last_word==(0x8000u|(IMU_SAFETY_LIFT<<8)));
            assert(!snapshot.active && snapshot.state==VSM_IDLE);
        } else {
            assert(imu_pending==0u && imu_cleared==0u);
            assert(!snapshot.active && snapshot.state==VSM_FAULT);
        }
        assert(vehicle_summary_try_read(&summary)==E_OK);
        assert(summary.stop_reason==(mode==4u ? 16u : 14u));
        assert(summary.shell_fault==VSM_FAULT_IMU);
        return 0;
    }
    if (barcode_case != 0u) {
        rm_summary_t summary;
        assert(barcode_reads > 0u);
        assert(vehicle_summary_try_read(&summary) == E_OK);
        const bool available = barcode_case != 2u && barcode_case != 3u &&
                               barcode_case != 4u && barcode_case != 9u;
        assert(((summary.flags & RM_SUMMARY_BARCODE_IMPLEMENTED) != 0u)
               == available);
        const uint8_t ok = !available || barcode_case == 6u ? 0u :
            barcode_case == 5u ? UINT8_MAX : barcode_case == 7u ? 4u : 7u;
        const uint8_t failed = !available || barcode_case == 6u ? 0u :
            barcode_case == 5u ? UINT8_MAX : barcode_case == 7u ? 2u : 3u;
        assert(summary.barcode_ok == ok && summary.barcode_failed == failed);
        if (barcode_case == 7u || barcode_case == 9u) {
            assert(summary.run_id == 2u);
            assert(vehicle_run_epoch() > first_barcode_epoch + 1u);
        }
        return 0;
    }
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

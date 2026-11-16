/* SIL-owned SDK/acquisition glue; unchanged native reader, shell and actions.
 * No valid/OFFSET_VERIFIED flags or commands are manufactured. */
#define sil_reset sil_motor_baseline_reset
#include "motor_adapter.c"
#undef sil_reset
#include "app_program/subsystem3/line_search.h"
#include "app_program/subsystem3/barcode_live_producer.h"
#include "app_program/vehicle/junction_line.h"
#include "app_program/vehicle/junction_live.h"
static lt_t trace;
static ls_t search;
static jl_t junction;
static vsm_t shell;
static jlive_t live;
static barcode_live_producer_t producer;
static barcode_calibration_t bc_cal[3];
static unsigned int bc_family;
static barcode_live_control_t control_copy;
static uint32_t control_tag;
static vc_event_t pending_line_event;
static uint32_t found_by_search;
static uint32_t motor_us;
static int32_t motor_counts[2];
static vsm_drive_t drive;
static uint32_t attempts,quiet_pass,start_pass,decoded,frame_admission;
static uint32_t previous_window_serial;
static uint32_t event_count,event_full;
static int32_t events[128][14];

/* Replacement for the SDK publication call: synchronous coherent value + tag.
 * The actual jlive code remains the only control writer. */
int barcode_live_control_publish(const barcode_live_control_t *value) {
    control_copy=*value;control_tag+=2u;return 0;
}
void sil_reset(void) {
    sil_motor_baseline_reset();
    trace=(lt_t){0};search=(ls_t){0};junction=(jl_t){0};producer=(barcode_live_producer_t){0};
    for (unsigned int i=0u;i<3u;++i) bc_cal[i]=(barcode_calibration_t){0};
    control_copy=(barcode_live_control_t){0};control_tag=0u;bc_family=0u;
    pending_line_event=VC_EVENT_NONE;found_by_search=0u;motor_us=0u;
    motor_counts[0]=0;motor_counts[1]=0;
    drive=(vsm_drive_t){0,0,0,VSM_ACT_STOP};
    attempts=0u;quiet_pass=0u;start_pass=0u;decoded=0u;frame_admission=0u;previous_window_serial=0u;
    event_count=0u;event_full=0u;
    lt_init(&trace,&LINE_CFG);ls_init(&search,&SEARCH_CFG);jl_init(&junction,&trace,SEARCH_CFG.speed_mm_s);
    ja_init(&junction.action,SEARCH_CFG.speed_mm_s,1u);
    vsm_init(&shell,VSM_DEFAULT_ROUTES,VSM_DEFAULT_ROUTE_COUNT);
    (void)vsm_set_module(&shell,VSM_MOD_LINE,jl_module(&junction));
    (void)vsm_set_module(&shell,VSM_MOD_SEARCH,ls_module(&search));
    jlive_start(&live,1u,0u);(void)vsm_start(&shell,VSM_MOD_LINE,0u,0u);
}
void sil_bc_select(uint32_t family) { if (family<3u) bc_family=family; }
void sil_bc_cal_begin(void) {
    for (unsigned int i=0u;i<3u;++i) barcode_calibration_begin(&bc_cal[i]);
}
void sil_bc_cal_add(uint32_t family,uint32_t us,uint16_t raw) {
    if (family<3u) barcode_calibration_add(&bc_cal[family],us,raw);
}
void sil_bc_cal_end(int32_t *out) {
    for (unsigned int i=0u;i<3u;++i) {
        barcode_calibration_end(&bc_cal[i],true);
        const barcode_cal_report_t *r=&bc_cal[i].report;
        const uint32_t v[14]={r->generation,r->samples,r->minimum,r->maximum,r->white_samples,
            r->black_samples,r->white_mean,r->black_mean,r->black_hold_us,r->max_gap_us,
            r->rise,r->fall,r->mode,r->faults};
        for (unsigned int j=0u;j<14u;++j) out[i*14u+j]=(int32_t)v[j];
    }
}
void sil_search_config(int32_t *out) {
    out[0]=SEARCH_CFG.speed_mm_s;out[1]=SEARCH_CFG.turn_counts;
    out[2]=LINE_CFG.straight_counts;out[3]=LINE_CFG.hint_counts;
}
static void reader_event(uint32_t ms) {
    const barcode_window_t *w=&producer.window;
    if (w->result.serial!=0u && w->result.serial!=previous_window_serial) {
        ++attempts;
        if (event_count<128u) {
            int32_t *r=events[event_count++];
            r[0]=(int32_t)ms;r[1]=(int32_t)w->result.serial;r[2]=(int32_t)w->result.reason;
            r[3]=w->result.letter;r[4]=w->result.accepted;r[5]=(int32_t)w->result.start_us;
            r[6]=(int32_t)w->result.read_us;r[7]=w->result.rise;r[8]=w->result.fall;
            r[9]=(int32_t)(w->result.leading_mm*1000.f);r[10]=(int32_t)(w->result.narrow_mm*1000.f);
            r[11]=(int32_t)(w->result.wide_mm*1000.f);r[12]=(int32_t)producer.failures;
            r[13]=(int32_t)producer.publications;
        } else event_full=1u;
    }
    previous_window_serial=w->result.serial;
    /* direction is set by native start_star only AFTER fitted10n quiet proof. */
    if (w->direction!=0u) {quiet_pass=1u;start_pass=1u;}
    if (producer.event_pending) producer.event_pending=false;
    if (producer.record_pending) {
        decoded=producer.slot.letter;producer.record_pending=false;
    }
}
void sil_barcode_update(uint16_t left,uint16_t centre,uint32_t ms,int32_t left_count,int32_t right_count,uint16_t raw_bc,int32_t *out) {
    const uint16_t raw[3]={left,centre,0};const int32_t counts[2]={left_count,right_count};
    const vc_event_t event=lp_update(&lp,raw,ms);
    if (vc_event_type(event)==VC_EVENT_LINE_LOST || vc_event_type(event)==VC_EVENT_LINE_FOUND) pending_line_event=event;
    if (ms%10u==0u) {
        mc_tick(&mc,counts,ms,&output);motor_us=ms*1000u;motor_counts[0]=left_count;motor_counts[1]=right_count;
    }
    const barcode_live_input_t input={.calibration=&bc_cal[bc_family].report,.control=control_copy,
        .tag=control_tag,.now_us=ms*1000u,.now_ms=ms,.adc_us=ms*1000u,.motor_us=motor_us,
        .left=(uint32_t)motor_counts[0],.right=(uint32_t)motor_counts[1],.motor_faults=mc_faults(&mc),
        .missed=0u,.dropped=0u,.raw=raw_bc,.control_ok=true,.motor_ok=true};
    barcode_live_producer_observe(&producer,&input);reader_event(ms);
    if (ms%10u==0u) {
        junction.sample=(ja_sample_t){.line_stamp=ms*1000u,.line_age_ms=0u,.valid=true};
        const vsm_inputs_t in={.line=&lp,.line_event=pending_line_event,.counts={left_count,right_count},
            .echo_new=false,.echo=0,.ping_ms=0,.ping_sent=false};
        const uint32_t before=shell.handoffs;
        drive=vsm_step(&shell,&in,(uint8_t)(mc_faults(&mc)&MC_FAULT_LATCHED),ms);
        pending_line_event=VC_EVENT_NONE;
        if (shell.handoffs!=before && shell.last.from==VSM_MOD_SEARCH && vsm_outcome_type(shell.last.outcome)==VSM_OUT_FOUND) ++found_by_search;
        if (shell.state==VSM_RUNNING && (shell.running==VSM_MOD_LINE || shell.running==VSM_MOD_SEARCH)) ja_cross_progress(&junction.action,counts,ms);
        const uint8_t module=shell.state==VSM_RUNNING?shell.running:VSM_MOD_NONE;
        jlive_route(&live,module,ms);
        jlive_tick(&live,&lp,&junction.sample,counts,ms,&junction.action,junction.action.phase==JA_PENDING || ja_active(&junction.action));
        ja_command_t command;
        if (jlive_read(&live,producer.occupied?&producer.slot:0,ms,&command)) {
            const bool accepted=ja_offer(&junction.action,&command);jlive_disposition(&live,command.sequence,accepted);
        }
        if (producer.occupied && producer.slot.sequence!=0u && live.control.ack_sequence==producer.slot.sequence)
            frame_admission=live.report.reason;
        desired[0]=drive.left_mm_s;desired[1]=drive.right_mm_s;
        if (drive.action==VSM_ACT_STOP) {mc_stop(&mc);output.duty_permille[0]=0;output.duty_permille[1]=0;}
        else mc_set_speed(&mc,desired[0],desired[1],ms);
        fault=shell.state==VSM_FAULT;
    }
    out[0]=lp.position;out[1]=(int32_t)event;out[2]=lp_line_seen(&lp);out[3]=lp_junction(&lp);
    out[4]=fault?0:output.duty_permille[0];out[5]=fault?0:output.duty_permille[1];
    out[6]=lp.norm[0];out[7]=lp.norm[1];out[8]=fault;
    out[9]=desired[0];out[10]=desired[1];out[11]=mc_faults(&mc);
    out[12]=shell.running;out[13]=shell.state;out[14]=shell.fault;out[15]=shell.fault_detail;
    out[16]=(int32_t)shell.handoffs;out[17]=(int32_t)search.searches;out[18]=(int32_t)found_by_search;
    out[19]=shell.last.outcome;out[20]=drive.action;
    out[21]=raw_bc;out[22]=(int32_t)attempts;out[23]=(int32_t)decoded;
    out[24]=(int32_t)producer.publications;out[25]=(int32_t)producer.failures;
    out[26]=(int32_t)producer.window.result.reason;out[27]=(int32_t)quiet_pass;out[28]=(int32_t)start_pass;
    out[29]=producer.slot.valid;out[30]=producer.slot.flags;out[31]=(int32_t)frame_admission;
    out[32]=live.reasons[JLIVE_CONFIRMED];out[33]=junction.action.phase;
    out[34]=junction.action.command.command;out[35]=(int32_t)event_full;
}
void sil_bc_events(int32_t *out) {
    out[0]=(int32_t)event_count;
    for (unsigned int i=0u;i<event_count;++i) for (unsigned int j=0u;j<14u;++j) out[1u+i*14u+j]=events[i][j];
}

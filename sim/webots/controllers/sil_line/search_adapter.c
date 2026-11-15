/* SIL ABI: actual shell + LINE/SEARCH only. No module stubs. */
#define sil_reset sil_motor_baseline_reset
#include "motor_adapter.c"
#undef sil_reset
#include "app_program/subsystem3/line_search.h"
static lt_t trace;
static ls_t search;
static vsm_t shell;
static vc_event_t pending_line_event;
static uint32_t found_by_search;
static vsm_drive_t drive;

void sil_reset(void) {
    sil_motor_baseline_reset();
    lt_init(&trace,&LINE_CFG);ls_init(&search,&SEARCH_CFG);
    vsm_init(&shell,VSM_DEFAULT_ROUTES,VSM_DEFAULT_ROUTE_COUNT);
    (void)vsm_set_module(&shell,VSM_MOD_LINE,lt_module(&trace));
    (void)vsm_set_module(&shell,VSM_MOD_SEARCH,ls_module(&search));
    (void)vsm_start(&shell,VSM_MOD_LINE,0u,0u);
    pending_line_event=VC_EVENT_NONE;found_by_search=0u;
    drive=(vsm_drive_t){0,0,0,VSM_ACT_STOP};
}
static void shell_step(uint32_t ms,const int32_t counts[2],vc_event_t event,uint8_t motor_fault) {
    const vsm_inputs_t in={.line=&lp,.line_event=event,.counts={counts[0],counts[1]},
        .echo_new=false,.echo=0,.ping_ms=0,.ping_sent=false};
    const uint32_t before=shell.handoffs;
    drive=vsm_step(&shell,&in,motor_fault,ms);
    if (shell.handoffs!=before && shell.last.from==VSM_MOD_SEARCH &&
        vsm_outcome_type(shell.last.outcome)==VSM_OUT_FOUND) ++found_by_search;
    desired[0]=drive.left_mm_s;desired[1]=drive.right_mm_s;
    if (drive.action==VSM_ACT_STOP) {
        mc_stop(&mc);output.duty_permille[0]=0;output.duty_permille[1]=0;
    } else mc_set_speed(&mc,desired[0],desired[1],ms);
    fault=shell.state==VSM_FAULT;
}
static void shell_result(vc_event_t event,int32_t *out) {
    out[0]=lp.position;out[1]=(int32_t)event;out[2]=lp_line_seen(&lp);out[3]=lp_junction(&lp);
    out[4]=fault?0:output.duty_permille[0];out[5]=fault?0:output.duty_permille[1];
    out[6]=lp.norm[0];out[7]=lp.norm[1];out[8]=fault;
    out[9]=desired[0];out[10]=desired[1];out[11]=mc_faults(&mc);
    out[12]=shell.running;out[13]=shell.state;out[14]=shell.fault;out[15]=shell.fault_detail;
    out[16]=(int32_t)shell.handoffs;out[17]=(int32_t)search.searches;
    out[18]=(int32_t)found_by_search;out[19]=shell.last.outcome;out[20]=drive.action;
}
void sil_search_update(uint16_t left,uint16_t centre,uint32_t ms,int32_t left_count,int32_t right_count,int32_t *out) {
    const uint16_t raw[3]={left,centre,0};const int32_t counts[2]={left_count,right_count};
    const vc_event_t event=lp_update(&lp,raw,ms);
    if (vc_event_type(event)==VC_EVENT_LINE_LOST || vc_event_type(event)==VC_EVENT_LINE_FOUND)
        pending_line_event=event;
    if (ms%10u==0u) {
        mc_tick(&mc,counts,ms,&output);
        shell_step(ms,counts,pending_line_event,(uint8_t)(mc_faults(&mc)&MC_FAULT_LATCHED));
        pending_line_event=VC_EVENT_NONE;
    }
    shell_result(event,out);
}
/* Host test seam accepts detector events via the same public module input.
 * It does not alter lp fields, shell state or search counters. */
void sil_search_module_step(uint32_t ms,int32_t left_count,int32_t right_count,uint16_t event,uint8_t motor_fault,int32_t *out) {
    const int32_t counts[2]={left_count,right_count};
    shell_step(ms,counts,event,motor_fault);shell_result(event,out);
}
void sil_search_config(int32_t *out) {
    out[0]=SEARCH_CFG.speed_mm_s;out[1]=SEARCH_CFG.turn_counts;
    out[2]=LINE_CFG.straight_counts;out[3]=LINE_CFG.hint_counts;
}

/* Pure action and locked-LINE adapter fixtures. No motor hardware. */
#include <assert.h>
#include <limits.h>
#include <string.h>
#include "../app_program/vehicle/junction_line.h"
#include "../app_program/vehicle/junction_source.h"

static const ja_config_t CFG = {123u,18u};
typedef struct {
    ja_t action;lp_t lp;vsm_inputs_t in;ja_sample_t sample;
    vsm_drive_t drive;uint32_t now;int32_t remainder[2];
} fixture_t;
static void line(fixture_t *f,bool cross) {
    f->lp.flags=LP_FLAG_CALIBRATED|LP_FLAG_LINE_SEEN;
    f->lp.flags|=cross ? LP_FLAG_JUNCTION : LP_FLAG_CENTRED;
    f->lp.norm[0]=f->lp.norm[1]=cross ? 1000u : 400u;
    f->lp.pair=true;
}
static void init(fixture_t *f,uint8_t command) {
    *f=(fixture_t){0};ja_init(&f->action,&CFG,1u);f->in.line=&f->lp;
    f->sample.valid=true;line(f,false);
    const ja_command_t c={.epoch=1u,.sequence=1u,.command=command,
                          .valid=1u,.source=1u};
    assert(ja_offer(&f->action,&c));
}
static bool tick(fixture_t *f,int32_t dl,int32_t dr) {
    f->now+=10u;f->sample.line_stamp=f->now;
    f->in.counts[0]=(int32_t)((uint32_t)f->in.counts[0]+(uint32_t)dl);
    f->in.counts[1]=(int32_t)((uint32_t)f->in.counts[1]+(uint32_t)dr);
    return ja_step(&f->action,&f->in,&f->sample,f->now,&f->drive);
}
static void claim(fixture_t *f) {
    for (unsigned int i=0u;i<12u;++i) assert(!tick(f,10,10));
    assert(f->action.clear);line(f,true);
    for (unsigned int i=0u;i<20u && !ja_active(&f->action);++i) (void)tick(f,10,10);
    assert(ja_active(&f->action) && f->action.result.claimed==1u);
    assert(f->action.consumed==1u && !f->action.command.valid);
}
static int32_t difference(int32_t a,int32_t b) {
    return (int32_t)((uint32_t)a-(uint32_t)b);
}
static void run_command(uint8_t command,bool wrap) {
    fixture_t f;init(&f,command);
    if (wrap) {
        f.in.counts[0]=f.in.counts[1]=INT32_MAX-20;
        f.now=UINT32_MAX-1000u;f.action.command.read_ms=f.now;
    }
    claim(&f);const int32_t edge=f.action.result.edge_counts[0];
    for (unsigned int t=0u;t<1300u && ja_active(&f.action);++t) {
        line(&f,false);
        /* A stale FOUND on the old stem must NOT end an encoder turn. */
        f.in.line_event=vc_event(VC_EVENT_LINE_FOUND,0u);
        const int16_t speeds[2]={f.drive.left_mm_s,f.drive.right_mm_s};
        int32_t step[2];
        for (unsigned int i=0u;i<2u;++i) {
            f.remainder[i]+=(int32_t)speeds[i]*(int32_t)JA_COUNTS_PER_M;
            step[i]=f.remainder[i]/100000;f.remainder[i]%=100000;
            f.sample.speed_mm_s[i]=speeds[i];
        }
        assert(tick(&f,step[0],step[1]));
    }
    assert(f.action.phase==JA_DONE && f.action.result.reason==JA_OK);
    assert(f.action.result.marks==7u);
    assert(f.action.result.elapsed_ms<12000u);
    const int32_t travelled=difference(f.action.result.centre_counts[0],edge);
    assert(travelled>=1639 && travelled<=1660);
    const int32_t turn=f.action.result.turn_cdeg;
    if (command==JA_LEFT) assert(turn>=9000 && turn<9100);
    if (command==JA_RIGHT) assert(turn<=-9000 && turn>-9100);
    if (command==JA_UTURN) assert(turn>=18000 && turn<18100);
    if (command==JA_STRAIGHT) {
        assert(turn==0);
        assert(difference(f.action.result.final_counts[0],edge)>=1987);
    }
    /* The same source record cannot cause a second action. */
    const ja_command_t old={.epoch=1u,.sequence=1u,.valid=1u,
                            .command=command,.source=1u};
    assert(!ja_offer(&f.action,&old));
    line(&f,true);for (unsigned int i=0u;i<40u;++i) assert(!tick(&f,10,10));
    assert(f.action.phase==JA_DONE);
}
static void detector(void) {
    fixture_t f;init(&f,JA_LEFT);
    line(&f,true);for (unsigned int i=0u;i<60u;++i) assert(!tick(&f,10,10));
    assert(!f.action.clear); /* Already on the cross at START: NEXT only. */
    line(&f,false);for (unsigned int i=0u;i<12u;++i) assert(!tick(&f,10,10));
    assert(f.action.clear);
    line(&f,true);assert(!tick(&f,10,10));
    for (unsigned int i=0u;i<30u;++i) assert(!tick(&f,0,0));
    assert(!f.action.candidate && !f.action.clear);
    line(&f,false);for (unsigned int i=0u;i<12u;++i) (void)tick(&f,10,10);
    line(&f,true);assert(!tick(&f,10,10));
    assert(!tick(&f,-1,0) && !f.action.clear);
    line(&f,false);for (unsigned int i=0u;i<12u;++i) (void)tick(&f,10,10);
    /* One dark channel plus weak channel/barcode-like pulses do not claim. */
    line(&f,true);f.lp.norm[1]=699u;
    for (unsigned int i=0u;i<80u;++i) assert(!tick(&f,10,10));
    assert(!f.action.candidate);
    line(&f,false);for (unsigned int i=0u;i<12u;++i) (void)tick(&f,10,10);
    for (unsigned int n=0u;n<20u;++n) {
        line(&f,true);for (unsigned int i=0u;i<5u;++i) assert(!tick(&f,10,10));
        line(&f,false);assert(!tick(&f,10,10));
    }
    /* 50-count black spans are shorter than N=75 counts. */
    assert(!f.action.result.claimed);
    line(&f,true);(void)tick(&f,10,10);
    const uint32_t stamp=f.sample.line_stamp;
    f.now+=10u;f.in.counts[0]+=100;f.in.counts[1]+=100;
    assert(!ja_step(&f.action,&f.in,&f.sample,f.now,&f.drive));
    assert(f.sample.line_stamp==stamp && !f.action.candidate);
    ja_suspend(&f.action);assert(!f.action.have_sample);
    /* A real cross rises through the global 1250 sum BEFORE both700.
     * That ramp must not disarm the already-clear approach. */
    init(&f,JA_LEFT);
    for (unsigned int i=0u;i<12u;++i) (void)tick(&f,10,10);
    line(&f,true);f.lp.norm[0]=f.lp.norm[1]=650u;
    for (unsigned int i=0u;i<5u;++i) assert(!tick(&f,10,10));
    assert(f.action.clear);f.lp.norm[0]=f.lp.norm[1]=700u;
    for (unsigned int i=0u;i<10u && !ja_active(&f.action);++i) (void)tick(&f,10,10);
    assert(ja_active(&f.action));
}
static void failures(void) {
    fixture_t f;init(&f,JA_LEFT);claim(&f);
    for (unsigned int i=0u;i<51u && ja_active(&f.action);++i) (void)tick(&f,0,0);
    assert(f.action.phase==JA_FAILED && f.action.result.reason==JA_STALL);
    assert(!f.drive.left_mm_s && !f.drive.right_mm_s);
    init(&f,JA_LEFT);claim(&f);f.sample.valid=false;(void)tick(&f,10,10);
    assert(f.action.result.reason==JA_STALE);
    init(&f,JA_LEFT);claim(&f);f.sample.line_age_ms=51u;(void)tick(&f,10,10);
    assert(f.action.result.reason==JA_STALE);
    init(&f,JA_LEFT);claim(&f);(void)tick(&f,-200,-200);
    assert(f.action.result.reason==JA_REVERSE);
    init(&f,JA_LEFT);claim(&f);(void)tick(&f,1000,100);
    assert(f.action.result.reason==JA_WHEEL_IMBALANCE);
    init(&f,JA_LEFT);claim(&f);f.now+=3000u;(void)tick(&f,1,1);
    assert(f.action.result.reason==JA_APPROACH_LIMIT);
    init(&f,JA_LEFT);claim(&f);f.now+=10000u;(void)tick(&f,1,1);
    assert(f.action.result.reason==JA_TOTAL_LIMIT);
    init(&f,JA_LEFT);claim(&f);ja_finish(&f.action,f.in.counts,f.now);
    assert(f.action.phase==JA_FAILED && f.action.result.reason==JA_CANCELLED);
    init(&f,JA_LEFT);ja_finish(&f.action,f.in.counts,10u);
    assert(f.action.phase==JA_UNUSED && !f.action.result.claimed);
    init(&f,JA_LEFT);claim(&f);f.in.echo_new=true;
    f.in.echo=60u;
    (void)tick(&f,10,10);(void)tick(&f,10,10);
    assert(f.action.result.reason==JA_OBSTACLE);
}
static void prepare_turn(fixture_t *f) {
    init(f,JA_LEFT);claim(f);line(f,false);
    while (f->action.phase==JA_APPROACH) (void)tick(f,10,10);
    assert(f->action.phase==JA_SETTLE);
    for (unsigned int i=0u;i<11u;++i) (void)tick(f,0,0);
    assert(f->action.phase==JA_TURN);
}
static void turn_and_acquire(void) {
    fixture_t f;prepare_turn(&f);(void)tick(&f,1300,-1300);
    assert(f.action.result.reason==JA_REVERSE);
    prepare_turn(&f);(void)tick(&f,-1400,1400);
    assert(f.action.result.reason==JA_TURN_LIMIT);
    prepare_turn(&f);f.now+=4000u;(void)tick(&f,-1,1);
    assert(f.action.result.reason==JA_TURN_LIMIT);
    prepare_turn(&f);(void)tick(&f,-1082,1083);
    assert(f.action.phase==JA_TURN_SETTLE);
    for (unsigned int i=0u;i<11u;++i) (void)tick(&f,0,0);
    assert(f.action.phase==JA_REACQUIRE);
    /* No outgoing line: ignore a stale global FOUND and sweep both sides. */
    f.lp.flags&=(uint16_t)~LP_FLAG_CENTRED;f.in.line_event=vc_event(VC_EVENT_LINE_FOUND,0u);
    unsigned int reversed=0u;
    while (ja_active(&f.action)) {
        (void)tick(&f,f.drive.left_mm_s<0 ? -7 : 7,f.drive.right_mm_s>0 ? 7 : -7);
        if (f.drive.left_mm_s>0) ++reversed;
    }
    assert(reversed && f.action.result.reason==JA_NO_LINE);
    /* A persistent line level, not an edge event, is sufficient. */
    prepare_turn(&f);(void)tick(&f,-1082,1083);
    for (unsigned int i=0u;i<11u;++i) (void)tick(&f,0,0);
    f.in.line_event=VC_EVENT_NONE;
    /* LP can keep SEEN and CENTRED on weak floor during LOST grace. */
    f.lp.norm[0]=f.lp.norm[1]=100u;
    for (unsigned int i=0u;i<4u;++i) (void)tick(&f,0,0);
    assert(f.action.phase==JA_REACQUIRE);
    f.lp.norm[0]=f.lp.norm[1]=400u;
    for (unsigned int i=0u;i<4u;++i) (void)tick(&f,0,0);
    assert(f.action.phase==JA_DONE);
    /* Settling is bounded even when wheel-speed observations stay high. */
    prepare_turn(&f);(void)tick(&f,-1082,1083);f.sample.speed_mm_s[0]=80;
    f.now+=500u;(void)tick(&f,0,0);
    assert(f.action.result.reason==JA_SETTLE_LIMIT);
    prepare_turn(&f);(void)tick(&f,-1082,1083);(void)tick(&f,-400,400);
    assert(f.action.result.reason==JA_TURN_LIMIT); /* Coast cap, before settle. */
    init(&f,JA_LEFT);claim(&f);line(&f,false);
    while (f.action.phase==JA_APPROACH) (void)tick(&f,10,10);
    (void)tick(&f,500,500);assert(f.action.result.reason==JA_APPROACH_LIMIT);
    init(&f,JA_LEFT);claim(&f);line(&f,false);
    while (f.action.phase==JA_APPROACH) (void)tick(&f,10,10);
    (void)tick(&f,-100,-100);assert(f.action.result.reason==JA_REVERSE);
    init(&f,JA_STRAIGHT);claim(&f);line(&f,false);f.lp.norm[0]=f.lp.norm[1]=100u;
    while (ja_active(&f.action)) (void)tick(&f,10,10);
    assert(f.action.result.reason==JA_APPROACH_LIMIT); /* No outgoing branch. */
}
static void source(void) {
    ja_command_t c={.epoch=999u};assert(!junction_source_read(NULL));
    assert(!junction_source_read(&c) && c.epoch==999u);
    assert(!junction_source_busy());junction_source_set_busy(true);
    assert(junction_source_busy());junction_source_set_busy(false);
    const int32_t counts[2]={INT32_MAX,-100};
    junction_source_start(JA_STRAIGHT,1u,42u,counts);
    assert(junction_source_read(&c) && c.valid==1u && c.command==JA_STRAIGHT);
    assert(c.read_counts[0]==INT32_MAX && c.read_counts[1]==-100 && c.read_ms==42u);
    fixture_t f;init(&f,JA_LEFT);assert(!ja_offer(&f.action,&c));
    ja_init(&f.action,&CFG,1u);assert(ja_offer(&f.action,&c));
    ja_command_t bad=c;bad.epoch=3u;assert(!ja_offer(&f.action,&bad));
    ja_init(&f.action,&CFG,1u);bad=c;bad.reserved=1u;assert(!ja_offer(&f.action,&bad));
    bad=c;bad.command=5u;assert(!ja_offer(&f.action,&bad));
    bad=c;bad.sequence=UINT32_MAX;assert(ja_offer(&f.action,&bad));
    ja_finish(&f.action,counts,100u);c.sequence=1u;assert(ja_offer(&f.action,&c));
    junction_source_acknowledge(c.sequence);assert(junction_source_acknowledged()==c.sequence);
    junction_source_start(JA_NONE,3u,99u,counts);
    assert(junction_source_read(&c) && !c.valid && c.epoch==3u);
    assert(!strcmp(junction_source_name(JA_UTURN),"D"));
}
static void unchanged_line(void) {
    const lt_config_t cfg={.steer={.base_mm_s=146,.gain_pct=85u,.kd_pct=85u,
        .min_mm_s=62,.max_mm_s=378,.far_from=50u,.far_gain_pct=340u},
        .straight_counts=450,.hint_counts=20};
    lt_t original,wrapped;lt_init(&original,&cfg);lt_init(&wrapped,&cfg);
    jl_t adapter;jl_init(&adapter,&wrapped,&CFG);
    lp_t a={0},b={0};lp_init(&a);lp_set_pair(&a,true,0);
    lp_calib_begin(&a);unsigned int bad;
    const uint16_t floor[3]={500u,500u,0u},ink[3]={3500u,3500u,0u};
    for (unsigned int i=0u;i<10u;++i) {lp_calib_add(&a,floor);lp_calib_add(&a,ink);}
    assert(lp_calib_end(&a,&bad)==LP_CALIB_OK);b=a;
    vsm_inputs_t ia={.line=&a},ib={.line=&b};
    lt_start(&original,0u,0u);jl_start(&adapter,0u,0u);
    for (unsigned int i=0u;i<1000u;++i) {
        uint16_t raw[3]={(uint16_t)(500u+i%9u*300u),
                         (uint16_t)(3500u-i%9u*300u),0u};
        if (i%125u<40u) raw[0]=raw[1]=500u; /* LOST/FOUND episodes. */
        else if (i%50u<8u) raw[0]=raw[1]=3500u; /* Uncommanded cross. */
        ia.line_event=lp_update(&a,raw,i*10u);ib.line_event=lp_update(&b,raw,i*10u);
        ia.counts[0]+=9;ia.counts[1]+=11;ib.counts[0]=ia.counts[0];ib.counts[1]=ia.counts[1];
        vsm_drive_t da={0},db={0};
        assert(lt_step(&original,&ia,i*10u,&da)==jl_step(&adapter,&ib,i*10u,&db));
        assert(!memcmp(&da,&db,sizeof(da)) && !memcmp(&a,&b,sizeof(a)));
    }
    ia.line_event=ib.line_event=vc_event(VC_EVENT_LINE_LOST,VC_LINE_SIDE_RIGHT);
    vsm_drive_t da={0},db={0};assert(lt_step(&original,&ia,10000u,&da)==jl_step(&adapter,&ib,10000u,&db));
}
int main(void) {
    _Static_assert(sizeof(ja_t)<=256u,"bounded action state");
    _Static_assert(sizeof(ja_command_t)==24u,"draft source record");
    _Static_assert(sizeof(ja_result_t)==48u,"result value copy");
    detector();failures();turn_and_acquire();source();unchanged_line();
    for (uint8_t c=JA_LEFT;c<=JA_UTURN;++c) {run_command(c,false);run_command(c,true);}
    return 0;
}

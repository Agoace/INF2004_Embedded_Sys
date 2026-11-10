/* Forced corner fixtures; no physical turn model or motor hardware. */
#include <assert.h>
#include <limits.h>
#include <string.h>
#include "../app_program/vehicle/junction_line.h"
#include "../app_program/vehicle/junction_source.h"
static const lt_config_t LINE={.steer={.base_mm_s=146,.gain_pct=85u,.kd_pct=85u,
    .min_mm_s=62,.max_mm_s=378,.far_from=50u,.far_gain_pct=340u},
    .straight_counts=450,.hint_counts=20};
typedef struct {
    ja_t action;lp_t lp;vsm_inputs_t in;ja_sample_t sample;
    vsm_drive_t drive;uint32_t now;
} fixture_t;
static void line(fixture_t *f,bool cross) {
    f->lp.flags=LP_FLAG_CALIBRATED|LP_FLAG_LINE_SEEN;
    f->lp.flags|=cross ? LP_FLAG_JUNCTION : LP_FLAG_CENTRED;
    f->lp.norm[0]=f->lp.norm[1]=cross ? 1000u : 400u;f->lp.pair=true;
}
static void init(fixture_t *f,uint8_t command) {
    *f=(fixture_t){0};ja_init(&f->action,131,1u);f->in.line=&f->lp;
    f->sample.valid=true;line(f,false);
    const ja_command_t c={.epoch=1u,.sequence=1u,.command=command,.valid=1u,.source=1u};
    assert(ja_offer(&f->action,&c));
}
static bool tick(fixture_t *f,int32_t dl,int32_t dr) {
    f->now+=10u;f->sample.line_stamp=f->now;
    f->in.counts[0]=(int32_t)((uint32_t)f->in.counts[0]+(uint32_t)dl);
    f->in.counts[1]=(int32_t)((uint32_t)f->in.counts[1]+(uint32_t)dr);
    return ja_step(&f->action,&f->in,&f->sample,f->now,&f->drive);
}
static int32_t diff(int32_t a,int32_t b) {return (int32_t)((uint32_t)a-(uint32_t)b);}
static void claim(fixture_t *f) {
    for (unsigned int i=0u;i<12u;++i) assert(!tick(f,10,10));
    assert(f->action.clear);line(f,true);
    for (unsigned int i=0u;i<12u && !ja_active(&f->action);++i) (void)tick(f,10,10);
    assert(ja_active(&f->action) && f->action.result.claimed && f->action.consumed==1u);
}
static void begin(fixture_t *f) {
    claim(f);line(f,false);
    while (!ja_searching(&f->action)) {
        f->drive=(vsm_drive_t){.left_mm_s=101,.right_mm_s=149};
        if (!tick(f,10,10)) assert(f->drive.left_mm_s==101 && f->drive.right_mm_s==149);
    }
    const int32_t d=diff(f->action.result.search_counts[0],f->action.result.edge_counts[0]);
    assert(d>=410 && d<=430); /*33mm and two fresh clear observations.*/
    assert(f->drive.left_mm_s==-f->drive.right_mm_s);
    assert(f->drive.left_mm_s==(f->action.result.command==JA_RIGHT ? 131 : -131));
}
static void white(fixture_t *f) {
    f->lp.norm[0]=f->lp.norm[1]=0u;
    const int d=f->action.result.command==JA_RIGHT ? -1 : 1;
    for (unsigned int i=0u;i<3u;++i) (void)tick(f,-d,d);
    assert(f->action.departed);
}
static void found(fixture_t *f,int32_t angle) {
    const int32_t dc=(int32_t)((int64_t)angle*JA_COUNTS_PER_DEG_X100/10000);
    f->in.counts[0]=(int32_t)((uint32_t)f->action.result.search_counts[0]-(uint32_t)(dc/2));
    f->in.counts[1]=(int32_t)((uint32_t)f->action.result.search_counts[1]+(uint32_t)(dc-dc/2));
    f->lp.norm[0]=500u;f->lp.norm[1]=0u;
    (void)tick(f,0,0);
}
static void detector(void) {
    fixture_t f;init(&f,JA_LEFT);line(&f,true);
    for (unsigned int i=0u;i<60u;++i) assert(!tick(&f,10,10));
    assert(!f.action.clear);line(&f,false);
    for (unsigned int i=0u;i<12u;++i) (void)tick(&f,10,10);
    line(&f,true);f.lp.norm[0]=f.lp.norm[1]=650u;
    for (unsigned int i=0u;i<5u;++i) assert(!tick(&f,10,10));
    assert(f.action.clear);f.lp.norm[0]=f.lp.norm[1]=700u;
    for (unsigned int i=0u;i<8u;++i) (void)tick(&f,10,10);
    assert(!ja_active(&f.action));(void)tick(&f,10,10);
    assert(f.action.phase==JA_BAR_CLEAR);
    init(&f,JA_LEFT);for (unsigned int i=0u;i<12u;++i) (void)tick(&f,10,10);
    for (unsigned int j=0u;j<20u;++j) {
        line(&f,true);for (unsigned int i=0u;i<5u;++i) (void)tick(&f,10,10);
        line(&f,false);(void)tick(&f,10,10);
    }
    assert(!f.action.result.claimed);
    line(&f,true);f.lp.norm[1]=699u;
    for (unsigned int i=0u;i<20u;++i) (void)tick(&f,10,10);
    assert(!f.action.result.claimed);
    f.lp.norm[1]=1000u;(void)tick(&f,0,0);assert(!f.action.clear);
    line(&f,false);for (unsigned int i=0u;i<12u;++i) (void)tick(&f,10,10);
    line(&f,true);(void)tick(&f,10,10);(void)tick(&f,-1,0);
    assert(!f.action.clear && !f.action.candidate);
    f.sample.valid=false;(void)tick(&f,10,10);assert(!f.action.have_sample);
}
static void commands(bool wrap) {
    for (uint8_t c=JA_LEFT;c<=JA_UTURN;++c) {
        fixture_t f;init(&f,c);
        if (wrap) {f.in.counts[0]=f.in.counts[1]=INT32_MAX-20;f.now=UINT32_MAX-1000u;}
        if (c==JA_STRAIGHT) {
            claim(&f);
            while (ja_active(&f.action)) {
                /* Side arms cannot re-fire the detector; LINE still drives. */
                line(&f,(f.now%100u)<30u);f.drive=(vsm_drive_t){.left_mm_s=100,.right_mm_s=147};
                assert(!tick(&f,10,10));assert(f.drive.left_mm_s==100 && f.drive.right_mm_s==147);
            }
            assert(f.action.phase==JA_DONE && !(f.action.result.marks&JA_MARK_SEARCH));
            assert(diff(f.action.result.final_counts[0],f.action.result.edge_counts[0])>=1987);
        } else {
            begin(&f);f.in.line_event=vc_event(VC_EVENT_LINE_FOUND,0u);
            f.lp.norm[0]=900u;f.lp.norm[1]=0u;
            for (unsigned int i=0u;i<10u;++i) (void)tick(&f,c==JA_RIGHT ? 7 : -7,c==JA_RIGHT ? -7 : 7);
            assert(f.action.phase==JA_SEARCH && !(f.action.result.marks&JA_MARK_FIRST));
            white(&f);found(&f,c==JA_RIGHT ? -3300 : 3300);
            if (c==JA_UTURN) {
                assert(f.action.phase==JA_SECOND && !f.action.departed);
                found(&f,6000);assert(f.action.phase==JA_SECOND); /*same arm*/
                white(&f);found(&f,18000);
            }
            assert(f.action.phase==JA_DONE && (f.action.result.marks&JA_MARK_ACCEPTED));
            assert(f.action.result.first_line_cdeg>(c==JA_RIGHT ? -3310 : 3290));
            assert(f.action.result.accepted_line_cdeg>(c==JA_RIGHT ? -3310 : c==JA_UTURN ? 17990 : 3290));
        }
        const ja_command_t old={.epoch=1u,.sequence=1u,.command=c,.valid=1u,.source=1u};
        assert(!ja_offer(&f.action,&old));
    }
}
static void failures(void) {
    fixture_t f;init(&f,JA_LEFT);begin(&f);white(&f);found(&f,15100);
    assert(f.action.result.reason==JA_TURN_LIMIT && f.drive.left_mm_s==0);
    init(&f,JA_RIGHT);begin(&f);white(&f);found(&f,-15100);
    assert(f.action.result.reason==JA_TURN_LIMIT);
    init(&f,JA_UTURN);begin(&f);white(&f);found(&f,3300);white(&f);found(&f,11900);
    assert(f.action.result.reason==JA_NO_LINE && !(f.action.result.marks&JA_MARK_ACCEPTED));
    init(&f,JA_UTURN);begin(&f);white(&f);found(&f,3300);white(&f);found(&f,27100);
    assert(f.action.result.reason==JA_TURN_LIMIT);
    init(&f,JA_LEFT);begin(&f);f.now+=4000u;(void)tick(&f,-1,1);
    assert(f.action.result.reason==JA_TURN_LIMIT);
    init(&f,JA_UTURN);begin(&f);f.now+=6000u;(void)tick(&f,-1,1);
    assert(f.action.result.reason==JA_TURN_LIMIT);
    init(&f,JA_LEFT);begin(&f);f.now+=6000u;(void)tick(&f,-1,1);
    assert(f.action.result.reason==JA_TOTAL_LIMIT);
    init(&f,JA_LEFT);begin(&f);for (unsigned int i=0u;i<50u;++i) (void)tick(&f,0,0);
    assert(f.action.result.reason==JA_STALL);
    init(&f,JA_LEFT);begin(&f);found(&f,-1000);assert(f.action.result.reason==JA_REVERSE);
    init(&f,JA_LEFT);begin(&f);(void)tick(&f,500,500);
    assert(f.action.result.reason==JA_WHEEL_IMBALANCE);
    init(&f,JA_LEFT);claim(&f);f.now+=1500u;(void)tick(&f,0,0);
    assert(f.action.result.reason==JA_TRAVEL_LIMIT);
    init(&f,JA_LEFT);claim(&f);(void)tick(&f,1000,1000);
    assert(f.action.result.reason==JA_TRAVEL_LIMIT);
    init(&f,JA_STRAIGHT);claim(&f);f.now+=3000u;(void)tick(&f,1,1);
    assert(f.action.result.reason==JA_TRAVEL_LIMIT);
    init(&f,JA_LEFT);claim(&f);f.sample.valid=false;(void)tick(&f,1,1);
    assert(f.action.result.reason==JA_STALE);
    init(&f,JA_LEFT);begin(&f);ja_finish(&f.action,f.in.counts,f.now);
    assert(f.action.phase==JA_FAILED && f.action.result.reason==JA_CANCELLED);
    init(&f,JA_LEFT);ja_finish(&f.action,f.in.counts,f.now);
    assert(f.action.phase==JA_UNUSED && f.action.consumed==1u);
}
static void wrapper(void) {
    lt_t a,b;lt_init(&a,&LINE);lt_init(&b,&LINE);jl_t j;jl_init(&j,&b,131);
    fixture_t f;init(&f,JA_STRAIGHT);j.action=f.action;f.in.line=&f.lp;
    /* C calls the actual locked LINE, not equal-wheel action commands. */
    for (unsigned int i=0u;i<200u;++i) {
        line(&f,i>12u && i<24u);f.lp.position=(int8_t)((int)(i%9u)*10-40);
        f.in.counts[0]+=11;f.in.counts[1]+=11;f.now+=10u;
        j.sample=(ja_sample_t){.line_stamp=f.now,.valid=true};
        lp_t copy=f.lp;vsm_inputs_t in=f.in;in.line=&copy;
        vsm_drive_t da={0},db={0};
        const vsm_outcome_t out=lt_step(&a,&in,f.now,&da);
        assert(jl_step(&j,&f.in,f.now,&db)==out);
        assert(da.left_mm_s==db.left_mm_s && da.right_mm_s==db.right_mm_s);
    }
    assert(j.action.phase==JA_DONE);
    /* Immediate metadata cancellation preserves the natural SEARCH route. */
    init(&f,JA_LEFT);j.action=f.action;f.in.line_event=vc_event(VC_EVENT_LINE_LOST,0u);
    vsm_drive_t d={0};assert(vsm_outcome_type(jl_step(&j,&f.in,10u,&d))==VSM_OUT_LINE_LOST);
    assert(j.action.phase==JA_UNUSED);
    init(&f,JA_STRAIGHT);claim(&f);j.action=f.action;
    f.in.line_event=vc_event(VC_EVENT_LINE_LOST,0u);
    assert(vsm_outcome_type(jl_step(&j,&f.in,20u,&d))==VSM_OUT_FAILED);
    assert(j.action.result.reason==JA_NO_LINE); /*claimed C cannot hide loss*/
    /* NONE delegates the exact old outcome/drive on all positions. */
    ja_init(&j.action,131,1u);lt_start(&a,0u,0u);jl_start(&j,0u,0u);
    f.in.line_event=VC_EVENT_NONE;
    for (unsigned int i=0u;i<1000u;++i) {
        f.lp.position=(int8_t)((int)(i%201u)-100);lp_t copy=f.lp;
        vsm_inputs_t in=f.in;in.line=&copy;vsm_drive_t da={0},db={0};
        assert(lt_step(&a,&in,i,&da)==jl_step(&j,&f.in,i,&db));
        assert(!memcmp(&da,&db,sizeof(da)));
    }
}
static void fresh_gates(void) {
    fixture_t f;init(&f,JA_LEFT);begin(&f);
    f.lp.norm[0]=f.lp.norm[1]=299u;
    (void)tick(&f,-1,1);(void)tick(&f,-1,1);assert(!f.action.departed);
    f.lp.norm[0]=f.lp.norm[1]=300u;(void)tick(&f,-1,1);assert(!f.action.white);
    white(&f);f.lp.norm[0]=499u;f.lp.norm[1]=0u;
    (void)tick(&f,-1,1);assert(f.action.phase==JA_SEARCH);
    f.lp.norm[0]=900u;f.now+=10u;
    (void)ja_step(&f.action,&f.in,&f.sample,f.now,&f.drive);
    assert(f.action.phase==JA_SEARCH); /*same stamp cannot be FOUND*/
    found(&f,3300);assert(f.action.phase==JA_DONE);
    init(&f,JA_UTURN);begin(&f);white(&f);found(&f,3300);white(&f);found(&f,12000);
    assert(f.action.phase==JA_DONE && f.action.result.accepted_line_cdeg==12000);
    init(&f,JA_LEFT);begin(&f);white(&f);f.now+=500u;
    f.lp.norm[0]=500u;(void)tick(&f,0,20);
    assert(f.action.result.reason==JA_STALL && !(f.action.result.marks&JA_MARK_FIRST));
    init(&f,JA_LEFT);begin(&f);f.in.echo_new=true;f.in.echo=60u;
    (void)tick(&f,-1,1);(void)tick(&f,-1,1);
    assert(f.action.result.reason==JA_OBSTACLE && !f.drive.left_mm_s);
    init(&f,JA_LEFT);begin(&f);f.sample.line_age_ms=51u;(void)tick(&f,-1,1);
    assert(f.action.result.reason==JA_STALE);
    init(&f,JA_LEFT);claim(&f);line(&f,false);
    const int32_t target=f.action.result.edge_counts[0]+410;
    f.in.counts[0]=f.in.counts[1]=target;
    (void)tick(&f,0,0);assert(f.action.exit_samples==1u);
    f.now+=10u;(void)ja_step(&f.action,&f.in,&f.sample,f.now,&f.drive);
    assert(f.action.phase==JA_BAR_CLEAR && f.action.exit_samples==1u);
    line(&f,true);(void)tick(&f,0,0);assert(!f.action.exit_samples);
    line(&f,false);(void)tick(&f,0,0);assert(f.action.phase==JA_BAR_CLEAR);
    (void)tick(&f,0,0);assert(f.action.phase==JA_SEARCH);
}
static void source(void) {
    ja_command_t c={.epoch=999u};assert(!junction_source_read(NULL));
    assert(!junction_source_read(&c) && c.epoch==999u);
    const int32_t counts[2]={INT32_MAX,-100};junction_source_start(JA_STRAIGHT,1u,42u,counts);
    assert(junction_source_read(&c) && c.valid && c.read_counts[0]==INT32_MAX && c.read_ms==42u);
    ja_t a;ja_init(&a,131,1u);ja_command_t bad=c;
    bad.epoch=3u;assert(!ja_offer(&a,&bad));bad=c;bad.reserved=1u;assert(!ja_offer(&a,&bad));
    bad=c;bad.valid=2u;assert(!ja_offer(&a,&bad));bad=c;bad.command=5u;assert(!ja_offer(&a,&bad));
    bad=c;bad.sequence=0u;assert(!ja_offer(&a,&bad));bad=c;bad.source=0u;assert(!ja_offer(&a,&bad));
    assert(ja_offer(&a,&c) && !ja_offer(&a,&c));
    ja_finish(&a,counts,100u);c.sequence=UINT32_MAX;a.consumed=0u;assert(ja_offer(&a,&c));
    ja_finish(&a,counts,100u);c.sequence=1u;assert(ja_offer(&a,&c));
    junction_source_set_busy(true);assert(junction_source_busy());junction_source_set_busy(false);
    junction_source_acknowledge(1u);assert(junction_source_acknowledged()==1u);
}
static void unchanged_line(void) {
    const lt_config_t cfg={.steer={.base_mm_s=146,.gain_pct=85u,.kd_pct=85u,
        .min_mm_s=62,.max_mm_s=378,.far_from=50u,.far_gain_pct=340u},
        .straight_counts=450,.hint_counts=20};
    lt_t original,wrapped;lt_init(&original,&cfg);lt_init(&wrapped,&cfg);
    jl_t adapter;jl_init(&adapter,&wrapped,131);
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
    _Static_assert(sizeof(ja_command_t)==24u,"source unchanged");
    _Static_assert(sizeof(ja_result_t)==52u,"bounded result");
    detector();commands(false);commands(true);failures();wrapper();fresh_gates();source();unchanged_line();return 0;
}

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../app_program/subsystem3/barcode_live_producer.h"

static barcode_cal_report_t cal = {.generation=1u, .mode=BC_CAL_LEARNED,
    .white_mean=160u, .white_samples=1904u, .rise=310u, .fall=240u};
static barcode_live_input_t input(uint32_t ms, uint32_t tag, bool ok)
{
    return (barcode_live_input_t){.calibration=&cal,
        .control={.epoch=7u,.eligibility_revision=3u,.revision_ms=1u,
                  .mode=BC_LIVE_PERMITTED},
        .now_ms=ms,.now_us=ms*1000u,.adc_us=ms*1000u,
        .motor_us=ms*1000u,.raw=160u,.motor_ok=true,.control_ok=ok,.tag=tag};
}
static void good(barcode_live_producer_t *p)
{
    barcode_live_input_t in=input(100u,2u,true);
    barcode_live_producer_observe(p,&in);
    in=input(110u,4u,true);
    barcode_live_producer_observe(p,&in);
    assert(p->qualified && p->window_live);
}
int main(void)
{
    barcode_live_producer_t p={0};
    barcode_live_input_t in=input(100u,2u,false);
    barcode_live_producer_observe(&p,&in);
    assert(!p.seen && !p.window_live); /* no first-read cache */
    p=(barcode_live_producer_t){0}; good(&p);
    in=input(120u,900u,false);
    in.control.epoch=99u; in.control.mode=BC_LIVE_DISABLED;
    barcode_live_producer_observe(&p,&in);
    assert(p.window_live && p.run_epoch==7u); /* ignore failed-read contents */
    assert(p.success_ms==110u && p.progress_ms==100u && p.tag==4u);
    in=input(150u,900u,false);
    barcode_live_producer_observe(&p,&in);
    assert(p.window_live && p.control_age_ms==50u);
    in=input(151u,900u,false);
    barcode_live_producer_observe(&p,&in);
    assert(!p.window_live && p.success_ms==110u);
    /* Successful observations with no new publication cannot renew age. */
    in=input(152u,4u,true); barcode_live_producer_observe(&p,&in);
    assert(!p.window_live);
    p=(barcode_live_producer_t){0}; good(&p);
    in=input(120u,6u,true); in.control.mode=BC_LIVE_DISABLED;
    barcode_live_producer_observe(&p,&in); assert(!p.window_live);
    p=(barcode_live_producer_t){0}; good(&p);
    p.bound=true; p.context=input(110u,4u,true).control;
    in=input(120u,6u,true); ++in.control.eligibility_revision;
    barcode_live_producer_observe(&p,&in);
    assert(p.event_pending && p.event.reason==BP_CONTEXT);
    p=(barcode_live_producer_t){0}; good(&p);
    p.occupied=true; p.slot.sequence=9u;
    in=input(120u,4u,false); in.control.ack_sequence=9u;
    barcode_live_producer_observe(&p,&in);
    assert(p.occupied); /* cannot accept a cached or failed-read ACK */
    /* Long observation pause requires actual new publication to qualify. */
    p=(barcode_live_producer_t){0}; good(&p);
    in=input(200u,4u,true); barcode_live_producer_observe(&p,&in);
    assert(!p.qualified && !p.window_live);
    in=input(201u,4u,false); barcode_live_producer_observe(&p,&in);
    assert(!p.qualified && !p.window_live);
    puts("control cache PASS");
    return 0;
}

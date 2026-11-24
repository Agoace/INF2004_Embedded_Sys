/* Included ONLY by the enabled source provider (or the host test).
 * No default-build object, storage, call, header dependency or wire change.
 * LINE owns ACTIVE slots. DUMP advertises one hazard slot before reading
 * FROZEN data. Writer makes a slot ACTIVE before checking that hazard:
 * either the reader sees ACTIVE and skips, or writer sees the hazard and
 * skips. Only aligned atomic loads/stores, no RP2040 libatomic/CAS. */
#include <math.h>
#include "barcode_evidence_pack.h"
#ifndef BCFC_FAILED_SLOTS
#define BCFC_FAILED_SLOTS 2u
#endif
_Static_assert(BCFC_FAILED_SLOTS == 1u || BCFC_FAILED_SLOTS == 2u,
               "Capture retains one or two failures");
#define BCFC_SLOTS (BCFC_FAILED_SLOTS + 1u)
#define BCFC_EDGES 64u
#define BCFC_ADC 512u
#define BCFC_PRE 64u
#define BCFC_DIV_US 4000u
/* State: FREE0, ACTIVE1, FROZEN2. LINE is the only state writer. */
typedef bcep_edge_t bcfc_edge_t;
typedef struct { uint32_t us; uint16_t raw, flags; } bcfc_adc_t;
typedef struct {
    uint32_t state;
    bcep_frame_t frame;
    uint32_t epoch, pre_head, pre_count, adc_count, edge_count;
    uint32_t adc_total, edge_total, last_adc_us, elements, window_reason;
    uint32_t enable_us, reset_us, reset_reason;
    float leading_mm, bias, space, first_mm, last_mm;
    uint16_t rise, fall;
    bool bound, closed, sampled;
    uint32_t edges[BCEP_EDGE_WORDS];
    bcfc_adc_t adc[BCFC_ADC];
} bcfc_slot_t;
static bcfc_slot_t bcfc_slots[BCFC_SLOTS];
static uint32_t bcfc_run, bcfc_saved, bcfc_missed;
static uint32_t bcfc_reading; /* DUMP-written slot+1, zero when not reading. */
static int bcfc_active = -1;
/* LINE-only context: last disabled->enabled control poll and last reset of
 * an UNBOUND window (both drv_time_us at the poll). Zero = unobserved. */
static uint32_t bcfc_now_us, bcfc_enable_us, bcfc_reset_us, bcfc_reset_reason;
static bool bcfc_enabled;
/* DUMP-only bit mask; no LINE access. */
static uint32_t bcfc_printed;
_Static_assert(sizeof(bcfc_edge_t)==24u && sizeof(bcfc_adc_t)==8u,
               "Capture row ABI");
_Static_assert(sizeof(bcfc_slot_t)==5660u, "CR1 capture slot; capacity unchanged");
_Static_assert(sizeof(bcfc_slots) + 32u <= 18u*1024u,
               "Failed capture RAM ceiling");

static bcfc_slot_t *bcfc_work(void)
{
    if (bcfc_active >= 0) return &bcfc_slots[bcfc_active];
    if (bcfc_run == 0u || bcfc_saved >= BCFC_FAILED_SLOTS) return NULL;
    for (unsigned int i=0u; i<BCFC_SLOTS; ++i) {
        if (__atomic_load_n(&bcfc_slots[i].state,__ATOMIC_SEQ_CST)!=0u) continue;
        __atomic_store_n(&bcfc_slots[i].state,1u,__ATOMIC_SEQ_CST);
        if (__atomic_load_n(&bcfc_reading,__ATOMIC_SEQ_CST)==i+1u) {
            __atomic_store_n(&bcfc_slots[i].state,0u,__ATOMIC_SEQ_CST);
            continue;
        }
        /* Reset metadata only; bounded records are never copied/cleared. */
        bcfc_slot_t *s=&bcfc_slots[i];
        s->frame=(bcep_frame_t){{0}};s->epoch=bcfc_run;
        s->pre_head=0u;s->pre_count=0u;s->adc_count=0u;s->edge_count=0u;
        s->adc_total=0u;s->edge_total=0u;s->elements=0u;s->window_reason=0u;
        s->bound=false;s->closed=false;s->sampled=false;
        s->leading_mm=0.0f;s->bias=0.0f;s->space=0.0f;
        s->enable_us=0u;s->reset_us=0u;s->reset_reason=0u;
        s->first_mm=0.0f;s->last_mm=0.0f;s->rise=0u;s->fall=0u;
        bcfc_active=(int)i;return s;
    }
    return NULL; /* A concurrent dump may own an old-run slot. Never wait. */
}

void bcfc_epoch(uint32_t epoch)
{
    if (epoch==0u || epoch==bcfc_run) return;
    bcfc_run=epoch;bcfc_saved=0u;bcfc_active=-1;
    bcfc_enable_us=0u;bcfc_reset_us=0u;bcfc_reset_reason=0u;bcfc_enabled=false;
    for (unsigned int i=0u;i<BCFC_SLOTS;++i) {
        __atomic_store_n(&bcfc_slots[i].state,0u,__ATOMIC_SEQ_CST);
    }
}

void bcfc_control(uint32_t mode, uint32_t now_us)
{
    const bool enabled=mode!=0u && mode<=2u;
    bcfc_now_us=now_us;
    if (enabled && !bcfc_enabled) bcfc_enable_us=now_us;
    bcfc_enabled=enabled;
}

static void bcfc_edge(bcfc_slot_t *s, const barcode_window_t *w,
                      barcode_window_adc_t a, float mm,
                      const barcode_window_motor_t *upper, bool black)
{
    ++s->edge_total;
    if (s->edge_count==BCFC_EDGES) return;
    const bcfc_edge_t edge={.us=a.us,.motor_us=upper->us,
        .left=upper->left,.right=upper->right,.mm=mm,.raw=a.raw,
        .black=black?1u:0u,.phase=w->phase};
    if (bcep_edge_pack(s->edges,(unsigned int)s->edge_count,&edge)) ++s->edge_count;
}

void bcfc_observe(const barcode_window_t *w, barcode_window_adc_t a,
                  float mm, const barcode_window_motor_t *upper)
{
    bcfc_slot_t *s=bcfc_work();
    if (!s || s->closed) return;
    if (!s->sampled || a.us-s->last_adc_us>=BCFC_DIV_US) {
        s->sampled=true;s->last_adc_us=a.us;
        const bcfc_adc_t value={a.us,a.raw,a.flags};
        if (!s->bound) {
            s->adc[s->pre_head]=value;
            s->pre_head=(s->pre_head+1u)%BCFC_PRE;
            if (s->pre_count<BCFC_PRE) ++s->pre_count;
        } else {
            ++s->adc_total;
            if (s->adc_count<BCFC_ADC-BCFC_PRE)
                s->adc[BCFC_PRE+s->adc_count++]=value;
        }
    }
    if (s->bound && (w->phase==2u || w->phase==3u)) {
        const bool black=w->black ? a.raw>w->fall : a.raw>=w->rise;
        if (black!=w->black) bcfc_edge(s,w,a,mm,upper,black);
        s->last_mm=mm;
    }
}

void bcfc_start(const barcode_window_t *w, barcode_window_adc_t a,
                float mm, const barcode_window_motor_t *upper)
{
    bcfc_slot_t *s=bcfc_work();
    if (!s || s->closed || s->bound) return;
    s->bound=true;bcep_put(s->frame.words,BCEP_F_START,32u,w->start_us);
    s->leading_mm=w->leading_mm;s->rise=w->rise;s->fall=w->fall;
    s->first_mm=mm;s->last_mm=mm;
    s->enable_us=bcfc_enable_us;s->reset_us=bcfc_reset_us;
    s->reset_reason=bcfc_reset_reason;
    bcfc_edge(s,w,a,mm,upper,true);
}

void bcfc_bad_adc(barcode_window_adc_t a, uint16_t fault)
{
    if (bcfc_active<0) return;
    bcfc_slot_t *s=&bcfc_slots[bcfc_active];
    if (!s->bound || s->closed) return;
    ++s->adc_total;
    a.flags=(uint16_t)(a.flags | UINT16_C(0x8000) | fault);
    if (s->adc_count<BCFC_ADC-BCFC_PRE)
        s->adc[BCFC_PRE+s->adc_count++]=(bcfc_adc_t){a.us,a.raw,a.flags};
}

static void bcfc_snapshot(bcfc_slot_t *s, const barcode_window_t *w)
{
    s->elements=w->elements;s->bias=w->bias;s->space=w->space;
    s->leading_mm=w->leading_mm;s->rise=w->rise;s->fall=w->fall;
}

void bcfc_reject(const barcode_window_t *w, uint32_t reason)
{
    bcfc_slot_t *s=bcfc_active>=0 ? &bcfc_slots[bcfc_active] : NULL;
    if (!s || !s->bound) {
        bcfc_reset_us=bcfc_now_us;bcfc_reset_reason=reason;return;
    }
    if (s->closed) return;
    bcfc_snapshot(s,w);s->window_reason=reason;s->closed=true;
}
void bcfc_close(const barcode_window_t *w)
{
    if (bcfc_active<0) return;
    bcfc_slot_t *s=&bcfc_slots[bcfc_active];
    if (!s->bound || s->closed) return;
    bcfc_snapshot(s,w);s->closed=true;
}
void bcfc_terminal(const barcode_live_frame_t *frame)
{
    if (frame->record.epoch!=bcfc_run || bcfc_saved>=BCFC_FAILED_SLOTS) return;
    bcfc_slot_t *s=bcfc_active>=0 ? &bcfc_slots[bcfc_active] : NULL;
    if (!s || !s->bound ||
        bcep_get(s->frame.words,BCEP_F_START,32u)!=frame->start_us) {
        if (frame->reason!=BW_OK)
            __atomic_store_n(&bcfc_missed,
                __atomic_load_n(&bcfc_missed,__ATOMIC_SEQ_CST)+1u,__ATOMIC_SEQ_CST);
        return;
    }
    bcfc_active=-1;
    if (frame->reason==BW_OK) {
        __atomic_store_n(&s->state,0u,__ATOMIC_RELEASE);return;
    }
    if (!bcep_frame_pack(&s->frame,frame)) {
        __atomic_store_n(&bcfc_missed,
            __atomic_load_n(&bcfc_missed,__ATOMIC_SEQ_CST)+1u,__ATOMIC_SEQ_CST);
        __atomic_store_n(&s->state,0u,__ATOMIC_RELEASE);return;
    }
    ++bcfc_saved;
    __atomic_store_n(&s->state,2u,__ATOMIC_RELEASE);
}

void bcfc_dump_begin(void)
{
    bcfc_printed=0u;
    tm_printf((UB *)"[BC_FAIL_STATUS] slots %u edges 64 samples 512 pre 64 "
        "div_us 4000 missed_lifetime %lu\n",(unsigned int)BCFC_FAILED_SLOTS,
        (unsigned long)__atomic_load_n(&bcfc_missed,__ATOMIC_SEQ_CST));
}
static void bcfc_print(unsigned int index, bool saved_ref)
{
    if ((bcfc_printed & (1u<<index))!=0u) return;
    bcfc_slot_t *s=&bcfc_slots[index];
    barcode_live_frame_t decoded_frame;
    if (!bcep_frame_unpack(&decoded_frame,&s->frame)) return;
    const barcode_live_frame_t *f=&decoded_frame;
    if (saved_ref)
        tm_printf((UB *)"[BC_FAIL_REF] epoch %lu index %lu start_us %lu terminal_us %lu reason %lu\n",
            (unsigned long)f->record.epoch,(unsigned long)f->index,
            (unsigned long)f->start_us,(unsigned long)f->terminal_us,
            (unsigned long)f->reason);
    tm_printf((UB *)"[BC_FAIL_CAPTURE] epoch %lu index %lu start_us %lu terminal_us %lu "
        "reason %lu window_reason %lu elements %lu rise %u fall %u lead_cmm %ld "
        "fit_valid %u "
        "n_cmm %ld w_cmm %ld bias_cmm %ld space_cmm %ld "
        "pre %lu adc %lu edge %lu adc_lost %lu edge_lost %lu div_us 4000\n",
        (unsigned long)f->record.epoch,(unsigned long)f->index,
        (unsigned long)f->start_us,(unsigned long)f->terminal_us,
        (unsigned long)f->reason,(unsigned long)s->window_reason,
        (unsigned long)s->elements,(unsigned int)s->rise,(unsigned int)s->fall,
        (long)lroundf(s->leading_mm*100.0f),
        (unsigned int)(isfinite(f->narrow_mm) && isfinite(f->wide_mm) &&
                       f->narrow_mm>0.0f && f->wide_mm>0.0f),
        (long)lroundf(f->narrow_mm*100.0f),(long)lroundf(f->wide_mm*100.0f),
        (long)lroundf(s->bias*100.0f),(long)lroundf(s->space*100.0f),
        (unsigned long)s->pre_count,(unsigned long)s->adc_count,
        (unsigned long)s->edge_count,
        (unsigned long)(s->adc_total-s->adc_count),
        (unsigned long)(s->edge_total-s->edge_count));
    tm_printf((UB *)"[BC_FAIL_CONTEXT] epoch %lu index %lu enable_us %lu reset_us %lu reset_reason %lu\n",
        (unsigned long)f->record.epoch,(unsigned long)f->index,
        (unsigned long)s->enable_us,(unsigned long)s->reset_us,
        (unsigned long)s->reset_reason);
    for (uint32_t i=0u;i<s->edge_count;++i) {
        bcfc_edge_t decoded_edge;
        if (!bcep_edge_unpack(&decoded_edge,s->edges,(unsigned int)i)) continue;
        const bcfc_edge_t *e=&decoded_edge;
        tm_printf((UB *)"[BC_FAIL_EDGE] epoch %lu index %lu row %lu adc_us %lu "
            "raw %u black %u phase %u mm_cmm %ld bracket_us %lu counts %ld/%ld\n",
            (unsigned long)f->record.epoch,(unsigned long)f->index,(unsigned long)i,
            (unsigned long)e->us,(unsigned int)e->raw,(unsigned int)e->black,
            (unsigned int)e->phase,(long)lroundf(e->mm*100.0f),
            (unsigned long)e->motor_us,(long)(int32_t)e->left,(long)(int32_t)e->right);
    }
    for (uint32_t i=0u;i<s->pre_count+s->adc_count;++i) {
        const bool pre=i<s->pre_count;
        const uint32_t at=pre ?
            (s->pre_head+BCFC_PRE-s->pre_count+i)%BCFC_PRE :
            BCFC_PRE+i-s->pre_count;
        const bcfc_adc_t *a=&s->adc[at];
        tm_printf((UB *)"[BC_FAIL_ADC] epoch %lu index %lu row %lu us %lu raw %u flags %u pre %u\n",
            (unsigned long)f->record.epoch,(unsigned long)f->index,(unsigned long)i,
            (unsigned long)a->us,(unsigned int)a->raw,(unsigned int)a->flags,pre?1u:0u);
    }
    bcfc_printed|=1u<<index;
}
void bcfc_dump(uint32_t epoch, uint32_t index)
{
    for (unsigned int i=0u;i<BCFC_SLOTS;++i) {
        __atomic_store_n(&bcfc_reading,i+1u,__ATOMIC_SEQ_CST);
        if (__atomic_load_n(&bcfc_slots[i].state,__ATOMIC_SEQ_CST)==2u &&
            bcep_get(bcfc_slots[i].frame.words,0u,32u)==epoch &&
            bcep_get(bcfc_slots[i].frame.words,BCEP_F_INDEX,32u)==index)
            bcfc_print(i,false);
        __atomic_store_n(&bcfc_reading,0u,__ATOMIC_SEQ_CST);
    }
}
void bcfc_dump_remaining(void)
{
    for (unsigned int i=0u;i<BCFC_SLOTS;++i) {
        __atomic_store_n(&bcfc_reading,i+1u,__ATOMIC_SEQ_CST);
        if (__atomic_load_n(&bcfc_slots[i].state,__ATOMIC_SEQ_CST)==2u)
            bcfc_print(i,true);
        __atomic_store_n(&bcfc_reading,0u,__ATOMIC_SEQ_CST);
    }
}

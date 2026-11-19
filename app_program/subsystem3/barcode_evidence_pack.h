/* CR1 CATEGORY A: PRIVATE evidence only. Public decoded types are unchanged.
 * Capacities, timestamps, counters, binary32 values and output units stay exact.
 * Storage is aligned uint32_t words; bit 0 is the least-significant word bit.
 * Fields can cross words. No C bit-fields, unaligned loads or shared RMW.
 *
 * Frame (22 words, 88 B), offsets inclusive, least significant bit first:
 *   0..31 epoch, 32..63 sequence, 64..95 read_ms,
 *   96..127 read_L, 128..159 read_R, 160..191 frame_L,
 *   192..223 frame_R, 224..255 eligibility_revision (all 32-bit exact).
 *   256..258 command(0..4),259 valid,260..261 source(0..2),
 *   262..267 letter(code below),268..269 direction(0..2),270..277 flags,
 *   278 version(0..1),279..310 index,311..315 reason(code below),
 *   316..347 start_us,348..379 edge_us,380..411 frame_us,
 *   412..443 eligible_us,444..475 terminal_us,476..507 control_age_ms,
 *   508..539 record_age_ms,540..571 spin_white (all 32-bit exact),
 *   572..579 leading_white(0..220),580..611 narrow_mm binary32,
 *   612..643 wide_mm binary32,644..675 leading_mm binary32,
 *   676..684 rise(0..350),685..693 fall(0..275),694..703 ZERO.
 * record.reserved is always reconstructed as zero.
 * Letter code 0 = unknown/NUL; 1..44 index the Code39 alphabet plus '*':
 * "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*". Codes45..63 invalid.
 * Reason codes0..10 = BW0..10;11..18 = BP100..107;19..31 invalid.
 *
 * Edge array: 64 records x174 bits =348 words =1392 B (was1536 B).
 * Edge i starts at bit174*i: +0 us32,+32 motor_us32,+64 left32,
 * +96 right32,+128 mm binary32,+160 raw12,+172 black1,+173 phase1.
 * phase bit0 means WINDOW(2), bit1 means TAIL(3). These are the ONLY
 * phases at bcfc_start/edge. Rejected ADCs use the unchanged 8 B ADC rows,
 * not edge storage; their full raw16/flags16 remain intact.
 *
 * LINE is sole writer of private ACTIVE slots. Existing FROZEN/hazard
 * transfer protects DUMP reads. Frames use app_board: pack a local value,
 * publish whole atomic words, copy a coherent value, THEN unpack locally.
 * Invalid values reject without modifying output; no narrowing on error.
 */
#ifndef BARCODE_EVIDENCE_PACK_H
#define BARCODE_EVIDENCE_PACK_H
#include <string.h>
#include "barcode_live_producer.h"
#define BCEP_FRAME_WORDS 22u
#define BCEP_EDGE_WORDS 348u
#define BCEP_EDGE_CAPACITY 64u
#define BCEP_F_INDEX 279u
#define BCEP_F_START 316u

typedef struct { uint32_t words[BCEP_FRAME_WORDS]; } bcep_frame_t;
typedef struct {
    uint32_t us, motor_us, left, right;
    float mm;
    uint16_t raw;
    uint8_t black, phase;
} bcep_edge_t;
_Static_assert(sizeof(float)==4u && sizeof(bcep_frame_t)==88u,
               "CR1 evidence uses binary32 and aligned 88 B frames");

static inline uint32_t bcep_get(const uint32_t *w, unsigned int bit, unsigned int n)
{
    const unsigned int at=bit/32u, shift=bit%32u;
    uint32_t v=w[at]>>shift;
    if (shift && n>32u-shift) v|=w[at+1u]<<(32u-shift);
    return n==32u ? v : v&((UINT32_C(1)<<n)-1u);
}
static inline void bcep_put(uint32_t *w, unsigned int bit, unsigned int n, uint32_t v)
{
    const unsigned int at=bit/32u, shift=bit%32u;
    const uint32_t mask=n==32u ? UINT32_MAX : (UINT32_C(1)<<n)-1u;
    w[at]=(w[at]&~(mask<<shift))|((v&mask)<<shift);
    if (shift && n>32u-shift) {
        const uint32_t upper=mask>>(32u-shift);
        w[at+1u]=(w[at+1u]&~upper)|((v>>(32u-shift))&upper);
    }
}
static inline uint32_t bcep_float_bits(float v)
{ uint32_t bits; memcpy(&bits,&v,4u); return bits; }
static inline float bcep_float(uint32_t bits)
{ float v; memcpy(&v,&bits,4u); return v; }
static inline int32_t bcep_signed(uint32_t bits)
{ int32_t v; memcpy(&v,&bits,4u); return v; }
static inline unsigned int bcep_letter_code(uint8_t letter)
{
    const char alphabet[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*";
    if (!letter) return 0u;
    for (unsigned int i=0u;i<sizeof(alphabet)-1u;++i)
        if ((uint8_t)alphabet[i]==letter) return i+1u;
    return 64u;
}
static inline bool bcep_frame_valid(const barcode_live_frame_t *f)
{
    return f && f->record.command<=4u && f->record.valid<=1u &&
        f->record.source<=2u && f->record.reserved==0u &&
        bcep_letter_code(f->record.letter)<=44u && f->record.direction<=2u &&
        f->record.version<=1u && (f->reason<=10u ||
        (f->reason>=100u && f->reason<=107u)) && f->leading_white<=220u &&
        f->rise<=350u && f->fall<=275u;
}
static inline bool bcep_frame_pack(bcep_frame_t *out,const barcode_live_frame_t *f)
{
    if (!out || !bcep_frame_valid(f)) return false;
    bcep_frame_t p={{0}};
#define BCEP_P(bit,n,value) bcep_put(p.words,(bit),(n),(uint32_t)(value))
    BCEP_P(0u,32u,f->record.epoch); BCEP_P(32u,32u,f->record.sequence);
    BCEP_P(64u,32u,f->record.read_ms); BCEP_P(96u,32u,f->record.read_counts[0]);
    BCEP_P(128u,32u,f->record.read_counts[1]); BCEP_P(160u,32u,f->record.frame_end_counts[0]);
    BCEP_P(192u,32u,f->record.frame_end_counts[1]); BCEP_P(224u,32u,f->record.eligibility_revision);
    BCEP_P(256u,3u,f->record.command); BCEP_P(259u,1u,f->record.valid);
    BCEP_P(260u,2u,f->record.source); BCEP_P(262u,6u,bcep_letter_code(f->record.letter));
    BCEP_P(268u,2u,f->record.direction); BCEP_P(270u,8u,f->record.flags);
    BCEP_P(278u,1u,f->record.version); BCEP_P(279u,32u,f->index);
    BCEP_P(311u,5u,f->reason<=10u?f->reason:11u+f->reason-100u);
    BCEP_P(316u,32u,f->start_us); BCEP_P(348u,32u,f->edge_us);
    BCEP_P(380u,32u,f->frame_us); BCEP_P(412u,32u,f->eligible_us);
    BCEP_P(444u,32u,f->terminal_us); BCEP_P(476u,32u,f->control_age_ms);
    BCEP_P(508u,32u,f->record_age_ms); BCEP_P(540u,32u,f->spin_white);
    BCEP_P(572u,8u,f->leading_white); BCEP_P(580u,32u,bcep_float_bits(f->narrow_mm));
    BCEP_P(612u,32u,bcep_float_bits(f->wide_mm)); BCEP_P(644u,32u,bcep_float_bits(f->leading_mm));
    BCEP_P(676u,9u,f->rise); BCEP_P(685u,9u,f->fall);
#undef BCEP_P
    *out=p; return true;
}
static inline bool bcep_frame_unpack(barcode_live_frame_t *out,const bcep_frame_t *p)
{
    if (!out || !p || (p->words[21]>>22u)) return false;
    const char alphabet[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*";
    const uint32_t letter=bcep_get(p->words,262u,6u), reason=bcep_get(p->words,311u,5u);
    if (letter>44u || reason>18u) return false;
    barcode_live_frame_t f={0};
#define BCEP_G(bit,n) bcep_get(p->words,(bit),(n))
    f.record.epoch=BCEP_G(0u,32u); f.record.sequence=BCEP_G(32u,32u);
    f.record.read_ms=BCEP_G(64u,32u); f.record.read_counts[0]=bcep_signed(BCEP_G(96u,32u));
    f.record.read_counts[1]=bcep_signed(BCEP_G(128u,32u));
    f.record.frame_end_counts[0]=bcep_signed(BCEP_G(160u,32u));
    f.record.frame_end_counts[1]=bcep_signed(BCEP_G(192u,32u));
    f.record.eligibility_revision=BCEP_G(224u,32u);
    f.record.command=(uint8_t)BCEP_G(256u,3u); f.record.valid=(uint8_t)BCEP_G(259u,1u);
    f.record.source=(uint8_t)BCEP_G(260u,2u);
    f.record.letter=letter?(uint8_t)alphabet[letter-1u]:0u;
    f.record.direction=(uint8_t)BCEP_G(268u,2u); f.record.flags=(uint8_t)BCEP_G(270u,8u);
    f.record.version=(uint8_t)BCEP_G(278u,1u); f.index=BCEP_G(279u,32u);
    f.reason=reason<=10u?reason:100u+reason-11u;
    f.start_us=BCEP_G(316u,32u); f.edge_us=BCEP_G(348u,32u); f.frame_us=BCEP_G(380u,32u);
    f.eligible_us=BCEP_G(412u,32u); f.terminal_us=BCEP_G(444u,32u);
    f.control_age_ms=BCEP_G(476u,32u); f.record_age_ms=BCEP_G(508u,32u);
    f.spin_white=BCEP_G(540u,32u); f.leading_white=BCEP_G(572u,8u);
    f.narrow_mm=bcep_float(BCEP_G(580u,32u)); f.wide_mm=bcep_float(BCEP_G(612u,32u));
    f.leading_mm=bcep_float(BCEP_G(644u,32u));
    f.rise=(uint16_t)BCEP_G(676u,9u); f.fall=(uint16_t)BCEP_G(685u,9u);
#undef BCEP_G
    if (!bcep_frame_valid(&f)) return false;
    *out=f; return true;
}
static inline bool bcep_edge_pack(uint32_t *words,unsigned int index,const bcep_edge_t *e)
{
    if (!words || !e || index>=BCEP_EDGE_CAPACITY || e->raw>4095u ||
        e->black>1u || (e->phase!=2u && e->phase!=3u)) return false;
    const unsigned int b=index*174u;
    bcep_put(words,b,32u,e->us); bcep_put(words,b+32u,32u,e->motor_us);
    bcep_put(words,b+64u,32u,e->left); bcep_put(words,b+96u,32u,e->right);
    bcep_put(words,b+128u,32u,bcep_float_bits(e->mm));
    bcep_put(words,b+160u,12u,e->raw); bcep_put(words,b+172u,1u,e->black);
    bcep_put(words,b+173u,1u,(uint32_t)e->phase-2u); return true;
}
static inline bool bcep_edge_unpack(bcep_edge_t *out,const uint32_t *words,unsigned int index)
{
    if (!out || !words || index>=BCEP_EDGE_CAPACITY) return false;
    const unsigned int b=index*174u;
    const bcep_edge_t e={.us=bcep_get(words,b,32u),.motor_us=bcep_get(words,b+32u,32u),
        .left=bcep_get(words,b+64u,32u),.right=bcep_get(words,b+96u,32u),
        .mm=bcep_float(bcep_get(words,b+128u,32u)),.raw=(uint16_t)bcep_get(words,b+160u,12u),
        .black=(uint8_t)bcep_get(words,b+172u,1u),.phase=(uint8_t)(2u+bcep_get(words,b+173u,1u))};
    *out=e; return true;
}
#endif

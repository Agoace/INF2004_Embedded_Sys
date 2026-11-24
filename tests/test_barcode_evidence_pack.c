/* CR1 private evidence: round trips, invalid inputs and torn board reads. */
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "../app_program/subsystem3/barcode_evidence_pack.h"
static void read_hook(const uint32_t *sequence, unsigned int index);
#define APP_BOARD_READ_HOOK(s, i) read_hook((s), (i))
#include "../app_program/platform/app_board.h"
static APP_BOARD(bcep_frame_t) board;
static bcep_frame_t replacement;
static unsigned int interrupt_at;
static bool interrupt_read;
static void read_hook(const uint32_t *sequence, unsigned int index)
{
    if (interrupt_read && sequence == &board.sequence && index == interrupt_at) {
        interrupt_read = false;
        app_board_publish(&board.sequence, board.words, &replacement, sizeof(replacement));
    }
}
static barcode_live_frame_t fixture(void)
{
    barcode_live_frame_t f = {0};
    f.record.epoch=UINT32_MAX; f.record.sequence=0x80000000u;
    f.record.read_ms=1u; f.record.read_counts[0]=INT32_MIN;
    f.record.read_counts[1]=INT32_MAX; f.record.frame_end_counts[0]=-1;
    f.record.frame_end_counts[1]=1; f.record.eligibility_revision=0x76543210u;
    f.record.command=4u; f.record.valid=1u; f.record.source=2u;
    f.record.letter='F'; f.record.direction=2u; f.record.flags=255u;
    f.record.version=1u;
    f.index=UINT32_MAX; f.reason=107u; f.start_us=0x80000000u;
    f.edge_us=1u; f.frame_us=UINT32_MAX; f.eligible_us=0x55555555u;
    f.terminal_us=0xaaaaaaaau; f.control_age_ms=UINT32_MAX;
    f.record_age_ms=UINT32_MAX; f.spin_white=UINT32_MAX;
    f.leading_white=220u; f.narrow_mm=5.8f; f.wide_mm=17.5f;
    f.leading_mm=-0.0f; f.rise=350u; f.fall=275u;
    return f;
}
static void round_frame(const barcode_live_frame_t *f)
{
    bcep_frame_t packed;
    barcode_live_frame_t decoded;
    assert(bcep_frame_pack(&packed,f));
    assert((packed.words[21] >> 22u)==0u);
    assert(bcep_frame_unpack(&decoded,&packed));
    assert(memcmp(&decoded,f,sizeof(decoded))==0);
}
static void reject_frame(const barcode_live_frame_t *f)
{
    bcep_frame_t saved, out; const barcode_live_frame_t good=fixture();
    assert(bcep_frame_pack(&saved,&good)); out=saved;
    assert(!bcep_frame_pack(&out,f) && memcmp(&out,&saved,sizeof(out))==0);
}
static void reject_packed(const bcep_frame_t *p)
{
    barcode_live_frame_t out=fixture(), saved=out;
    assert(!bcep_frame_unpack(&out,p) && memcmp(&out,&saved,sizeof(out))==0);
}
int main(void)
{
    assert(sizeof(bcep_frame_t)==88u && sizeof(board)==92u);
    assert(BCEP_EDGE_WORDS*sizeof(uint32_t)==1392u);
    barcode_live_frame_t f=fixture(); round_frame(&f);
    for (unsigned int i=0u;i<=350u;++i) { f.rise=(uint16_t)i; round_frame(&f); }
    for (unsigned int i=0u;i<=275u;++i) { f.fall=(uint16_t)i; round_frame(&f); }
    for (unsigned int i=0u;i<=220u;++i) { f.leading_white=i; round_frame(&f); }
    for (unsigned int i=0u;i<256u;++i) { f.record.flags=(uint8_t)i; round_frame(&f); }
    for (unsigned int i=0u;i<=4u;++i) { f.record.command=(uint8_t)i; round_frame(&f); }
    for (unsigned int i=0u;i<=2u;++i) {
        f.record.source=(uint8_t)i; f.record.direction=(uint8_t)i; round_frame(&f);
    }
    for (unsigned int i=0u;i<4u;++i) {
        f.record.valid=(uint8_t)(i&1u); f.record.version=(uint8_t)(i>>1u); round_frame(&f);
    }
    const char alphabet[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*";
    f.record.letter=0u; round_frame(&f);
    for (unsigned int i=0u;i<sizeof(alphabet)-1u;++i) {
        f.record.letter=(uint8_t)alphabet[i]; round_frame(&f);
    }
    for (unsigned int i=0u;i<19u;++i) { f.reason=i<11u?i:100u+i-11u; round_frame(&f); }
    const uint32_t float_bits[]={0u,0x80000000u,0x7f800000u,0xff800000u,
        0x7fc12345u,0x00000001u,0x7f7fffffu};
    for (unsigned int i=0u;i<sizeof(float_bits)/sizeof(float_bits[0]);++i) {
        memcpy(&f.narrow_mm,&float_bits[i],4u); memcpy(&f.wide_mm,&float_bits[i],4u);
        memcpy(&f.leading_mm,&float_bits[i],4u); round_frame(&f);
    }
    /* Walk every bit in every full-width integer, including wrap endpoints. */
    f=fixture();
    uint32_t *full[]={&f.record.epoch,&f.record.sequence,&f.record.read_ms,
        (uint32_t *)&f.record.read_counts[0],(uint32_t *)&f.record.read_counts[1],
        (uint32_t *)&f.record.frame_end_counts[0],(uint32_t *)&f.record.frame_end_counts[1],
        &f.record.eligibility_revision,&f.index,&f.start_us,&f.edge_us,&f.frame_us,
        &f.eligible_us,&f.terminal_us,&f.control_age_ms,&f.record_age_ms,&f.spin_white};
    for (unsigned int i=0u;i<sizeof(full)/sizeof(full[0]);++i) {
        for (unsigned int b=0u;b<32u;++b) { *full[i]=UINT32_C(1)<<b; round_frame(&f); }
        *full[i]=UINT32_MAX; round_frame(&f); ++*full[i]; round_frame(&f);
    }
    f=fixture(); f.record.valid=2u; reject_frame(&f);
    f=fixture(); f.record.source=3u; reject_frame(&f);
    f=fixture(); f.record.direction=3u; reject_frame(&f);
    f=fixture(); f.record.version=2u; reject_frame(&f);
    f=fixture(); f.record.reserved=1u; reject_frame(&f);
    f=fixture(); f.record.letter='?'; reject_frame(&f);
    f=fixture(); f.leading_white=221u; reject_frame(&f);
    f=fixture(); f.rise=351u; reject_frame(&f);
    f=fixture(); f.fall=276u; reject_frame(&f);
    f=fixture();
    for (unsigned int i=11u;i<100u;++i) { f.reason=i; reject_frame(&f); }
    f.reason=108u; reject_frame(&f); f.reason=UINT32_MAX; reject_frame(&f);
    bcep_frame_t packed, saved; f=fixture(); assert(bcep_frame_pack(&saved,&f));
    packed=saved; f.record.command=5u;
    assert(!bcep_frame_pack(&packed,&f) && memcmp(&saved,&packed,sizeof(saved))==0);
    barcode_live_frame_t decoded=fixture(), unchanged=decoded;
    packed=saved; packed.words[21]|=UINT32_C(1)<<31u;
    assert(!bcep_frame_unpack(&decoded,&packed) && memcmp(&decoded,&unchanged,sizeof(decoded))==0);
    assert(!bcep_frame_pack(NULL,&f) && !bcep_frame_unpack(NULL,&saved));
    assert(!bcep_frame_pack(&packed,NULL) && !bcep_frame_unpack(&decoded,NULL));
    for (unsigned int i=45u;i<64u;++i) {
        packed=saved; bcep_put(packed.words,262u,6u,i); reject_packed(&packed);
    }
    for (unsigned int i=19u;i<32u;++i) {
        packed=saved; bcep_put(packed.words,311u,5u,i); reject_packed(&packed);
    }
    for (unsigned int i=694u;i<704u;++i) {
        packed=saved; bcep_put(packed.words,i,1u,1u); reject_packed(&packed);
    }
    packed=saved; bcep_put(packed.words,256u,3u,7u); reject_packed(&packed);
    packed=saved; bcep_put(packed.words,260u,2u,3u); reject_packed(&packed);
    packed=saved; bcep_put(packed.words,268u,2u,3u); reject_packed(&packed);
    packed=saved; bcep_put(packed.words,572u,8u,255u); reject_packed(&packed);
    packed=saved; bcep_put(packed.words,676u,9u,511u); reject_packed(&packed);
    packed=saved; bcep_put(packed.words,685u,9u,511u); reject_packed(&packed);
    uint32_t edges[BCEP_EDGE_WORDS]={0};
    for (unsigned int raw=0u;raw<4096u;++raw) {
        for (unsigned int flags=0u;flags<4u;++flags) {
            const unsigned int at=raw%64u;
            bcep_edge_t e={.us=UINT32_MAX,.motor_us=0x80000000u,
                .left=UINT32_MAX,.right=1u,.mm=-0.0f,.raw=(uint16_t)raw,
                .black=(uint8_t)(flags&1u),.phase=(uint8_t)(2u+(flags>>1u))};
            bcep_edge_t out;
            assert(bcep_edge_pack(edges,at,&e)); assert(bcep_edge_unpack(&out,edges,at));
            assert(memcmp(&e,&out,sizeof(e))==0);
        }
    }
    /* Every slot and crossing-word boundary remains independently intact. */
    for (unsigned int i=0u;i<64u;++i) {
        bcep_edge_t e={.us=i,.raw=(uint16_t)(i*63u),.phase=2u,.black=1u};
        assert(bcep_edge_pack(edges,i,&e));
    }
    for (unsigned int i=0u;i<64u;++i) {
        bcep_edge_t out; assert(bcep_edge_unpack(&out,edges,i));
        assert(out.us==i && out.raw==i*63u && out.phase==2u && out.black==1u);
    }
    bcep_edge_t bad={.phase=1u}; uint32_t before[BCEP_EDGE_WORDS]; memcpy(before,edges,sizeof(edges));
    assert(!bcep_edge_pack(edges,0u,&bad) && memcmp(before,edges,sizeof(edges))==0);
    assert(!bcep_edge_unpack(&bad,edges,64u));
    bad.phase=2u; bad.raw=4096u;
    assert(!bcep_edge_pack(edges,0u,&bad) && memcmp(before,edges,sizeof(edges))==0);
    bad.raw=UINT16_MAX;
    assert(!bcep_edge_pack(edges,0u,&bad) && memcmp(before,edges,sizeof(edges))==0);
    bad.raw=0u; bad.black=2u;
    assert(!bcep_edge_pack(edges,0u,&bad) && memcmp(before,edges,sizeof(edges))==0);
    assert(!bcep_edge_pack(edges,64u,&bad));
    for (unsigned int i=0u;i<sizeof(float_bits)/sizeof(float_bits[0]);++i) {
        bcep_edge_t e={.phase=3u}, out;
        memcpy(&e.mm,&float_bits[i],4u); assert(bcep_edge_pack(edges,63u,&e));
        assert(bcep_edge_unpack(&out,edges,63u) && memcmp(&out,&e,sizeof(e))==0);
    }
    f=fixture(); assert(bcep_frame_pack(&packed,&f));
    f.index=123u; assert(bcep_frame_pack(&replacement,&f));
    for (unsigned int i=0u;i<BCEP_FRAME_WORDS;++i) {
        app_board_publish(&board.sequence,board.words,&packed,sizeof(packed));
        interrupt_at=i; interrupt_read=true;
        bcep_frame_t scratch;
        decoded=unchanged;
        if (app_board_try_read(&board.sequence,board.words,&scratch,sizeof(scratch)))
            assert(bcep_frame_unpack(&decoded,&scratch));
        assert(!interrupt_read && memcmp(&decoded,&unchanged,sizeof(decoded))==0);
        assert(app_board_try_read(&board.sequence,board.words,&scratch,sizeof(scratch)));
        assert(bcep_frame_unpack(&decoded,&scratch) && decoded.index==123u);
    }
    /* A wrapped zero sequence is unavailable for one publication. */
    bcep_frame_t scratch;
    __atomic_store_n(&board.sequence,0u,__ATOMIC_SEQ_CST);
    assert(!app_board_try_read(&board.sequence,board.words,&scratch,sizeof(scratch)));
    __atomic_store_n(&board.sequence,1u,__ATOMIC_SEQ_CST);
    assert(!app_board_try_read(&board.sequence,board.words,&scratch,sizeof(scratch)));
    __atomic_store_n(&board.sequence,UINT32_MAX-1u,__ATOMIC_SEQ_CST);
    app_board_publish(&board.sequence,board.words,&packed,sizeof(packed));
    assert(board.sequence==0u);
    assert(!app_board_try_read(&board.sequence,board.words,&scratch,sizeof(scratch)));
    app_board_publish(&board.sequence,board.words,&replacement,sizeof(replacement));
    assert(app_board_try_read(&board.sequence,board.words,&scratch,sizeof(scratch)));
    assert(bcep_frame_unpack(&decoded,&scratch) && decoded.index==123u);
    puts("private barcode evidence packing PASS"); return 0;
}

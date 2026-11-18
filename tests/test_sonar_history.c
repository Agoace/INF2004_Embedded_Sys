#include <stdint.h>
#include <stdbool.h>
#include <string.h>
static uint32_t test_load(const uint32_t *p,int order);
#define __atomic_load_n test_load
#include "../app_program/subsystem5/sonar_service.c"
#undef __atomic_load_n
#define main original_service_test
#include "test_sonar_service.c"
#undef main
static const uint32_t *change_on_load;
static unsigned int change_countdown;
/* Torn-read injection: a whole writer publication runs while the reader is
 * between two payload word loads. */
static const uint32_t *tear_on_load;
static sonar_snapshot_t tear_value;
static bool tear_append;
static uint32_t test_load(const uint32_t *p,int order) {
    const uint32_t value=__atomic_load_n(p,order);
    if (p==change_on_load && --change_countdown==0u) {
        __atomic_store_n((uint32_t *)p,value+2u,__ATOMIC_SEQ_CST);
        change_on_load=NULL;
    }
    if (p==tear_on_load) {
        tear_on_load=NULL;
        board_publish(&tear_value,tear_append);
    }
    return value;
}
static sonar_snapshot_t pattern(uint32_t seed) {
    sonar_snapshot_t v;
    memset(&v,0,sizeof(v));
    v.generation=seed+1u;v.id=seed+2u;v.ping_sequence=seed+3u;
    v.result_sequence=seed+4u;v.trigger_ms=seed+5u;v.timestamp_ms=seed+6u;
    v.deadline_ms=seed+7u;v.ping_generation=seed+8u;v.ping_id=seed+9u;
    v.angle=(int8_t)((int)(seed%61u)-30);v.ping_angle=(int8_t)(30-(int)(seed%61u));
    v.echo=(ultrasonic_result_t)(seed*7u);v.status=(uint8_t)(seed%4u);
    v.pending=(seed&1u)!=0u;v.valid=(seed&2u)!=0u;
    return v;
}
static bool same_snapshot(const sonar_snapshot_t *a,const sonar_snapshot_t *b) {
    uint32_t wa[SONAR_PACK_WORDS],wb[SONAR_PACK_WORDS];
    return sonar_pack(a,wa) && sonar_pack(b,wb) && memcmp(wa,wb,sizeof(wa))==0;
}
/* CR1 A: torn reads after each payload word of the board and the history. */
static void test_torn_reads(void) {
    sonar_snapshot_t old=pattern(1000u),out;
    sonar_history_entry_t item;uint32_t cursor,missed;
    for (unsigned int k=0u;k<SONAR_PACK_WORDS;++k) {
        board_publish(&old,false);
        tear_value=pattern(2000u+k);tear_append=false;
        tear_on_load=&board_words[k];
        assert(sonar_snapshot_try_read(&out)==E_TMOUT && tear_on_load==NULL);
        assert(sonar_snapshot_try_read(&out)==E_OK && same_snapshot(&out,&tear_value));
    }
    for (unsigned int k=0u;k<SONAR_PACK_WORDS;++k) {
        (void)sonar_history_skip(&cursor);
        board_publish(&old,true);
        const uint32_t slot=(load_word(&history_count)-1u)%SONAR_HISTORY_CAPACITY;
        tear_value=pattern(3000u+k);tear_append=true;
        tear_on_load=&history_words[slot][k];
        assert(sonar_history_read(&cursor,&item,&missed)==E_TMOUT && tear_on_load==NULL);
        assert(sonar_history_read(&cursor,&item,&missed)==E_OK && same_snapshot(&item.result,&old));
        assert(sonar_history_read(&cursor,&item,&missed)==E_OK && same_snapshot(&item.result,&tear_value));
        assert(sonar_history_read(&cursor,&item,&missed)==E_NOEXS);
    }
}
int main(void) {
    /* CR1 A budget: 10 words per snapshot. */
    assert(sizeof(board_words)==40u && sizeof(history_words)==640u);
    sonar_history_entry_t item;uint32_t cursor=0u,missed=55u;
    sonar_snapshot_t board;
    assert(sonar_history_read(&cursor,&item,&missed)==E_OBJ && missed==0u);
    assert(sonar_history_skip(&cursor)==E_OBJ && cursor==0u);
    assert(sonar_history_skip(NULL)==E_PAR);
    assert(sonar_snapshot_try_read(&board)==E_OBJ);
    assert(sonar_history_read(NULL,&item,&missed)==E_PAR);
    assert(sonar_snapshot_try_read(NULL)==E_PAR);
    assert(original_service_test(0,NULL)==0);
    /* Original result flag was consumed; independent history remains readable. */
    assert(sonar_history_read(&cursor,&item,&missed)==E_OK && cursor==1u && missed==0u);
    assert(item.result.id==1u && item.result.trigger_ms==0u);
    const uint32_t count=load_word(&history_count);
    for (uint32_t i=0u;i<20u;++i) {
        sonar_snapshot_t value={.id=100u+i,.status=SONAR_CANCELLED};
        board_publish(&value,true);
    }
    assert(sonar_history_read(&cursor,&item,&missed)==E_OK);
    assert(missed==count+20u-1u-SONAR_HISTORY_CAPACITY && item.result.id==104u);
    while (sonar_history_read(&cursor,&item,&missed)==E_OK) assert(missed==0u);
    assert(cursor==count+20u && missed==0u);
    const sonar_history_entry_t saved=item;
    store_word(&history_guard,1u);missed=99u;
    assert(sonar_history_read(&cursor,&item,&missed)==E_TMOUT && missed==0u);
    assert(memcmp(&saved,&item,sizeof(item))==0 && cursor==count+20u);
    store_word(&history_guard,2u);
    change_on_load=&history_guard;change_countdown=1u;
    assert(sonar_history_read(&cursor,&item,&missed)==E_TMOUT);
    store_word(&board_guard,1u);
    assert(sonar_snapshot_try_read(&board)==E_TMOUT);
    store_word(&board_guard,2u);change_on_load=&board_guard;change_countdown=1u;
    assert(sonar_snapshot_try_read(&board)==E_TMOUT);
    assert(sonar_snapshot_try_read(&board)==E_OK && board.id==119u);
    /* IDLE skip: 40 results overwrite the ring; skip reports no loss and the
     * next read returns only a result published after the skip. */
    for (uint32_t i = 0u; i < 40u; ++i) {
        sonar_snapshot_t value = {.id = 200u + i, .status = SONAR_CANCELLED};
        board_publish(&value, true);
    }
    assert(sonar_history_skip(&cursor) == E_OK);
    assert(cursor == load_word(&history_count));
    assert(sonar_history_read(&cursor, &item, &missed) == E_NOEXS);
    {
        sonar_snapshot_t value = {.id = 300u, .status = SONAR_CANCELLED};
        board_publish(&value, true);
    }
    assert(sonar_history_read(&cursor, &item, &missed) == E_OK);
    assert((missed == 0u) && (item.result.id == 300u));
    /* Sequence zero is a valid publication after unsigned wrap. */
    store_word(&history_count,UINT32_MAX-1u);cursor=UINT32_MAX-1u;
    board_publish(&board,true);board_publish(&board,true);board_publish(&board,true);
    assert(sonar_history_read(&cursor,&item,&missed)==E_OK && cursor==UINT32_MAX);
    assert(sonar_history_read(&cursor,&item,&missed)==E_OK && cursor==0u);
    assert(sonar_history_read(&cursor,&item,&missed)==E_OK && cursor==1u);
    assert(sonar_history_read(&cursor,&item,&missed)==E_NOEXS);
    test_torn_reads();
    return 0;
}

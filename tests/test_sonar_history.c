#include <stdint.h>
static uint32_t test_load(const uint32_t *p,int order);
#define __atomic_load_n test_load
#include "../app_program/subsystem5/sonar_service.c"
#undef __atomic_load_n
#define main original_service_test
#include "test_sonar_service.c"
#undef main
static const uint32_t *change_on_load;
static unsigned int change_countdown;
static uint32_t test_load(const uint32_t *p,int order) {
    const uint32_t value=__atomic_load_n(p,order);
    if (p==change_on_load && --change_countdown==0u) {
        __atomic_store_n((uint32_t *)p,value+2u,__ATOMIC_SEQ_CST);
        change_on_load=NULL;
    }
    return value;
}
int main(void) {
    sonar_history_entry_t item;uint32_t cursor=0u,missed=55u;
    sonar_snapshot_t board;
    assert(sonar_history_read(&cursor,&item,&missed)==E_OBJ && missed==0u);
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
    assert(sonar_history_read(&cursor,&item,&missed)==E_OBJ && missed==0u);
    assert(memcmp(&saved,&item,sizeof(item))==0 && cursor==count+20u);
    store_word(&history_guard,2u);
    change_on_load=&history_guard;change_countdown=1u;
    assert(sonar_history_read(&cursor,&item,&missed)==E_OBJ);
    store_word(&board_guard,1u);
    assert(sonar_snapshot_try_read(&board)==E_TMOUT);
    store_word(&board_guard,2u);change_on_load=&board_guard;change_countdown=1u;
    assert(sonar_snapshot_try_read(&board)==E_TMOUT);
    assert(sonar_snapshot_try_read(&board)==E_OK && board.id==119u);
    /* Sequence zero is a valid publication after unsigned wrap. */
    store_word(&history_count,UINT32_MAX-1u);cursor=UINT32_MAX-1u;
    board_publish(&board,true);board_publish(&board,true);board_publish(&board,true);
    assert(sonar_history_read(&cursor,&item,&missed)==E_OK && cursor==UINT32_MAX);
    assert(sonar_history_read(&cursor,&item,&missed)==E_OK && cursor==0u);
    assert(sonar_history_read(&cursor,&item,&missed)==E_OK && cursor==1u);
    assert(sonar_history_read(&cursor,&item,&missed)==E_TMOUT);
    return 0;
}

#include <assert.h>
#include <stdint.h>
#include <string.h>
static int change_index=-1;
static void read_hook(const uint32_t *sequence,unsigned int index) {
    if (change_index>=0 && index==(unsigned int)change_index) {
        const uint32_t value=__atomic_load_n(sequence,__ATOMIC_SEQ_CST);
        __atomic_store_n((uint32_t *)sequence,value+2u,__ATOMIC_SEQ_CST);change_index=-1;
    }
}
#define APP_BOARD_READ_HOOK(sequence,index) read_hook(sequence,index)
#include "../app_program/platform/app_board.h"
typedef struct { unsigned char bytes[7]; } sample_t;
int main(void) {
    APP_BOARD(sample_t) board={0};
    const sample_t input={{0x12u,0x34u,0x56u,0x78u,0x9au,0xbcu,0xdeu}};
    sample_t scratch={{0}},cache=input;
    assert(APP_BOARD_WORDS(sample_t)==2u);
    assert(!app_board_try_read(&board.sequence,board.words,&scratch,sizeof(scratch)));
    app_board_publish(&board.sequence,board.words,&input,sizeof(input));
    assert(board.sequence==2u && board.words[0]==0x78563412u && board.words[1]==0x00debc9au);
    assert(app_board_try_read(&board.sequence,board.words,&scratch,sizeof(scratch)));
    assert(memcmp(&scratch,&input,sizeof(input))==0);
    __atomic_store_n(&board.sequence,3u,__ATOMIC_SEQ_CST);
    memset(&scratch,0xa5,sizeof(scratch));
    assert(!app_board_try_read(&board.sequence,board.words,&scratch,sizeof(scratch)));
    for (unsigned int i=0u;i<sizeof(scratch);++i) assert(scratch.bytes[i]==0xa5u);
    __atomic_store_n(&board.sequence,4u,__ATOMIC_SEQ_CST);
    for (int i=0;i<2;++i) {
        change_index=i;
        const bool stable=app_board_try_read(&board.sequence,board.words,&scratch,sizeof(scratch));
        assert(!stable);if (stable) cache=scratch;
        assert(change_index==-1 && memcmp(&cache,&input,sizeof(input))==0);
    }
    assert(app_board_try_read(&board.sequence,board.words,&scratch,sizeof(scratch)));
    __atomic_store_n(&board.sequence,UINT32_MAX-1u,__ATOMIC_SEQ_CST);
    app_board_publish(&board.sequence,board.words,&input,sizeof(input));
    assert(board.sequence==0u);
    assert(!app_board_try_read(&board.sequence,board.words,&scratch,sizeof(scratch)));
    app_board_publish(&board.sequence,board.words,&input,sizeof(input));
    assert(board.sequence==2u && app_board_try_read(&board.sequence,board.words,&scratch,sizeof(scratch)));
    /* Zero-byte objects are permitted; no payload is read or written. */
    app_board_publish(&board.sequence,board.words,&input,0u);
    assert(app_board_try_read(&board.sequence,board.words,&scratch,0u));
    return 0;
}

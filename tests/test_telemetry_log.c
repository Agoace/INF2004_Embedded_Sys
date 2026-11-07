/* Reuse the established telemetry task doubles. Include the implementation
 * only to stop at private publication boundaries and inspect committed data. */
#define main original_telemetry_test
#define bytes telemetry_test_bytes
#include "test_telemetry.c"
#undef bytes
#undef main
static bool change_log_guard;
static void log_read_hook(const uint32_t *sequence,unsigned int index) {
    if (change_log_guard && index==0u) {
        const uint32_t value=__atomic_load_n(sequence,__ATOMIC_SEQ_CST);
        __atomic_store_n((uint32_t *)sequence,value+2u,__ATOMIC_SEQ_CST);change_log_guard=false;
    }
}
#define APP_BOARD_READ_HOOK(sequence,index) log_read_hook(sequence,index)
#include "../app_program/subsystem1/telemetry.c"
static void send_message(telemetry_message_t *message) { assert(telemetry_send(message)==E_OK);pump(); }
int main(void) {
    telemetry_log_index_t opened={.generation=99u,.rows=88u,.events=77u,.lost=66u};
    const telemetry_log_index_t untouched=opened;
    uint8_t row[TELEMETRY_BYTES];telemetry_event_t event;
    assert(telemetry_log_open(NULL)==E_PAR);
    assert(telemetry_log_open(&opened)==E_TMOUT && memcmp(&opened,&untouched,sizeof(opened))==0);
    assert(telemetry_log_row(0u,0u,NULL)==E_PAR && telemetry_log_event(0u,0u,NULL)==E_PAR);
    assert(telemetry_log_row(0u,0u,row)==E_TMOUT && telemetry_log_event(0u,0u,&event)==E_TMOUT);
    assert(original_telemetry_test()==0);
    assert(telemetry_log_open(&opened)==E_OK && opened.generation==2u && opened.rows==2u && opened.events==1u);
    assert(telemetry_log_row(opened.generation,0u,row)==E_OK && row[0]==0x12u);
    assert(telemetry_log_row(opened.generation,1u,row)==E_OK && row[0]==0x34u);
    assert(telemetry_log_row(opened.generation,2u,row)==E_PAR);
    assert(telemetry_log_event(opened.generation,0u,&event)==E_OK && event.ms==10u && event.word==1u && event.kind==2u);
    assert(telemetry_log_event(opened.generation,1u,&event)==E_PAR);
    /* A partially copied next row, then the unadvertised count, remain hidden. */
    rows[2][0]=0xabu;
    assert(telemetry_log_row(opened.generation,2u,row)==E_PAR);
    for (unsigned int i=1u;i<TELEMETRY_BYTES;++i) rows[2][i]=(uint8_t)i;
    ++stats.rows;
    assert(telemetry_log_row(opened.generation,2u,row)==E_PAR);
    publish_index();
    assert(telemetry_log_row(opened.generation,2u,row)==E_OK && row[0]==0xabu && row[15]==15u);
    events[1]=(telemetry_event_t){.ms=50u,.kind=6u,.word=0x1234u,.detail=3u};
    ++stats.events;
    assert(telemetry_log_event(opened.generation,1u,&event)==E_PAR);
    publish_index();
    assert(telemetry_log_event(opened.generation,1u,&event)==E_OK && event.word==0x1234u && event.detail==3u);
    telemetry_log_index_t highwater;
    assert(telemetry_log_open(&highwater)==E_OK && highwater.rows==3u && highwater.events==2u);
    telemetry_message_t message={.kind=TELEMETRY_ROW,.data.row={0xcdu}};
    send_message(&message); /* Open readers do not set busy or stop appending. */
    telemetry_log_index_t current_index;
    assert(telemetry_log_open(&current_index)==E_OK && current_index.rows==4u && highwater.rows==3u);
    assert(telemetry_log_row(highwater.generation,3u,row)==E_OK && row[0]==0xcdu);
    const uint32_t guard=log_index.sequence;
    log_index.sequence=guard+1u;
    assert(telemetry_log_open(&opened)==E_TMOUT);
    assert(telemetry_log_row(highwater.generation,0u,row)==E_TMOUT);
    assert(telemetry_log_event(highwater.generation,0u,&event)==E_TMOUT);
    log_index.sequence=guard+2u;change_log_guard=true;
    opened=untouched;
    assert(telemetry_log_open(&opened)==E_TMOUT && !change_log_guard && memcmp(&opened,&untouched,sizeof(opened))==0);
    message.kind=TELEMETRY_BEGIN;send_message(&message);
    assert(telemetry_log_open(&opened)==E_OK && opened.generation==highwater.generation+1u && opened.rows==0u && opened.events==0u);
    assert(telemetry_log_row(highwater.generation,0u,row)==E_OBJ);
    assert(telemetry_log_event(highwater.generation,0u,&event)==E_OBJ);
    message.kind=TELEMETRY_ROW;message.data.row[0]=0xefu;send_message(&message);
    assert(telemetry_log_row(opened.generation,0u,row)==E_OK && row[0]==0xefu);
    assert(telemetry_log_row(highwater.generation,0u,row)==E_OBJ);
    assert(!stats.busy); /* Log access does not claim the USB-dump exclusion. */
    return 0;
}

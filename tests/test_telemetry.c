#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <string.h>
#include <tk/tkernel.h>
#include "../app_program/subsystem1/telemetry.h"
static FP entry;static jmp_buf end;
static telemetry_message_t queue[64];static unsigned int head,tail;
static bool full;static int irq_off;static unsigned int kr,ve,bytes,reports;
ID tk_cre_mbf(const T_CMBF *c) { assert(c->maxmsz==(INT)sizeof(telemetry_message_t) && c->bufsz==4096);return 1; }
ID app_start_task(FP f,PRI p,UW core,const char *name) { (void)name;assert(p==30 && core==1u);entry=f;return 7; }
ER tk_snd_mbf(ID id,const void *m,INT size,TMO timeout) {
    assert(id==1 && size==(INT)sizeof(telemetry_message_t) && timeout==TMO_POL && !irq_off);
    if (full) return E_TMOUT;
    queue[head++%64u]=*(const telemetry_message_t *)m;return E_OK;
}
INT tk_rcv_mbf(ID id,void *m,TMO timeout) {
    assert(id==1 && timeout==TMO_FEVR);
    if (tail==head) longjmp(end,1);
    *(telemetry_message_t *)m=queue[tail++%64u];return (INT)sizeof(telemetry_message_t);
}
void mock_di(UINT *irq) { assert(!irq_off);*irq=0u;irq_off=1; }
void mock_ei(UINT irq) { assert(irq==0u && irq_off);irq_off=0; }
ER tk_dly_tsk(RELTIM delay) { assert(delay==20u && !irq_off);return E_OK; }
const char *drv_watchdog_reset_name(void) { return "watchdog"; }
void app_task_report(void) { ++reports; }
int tm_printf(const UB *fmt,...) {
    va_list a;va_start(a,fmt);
    if (!strcmp((const char *)fmt,"KR %lu ")) ++kr;
    if (!strcmp((const char *)fmt,"VE %lu %u %u %u %u\n")) ++ve;
    if (!strcmp((const char *)fmt,"%02x")) {
        const unsigned int v=va_arg(a,unsigned int);
        if (bytes==0u) assert(v==0x12u);
        if (bytes==16u) assert(v==0x34u);
        ++bytes;
    }
    va_end(a);return 0;
}
static void pump(void) { if (setjmp(end)==0) ((void (*)(INT,void *))entry)(0,0);assert(!irq_off); }
int main(void) {
    telemetry_message_t m={.kind=TELEMETRY_BEGIN};
    assert(telemetry_send(&m)==E_OBJ);
    assert(telemetry_init()==E_OK && telemetry_start()==E_OK && telemetry_start()==E_OBJ);
    assert(telemetry_send(&m)==E_OK);pump();
    m.kind=TELEMETRY_ROW;full=true;assert(telemetry_send(&m)==E_TMOUT);full=false;
    for (unsigned int i=0u;i<TELEMETRY_ROWS+1u;++i) { assert(telemetry_send(&m)==E_OK);pump(); }
    telemetry_stats_t s;telemetry_stats(&s);
    assert(s.rows==TELEMETRY_ROWS && s.storage_full && s.lost==2u);
    m.kind=TELEMETRY_BEGIN;assert(telemetry_send(&m)==E_OK);pump();
    m.kind=TELEMETRY_ROW;m.data.row[0]=0x12u;assert(telemetry_send(&m)==E_OK);
    m.data.row[0]=0x34u;assert(telemetry_send(&m)==E_OK);
    m.data.row[0]=0xffu; /* Value copy, not a retained producer pointer. */
    m.kind=TELEMETRY_EVENT;m.data.event=(telemetry_event_t){.ms=10u,.word=1u,.kind=2u};
    assert(telemetry_send(&m)==E_OK);pump();
    m.kind=TELEMETRY_DUMP;memset(&m.data.info,0,sizeof(m.data.info));
    assert(telemetry_send(&m)==E_OK);telemetry_stats(&s);assert(s.busy);
    assert(telemetry_send(&m)==E_OBJ);pump();telemetry_stats(&s);
    assert(!s.busy && s.rows==2u && s.events==1u && kr==2u && ve==1u && bytes==32u && reports==1u);
    return 0;
}

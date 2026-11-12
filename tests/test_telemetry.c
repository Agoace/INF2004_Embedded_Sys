#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <string.h>
#include <tk/tkernel.h>
#include <tk/smp.h>
#include "../app_program/app.h"
#include "../app_program/subsystem1/telemetry.h"
#include "../app_program/vehicle/vehicle_task.h"
#include "../app_program/subsystem4/imu_settings.h"
static FP entry;static jmp_buf end;
static telemetry_message_t queue[64];static unsigned int head,tail;
#define irq_off test_smp_irq_mask
static bool full;static unsigned int kr,ve,bytes,reports;
unsigned int source_dumps;
static unsigned int source_lines,source_count_lines;
static int hump_result=E_NOEXS;static unsigned int hump_lines,hump_none,diag_lines,junction_lines;
ID tk_cre_mbf(const T_CMBF *c) { assert(c->maxmsz==(INT)sizeof(telemetry_message_t) && c->bufsz==4096);return 1; }
ID app_start_task(FP function, PRI priority, UW core, const char *name)
{
    (void)name;
    assert(priority == 30 && core == app_prc_core1());
    entry = function;
    return 7;
}
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
void mock_di(UINT *irq) { *irq = test_smp_irq_save(); }
void mock_ei(UINT irq) { test_smp_irq_restore(irq); }
ER tk_dly_tsk(RELTIM delay) { assert(delay==20u && !irq_off);return E_OK; }
const char *drv_watchdog_reset_name(void) { return "watchdog"; }
void app_task_report(void) { ++reports; }
int vehicle_hump_try_read(imu_hump_summary_t *out) {
    if (hump_result==E_OK) *out=(imu_hump_summary_t){.peak_01mm=108u,.count=2u,.flags=0x05u,.gated=3u};
    return hump_result;
}
int vehicle_hump_diag_try_read(imu_hump_diag_report_t *out) {
    if (hump_result==E_OK) *out=(imu_hump_diag_report_t){.valid=true};
    return hump_result;
}
int tm_printf(const UB *fmt,...) {
    va_list a;va_start(a,fmt);
    if (!strncmp((const char *)fmt,"[JUNCTION]",10u)) {
        assert(!strcmp(va_arg(a,const char *),"FAKE"));
        assert(!strcmp(va_arg(a,const char *),"B"));++junction_lines;
    }
    if (!strncmp((const char *)fmt,"[BC_ADMIT]",10u)) ++source_lines;
    if (strstr((const char *)fmt,"[JUNCTION_COUNTS]")) {
        ++source_count_lines;
        if (strstr((const char *)fmt,"line_age10")) {
            assert(va_arg(a,unsigned long)==1u);
            for (unsigned int i=0u;i<6u;++i) (void)va_arg(a,unsigned int);
            assert(va_arg(a,unsigned int)==3u);
        }
    }
    if (!strcmp((const char *)fmt,"KR %lu ")) ++kr;
    if (!strcmp((const char *)fmt,"VE %lu %u %u %u %u\n")) ++ve;
    if (!strncmp((const char *)fmt,"[robot] hump no finished run",28u)) ++hump_none;
    if (!strncmp((const char *)fmt,"[S4_HUMP_DIAG]",14u)) ++diag_lines;
    if (!strncmp((const char *)fmt,"[robot] hump peak_0.1mm",23u)) {
        const unsigned int peak=va_arg(a,unsigned int),count=va_arg(a,unsigned int);
        const unsigned int flags=va_arg(a,unsigned int),gated=va_arg(a,unsigned int);
        assert(peak==108u && count==2u && flags==5u && gated==3u);++hump_lines;
        assert(va_arg(a,unsigned int)==IMU_HUMP_SOURCE);
        assert(!strcmp(va_arg(a,const char *),IMU_HUMP_SOURCE_NAME));
    }
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
    assert(telemetry_send(&m)==E_OBJ && !telemetry_running());
    assert(telemetry_init()==E_OK && telemetry_start()==E_OK && telemetry_start()==E_OBJ && telemetry_running());
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
    assert(!s.busy && s.rows==2u && s.events==1u && kr==2u && ve==1u && bytes==32u && reports==2u);
    assert(hump_none==1u && hump_lines==0u);
    /* A finished run adds one hump line; KR1 rows and VE records unchanged. */
    hump_result=E_OK;m.kind=TELEMETRY_DUMP;assert(telemetry_send(&m)==E_OK);pump();
    assert(hump_none==1u && hump_lines==1u && diag_lines==1u && kr==4u && ve==2u);
    assert(junction_lines==0u);
    m.kind=TELEMETRY_JUNCTION;
    m.data.junction=(ja_result_t){.command=JA_RIGHT,.phase=JA_DONE,.source=1u};
    assert(telemetry_send(&m)==E_OK);pump();assert(junction_lines==1u);
    m.kind=TELEMETRY_DUMP;assert(telemetry_send(&m)==E_OK);pump();
    assert(junction_lines==2u); /* Saved result repeats at GP20, no VE row. */
    assert(source_dumps==3u);
    m.kind=TELEMETRY_BARCODE_DUMP;
    assert(telemetry_send(&m)==E_OK);pump();assert(source_dumps==4u);
    m.kind=TELEMETRY_JUNCTION_SOURCE;
    m.data.junction_source=(jlive_report_t){
        .frame={.epoch=1u,.sequence=3u,.source=2u,.letter='A'},
        .control={.epoch=1u},.max_delivery_ms=7u};
    assert(telemetry_send(&m)==E_OK);pump();assert(source_lines==1u);
    m.kind=TELEMETRY_JUNCTION_COUNTS;
    m.data.junction_counts=(jlive_counts_t){.epoch=1u};
    m.data.junction_counts.reason[JLIVE_LINE_STALE]=3u;
    assert(telemetry_send(&m)==E_OK);
    m.data.junction_counts.reason[JLIVE_LINE_STALE]=99u;
    pump();assert(source_count_lines==2u);
    m.kind=TELEMETRY_DUMP;
    assert(telemetry_send(&m)==E_OK);pump();
    assert(source_dumps==5u && source_lines==2u && source_count_lines==4u);
    assert(junction_lines==3u);
    return 0;
}

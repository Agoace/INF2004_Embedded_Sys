#include <assert.h>
#include <string.h>
#include "../app_program/platform/app_task.h"
static int create_error,start_error,deleted,prints;
static unsigned char *stack;
static SZ size;
void mock_di(UINT *irq) { *irq=0u; }
void mock_ei(UINT irq) { assert(irq==0u); }
INT tk_get_prc(void) { return 1; }
ID tk_cre_tsk(const T_CTSK *c) {
    assert((c->tskatr&(TA_USERBUF|TA_ASSPRC))==(TA_USERBUF|TA_ASSPRC));
    assert(c->assprc==TP_PRC1 && c->itskpri==10 && c->stksz==3072);
    stack=c->bufptr;size=c->stksz;
    assert(((uintptr_t)stack&7u)==0u);
    for (SZ i=0;i<size;++i) assert(stack[i]==0xa5u);
    return create_error ? E_SYS : 7;
}
ER tk_sta_tsk(ID id,INT code) { assert(id==7 && code==0);return start_error ? E_SYS : E_OK; }
ER tk_del_tsk(ID id) { assert(id==7);++deleted;return E_OK; }
int tm_printf(const UB *f,...) { (void)f;++prints;return 0; }
static void entry(void) { }
int main(void) {
    assert(app_task_start((FP)entry,10,2u,"motion_task")==E_CTX);
    assert(app_task_start((FP)entry,10,1u,"unknown")==E_PAR);
    create_error=1;assert(app_task_start((FP)entry,10,1u,"motion_task")==E_SYS);
    create_error=0;start_error=1;assert(app_task_start((FP)entry,10,1u,"motion_task")==E_SYS && deleted==1);
    start_error=0;assert(app_task_start((FP)entry,10,1u,"motion_task")==7);
    assert(app_task_start((FP)entry,10,1u,"motion_task")==E_OBJ);
    memset(stack+size-80,0,80u);
    app_stack_t s[6];assert(app_task_stacks(s,6u)==1u && s[0].used==80u && s[0].free==2992u);
    memset(stack+size-80,0xa5,80u);
    assert(app_task_stacks(s,6u)==1u && s[0].used==80u); /* Never forget high water. */
    app_task_report();assert(prints==1);
    return 0;
}

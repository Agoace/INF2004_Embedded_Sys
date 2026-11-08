#include <assert.h>
#include <string.h>
#include "../app_program/platform/app_task.h"
static int create_error,start_error,deleted,prints;
static unsigned char *stack;
static SZ size;
static PRI expected_priority = 10;
static SZ expected_size = 3072;
static ID expected_id = 7;
void mock_di(UINT *irq) { *irq=0u; }
void mock_ei(UINT irq) { assert(irq==0u); }
INT tk_get_prc(void) { return 1; }
ID tk_cre_tsk(const T_CTSK *c) {
    assert((c->tskatr&(TA_USERBUF|TA_ASSPRC))==(TA_USERBUF|TA_ASSPRC));
    assert(c->assprc == TP_PRC1 && c->itskpri == expected_priority &&
           c->stksz == expected_size);
    stack=c->bufptr;size=c->stksz;
    assert(((uintptr_t)stack&7u)==0u);
    for (SZ i=0;i<size;++i) assert(stack[i]==0xa5u);
    return create_error ? E_SYS : expected_id;
}
ER tk_sta_tsk(ID id, INT code)
{
    assert(id == expected_id && code == 0);
    return start_error ? E_SYS : E_OK;
}
ER tk_del_tsk(ID id)
{
    assert(id == expected_id);
    ++deleted;
    return E_OK;
}
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
    expected_priority = 14;
    expected_size = 2048;
    expected_id = 8;
    assert(app_task_start((FP)entry, 14, 1u, "imu_task") == 8);
    memset(stack + size - 160, 0, 160u);
    assert(app_task_stacks(s, 6u) == 2u);
    assert(strcmp(s[1].name, "imu_task") == 0);
    assert(s[1].used == 160u && s[1].free == 1888u);
    return 0;
}

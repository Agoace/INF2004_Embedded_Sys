/* M9: same-TU real registry access forces a held diagnostic lock. No
 * production test hook, linker wrapper or kernel-private access is used. */
#include <assert.h>
#include <tk/smp.h>
#include "../app_program/platform/app_task.c"

static unsigned char *region;
INT tk_get_prc(void) { return TP_PRC1; }
ID tk_cre_tsk(const T_CTSK *config)
{
    assert(config->bufptr && config->stksz == 8320);
    assert((config->tskatr & TA_USERBUF) != 0u);
    region = config->bufptr;
    return 42;
}
ER tk_sta_tsk(ID id, INT code)
{
    assert(id == 42 && code == 0);
    return E_OK;
}
ER tk_del_tsk(ID id) { (void)id; assert(0); return E_SYS; }
void mock_di(UINT *irq) { *irq = test_smp_irq_save(); }
void mock_ei(UINT irq) { test_smp_irq_restore(irq); }
int tm_printf(const UB *format, ...) { (void)format; return 0; }

static void check_free(uint32_t expected)
{
    app_stack_t row;
    assert(app_task_stacks(&row, 1u) == 1u);
    assert(row.free == expected && row.used == 8320u - expected);
    assert(test_smp_irq_mask == 0u);
}
static void entry(void) { }
int main(void)
{
    T_CTSK request = {0};
    request.task = (FP)entry;
    request.itskpri = 20;
    request.tskatr = TA_HLNG | TA_RNG1;
    request.stksz = 8192;
#if TK_SUPPORT_SMP
    request.tskatr |= TA_ASSPRC;
    request.assprc = app_prc_core1();
#endif
    assert(app_task_start_config(&request, "cyw43") == 42);
    region[512] = 0;
    check_free(512u);
    region[512] = 0xa5u; /* Lower new scan must not erase old peak. */
    assert(SpinLock(&stacks_lock) == E_OK); /* Simulated remote holder. */
    check_free(512u); /* Try-lock failure returns the latched mark. */
    region[255] = 0;
    check_free(512u); /* New deeper observation cannot latch while held. */
    assert(SpinUnlock(&stacks_lock) == E_OK);
    check_free(255u); /* Quiet scan latches the retained deep write. */
    region[255] = 0xa5u;
    assert(SpinLock(&stacks_lock) == E_OK);
    check_free(255u); /* Contended fallback cannot regress. */
    assert(SpinUnlock(&stacks_lock) == E_OK);
    check_free(255u);
    assert(test_smp_saves == test_smp_restores);
    return 0;
}

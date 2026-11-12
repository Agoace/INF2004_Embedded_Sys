/* Real registry placement helpers; host diagnostic doubles. */
#include <assert.h>
#include <string.h>
#include <tk/smp.h>
#include "../app_program/app.h"
#include "../app_program/platform/app_task.h"
#ifndef TM_MQTT
#define TM_MQTT 0
#endif

static int create_error, start_error, deleted, prints;
static unsigned char *stack;
static SZ size;
static PRI expected_priority = 10;
static SZ expected_size = 3072;
static ID expected_id = 7;
static UW expected_core = TP_PRC1;
static INT creator_core = TP_PRC1;
static unsigned int loads, wifi_reads;

void mock_di(UINT *irq) { *irq = test_smp_irq_save(); }
void mock_ei(UINT irq) { test_smp_irq_restore(irq); }
INT tk_get_prc(void) { return creator_core; }
ID tk_cre_tsk(const T_CTSK *config)
{
    assert(test_smp_irq_mask == 0u);
    assert((config->tskatr & TA_USERBUF) != 0u);
#if TK_SUPPORT_SMP
    assert((config->tskatr & TA_ASSPRC) != 0u);
    assert(config->assprc == expected_core);
#else
    assert((config->tskatr & TA_ASSPRC) == 0u && config->assprc == 0u);
#endif
    assert(config->itskpri == expected_priority);
    assert(config->stksz == expected_size);
    stack = config->bufptr;
    size = config->stksz;
    assert(((uintptr_t)stack & 7u) == 0u);
    for (SZ i = 0; i < size; ++i) {
        assert(stack[i] == 0xa5u);
    }
    return create_error ? E_SYS : expected_id;
}
ER tk_sta_tsk(ID id, INT code)
{
    assert(id == expected_id && code == 0 && test_smp_irq_mask == 0u);
    return start_error ? E_SYS : E_OK;
}
ER tk_del_tsk(ID id)
{
    assert(id == expected_id && test_smp_irq_mask == 0u);
    ++deleted;
    return E_OK;
}
uint16_t idle_meter_load_permille(void) { ++loads; return 100u; }
uint16_t idle_meter_load_permille_core1(void) { ++loads; return 200u; }
/* Regression trap: no production caller may use the removed SP estimate. */
uint32_t cyw43_utk_stack_free(void) { ++wifi_reads; return 1024u; }
int tm_printf(const UB *format, ...)
{
    assert(test_smp_irq_mask == 0u);
    assert(strstr((const char *)format, "[stack] wifi") == NULL);
    ++prints;
    return 0;
}
static void entry(void) { }

int main(void)
{
    assert(app_prc_core0() == TP_PRC1);
#if TK_SUPPORT_SMP && !APP_FORCE_CORE0
    assert(app_prc_core1() == TP_PRC1 + 1u);
#else
    assert(app_prc_core1() == TP_PRC1);
#endif
#if TK_SUPPORT_SMP
    creator_core = 2;
    assert(app_task_start((FP)entry, 10, 1u, "motion_task") == E_CTX);
    creator_core = 1;
#endif
    assert(app_task_start((FP)entry, 10, 2u, "motion_task") == E_CTX);
    assert(app_task_start((FP)entry, 10, 1u, "unknown") == E_PAR);
    assert(app_task_start((FP)entry, 20, app_prc_core1(), "cyw43") == E_PAR);
    create_error = 1;
    assert(app_task_start((FP)entry, 10, 1u, "motion_task") == E_SYS);
    create_error = 0;
    start_error = 1;
    assert(app_task_start((FP)entry, 10, 1u, "motion_task") == E_SYS);
    assert(deleted == 1);
    start_error = 0;
    assert(app_task_start((FP)entry, 10, 1u, "motion_task") == 7);
    assert(app_task_start((FP)entry, 10, 1u, "motion_task") == E_OBJ);
    memset(stack + size - 80, 0, 80u);
    app_stack_t rows[APP_TASK_MAX_SLOTS];
    assert(app_task_stacks(rows, APP_TASK_MAX_SLOTS) == 1u);
    assert(rows[0].used == 80u && rows[0].free == 2992u);
    memset(stack + size - 80, 0xa5, 80u);
    assert(app_task_stacks(rows, APP_TASK_MAX_SLOTS) == 1u);
    assert(rows[0].used == 80u); /* Never forget high water. */
    app_task_report();
    assert(prints == (TM_MQTT ? 2 : 1));
    assert(app_task_start((FP)entry, 10, 1u, NULL) == E_PAR);
    expected_id = 8;
#if APP_BENCH
    expected_priority = 30;
    expected_size = 4096;
    const char *secondary = "motion_bench";
    assert(app_task_start((FP)entry, 14, 1u, "imu_task") == E_PAR);
#else
    expected_priority = 14;
    expected_size = 2048;
    expected_core = app_prc_core1();
    const char *secondary = "imu_task";
    unsigned char *const motion_stack = stack;
    assert(app_task_start((FP)entry, 30, 1u, "motion_bench") == E_PAR);
    assert(stack == motion_stack);
#endif
    assert(app_task_start((FP)entry, expected_priority, expected_core,
                          secondary) == 8);
    assert(app_task_start((FP)entry, expected_priority, expected_core,
                          secondary) == E_OBJ);
    memset(stack + size - 160, 0, 160u);
    assert(app_task_stacks(rows, APP_TASK_MAX_SLOTS) == 2u);
    assert(strcmp(rows[1].name, secondary) == 0);
    assert(rows[1].used == 160u);
    assert(rows[1].free == (uint32_t)expected_size - 160u);
    assert(rows[1].core == (uint8_t)(expected_core - TP_PRC1));
    const char *names[] = {
        "line_task", "sonar_task", "vehicle_task", "telemetry_task"
    };
    const SZ sizes[] = {2048, 2048, 4096, 4096};
    for (unsigned int i = 0u; i < 4u; ++i) {
        expected_priority = 20;
        expected_size = sizes[i];
        expected_id = (ID)(9u + i);
        expected_core = i == 3u ? app_prc_core1() : app_prc_core0();
        assert(app_task_start((FP)entry, expected_priority, expected_core,
                              names[i]) == expected_id);
    }
    assert(app_task_stacks(rows, APP_TASK_MAX_SLOTS) == 6u);
    assert(app_task_stacks(rows, 1u) == 1u);
#if TM_MQTT
    expected_priority = 31;
    expected_size = 2048;
    expected_id = 13;
    expected_core = app_prc_core0();
    assert(app_task_start((FP)entry, 31, expected_core, "health_task") == 13);
    expected_priority = 32;
    expected_size = 768;
    expected_id = 14;
    assert(app_task_start((FP)entry, 32, expected_core, "idle_meter") == 14);
#if TK_SUPPORT_SMP
    expected_id = 15;
    expected_core = app_prc_core1();
    assert(app_task_start((FP)entry, 32, expected_core, "idle_meter_1") == 15);
    assert(app_task_stacks(rows, APP_TASK_MAX_SLOTS) == 9u);
    assert(rows[8].core == (uint8_t)(expected_core - TP_PRC1));
#else
    assert(app_task_start((FP)entry, 32, 1u, "idle_meter_1") == E_PAR);
    assert(app_task_stacks(rows, APP_TASK_MAX_SLOTS) == 8u);
#endif
    assert(loads == (TK_SUPPORT_SMP ? 2u : 1u) && wifi_reads == 0u);
#else
    assert(app_task_start((FP)entry, 31, 1u, "health_task") == E_PAR);
    assert(app_task_start((FP)entry, 32, 1u, "idle_meter") == E_PAR);
    assert(loads == 0u && wifi_reads == 0u);
#endif
    assert(test_smp_irq_mask == 0u && test_smp_saves == test_smp_restores);
    return 0;
}

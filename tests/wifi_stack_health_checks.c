/* M9: run the real Health refresh once against the real stack registry. */
#if TM_MQTT
#include <assert.h>
#include <setjmp.h>
#include <string.h>
#include "../app_program/app.h"
#include "../app_program/platform/app_task.h"
#include "../app_program/platform/drv_watchdog.h"
#include "../app_program/subsystem1/robot_health.h"
#include "../app_program/subsystem1/robot_network.h"
#include "../app_program/subsystem1/telemetry.h"

static FP health_entry;
static jmp_buf refreshed;
ID app_start_task(FP entry, PRI priority, UW core, const char *name)
{
    assert(priority == 31 && core == TP_PRC1);
    assert(strcmp(name, "health_task") == 0);
    health_entry = entry;
    return app_task_start(entry, priority, core, name);
}
uint32_t drv_time_us(void) { return 1234000u; }
uint32_t drv_time_ms(void) { return 1234u; }
drv_watchdog_reset_t drv_watchdog_last_reset(void)
{
    return DRV_RESET_POWER_RUN;
}
void idle_meter_sample_all(void) { }
void telemetry_stats(telemetry_stats_t *out) { memset(out, 0, sizeof(*out)); }
void robot_network_metrics(rm_heartbeat_t *out) { (void)out; }
ID tk_cre_cyc(const T_CCYC *config)
{
    assert(config->cyctim == 1000u);
    return 99;
}
ER tk_sta_cyc(ID id) { assert(id == 99); return E_OK; }
ER tk_wup_tsk(ID id) { (void)id; return E_OK; }
INT tk_can_wup(ID id) { (void)id; return 0; }
ER tk_slp_tsk(TMO timeout)
{
    assert(timeout == TMO_FEVR);
    longjmp(refreshed, 1);
}
void wifi_stack_health_check(uint32_t expected_free)
{
    assert(robot_health_init() == E_OK);
    assert(robot_health_start() == E_OK && health_entry);
    if (setjmp(refreshed) == 0) {
        ((void (*)(INT, void *))health_entry)(0, NULL);
        assert(0);
    }
    rm_heartbeat_t heartbeat;
    uint32_t at;
    assert(robot_health_try_read(&heartbeat, &at) == E_OK);
    assert(at == 1234000u && heartbeat.stack_min_free == expected_free);
}
#endif

/* M5: run the actual TEST assertion with deterministic consumer schedules. */
#include <assert.h>
#include <stdarg.h>
#include <string.h>
#include <tk/tkernel.h>
#include "../app_program/subsystem1/telemetry.h"

int telemetry_test_probe_send(uint32_t token);
int telemetry_test_probe_result(uint32_t token, bool *discarded);
#include "../app_program/subsystem1/telemetry_contention_test.c"

static telemetry_stats_t model;
static bool full_seen, waiting, noise, reset_probe;
static unsigned int retries = 2u;
static uint32_t completed;
static unsigned int waits;

int tm_printf(const UB *format, ...)
{
    (void)format;
    return 0;
}
UW drv_time_us(void) { return 0u; }
bool telemetry_running(void) { return true; }
void telemetry_test_hold_payload_lock(void) { }
void telemetry_test_release_payload_lock(void) { }
void telemetry_stats(telemetry_stats_t *out)
{
    *out = model;
    if (model.storage_full) full_seen = true;
}
static int probe(uint32_t token)
{
    if (retries != 0u) {
        --retries;
        ++model.lost;
        return E_TMOUT;
    }
    waiting = true;
    if (!noise) {
        completed = token;
        if (!reset_probe) ++model.lost; /* consumer runs before send returns */
        else { model.rows = 1u; model.storage_full = false; }
    }
    return E_OK;
}
int telemetry_send(const telemetry_message_t *message)
{
    if (message->kind == TELEMETRY_BEGIN) {
        model = (telemetry_stats_t){0};
        full_seen = false;
    } else if (message->kind == TELEMETRY_ROW) {
        if (full_seen) return probe(1u);
        if (model.rows == TELEMETRY_ROWS) {
            model.storage_full = true;
            ++model.lost;
        } else ++model.rows;
    }
    return E_OK;
}
int telemetry_test_probe_send(uint32_t token) { return probe(token); }
int telemetry_test_probe_result(uint32_t token, bool *discarded)
{
    if (completed != token) return E_TMOUT;
    *discarded = !reset_probe;
    return E_OK;
}
ER tk_dly_tsk(RELTIM delay)
{
    (void)delay;
    ++waits;
    assert(waits < 2000u);
    if (waiting && noise) {
        ++model.lost; /* unrelated queued row; selected probe NOT processed */
        waiting = false;
    }
    return E_OK;
}
int main(int argc, char **argv)
{
    noise = argc > 1 && !strcmp(argv[1], "noise");
    reset_probe = argc > 1 && !strcmp(argv[1], "reset");
    test_storage_full();
    if (noise || reset_probe) assert(fails > 0u);
    else assert(fails == 0u);
    return 0;
}

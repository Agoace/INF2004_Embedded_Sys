/* M5: real FIFO consumer, ordinary/probe rows share the storage decision. */
#define main original_telemetry_main
#define tk_snd_mbf base_send
#include "test_telemetry.c"
#undef tk_snd_mbf
#undef main

static bool immediate;
uint32_t drv_time_ms(void) { return 123u; }
ER tk_snd_mbf(ID id, const void *message, INT size, TMO timeout)
{
    const ER result = base_send(id, message, size, timeout);
    if (result == E_OK && immediate) pump();
    return result;
}
static void expect_result(uint32_t token, bool expected)
{
    bool discarded = !expected;
    assert(telemetry_test_probe_result(token, &discarded) == E_OK);
    assert(discarded == expected);
}
int main(void)
{
    bool untouched = true;
    assert(telemetry_init() == E_OK && telemetry_start() == E_OK);
    assert(telemetry_test_probe_send(0u) == E_PAR);
    assert(telemetry_test_probe_result(0u, &untouched) == E_PAR);
    assert(telemetry_test_probe_result(1u, NULL) == E_PAR);
    assert(telemetry_test_probe_result(1u, &untouched) == E_TMOUT);
    assert(untouched);
    telemetry_message_t message = {.kind = TELEMETRY_BEGIN};
    assert(telemetry_send(&message) == E_OK); pump();
    message.kind = TELEMETRY_ROW;
    message.data.row[0] = 0x12u;
    assert(telemetry_send(&message) == E_OK);
    assert(telemetry_test_probe_send(1u) == E_OK);
    assert(telemetry_test_probe_result(1u, &untouched) == E_TMOUT);
    assert(untouched);
    pump();
    expect_result(1u, false); /* not full: the probe was stored */
    telemetry_log_index_t index;
    assert(telemetry_log_open(&index) == E_OK && index.rows == 2u);
    uint8_t row[TELEMETRY_BYTES];
    assert(telemetry_log_row(index.generation, 0u, row) == E_OK);
    assert(row[0] == 0x12u);
    for (unsigned int i = 2u; i <= TELEMETRY_ROWS; ++i) {
        assert(telemetry_send(&message) == E_OK); pump();
    }
    /* Older queued excess rows cannot acknowledge an unprocessed probe. */
    for (unsigned int i = 0u; i < 3u; ++i)
        assert(telemetry_send(&message) == E_OK);
    assert(telemetry_test_probe_send(2u) == E_OK);
    assert(telemetry_test_probe_result(2u, &untouched) == E_TMOUT);
    pump();
    expect_result(2u, true);
    assert(telemetry_test_probe_result(99u, &untouched) == E_TMOUT);
    full = true;
    assert(telemetry_test_probe_send(3u) == E_TMOUT);
    full = false;
    assert(telemetry_test_probe_result(3u, &untouched) == E_TMOUT);
    /* Complete before send returns, including after a failed enqueue. */
    immediate = true;
    assert(telemetry_test_probe_send(3u) == E_OK);
    expect_result(3u, true);
    immediate = false;
    message.kind = TELEMETRY_BEGIN;
    assert(telemetry_send(&message) == E_OK);
    assert(telemetry_test_probe_send(4u) == E_OK);
    pump();
    expect_result(4u, false); /* BEGIN/reset: do not claim a drop */
    telemetry_stats_t state;
    telemetry_stats(&state);
    assert(state.rows == 1u && !state.storage_full);
    assert(sizeof(telemetry_message_t) == 116u);
    assert(sizeof(telemetry_info_t) == 108u);
    assert(!test_smp_irq_mask && test_smp_saves == test_smp_restores);
    return 0;
}

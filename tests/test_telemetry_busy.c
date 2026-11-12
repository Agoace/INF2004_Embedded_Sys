/* M3: real admission with the consumer deliberately paused. */
#define main original_telemetry_main
#include "test_telemetry.c"
#undef main
#include <stdlib.h>

int main(int argc, char **argv)
{
    const unsigned int kind = argc > 1 ?
        (unsigned int)strtoul(argv[1], NULL, 10) : TELEMETRY_BEGIN;
    assert(kind <= TELEMETRY_JUNCTION && kind != TELEMETRY_DUMP);
    assert(telemetry_init() == E_OK && telemetry_start() == E_OK);
    telemetry_message_t dump = {.kind = TELEMETRY_DUMP};
    assert(telemetry_send(&dump) == E_OK);
    telemetry_stats_t before, after;
    telemetry_stats(&before);
    assert(before.busy);
    const unsigned int queued = head;
    const telemetry_message_t blocked = {.kind = kind};
    assert(telemetry_send(&blocked) == E_OBJ);
    telemetry_stats(&after);
    assert(after.busy && after.lost == before.lost + 1u && head == queued);
    assert(telemetry_send(&dump) == E_OBJ);
    telemetry_stats(&after);
    assert(after.busy && after.lost == before.lost + 2u && head == queued);
    pump();
    telemetry_stats(&after);
    assert(!after.busy);
    /* Failed DUMP enqueue releases only its own reservation. */
    full = true;
    assert(telemetry_send(&dump) == E_TMOUT);
    telemetry_stats(&after);
    assert(!after.busy && after.lost == before.lost + 3u);
    full = false;
    assert(telemetry_send(&blocked) == E_OK);
    assert(head == queued + 1u);
    return 0;
}

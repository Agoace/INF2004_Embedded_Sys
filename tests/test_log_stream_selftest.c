/* M4: the core-0 TEST runner must not mutate a live Wi-Fi-owned stream. */
#include <assert.h>
#include <stdarg.h>
#include <string.h>
#include <tk/tkernel.h>
#include "../app_program/subsystem1/log_stream.h"

void log_stream_admission_selftest(void);
static unsigned int lines;
static const uint32_t live_boot_id = 0xa1b2c3d4u;
static const uint32_t pending_id = 987u;
static uint8_t frame_sentinel[32];

static void forbidden(void)
{
    assert(0 && "core-0 TEST runner touched Wi-Fi-owned stream");
}
void log_stream_init(uint32_t boot_id) { (void)boot_id; forbidden(); }
int log_stream_dump_request(uint32_t id) { (void)id; forbidden(); return -1; }
bool log_stream_dump_active(void) { forbidden(); return false; }
void log_stream_service(log_stream_publish_fn publish, bool live)
{
    (void)publish; (void)live; forbidden();
}
void log_stream_complete(uint32_t token, bool success)
{
    (void)token; (void)success; forbidden();
}
int tm_printf(const UB *format, ...)
{
    assert(strstr((const char *)format, "SKIP") != NULL);
    ++lines;
    return 0;
}
int main(void)
{
    memset(frame_sentinel, 0xa5, sizeof(frame_sentinel));
    log_stream_admission_selftest();
    assert(lines == 1u && live_boot_id == 0xa1b2c3d4u && pending_id == 987u);
    for (unsigned int i = 0u; i < sizeof(frame_sentinel); ++i)
        assert(frame_sentinel[i] == 0xa5u);
    return 0;
}

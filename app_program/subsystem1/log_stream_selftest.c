/*
 * log_stream_selftest.c - admission interleaving tests for the DUMP state split.
 *
 * RT-0060 asked for three interleavings:
 *   1. Health's active read, before and after a request;
 *   2. the pending-to-active handoff performed by the service step;
 *   3. a request racing END-to-IDLE: drive a whole DUMP to completion, then
 *      submit a second request and see it accepted.
 *
 * These drive the real public API - request, service and complete - rather than a
 * test-only hook, so they exercise the same code the Vehicle and Wi-Fi tasks use.
 * On-target only: the stream sits on the kernel and on the telemetry log.
 */
/*
 * The T-Kernel headers define size_t themselves, so this tree suppresses the
 * toolchain definition with PROHIBIT_DEF_SIZE_T before stddef is reached - the
 * same order robot_mqtt.c and idle_meter.c use.
 */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "../app.h"

#if APP_TEST

#include <stdbool.h>
#include "log_stream.h"

static uint32_t last_token;
static int publish_calls;

/* Accept every frame and remember its token so the test can acknowledge it. */
static int capture_publish(const uint8_t *data, size_t length, uint8_t qos,
                           uint32_t token)
{
    (void)data;
    (void)length;
    (void)qos;
    last_token = token;
    ++publish_calls;
    return 0;
}

static void expect(const char *what, bool ok)
{
    tm_printf((UB *)"[dump] %s %s\n", ok ? "ok  " : "FAIL", what);
}

void log_stream_admission_selftest(void)
{
    log_stream_init(1u);

    /* 1. Health's active read, and a request being accepted. */
    expect("idle before any request", !log_stream_dump_active());
    expect("first request accepted", log_stream_dump_request(11u) == 0);
    expect("active while pending", log_stream_dump_active());
    /* A second request must still be rejected while one is in flight. */
    expect("second request rejected", log_stream_dump_request(12u) < 0);

    /* 2 and 3. Handoff by the service step, then run the DUMP to completion. */
    publish_calls = 0;
    last_token = 0u;
    for (unsigned int i = 0u; i < 200u; ++i) {
        log_stream_service(capture_publish, false);
        if (publish_calls != 0) {
            log_stream_complete(last_token, true);
            publish_calls = 0;
        }
        if (!log_stream_dump_active()) {
            break;
        }
    }

    expect("transfer finished", !log_stream_dump_active());
    expect("request accepted after END", log_stream_dump_request(13u) == 0);
    expect("active again", log_stream_dump_active());

    tm_printf((UB *)"[dump] admission selftest done\n");
}

#endif /* APP_TEST */

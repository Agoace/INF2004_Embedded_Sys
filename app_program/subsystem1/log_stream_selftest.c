/*
 * log_stream_selftest.c - report the scope of DUMP admission tests.
 *
 * M4: the TEST runner is not the Wi-Fi owner. Do not initialize, service,
 * acknowledge or submit requests to the live stream here. Saving/restoring
 * its state would also race the owner. The isolated host fixture exercises
 * the real implementation and completes both requests before teardown.
 */
#include "../app.h"

#if APP_TEST
void log_stream_admission_selftest(void)
{
#if TM_MQTT
    tm_printf((UB *)"[dump] SKIP live-instance test: admission lifecycle is "
                    "host-only; no cross-core hardware result\n");
#else
    tm_printf((UB *)"[dump] SKIP: no MQTT in this profile\n");
#endif
}
#endif /* APP_TEST */

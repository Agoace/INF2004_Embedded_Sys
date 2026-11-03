#include "sim_runtime.h"
#include "sim_devices.h"
#include "sim_world.h"
#include "pico/stdlib.h"
#include "servo.h"
#include "ultrasonic.h"
#include "obstacle_scan.h"
#include <assert.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

static unsigned int falls;
static void falling_edge(void) { ++falls; }
static void result(void) {
    const ultrasonic_debug_t d=ultrasonic_debug_latest();
    sim_event("probe_result","\"word\":%u,\"latency_us\":%" PRIu32 ",\"pulse_us\":%" PRIu32 ",\"distance_mm\":%" PRIu32 ",\"falls\":%u",
        (unsigned int)ultrasonic_latest(),d.rise_latency_us,d.pulse_us,d.distance_mm,falls);
}
int firmware_main(void) {
    const char *mode=getenv("SIM_PROBE_MODE");
    assert(mode!=NULL);
    ultrasonic_init(3u,2u,falling_edge);
    servo_init(15u);
    assert(ultrasonic_latest()==ULTRASONIC_RESULT_NONE);
    assert(ultrasonic_debug_latest().pulse_us==0u);
    assert(servo_set_pulse_us(0u)==1100u && sim_now()==0u);
    assert(servo_set_pulse_us(UINT16_MAX)==1900u && sim_now()==0u);
    (void)servo_set_pulse_us(1500u);
    if (strcmp(mode,"scan")==0) {
        obstacle_scan_init(15u);
        assert(obstacle_scan_start() && !obstacle_scan_start());
        obstacle_scan_event_t ev=OBSTACLE_SCAN_NONE;
        while (obstacle_scan_busy()) {
            ev=obstacle_scan_service();
            sleep_us(100u);
        }
        assert(ev==OBSTACLE_SCAN_DONE && obstacle_scan_progress()==13u);
        for (unsigned int i=0;i<13u;++i) {
            sim_event("scan_point","\"index\":%u,\"pulse\":%u,\"word\":%u",i,
                (unsigned int)obstacle_scan_pulse_us(i),(unsigned int)obstacle_scan_profile()->point[i]);
        }
    } else if (strcmp(mode,"faults")==0) {
        for (unsigned int i=0;i<7u;++i) {
            assert(ultrasonic_trigger() && ultrasonic_busy());
            sleep_us(1999u); assert(!ultrasonic_service());
            sleep_us(1u); assert(ultrasonic_service());
            assert(ultrasonic_result_status(ultrasonic_latest())==ULTRASONIC_STATUS_NO_RISE);
            assert(ultrasonic_result_streak(ultrasonic_latest())==(i<5u?i+1u:5u));
            result(); sleep_us(58000u);
        }
        assert(sim_now()==420000u);
        assert(ultrasonic_trigger() && !ultrasonic_busy() && !ultrasonic_service());
        assert(ultrasonic_result_status(ultrasonic_latest())==ULTRASONIC_STATUS_STUCK_HIGH);
        assert(ultrasonic_result_streak(ultrasonic_latest())==5u);
        result(); sleep_us(60000u);
        assert(ultrasonic_trigger()); sleep_us(4000u); assert(ultrasonic_service());
        assert(ultrasonic_result_status(ultrasonic_latest())==ULTRASONIC_STATUS_VALID);
        assert(ultrasonic_result_streak(ultrasonic_latest())==0u);
        result(); sleep_us(56000u);
        assert(ultrasonic_trigger()); sleep_us(2000u); assert(ultrasonic_service());
        assert(ultrasonic_result_streak(ultrasonic_latest())==1u);
        result(); sleep_us(58000u);
        assert(ultrasonic_trigger()); sleep_us(3215u); assert(ultrasonic_service());
        assert(ultrasonic_result_status(ultrasonic_latest())==ULTRASONIC_STATUS_NO_RETURN);
        assert(ultrasonic_result_streak(ultrasonic_latest())==0u);
        result();
    } else {
        if (strcmp(mode,"wrap")==0) { sleep_us(UINT64_C(4294966296)); }
        const uint64_t at=sim_now();
        assert(ultrasonic_trigger() && ultrasonic_busy());
        assert(!ultrasonic_trigger());
        (void)servo_set_pulse_us(1900u); /* Must not change the in-flight geometry. */
        const uint64_t due=sim_devices_deadline();
        sleep_us(due-sim_now()-1u);
        assert(!ultrasonic_service() && ultrasonic_latest()==ULTRASONIC_RESULT_NONE);
        sleep_us(1u);
        assert(ultrasonic_latest()==ULTRASONIC_RESULT_NONE && ultrasonic_busy());
        assert(ultrasonic_service() && !ultrasonic_busy() && !ultrasonic_service());
        result();
        (void)servo_set_pulse_us(1500u);
        assert(!ultrasonic_trigger());
        sleep_us(at+59999u-sim_now()); assert(!ultrasonic_trigger());
        sleep_us(1u); assert(ultrasonic_trigger());
        assert(sim_now()==at+60000u);
    }
    sim_event("device_probe_passed","\"passed\":true");
    sleep_us(UINT64_MAX);
    return 1;
}

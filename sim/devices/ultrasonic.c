/* Timed API replacement: geometry is sampled only when a trigger is accepted. */
#include "sim_devices.h"
#include "sim_runtime.h"
#include "sim_world.h"
#include "ultrasonic.h"
#include <inttypes.h>
#include <math.h>
#include <stdlib.h>

typedef struct { uint64_t at; unsigned int mode; } fault_t;
static fault_t *faults;
static size_t fault_count, next_fault;
static unsigned int fault_mode;
static bool busy, triggered, completed, pending_too_near;
static uint64_t trigger_us, ready_us;
static uint8_t streak;
static ultrasonic_echo_hook_t echo_hook;
static ultrasonic_result_t latest=ULTRASONIC_RESULT_NONE;
static ultrasonic_debug_t debug, pending_debug;
static ultrasonic_status_t pending_status;

void sim_devices_read(FILE *input) {
    if (fscanf(input,"%zu",&fault_count)!=1 || fault_count>100000u) { sim_error("invalid fault count"); }
    faults=calloc(fault_count==0u?1u:fault_count,sizeof *faults);
    if (faults==NULL) { sim_error("fault allocation failed"); }
    for (size_t i=0;i<fault_count;++i) {
        if (fscanf(input,"%" SCNu64 " %u",&faults[i].at,&faults[i].mode)!=2 || faults[i].mode>3u ||
            (i!=0u && faults[i].at<faults[i-1u].at)) { sim_error("invalid fault event"); }
    }
}
void sim_devices_close(void) { free(faults); faults=NULL; }
uint64_t sim_devices_deadline(void) {
    uint64_t next=busy && !completed ? ready_us : UINT64_MAX;
    if (next_fault<fault_count && faults[next_fault].at<next) { next=faults[next_fault].at; }
    return next;
}
void sim_devices_tick(void) {
    while (next_fault<fault_count && faults[next_fault].at<=sim_now()) {
        fault_mode=faults[next_fault++].mode;
        sim_event("ultrasonic_fault","\"mode\":%u",fault_mode);
    }
    if (busy && !completed && sim_now()>=ready_us) {
        completed=true;
        sim_event("echo_complete","\"ready_us\":%" PRIu64,ready_us);
        if ((pending_debug.pulse_us!=0u || pending_status==ULTRASONIC_STATUS_VALID) && echo_hook!=NULL) { echo_hook(); }
    }
}
static void publish(ultrasonic_status_t status, ultrasonic_debug_t timing) {
    if (status==ULTRASONIC_STATUS_VALID && timing.distance_mm>ULTRASONIC_DISTANCE_MAX_MM) { status=ULTRASONIC_STATUS_NO_RETURN; }
    if (status==ULTRASONIC_STATUS_VALID || status==ULTRASONIC_STATUS_NO_RETURN) { streak=0; }
    else if (streak<ULTRASONIC_FAULT_STREAK) { ++streak; }
    const uint16_t distance=status==ULTRASONIC_STATUS_VALID && !pending_too_near && timing.distance_mm>=ULTRASONIC_DISTANCE_MIN_MM ? (uint16_t)timing.distance_mm : 0u;
    latest=(ultrasonic_result_t)((unsigned int)distance | ((unsigned int)status<<ULTRASONIC_STATUS_SHIFT) | ((unsigned int)streak<<ULTRASONIC_STREAK_SHIFT));
    debug=timing;
    sim_event("ultrasonic_result","\"word\":%u,\"ready_us\":%" PRIu64 ",\"rise_latency_us\":%" PRIu32 ",\"pulse_us\":%" PRIu32 ",\"distance_mm\":%" PRIu32,
        (unsigned int)latest,ready_us,debug.rise_latency_us,debug.pulse_us,debug.distance_mm);
}
void ultrasonic_init(unsigned int trigger_pin,unsigned int echo_pin,ultrasonic_echo_hook_t hook) {
    (void)trigger_pin; (void)echo_pin; echo_hook=hook;
    busy=triggered=completed=false; streak=0;
    latest=ULTRASONIC_RESULT_NONE; debug=(ultrasonic_debug_t){0};
}
bool ultrasonic_trigger(void) {
    const uint64_t now=sim_now();
    if (busy || (triggered && (uint32_t)((uint32_t)now-(uint32_t)trigger_us)<60000u)) { return false; }
    trigger_us=now; triggered=true; completed=false;
    const world_echo_t echo=world_echo();
    pending_too_near=echo.hit && echo.distance<(double)ULTRASONIC_DISTANCE_MIN_MM;
    pending_debug=(ultrasonic_debug_t){0};
    pending_status=ULTRASONIC_STATUS_NO_RETURN;
    ready_us=now+3215u;
    if (fault_mode==2u) { ready_us=now; }
    else if (fault_mode==1u) { ready_us=now+2000u; pending_status=ULTRASONIC_STATUS_NO_RISE; }
    else {
        pending_debug.rise_latency_us=215u;
        if (fault_mode==0u && echo.hit && echo.distance<=514.5) {
            pending_debug.pulse_us=(uint32_t)llround(echo.distance*2000/343);
            pending_debug.distance_mm=(pending_debug.pulse_us*343u+1000u)/2000u;
            pending_status=echo.distance>(double)ULTRASONIC_DISTANCE_MAX_MM+1e-9 ?
                ULTRASONIC_STATUS_NO_RETURN : ULTRASONIC_STATUS_VALID;
            ready_us=now+215u+pending_debug.pulse_us;
        }
    }
    sim_event("ultrasonic_trigger","\"ready_us\":%" PRIu64 ",\"hit\":%s,\"obstacle_index\":%" PRId64 ",\"distance_mm\":%.9f,\"origin_mm\":[%.9f,%.9f],\"point_mm\":[%.9f,%.9f],\"beam_heading_deg\":%.9f,\"fault_mode\":%u",
        ready_us,echo.hit?"true":"false",echo.hit?(int64_t)echo.obstacle:INT64_C(-1),echo.hit?echo.distance:-1.0,
        echo.origin.x,echo.origin.y,echo.point.x,echo.point.y,echo.angle_deg,fault_mode);
    if (!echo.hit && echo.surface_hit) {
        sim_event("ultrasonic_no_echo", "\"obstacle_index\":%zu,\"point_mm\":[%.9f,%.9f],\"distance_mm\":%.9f,\"reason\":\"incidence\"",
            echo.surface_obstacle,echo.surface_point.x,echo.surface_point.y,echo.surface_distance);
    }
    if (fault_mode==2u) { publish(ULTRASONIC_STATUS_STUCK_HIGH,(ultrasonic_debug_t){0}); }
    else { busy=true; }
    return true;
}
bool ultrasonic_service(void) {
    if (!busy || sim_now()<ready_us) { return false; }
    sim_devices_tick(); busy=false;
    publish(pending_status,pending_debug);
    return true;
}
bool ultrasonic_busy(void) { return busy; }
ultrasonic_result_t ultrasonic_latest(void) { return latest; }
ultrasonic_debug_t ultrasonic_debug_latest(void) { return debug; }

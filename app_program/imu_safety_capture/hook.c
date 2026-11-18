/*
 * IMU_SAFETY_CAPTURE=1 glue. The IMU task calls isc_safety_step() in place
 * of imu_safety_step() (one 20-byte store per observation, no output). The
 * Vehicle task requests a capture at each ESTOP. The telemetry task prints
 * the frozen capture in the GP20 dump, paced by the dump pause, then
 * re-arms it.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "capture.h"
#include "../platform/run_timing.h"
#include "../subsystem2/motion.h"

void isc_safety_step(imu_safety_t *s, const imu_safety_input_t *in);
void isc_timeline_report(void (*pause)(void));
int isc_motion_estop(void);

static isc_t capture;

void isc_safety_step(imu_safety_t *s, const imu_safety_input_t *in)
{
    const uint8_t before = s->output.latched_trigger;

    imu_safety_step(s, in);
    isc_observe(&capture, in, s, before);
}

/* Vehicle task: every ESTOP also asks for a capture (gentle-lift runs). */
int isc_motion_estop(void)
{
    isc_request(&capture);
    return motion_estop();
}

static const char *state_name(uint8_t state)
{
    static const char *const names[] = {"ARMED", "COLLECTING", "FROZEN",
                                        "REARM"};
    return state < 4u ? names[state] : "?";
}

static void print_capture(void (*pause)(void))
{
    const uint8_t state = isc_state(&capture);

    tm_printf((UB *)"[IMU_CAP] state %s latches %lu missed %lu captures "
                    "%lu pre %u post %u\n",
              state_name(state),
              (unsigned long)__atomic_load_n(&capture.latches,
                                             __ATOMIC_SEQ_CST),
              (unsigned long)__atomic_load_n(&capture.missed,
                                             __ATOMIC_SEQ_CST),
              (unsigned long)__atomic_load_n(&capture.captures,
                                             __ATOMIC_SEQ_CST),
              (unsigned int)ISC_PRE, (unsigned int)ISC_POST);
    if (state != ISC_FROZEN) {
        return;
    }
    tm_printf((UB *)"[IMU_CAP] source %s trigger %u latch_ms %lu samples "
                    "%u pre %u\n",
              capture.source == ISC_SOURCE_LATCH ? "LATCH" : "ESTOP",
              (unsigned int)capture.trigger,
              (unsigned long)capture.latch_ms, isc_count(&capture),
              (unsigned int)capture.pre);
    tm_printf((UB *)"[IMU_CAP] cols i rel_ms ax ay az g_mg tilt_ddeg vert_mg "
                    "flags ff lift ot trig in\n");
    if (pause != NULL) { pause(); }
    for (unsigned int i = 0u; i < isc_count(&capture); ++i) {
        const isc_sample_t *s = isc_at(&capture, i);
        tm_printf((UB *)"[IMU_CAP_S] %u %ld %d %d %d %u %d %d %02x %u %u %u "
                        "%02x %x\n",
                  i, (long)isc_rel_ms(&capture, s), (int)s->a[0],
                  (int)s->a[1], (int)s->a[2], (unsigned int)s->g,
                  (int)s->tilt, (int)s->vert, (unsigned int)s->flags,
                  (unsigned int)s->ff, (unsigned int)s->lift,
                  (unsigned int)s->ot, (unsigned int)s->trig,
                  (unsigned int)s->in);
        if ((i + 1u) % 8u == 0u && pause != NULL) { pause(); }
    }
    tm_printf((UB *)"[IMU_CAP] end\n");
    if (pause != NULL) { pause(); }
    isc_rearm(&capture);
}

void isc_timeline_report(void (*pause)(void))
{
    print_capture(pause);
    rt_timeline_report(pause);
}

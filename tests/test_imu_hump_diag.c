/* Synthetic diagnostic C latch rules; not hardware timing evidence. */
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "../app_program/app.h"
#include "../app_program/subsystem4/imu_hump_diag.h"

static char printed[1024];
static unsigned int prints;

#if defined(__clang__)
/* Clang accepts the monitor's UB format type; GCC requires char *. */
int tm_printf(const UB *fmt, ...)
    __attribute__((format(printf, 1, 2)));
#endif
int tm_printf(const UB *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    (void)vsnprintf(printed, sizeof(printed), (const char *)fmt, args);
    va_end(args);
    ++prints;
    return 0;
}

typedef struct {
    imu_hump_t hump;
    imu_hump_diag_t diag;
    imu_hump_input_t h;
    imu_hump_diag_input_t in;
    imu_hump_diag_report_t out;
} rig_t;

static void init(rig_t *r)
{
    memset(r, 0, sizeof(*r));
    imu_hump_init(&r->hump);
    imu_hump_diag_init(&r->diag);
    r->h = (imu_hump_input_t){.run_epoch = 1u, .speed_mm_s = 100.0f,
        .travel_mm = 5.0f, .valid = true, .motion_valid = true};
    r->in = (imu_hump_diag_input_t){.hump = &r->hump.output,
        .run_start_ms = UINT32_MAX - 99u, .now_ms = UINT32_MAX - 99u,
        .line_active = true};
}

static void tick(rig_t *r, float corrected, float filtered)
{
    r->h.height_mm = corrected;
    r->in.now_ms += 10u;
    r->in.corrected_mm = corrected;
    r->in.filtered_mm = filtered;
    r->in.travel_mm = r->h.travel_mm;
    r->in.speed_mm_s = r->h.speed_mm_s;
    r->in.yaw_rad_s = r->h.yaw_rate_rad_s;
    r->in.lateral_mg = r->h.lateral_mg;
    r->in.valid = r->h.valid;
    r->in.motion_valid = r->h.motion_valid;
    imu_hump_step(&r->hump, &r->h);
    imu_hump_diag_step(&r->diag, &r->in, &r->out);
}

static void many(rig_t *r, float c, float f, unsigned int n)
{
    for (unsigned int i = 0u; i < n; ++i) {
        tick(r, c, f);
    }
}

static void latch_and_winner(void)
{
    rig_t r;

    init(&r);
    many(&r, 0.0f, 0.0f, 10u);
    /* Gated samples can contribute to a later accepted 100 ms mean. */
    r.h.lateral_mg = -45.0f;
    r.h.yaw_rate_rad_s = -1.6f;
    r.in.yaw_accel_rad_s2 = -12.0f;
    tick(&r, 10.0f, 8.0f);
    r.h.lateral_mg = -20.0f;
    r.h.yaw_rate_rad_s = -0.5f;
    r.in.yaw_accel_rad_s2 = -3.0f;
    many(&r, 10.0f, 8.0f, 9u);
    const uint32_t peak_ms = r.in.now_ms - r.in.run_start_ms;
    const float peak_travel = r.diag.travel_mm;

    many(&r, 10.0f, 1.0f, 10u); /* Ties keep the FIRST peak instant. */
    assert(r.hump.output.state == IMU_HUMP_ACTIVE);
    const imu_hump_peak_diag_t peak = r.out.peak;

    assert(peak.valid && !peak.left && peak.ms == peak_ms);
    assert(peak.travel_mm == peak_travel);
    assert(fabsf(peak.corrected_mm - 10.0f) < 0.001f);
    assert(fabsf(peak.filtered_mm - 8.0f) < 0.001f);
    assert(peak.yaw_rad_s == -0.5f && peak.lateral_mg == -20.0f);
    assert(peak.yaw_accel_rad_s2 == -3.0f && peak.gate == 0u);
    assert(peak.window_yaw_rad_s == 1.6f);
    assert(peak.window_yaw_accel_rad_s2 == 12.0f);
    assert(peak.window_lateral_mg == 45.0f);
    assert(peak.window_gate ==
        (IMU_DIAG_GATE_YAW | IMU_DIAG_GATE_LATERAL));
    /* Gated high values freeze the ACTIVE detector peak and its latch. */
    r.h.lateral_mg = 70.0f;
    r.in.yaw_accel_rad_s2 = 20.0f;
    many(&r, 15.0f, 20.0f, 10u);
    assert(memcmp(&r.out.peak, &peak, sizeof(peak)) == 0);
    r.h.lateral_mg = 0.0f;
    r.in.yaw_accel_rad_s2 = 0.0f;
    /* Close while still gated, to avoid accepting the transient. */
    r.h.yaw_rate_rad_s = 2.0f;
    many(&r, 0.0f, 0.0f, 30u);
    assert(r.hump.output.count == 1u);
    assert(memcmp(&r.out.peak, &peak, sizeof(peak)) == 0);
    r.h.yaw_rate_rad_s = 0.0f;
    many(&r, -6.0f, -4.0f, 20u);
    many(&r, 0.0f, 0.0f, 20u);
    assert(r.hump.output.count == 2u);
    assert(memcmp(&r.out.peak, &peak, sizeof(peak)) == 0);
    many(&r, -12.0f, -9.0f, 20u);
    many(&r, 0.0f, 0.0f, 20u);
    assert(r.hump.output.count == 3u && r.out.peak.left);
    assert(r.out.peak.corrected_mm == -12.0f);
    assert(r.out.peak.filtered_mm == -9.0f);
    assert(r.out.line_lateral_mg == 70.0f);
    assert(r.out.line_yaw_accel_rad_s2 == 20.0f);
    /* SEARCH measurements must not inflate LINE-only maxima. */
    const uint32_t line_samples = r.out.line_samples;
    r.in.line_active = false;
    r.h.lateral_mg = 150.0f;
    r.in.yaw_accel_rad_s2 = 100.0f;
    tick(&r, 0.0f, 0.0f);
    assert(r.out.line_samples == line_samples);
    assert(r.out.line_lateral_mg == 70.0f);
    /* Next epoch clears winners/maxima, but not the shared window. */
    r.h.run_epoch = 3u;
    tick(&r, 0.0f, 0.0f);
    assert(r.out.run_epoch == 3u && !r.out.peak.valid);
    assert(r.out.line_samples == 0u && r.out.line_lateral_mg == 0.0f);
}

static void reject_and_stop(void)
{
    rig_t r;

    init(&r);
    many(&r, 0.0f, 0.0f, 10u);
    /* Too little travel: only a candidate, never a reported peak. */
    r.h.travel_mm = 0.0f;
    many(&r, 10.0f, 8.0f, 15u);
    assert(!r.out.peak.valid);
    many(&r, 0.0f, 0.0f, 15u);
    r.h.travel_mm = 5.0f;
    many(&r, 10.0f, 8.0f, 15u);
    assert(r.out.peak.valid && r.hump.output.state == IMU_HUMP_ACTIVE);
    const imu_hump_peak_diag_t p = r.out.peak;

    /* Virtual STOP result and actual even-epoch closure agree. */
    imu_hump_diag_report_t final = r.out;
    imu_hump_diag_finish(&final, 1u, r.in.now_ms, true);
    assert(final.valid && memcmp(&final.peak, &p, sizeof(p)) == 0);
    r.h.run_epoch = 2u;
    tick(&r, 0.0f, 0.0f);
    assert(r.hump.output.count == 1u && !r.hump.output.running);
    assert(memcmp(&r.out.peak, &p, sizeof(p)) == 0);
    /* A LONG rejection must discard the candidate's diagnostic. */
    r.h.run_epoch = 3u;
    many(&r, 0.0f, 0.0f, 10u);
    many(&r, 15.0f, 8.0f, 150u);
    assert(r.hump.output.state == IMU_HUMP_LONG);
    assert(!r.out.peak.valid && r.hump.output.count == 0u);
    many(&r, 0.0f, 0.0f, 20u);
    many(&r, 5.0f, 4.0f, 20u);
    assert(r.out.peak.valid && r.out.peak.corrected_mm == 5.0f);
    /* Invalid/gap samples reset the diagnostic window, not the winner. */
    r.h.valid = false;
    tick(&r, 99.0f, 99.0f);
    assert(r.diag.samples == 0u);
    r.h.valid = true;
    tick(&r, 5.0f, 4.0f);
    assert(r.diag.samples == 1u);
    assert(r.out.peak.corrected_mm == 5.0f);
    /* Age wrap is modular; stale and other-epoch never give values. */
    final = r.out;
    imu_hump_diag_finish(&final, 3u, final.timestamp_ms + 50u, true);
    assert(final.valid);
    imu_hump_diag_finish(&final, 3u, final.timestamp_ms + 51u, true);
    assert(!final.valid && !final.peak.valid && final.line_samples == 0u);
    final = r.out;
    imu_hump_diag_finish(&final, 5u, final.timestamp_ms, true);
    assert(!final.valid && final.run_epoch == 5u);
    final = r.out;
    imu_hump_diag_finish(&final, 3u, final.timestamp_ms, false);
    assert(!final.valid);
    imu_hump_diag_finish(NULL, 3u, 0u, false);
}

static void print_units(void)
{
    const imu_hump_diag_report_t r = {.valid = true, .run_epoch = 1u,
        .line_samples = 45u, .line_lateral_mg = 60.0f,
        .peak = {.valid = true, .corrected_mm = 10.3f,
            .filtered_mm = -8.2f, .yaw_rad_s = -0.123f,
            .yaw_accel_rad_s2 = 12.0f, .gate = 2u, .window_gate = 6u}};

    imu_hump_diag_print(&r);
    assert(prints == 1u && strstr(printed, "[S4_HUMP_DIAG] epoch 1"));
    assert(strstr(printed, "corrected_0.1mm 103 filtered_0.1mm -82"));
    assert(strstr(printed, "yaw_mrad_s -123 yaw_accel_mrad_s2 12000"));
    assert(strstr(printed, "gate 02 win_gate 06"));
    assert(strstr(printed, "line_samples 45 line_lateral_mg 60"));
    assert(printed[strlen(printed) - 1u] == '\n');
}

int main(void)
{
    latch_and_winner();
    reject_and_stop();
    print_units();
    puts("hump diagnostic latch rules passed");
    return 0;
}

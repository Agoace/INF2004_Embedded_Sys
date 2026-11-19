/* CR1 diagnostic board: 36 B. Inside the field ranges the USB line from
 * the board is the same text as the line from the float report.
 * ref_print() is a verbatim copy of imu_hump_diag_print() before the board. */
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "../app_program/app.h"
#include "../app_program/platform/app_board.h"
#include "../app_program/subsystem4/imu_hump_diag.h"

_Static_assert(sizeof(imu_hump_diag_board_t) == 36u, "diag board is 36 B");
_Static_assert(APP_BOARD_WORDS(imu_hump_diag_board_t) == 9u, "9 words");

static char printed[1024];
static unsigned int prints;

#if defined(__clang__)
int tm_printf(const UB *fmt, ...) __attribute__((format(printf, 1, 2)));
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

static long scaled(float value, float scale)
{
    const float x = value * scale;

    if (!isfinite(x)) {
        return 0L;
    }
    if (x >= 2147483520.0f) {
        return (long)INT32_MAX;
    }
    if (x <= -2147483648.0f) {
        return (long)INT32_MIN;
    }
    return (long)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

static void ref_print(char *out, size_t size, const imu_hump_diag_report_t *r)
{
    const imu_hump_peak_diag_t *p = &r->peak;

    (void)snprintf(out, size, "[S4_HUMP_DIAG] epoch %lu valid %u peak_valid %u "
              "peak_ms %lu travel_mm %ld corrected_0.1mm %ld "
              "filtered_0.1mm %ld speed_mm_s %ld yaw_mrad_s %ld "
              "yaw_accel_mrad_s2 %ld lateral_mg %ld "
              "win_yaw_mrad_s %ld win_yaw_accel_mrad_s2 %ld "
              "win_lateral_mg %ld gate %02x win_gate %02x left %u "
              "line_samples %lu line_lateral_mg %ld "
              "line_yaw_accel_mrad_s2 %ld\n",
              (unsigned long)r->run_epoch, (unsigned int)r->valid,
              (unsigned int)p->valid, (unsigned long)p->ms,
              scaled(p->travel_mm, 1.0f), scaled(p->corrected_mm, 10.0f),
              scaled(p->filtered_mm, 10.0f), scaled(p->speed_mm_s, 1.0f),
              scaled(p->yaw_rad_s, 1000.0f),
              scaled(p->yaw_accel_rad_s2, 1000.0f),
              scaled(p->lateral_mg, 1.0f),
              scaled(p->window_yaw_rad_s, 1000.0f),
              scaled(p->window_yaw_accel_rad_s2, 1000.0f),
              scaled(p->window_lateral_mg, 1.0f),
              (unsigned int)p->gate, (unsigned int)p->window_gate,
              (unsigned int)p->left, (unsigned long)r->line_samples,
              scaled(r->line_lateral_mg, 1.0f),
              scaled(r->line_yaw_accel_rad_s2, 1000.0f));
}

static uint32_t seed = 0x1234567u;
static uint32_t rnd(void)
{
    seed ^= seed << 13;
    seed ^= seed >> 17;
    seed ^= seed << 5;
    return seed;
}

/* Uniform in [-limit, limit] (or [0, limit]) on a fine grid. */
static float span(float limit, bool sign)
{
    const float u = (float)(rnd() % 2000001u) / 2000000.0f;
    return sign ? (2.0f * u - 1.0f) * limit : u * limit;
}

/* Inside the approved ranges (MEASUREMENTS limits plus the field margin). */
static imu_hump_diag_report_t in_range(void)
{
    imu_hump_diag_report_t r;
    memset(&r, 0, sizeof(r));
    imu_hump_peak_diag_t *p = &r.peak;
    p->ms = rnd() % (1u << 20);
    p->travel_mm = span(65535.0f, false);
    p->corrected_mm = span(204.7f, true);
    p->filtered_mm = span(204.7f, true);
    p->speed_mm_s = span(2047.0f, true);
    p->yaw_rad_s = span(16.383f, true);
    p->yaw_accel_rad_s2 = span(262.143f, true);
    p->lateral_mg = span(4095.0f, true);
    p->window_yaw_rad_s = span(16.383f, false);
    p->window_yaw_accel_rad_s2 = span(262.143f, false);
    p->window_lateral_mg = span(4095.0f, false);
    p->gate = (uint8_t)(rnd() & 0xFu);
    p->window_gate = (uint8_t)(rnd() & 0xFu);
    p->left = (rnd() & 1u) != 0u;
    p->valid = (rnd() & 1u) != 0u;
    r.run_epoch = rnd();
    r.timestamp_ms = rnd();
    r.line_samples = rnd() % (1u << 20);
    r.line_lateral_mg = span(4095.0f, false);
    r.line_yaw_accel_rad_s2 = span(262.143f, false);
    r.valid = (rnd() & 1u) != 0u;
    return r;
}

static void same_line(const imu_hump_diag_report_t *r)
{
    char want[1024];
    ref_print(want, sizeof(want), r);
    imu_hump_diag_board_t b;
    memset(&b, 0xa5, sizeof(b));
    imu_hump_diag_pack(&b, r);
    const unsigned int before = prints;
    imu_hump_diag_board_print(&b);
    assert(prints == before + 1u);
    if (strcmp(want, printed) != 0) {
        fprintf(stderr, "want %sgot  %s", want, printed);
        assert(0);
    }
    imu_hump_diag_print(r);
    assert(strcmp(want, printed) == 0);
    assert(imu_hump_diag_board_epoch(&b) == r->run_epoch);
    assert(imu_hump_diag_board_time(&b) == r->timestamp_ms);
    assert(imu_hump_diag_board_valid(&b) == r->valid);
    /* The writer cache: peak bits OR run bits is the full pack. */
    imu_hump_diag_board_t peak, two;
    imu_hump_diag_pack_peak(&peak, &r->peak);
    imu_hump_diag_pack_run(&two, &peak, r);
    assert(memcmp(&two, &b, sizeof(b)) == 0);
    imu_hump_diag_pack_run(&peak, &peak, r);   /* aliasing allowed */
    assert(memcmp(&peak, &b, sizeof(b)) == 0);
}

static void ranges(void)
{
    for (unsigned int i = 0u; i < 200000u; ++i) {
        const imu_hump_diag_report_t r = in_range();
        same_line(&r);
    }
    /* Exact field edges print exactly. */
    imu_hump_diag_report_t r;
    memset(&r, 0, sizeof(r));
    r.peak.ms = (1u << 20) - 1u;
    r.peak.travel_mm = 65535.0f;
    r.peak.corrected_mm = -204.8f;
    r.peak.filtered_mm = 204.7f;
    r.peak.speed_mm_s = -2048.0f;
    r.peak.yaw_rad_s = -16.384f;
    r.peak.yaw_accel_rad_s2 = -262.144f;
    r.peak.lateral_mg = -4096.0f;
    r.peak.window_yaw_rad_s = 16.383f;
    r.peak.window_yaw_accel_rad_s2 = 262.143f;
    r.peak.window_lateral_mg = 4095.0f;
    r.peak.gate = 0xFu;
    r.peak.window_gate = 0xFu;
    r.peak.left = true;
    r.peak.valid = true;
    r.run_epoch = UINT32_MAX;
    r.timestamp_ms = UINT32_MAX;
    r.valid = true;
    r.line_samples = (1u << 20) - 1u;
    r.line_lateral_mg = 4095.0f;
    r.line_yaw_accel_rad_s2 = 262.143f;
    same_line(&r);
    /* Nonfinite prints 0, like the old line. */
    r.peak.corrected_mm = NAN;
    r.peak.yaw_rad_s = INFINITY;
    r.line_lateral_mg = -INFINITY;
    same_line(&r);
}

/* Outside the range the board saturates to the field limit. */
static void saturation(void)
{
    imu_hump_diag_report_t r;
    memset(&r, 0, sizeof(r));
    r.peak.ms = UINT32_MAX;
    r.peak.travel_mm = 1.0e6f;
    r.peak.corrected_mm = 1000.0f;
    r.peak.filtered_mm = -1000.0f;
    r.peak.speed_mm_s = 9999.0f;
    r.peak.yaw_rad_s = -99.0f;
    r.peak.yaw_accel_rad_s2 = 9999.0f;
    r.peak.lateral_mg = -9999.0f;
    r.peak.window_yaw_rad_s = 99.0f;
    r.peak.window_yaw_accel_rad_s2 = 9999.0f;
    r.peak.window_lateral_mg = 9999.0f;
    r.line_samples = UINT32_MAX;
    r.line_lateral_mg = 9999.0f;
    r.line_yaw_accel_rad_s2 = 9999.0f;
    imu_hump_diag_board_t b;
    imu_hump_diag_pack(&b, &r);
    imu_hump_diag_board_print(&b);
    assert(strstr(printed, "peak_ms 1048575 travel_mm 65535 "
                           "corrected_0.1mm 2047 filtered_0.1mm -2048 "
                           "speed_mm_s 2047 yaw_mrad_s -16384 "
                           "yaw_accel_mrad_s2 262143 lateral_mg -4096 "
                           "win_yaw_mrad_s 16383 win_yaw_accel_mrad_s2 "
                           "262143 win_lateral_mg 4095 "));
    assert(strstr(printed, "line_samples 1048575 line_lateral_mg 4095 "
                           "line_yaw_accel_mrad_s2 262143\n"));
    /* Negative magnitudes (never written) clamp to 0. */
    memset(&r, 0, sizeof(r));
    r.peak.window_lateral_mg = -5.0f;
    r.peak.travel_mm = -5.0f;
    imu_hump_diag_pack(&b, &r);
    imu_hump_diag_board_print(&b);
    assert(strstr(printed, "travel_mm 0 "));
    assert(strstr(printed, "win_lateral_mg 0 "));
}

/* imu_hump_diag_finish() then pack == pack then board finish. */
static void finish(void)
{
    for (unsigned int i = 0u; i < 100000u; ++i) {
        imu_hump_diag_report_t r = in_range();
        const uint32_t epochs[] = {r.run_epoch, r.run_epoch + 1u, rnd()};
        const uint32_t epoch = epochs[rnd() % 3u];
        const uint32_t ages[] = {0u, 50u, 51u, rnd()};
        const uint32_t now = r.timestamp_ms + ages[rnd() % 4u];
        const bool have = (rnd() % 4u) != 0u;
        imu_hump_diag_board_t b, want;
        imu_hump_diag_pack(&b, &r);
        imu_hump_diag_board_finish(&b, epoch, now, have);
        imu_hump_diag_finish(&r, epoch, now, have);
        imu_hump_diag_pack(&want, &r);
        assert(memcmp(&b, &want, sizeof(b)) == 0);
    }
    imu_hump_diag_board_finish(NULL, 3u, 0u, false);
}

int main(void)
{
    ranges();
    saturation();
    finish();
    puts("diag board: print and finish identical");
    return 0;
}

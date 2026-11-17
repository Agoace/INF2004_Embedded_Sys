/* Native module with a 10 ms scheduler/60 ms sonar double, not hardware. */
#include <assert.h>
#include <stdio.h>
#include "../app_program/subsystem5/servo_scan.h"

static uint32_t run(unsigned int lo, unsigned int hi)
{
    ss_t scan;
    const ss_config_t cfg = {.stop_ms = 300u};
    ss_init(&scan, &cfg);
    ss_start(&scan, 200u, 0u);
    vsm_drive_t drive = {0};
    uint16_t ping = 0u;
    vsm_inputs_t in = {.ping_ms = &ping};
    uint32_t last_ping = 0u;
    unsigned int pings = 0u, at = 0u;
    unsigned int seen[13] = {0};
    bool send = false, saw_fine = false;
    for (uint32_t t = 0u; t < 5000u; t += 10u) {
        in.ping_sent = send;
        in.echo_new = send;
        in.echo = at >= lo && at <= hi ? 200u :
            (ultrasonic_result_t)(ULTRASONIC_STATUS_NO_RETURN <<
                                  ULTRASONIC_STATUS_SHIFT);
        if (send) {
            if (pings > 0u) { assert(t - last_ping >= 60u); }
            last_ping = t;
            if (pings < 10u) { assert(at == (pings / 2u) * 3u); }
            ++pings; ++seen[at];
            assert(seen[at] <= 2u);
        }
        const vsm_outcome_t outcome = ss_step(&scan, &in, t, &drive);
        saw_fine |= scan.stage == SS_STAGE_FINE;
        if (scan.phase != SS_REVERSE) {
            assert(drive.left_mm_s == 0 && drive.right_mm_s == 0);
        }
        if (outcome != 0u || scan.phase == SS_REVERSE) {
            assert(pings == 26u && scan.measured_mask == 0x1fffu);
            for (unsigned int i = 0u; i < 13u; ++i) {
                assert(seen[i] == 2u);
            }
            const obstacle_profile_word_t legacy =
                obstacle_profile_build(&scan.profile);
            assert(scan.word == legacy);
            bool coarse_hit = false;
            for (unsigned int i = 0u; i < 13u; i += 3u) {
                coarse_hit |= i >= lo && i <= hi;
            }
            assert(saw_fine == coarse_hit);
            return t;
        }
        at = (unsigned int)((drive.servo_deg + 30) / 5);
        send = ping == VSM_PING_ASAP &&
               (pings == 0u || t + 10u - last_ping >= 60u);
    }
    assert(false);
    return 0u;
}

static void bounds(void)
{
    obstacle_profile_t profile;
    for (unsigned int i = 0u; i < 13u; ++i) {
        profile.point[i] = OBSTACLE_SCAN_POINT_EMPTY;
    }
    uint8_t lo = 99u, hi = 99u;
    assert(!obstacle_fine_bounds(&profile, &lo, &hi));
    assert(lo == 99u && hi == 99u);
    profile.point[6] = 200u;
    assert(obstacle_fine_bounds(&profile, &lo, &hi));
    assert(lo == 3u && hi == 9u);
    profile.point[0] = 350u; /* inclusive nearest+150 */
    profile.point[12] = 351u;
    assert(obstacle_fine_bounds(&profile, &lo, &hi));
    assert(lo == 0u && hi == 9u);
    profile.point[12] = 300u; /* disjoint outer hit => clipped full range */
    assert(obstacle_fine_bounds(&profile, &lo, &hi));
    assert(lo == 0u && hi == 12u);
    profile.point[6] = 0u; /* too-near is VALID, never treated as free */
    assert(obstacle_fine_bounds(&profile, &lo, &hi));
    assert(lo == 3u && hi == 9u);
}

int main(void)
{
    bounds();
    uint32_t worst = 0u;
    for (unsigned int lo = 0u; lo < 13u; ++lo) {
        for (unsigned int hi = lo; hi < 13u; ++hi) {
            const uint32_t duration = run(lo, hi);
            if (duration > worst) { worst = duration; }
        }
    }
    const uint32_t empty = run(13u, 13u);
    if (empty > worst) { worst = empty; }
    printf("scan scheduler worst incl 300ms stop: %lu ms\n",
           (unsigned long)worst);
    assert(worst <= 2600u); /* 10 ms fake-clock bound, not hardware proof. */
    return 0;
}

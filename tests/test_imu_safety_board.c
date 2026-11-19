/* CR1 safety board: 12 B. Lists the carried fields, proves they round
 * trip exactly, that the not-carried fields read 0, and that the Vehicle
 * gate gives the same actions with a board copy as with the report. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../app_program/platform/app_board.h"
#include "../app_program/subsystem4/imu_safety_board.h"
#include "../app_program/vehicle/imu_gate.h"

_Static_assert(sizeof(imu_safety_board_t) == 12u, "safety board is 12 B");
_Static_assert(APP_BOARD_WORDS(imu_safety_board_t) == 3u, "3 board words");

static uint32_t seed = 0x9E3779B9u;
static uint32_t rnd(void)
{
    seed ^= seed << 13;
    seed ^= seed >> 17;
    seed ^= seed << 5;
    return seed;
}

static imu_safety_report_t roundtrip(const imu_safety_report_t *in)
{
    imu_safety_board_t b;
    imu_safety_report_t out;
    memset(&b, 0xa5, sizeof(b));
    memset(&out, 0xa5, sizeof(out));
    imu_safety_board_pack(&b, in);
    imu_safety_board_unpack(&out, &b);
    return out;
}

/* A report as the writer makes it: clear_ms <= limit, latch 0..2. */
static imu_safety_report_t random_report(uint32_t now)
{
    imu_safety_report_t r;
    memset(&r, 0, sizeof(r));
    const int32_t ages[] = {0, 1, 49, 50, 51, -1, 1000};
    r.value.observation_ms = (rnd() % 4u) != 0u
        ? now - (uint32_t)ages[rnd() % 7u] : rnd();
    r.value.clear_ms = (rnd() & 1u) != 0u ? IMU_SAFETY_CLEAR_MS
                                          : rnd() % (IMU_SAFETY_CLEAR_MS + 1u);
    r.value.latched_trigger = (uint8_t)(rnd() % 3u);
    r.value.flags = (rnd() & 1u) != 0u
        ? (uint8_t)(IMU_SAFETY_FLAG_VALID | IMU_SAFETY_FLAG_UPRIGHT |
                    IMU_SAFETY_FLAG_NEAR_ONE_G)
        : (uint8_t)rnd();
    r.value.clear_blockers = (rnd() & 1u) != 0u ? 0u
                                                : (uint8_t)(rnd() & 0xFu);
    r.applied_request = rnd() % 6u;
    /* Not carried: any value; the gate never reads them. */
    r.value.tilt_ddeg = (int32_t)rnd();
    r.value.g_mg = (float)(rnd() % 3000u);
    r.value.vertical_mg = -(float)(rnd() % 3000u);
    r.value.overturn_samples = (uint16_t)rnd();
    r.value.lift_samples = (uint16_t)rnd();
    r.value.current_trigger = (uint8_t)(rnd() % 4u);
    r.value.stops = (uint8_t)rnd();
    return r;
}

/* The field list: carried exactly, or 0 after the board. */
static void carried_fields(void)
{
    imu_safety_report_t in;
    memset(&in, 0, sizeof(in));
    in.value.observation_ms = 0xDEADBEEFu;
    in.applied_request = 0xFEEDF00Du;
    in.value.clear_ms = IMU_SAFETY_CLEAR_MS;
    in.value.latched_trigger = (uint8_t)IMU_SAFETY_IMU_UNAVAILABLE;
    in.value.flags = 0xFFu;
    in.value.clear_blockers = 0x0Fu;
    in.value.tilt_ddeg = 1800;
    in.value.g_mg = 1000.0f;
    in.value.vertical_mg = -600.0f;
    in.value.overturn_samples = 7u;
    in.value.lift_samples = 3u;
    in.value.current_trigger = (uint8_t)IMU_SAFETY_LIFT;
    in.value.stops = 2u;
    const imu_safety_report_t out = roundtrip(&in);
    /* Carried (a reader uses them). */
    assert(out.value.observation_ms == in.value.observation_ms);
    assert(out.applied_request == in.applied_request);
    assert(out.value.clear_ms == in.value.clear_ms);
    assert(out.value.latched_trigger == in.value.latched_trigger);
    assert(out.value.flags == in.value.flags);
    assert(out.value.clear_blockers == in.value.clear_blockers);
    /* Not carried (no board reader): 0. */
    assert(out.value.tilt_ddeg == 0);
    assert(out.value.g_mg == 0.0f);
    assert(out.value.vertical_mg == 0.0f);
    assert(out.value.overturn_samples == 0u);
    assert(out.value.lift_samples == 0u);
    assert(out.value.current_trigger == 0u);
    assert(out.value.stops == 0u);
    /* Unused board bits stay 0. */
    imu_safety_board_t b;
    imu_safety_board_pack(&b, &in);
    assert((b.word[2] >> 24) == 0u);
}

static void every_value(void)
{
    imu_safety_report_t in;
    memset(&in, 0, sizeof(in));
    for (uint32_t c = 0u; c <= IMU_SAFETY_CLEAR_MS; ++c) {
        in.value.clear_ms = c;
        assert(roundtrip(&in).value.clear_ms == c);
    }
    in.value.clear_ms = IMU_SAFETY_CLEAR_MS + 500u; /* never written */
    assert(roundtrip(&in).value.clear_ms == IMU_SAFETY_W2_CLEAR_MASK);
    in.value.clear_ms = 0u;
    for (unsigned int t = 0u; t <= IMU_SAFETY_W2_LATCHED_MASK; ++t) {
        for (unsigned int f = 0u; f < 256u; ++f) {
            for (unsigned int k = 0u; k < 16u; ++k) {
                in.value.latched_trigger = (uint8_t)t;
                in.value.flags = (uint8_t)f;
                in.value.clear_blockers = (uint8_t)k;
                const imu_safety_report_t out = roundtrip(&in);
                assert(out.value.latched_trigger == t &&
                       out.value.flags == f && out.value.clear_blockers == k);
            }
        }
    }
    for (unsigned int i = 0u; i < 100000u; ++i) {
        in.value.observation_ms = rnd();
        in.applied_request = rnd();
        const imu_safety_report_t out = roundtrip(&in);
        assert(out.value.observation_ms == in.value.observation_ms);
        assert(out.applied_request == in.applied_request);
    }
}

static bool same_action(ig_action_t a, ig_action_t b)
{
    return a.stop_edge == b.stop_edge && a.cleared == b.cleared &&
           a.request_clear == b.request_clear && a.recovery == b.recovery;
}

/* Two gates, same inputs: one sees the report, one the board copy. */
static void gate_equivalence(unsigned int runs, unsigned int steps)
{
    unsigned long compared = 0u;
    unsigned long edges = 0u, cleared = 0u, requests = 0u, recoveries = 0u;
    for (unsigned int run = 0u; run < runs; ++run) {
        ig_t direct, board;
        ig_init(&direct);
        ig_init(&board);
        uint32_t now = rnd();
        uint32_t token = 0u;
        for (unsigned int i = 0u; i < steps; ++i) {
            now += 1u + rnd() % 40u;
            const bool active = (rnd() & 1u) != 0u;
            const bool have = (rnd() % 8u) != 0u;
            imu_safety_report_t r = random_report(now);
            if ((rnd() % 4u) != 0u) {
                /* Mostly a fresh, safe pose, so stops, clear requests,
                 * acknowledgements and refusals all happen. */
                r.value.observation_ms = now - rnd() % 45u;
                r.value.clear_ms = (rnd() % 8u) != 0u ? IMU_SAFETY_CLEAR_MS
                                                      : rnd() % 1000u;
                r.value.flags = (uint8_t)(IMU_SAFETY_FLAG_VALID |
                    IMU_SAFETY_FLAG_UPRIGHT | IMU_SAFETY_FLAG_NEAR_ONE_G);
                r.value.clear_blockers = (rnd() % 8u) != 0u ? 0u
                    : (uint8_t)(rnd() & 0xFu);
                r.value.latched_trigger = (rnd() % 16u) == 0u
                    ? (uint8_t)(1u + rnd() % 2u) : (uint8_t)IMU_SAFETY_NONE;
                r.applied_request = (rnd() % 4u) != 0u ? token : rnd() % 6u;
            }
            const imu_safety_report_t copy = roundtrip(&r);
            const ig_action_t a =
                ig_step(&direct, have ? &r : NULL, now, active);
            const ig_action_t b =
                ig_step(&board, have ? &copy : NULL, now, active);
            assert(same_action(a, b));
            edges += a.stop_edge ? 1u : 0u;
            cleared += a.cleared ? 1u : 0u;
            requests += a.request_clear ? 1u : 0u;
            recoveries += a.recovery != 0u ? 1u : 0u;
            if (a.request_clear) {
                token = token + 1u == 0u ? 1u : token + 1u;
                ig_queued(&direct, token, now);
                ig_queued(&board, token, now);
            }
            if ((rnd() % 2u) == 0u) {
                assert(ig_press(&direct, now) == ig_press(&board, now));
            }
            assert(ig_start_allowed(&direct, now) ==
                   ig_start_allowed(&board, now));
            assert(ig_stop_required(&direct) == ig_stop_required(&board));
            assert(ig_refusal(&direct, now) == ig_refusal(&board, now));
            assert(direct.latched == board.latched &&
                   direct.cause == board.cause &&
                   direct.pending == board.pending &&
                   direct.token == board.token);
            assert(imu_safety_evaluate(&direct.report.value, direct.have,
                                       now, IMU_SAFETY_MAX_AGE_MS) ==
                   imu_safety_evaluate(&board.report.value, board.have, now,
                                       IMU_SAFETY_MAX_AGE_MS));
            assert(imu_safety_unavailable_cause(&direct.report.value,
                       direct.have, now, IMU_SAFETY_MAX_AGE_MS) ==
                   imu_safety_unavailable_cause(&board.report.value,
                       board.have, now, IMU_SAFETY_MAX_AGE_MS));
            ++compared;
        }
    }
    printf("safety board: %lu gate steps identical (stop %lu, cleared %lu, "
           "clear requests %lu, refusals %lu)\n", compared, edges, cleared,
           requests, recoveries);
    fflush(stdout);
    assert(edges > 0u && cleared > 0u && requests > 0u && recoveries > 0u);
}

int main(void)
{
    carried_fields();
    every_value();
    gate_equivalence(2000u, 200u);
    return 0;
}

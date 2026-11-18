/* Synthetic bench stream must exercise the unmodified detector paths. */
#include "../app_program/imu_safety_bench/replay.h"
#include "../app_program/subsystem4/imu_safety.h"
#include <assert.h>
#include <math.h>

static imu_safety_t safety;

static void feed(const imu_probe_t *probe, const imu_units_t *raw,
                 uint32_t ms)
{
    assert(!probe->no_sample && probe->fifo_last_samples == 4u);
    assert(!raw->mag_valid);
    imu_safety_input_t input = {.now_ms = ms, .accel_valid = true,
        .calibrated = true, .wheels_still = true};
    for (unsigned int i = 0u; i < 3u; ++i) {
        input.accel_mg[i] = probe->mean_accel_mg[i];
        assert(isfinite(input.accel_mg[i]));
        assert((float)raw->accel_mg[i] == probe->mean_accel_mg[i]);
    }
    imu_safety_step(&safety, &input);
}

/* TEST profile: a 40 s periodic stream with both episodes. */
static void periodic(void)
{
    imu_safety_init(&safety);
    bool lift = false, tilt = false;
    for (uint32_t ms = 0u; ms < 70000u; ms += 10u) {
        imu_probe_t probe = {0};
        imu_units_t raw = {0};
        const unsigned int phase = imu_replay_read(&probe, &raw, ms);
        assert(phase < IMU_REPLAY_PHASES);
        feed(&probe, &raw, ms);
        lift |= safety.output.latched_trigger == IMU_SAFETY_LIFT;
        tilt |= safety.output.latched_trigger == IMU_SAFETY_OVERTURN;
        if (safety.output.clear_ms >= IMU_SAFETY_CLEAR_MS) {
            (void)imu_safety_clear(&safety);
        }
    }
    assert(lift && tilt);
    imu_probe_t a = {0}, b = {0};
    imu_units_t x = {0}, y = {0};
    assert(imu_replay_read(&a, &x, 1u) == imu_replay_read(&b, &y, 40001u));
    assert(x.accel_mg[2] == y.accel_mg[2]);
}

static imu_replay_robot_t robot;
static uint32_t now_ms;

/* Advance the robot stream to `until`, with a constant calibration epoch.
 * Return the first time the detector latched `want`, or 0. */
static uint32_t run_robot(uint32_t until, uint32_t epoch,
                          imu_safety_trigger_t want, unsigned int *phases)
{
    uint32_t seen = 0u;
    for (; now_ms < until; now_ms += 10u) {
        imu_probe_t probe = {0};
        imu_units_t raw = {0};
        const unsigned int phase =
            imu_replay_robot_read(&robot, &probe, &raw, now_ms, epoch);
        assert(phase < IMU_REPLAY_PHASES);
        *phases |= 1u << phase;
        feed(&probe, &raw, now_ms);
        if (seen == 0u && safety.output.latched_trigger == want &&
            want != IMU_SAFETY_NONE) {
            seen = now_ms;
        }
        assert(want != IMU_SAFETY_NONE ||
               safety.output.latched_trigger == IMU_SAFETY_NONE);
    }
    return seen;
}

/* Vehicle recovery: clear only after the detector's own clear window.
 * The epoch stays at its current (even) value: no new start. */
static void recover(uint32_t epoch)
{
    unsigned int phases = 0u;
    const imu_safety_trigger_t latched =
        (imu_safety_trigger_t)safety.output.latched_trigger;
    const uint32_t limit = now_ms + 5000u;
    assert(latched != IMU_SAFETY_NONE);
    while (safety.output.clear_ms < IMU_SAFETY_CLEAR_MS) {
        assert(now_ms < limit);
        (void)run_robot(now_ms + 10u, epoch, latched, &phases);
    }
    assert(imu_safety_clear(&safety));
    assert(safety.output.latched_trigger == IMU_SAFETY_NONE);
}

/* ROBOT profile: upright until each new calibration start. Starts
 * alternate OVERTURN and LIFT, both inside the 6 s calibration spin. */
static void robot_schedule(void)
{
    imu_safety_init(&safety);
    imu_replay_robot_init(&robot);
    now_ms = 0u;
    unsigned int phases = 0u;
    /* Long idle: no episode without a calibration start. */
    assert(run_robot(300000u, 0u, IMU_SAFETY_NONE, &phases) == 0u);
    assert(phases == 1u && imu_replay_robot_episodes(&robot) == 0u);

    /* Start 1 (odd epoch): OVERTURN after the delay, before 6 s. */
    const uint32_t press1 = now_ms;
    phases = 0u;
    const uint32_t tilt = run_robot(press1 + 6000u, 1u, IMU_SAFETY_OVERTURN,
                                    &phases);
    assert(tilt >= press1 + IMU_REPLAY_ROBOT_DELAY_MS);
    assert(tilt < press1 + IMU_REPLAY_ROBOT_DELAY_MS + 1000u);
    assert((phases & 3u) == 3u && (phases & 0x18u) == 0u);
    assert(imu_replay_robot_episodes(&robot) == 1u);
    /* Calibration end (even, same start count) adds no episode. The tilt
     * ends and the stream stays upright in phase 2 until a new start. */
    phases = 0u;
    (void)run_robot(press1 + 30000u, 2u, IMU_SAFETY_OVERTURN, &phases);
    assert(phases == 4u && imu_replay_robot_episodes(&robot) == 1u);
    recover(2u);

    /* Start 2 (after the recovery press): LIFT. */
    const uint32_t press2 = now_ms;
    phases = 0u;
    const uint32_t lift = run_robot(press2 + 6000u, 3u, IMU_SAFETY_LIFT,
                                    &phases);
    assert(lift >= press2 + IMU_REPLAY_ROBOT_DELAY_MS);
    assert(lift < press2 + IMU_REPLAY_ROBOT_DELAY_MS + 1000u);
    assert((phases & (1u << 3u)) != 0u && (phases & (1u << 1u)) == 0u);
    phases = 0u;
    (void)run_robot(press2 + 12000u, 4u, IMU_SAFETY_LIFT, &phases);
    assert(phases == (1u << 4u));
    recover(4u);

    /* Start 3 alternates back to OVERTURN. A whole start/end pair between
     * two reads (4 -> 6) is a start. A start during the episode (6 -> 7)
     * does not add, restart or extend an episode. */
    const uint32_t press3 = now_ms;
    phases = 0u;
    (void)run_robot(press3 + 2500u, 6u, IMU_SAFETY_OVERTURN, &phases);
    (void)run_robot(press3 + 2600u, 7u, IMU_SAFETY_OVERTURN, &phases);
    assert(imu_replay_robot_episodes(&robot) == 2u + 1u);
    (void)run_robot(press3 + 2000u + IMU_REPLAY_TILT_HOLD_MS +
                    IMU_REPLAY_TILT_RETURN_MS + 10u, 8u,
                    IMU_SAFETY_OVERTURN, &phases);
    assert(imu_replay_robot_episodes(&robot) == 3u);
    assert(safety.output.latched_trigger == IMU_SAFETY_OVERTURN);

    /* Uptime wrap: the delay comparison uses a signed difference. */
    imu_replay_robot_init(&robot);
    imu_safety_init(&safety);
    now_ms = UINT32_MAX - 990u;
    phases = 0u;
    (void)run_robot(UINT32_MAX - 500u, 0u, IMU_SAFETY_NONE, &phases);
    const uint32_t wrap = run_robot(UINT32_MAX - 10u, 1u,
                                    IMU_SAFETY_NONE, &phases);
    assert(wrap == 0u && phases == 1u);
    /* Epoch overflow keeps a defined start count. */
    imu_replay_robot_init(&robot);
    now_ms = 0u;
    imu_probe_t probe = {0};
    imu_units_t raw = {0};
    (void)imu_replay_robot_read(&robot, &probe, &raw, 0u, UINT32_MAX - 1u);
    (void)imu_replay_robot_read(&robot, &probe, &raw, 10u, UINT32_MAX);
    assert(imu_replay_robot_episodes(&robot) == 1u);
}

int main(void)
{
    periodic();
    robot_schedule();
    return 0;
}

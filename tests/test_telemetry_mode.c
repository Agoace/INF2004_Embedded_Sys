/*
 * Host test for telemetry_mode.c: MQTT publication mode transitions.
 */
#include <assert.h>
#include <stdio.h>

#include "../app_program/subsystem1/telemetry_mode.h"

static pm_tracker_t t;

static pm_mode_t at(uint32_t now, uint8_t phase, bool active)
{
    return pm_update(&t, PM_BOARD_OK, phase, active, now);
}

/* idle -> calibrate -> start delay -> run -> stop -> grace -> idle. */
static void test_run_cycle(void)
{
    pm_init(&t);
    assert(t.mode == PM_MODE_RUN);               /* before the first read */
    assert(at(0u, 0u, false) == PM_MODE_IDLE);   /* boot is idle at once */
    assert(at(100u, 1u, true) == PM_MODE_RUN);   /* CALIBRATE */
    assert(at(5000u, 2u, true) == PM_MODE_RUN);  /* start delay */
    assert(at(6000u, 3u, true) == PM_MODE_RUN);  /* running */
    assert(at(20000u, 0u, false) == PM_MODE_RUN); /* stopped: grace starts */
    assert(at(20000u + PM_RUN_GRACE_MS - 1u, 0u, false) == PM_MODE_RUN);
    assert(at(20000u + PM_RUN_GRACE_MS, 0u, false) == PM_MODE_IDLE);
    assert(at(30000u, 0u, false) == PM_MODE_IDLE);
}

/* Calibration alone (CALIBRATE command, no run) also gets the grace. */
static void test_calibrate_only(void)
{
    pm_init(&t);
    (void)at(0u, 0u, false);
    assert(at(10u, 1u, true) == PM_MODE_RUN);
    assert(at(3000u, 0u, false) == PM_MODE_RUN);
    assert(at(4999u, 0u, false) == PM_MODE_RUN);
    assert(at(5000u, 0u, false) == PM_MODE_IDLE);
}

/* ESTOP ends the run: the phase drops to 0 and the same grace applies. */
static void test_estop(void)
{
    pm_init(&t);
    (void)at(0u, 0u, false);
    assert(at(100u, 3u, true) == PM_MODE_RUN);
    assert(at(150u, 0u, false) == PM_MODE_RUN); /* ESTOP stop edge */
    assert(at(2149u, 0u, false) == PM_MODE_RUN);
    assert(at(2150u, 0u, false) == PM_MODE_IDLE);
    /* A new run inside the grace restarts it at its own stop edge. */
    assert(at(2200u, 1u, true) == PM_MODE_RUN);
    assert(at(2300u, 0u, false) == PM_MODE_RUN);
    assert(at(4299u, 0u, false) == PM_MODE_RUN);
    assert(at(4300u, 0u, false) == PM_MODE_IDLE);
}

/* active alone (phase not yet set) counts as busy too. */
static void test_active_flag(void)
{
    pm_init(&t);
    (void)at(0u, 0u, false);
    assert(at(10u, 0u, true) == PM_MODE_RUN);
    assert(at(20u, 0u, false) == PM_MODE_RUN);
}

/* Busy reads keep the decision; no vehicle consumer is always RUN. */
static void test_board_results(void)
{
    pm_init(&t);
    assert(at(0u, 0u, false) == PM_MODE_IDLE);
    assert(pm_update(&t, PM_BOARD_BUSY, 3u, true, 10u) == PM_MODE_IDLE);
    assert(at(20u, 3u, true) == PM_MODE_RUN);
    assert(pm_update(&t, PM_BOARD_BUSY, 0u, false, 9000u) == PM_MODE_RUN);
    assert(pm_update(&t, PM_BOARD_NONE, 0u, false, 9000u) == PM_MODE_RUN);
    pm_init(&t);
    for (uint32_t now = 0u; now < 20000u; now += 1000u) {
        assert(pm_update(&t, PM_BOARD_NONE, 0u, false, now) == PM_MODE_RUN);
    }
}

/* The grace works across the 32-bit millisecond wrap. */
static void test_wrap(void)
{
    pm_init(&t);
    (void)at(0xFFFFF000u, 3u, true);
    assert(at(0xFFFFFF00u, 0u, false) == PM_MODE_RUN);
    assert(at(0x00000100u, 0u, false) == PM_MODE_RUN); /* 512 ms later */
    assert(at(0xFFFFFF00u + PM_RUN_GRACE_MS, 0u, false) == PM_MODE_IDLE);
}

static void test_periods(void)
{
    assert(pm_state_period_ms(PM_MODE_RUN) == 200u);
    assert(pm_state_period_ms(PM_MODE_IDLE) == 1000u);
    assert(pm_heartbeat_period_ms(PM_MODE_RUN) == 1000u);
    assert(pm_heartbeat_period_ms(PM_MODE_IDLE) == 5000u);
    assert(pm_streams_live(PM_MODE_RUN) && !pm_streams_live(PM_MODE_IDLE));
}

int main(void)
{
    test_run_cycle();
    test_calibrate_only();
    test_estop();
    test_active_flag();
    test_board_results();
    test_wrap();
    test_periods();
    printf("test_telemetry_mode: all checks passed\n");
    return 0;
}

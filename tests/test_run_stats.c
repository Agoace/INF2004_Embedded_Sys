/*
 * Host tests for run_stats.c: run summary and detour summary observer.
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../app_program/vehicle/run_stats.h"

#define CPM 12414
#define STEP_MS 10u

static rs_t rs;
static int32_t counts[2];
static int16_t measured[2];
static int16_t command[2];
static uint32_t now;

static void setup(void)
{
    const rs_config_t cfg = {{CPM, 2405u}};

    rs_init(&rs, &cfg);
    memset(counts, 0, sizeof(counts));
    memset(measured, 0, sizeof(measured));
    memset(command, 0, sizeof(command));
    now = 1000u;
    rs_run_begin(&rs, now);
    rs_step(&rs, now, counts, measured, command, 0u);
}

/* One 10 ms step with the given count changes. */
static void move(int32_t left, int32_t right)
{
    now += STEP_MS;
    counts[0] += left;
    counts[1] += right;
    rs_step(&rs, now, counts, measured, command, 0u);
}

static void test_distance_and_spin(void)
{
    rm_summary_t summary;
    rm_detour_t detour;

    setup();
    for (int i = 0; i < 100; ++i) {
        move(124, 124); /* 12400 counts forward: 998 mm */
    }
    for (int i = 0; i < 50; ++i) {
        move(-60, 60); /* spin: no path length */
    }
    for (int i = 0; i < 100; ++i) {
        move(-124, -124); /* reverse counts as path length too */
    }
    assert(!rs_run_end(&rs, now, RM_STOP_GP21, 0u, 0u, &summary, &detour));
    assert(summary.run_id == 1u && summary.start_ms == 1000u);
    assert(summary.duration_ms == 2500u);
    assert(summary.distance_mm == 1997u); /* 24800 * 1000 / 12414 */
    assert(summary.stop_source == RM_STOP_SOURCE_GP21);
    assert(summary.stop_reason == RM_STOP_GP21);
    assert(summary.max_speed_error_mm_s == 0u); /* steady at 0, no error */
    assert((summary.flags & RM_SUMMARY_SPEED_ERROR_VALID) != 0u);
    assert(summary.hump_peak == RM_SUMMARY_NONE_U16);
    assert(summary.barcode_ok == 0u && summary.barcode_failed == 0u);
    assert((summary.flags & RM_SUMMARY_BARCODE_IMPLEMENTED) == 0u);
}

static void test_lateral_offset(void)
{
    rm_detour_t detour;
    const int quarter_turn = (int)lround(90.0 * 24.05 / 2.0);

    setup();
    for (int i = 0; i < 50; ++i) {
        move(124, 124);
    }
    rs_detour_begin(&rs, now, 2u, 7u);
    for (int i = 0; i < quarter_turn; ++i) {
        move(-1, 1); /* turn 90 degrees left on the spot */
    }
    for (int i = 0; i < 100; ++i) {
        move(124, 124); /* 998 mm sideways, to the left */
    }
    now += STEP_MS;
    assert(rs_outcome(&rs, VSM_MOD_DETOUR, VSM_OUT_DONE_LINE, 0u, now, &detour));
    assert(detour.outcome == RM_DETOUR_DONE_LINE && detour.side == 2u);
    assert(detour.scan_id == 7u && detour.sequence == 1u);
    assert((detour.flags & RM_DETOUR_LATERAL_VALID) != 0u);
    assert(detour.distance_mm == 998u);
    printf("lateral %u mm for 998 mm sideways\n", detour.max_lateral_mm);
    assert(detour.max_lateral_mm >= 985u && detour.max_lateral_mm <= 998u);
    assert(detour.duration_ms == (uint32_t)(quarter_turn + 100 + 1) * STEP_MS);
    assert(detour.end_ms == now && detour.start_ms == now - detour.duration_ms);
}

/* A detour that drives out and comes back has a lateral maximum, not a net. */
static void test_out_and_back(void)
{
    rm_detour_t detour;
    const int half_turn = (int)lround(180.0 * 24.05 / 2.0);
    const int quarter = (int)lround(90.0 * 24.05 / 2.0);

    setup();
    rs_detour_begin(&rs, now, 1u, 1u);
    for (int i = 0; i < quarter; ++i) {
        move(1, -1); /* right turn */
    }
    for (int i = 0; i < 50; ++i) {
        move(124, 124); /* about 499 mm to the right */
    }
    for (int i = 0; i < half_turn; ++i) {
        move(-1, 1);
    }
    for (int i = 0; i < 50; ++i) {
        move(124, 124); /* and back */
    }
    now += STEP_MS;
    assert(rs_outcome(&rs, VSM_MOD_DETOUR, VSM_OUT_DONE_NO_LINE, 1u, now,
                      &detour));
    assert(detour.outcome == RM_DETOUR_DONE_NO_LINE && detour.payload == 1u);
    assert(detour.distance_mm >= 990u && detour.distance_mm <= 1000u);
    assert(detour.max_lateral_mm >= 485u && detour.max_lateral_mm <= 500u);
}

static void test_straight_detour_has_no_lateral(void)
{
    rm_detour_t detour;

    setup();
    rs_detour_begin(&rs, now, 2u, 3u);
    for (int i = 0; i < 100; ++i) {
        move(124, 124);
    }
    now += STEP_MS;
    assert(rs_outcome(&rs, VSM_MOD_DETOUR, VSM_OUT_FAILED, 3u, now, &detour));
    assert(detour.outcome == RM_DETOUR_FAILED && detour.payload == 3u);
    assert(detour.max_lateral_mm == 0u);
}

static void test_counters_and_outcomes(void)
{
    rm_detour_t detour;
    rm_summary_t summary;

    setup();
    assert(!rs_outcome(&rs, VSM_MOD_LINE, VSM_OUT_LINE_LOST, 1u, now, &detour));
    assert(!rs_outcome(&rs, VSM_MOD_SEARCH, VSM_OUT_LINE_LOST, 1u, now, &detour));
    assert(!rs_outcome(&rs, VSM_MOD_LINE, VSM_OUT_OBSTACLE, 100u, now, &detour));
    assert(!rs_outcome(&rs, VSM_MOD_SCAN, VSM_OUT_SIDE, 5u, now, &detour));
    /* A detour outcome with no open detour is not a summary. */
    assert(!rs_outcome(&rs, VSM_MOD_DETOUR, VSM_OUT_DONE_LINE, 0u, now, &detour));
    rs_detour_begin(&rs, now, 1u, 1u);
    rs_detour_begin(&rs, now + 5u, 2u, 2u); /* ignored */
    move(10, 10);
    assert(rs_outcome(&rs, VSM_MOD_DETOUR, VSM_OUT_DONE_LINE, 0u, now, &detour));
    assert(detour.side == 1u && detour.scan_id == 1u);
    rs_detour_begin(&rs, now, 2u, 2u);
    move(10, 10);
    assert(rs_outcome(&rs, VSM_MOD_DETOUR, VSM_OUT_DONE_NO_LINE, 2u, now, &detour));
    assert(detour.sequence == 2u);
    rs_detour_begin(&rs, now, 2u, 3u);
    move(10, 10);
    assert(rs_outcome(&rs, VSM_MOD_DETOUR, VSM_OUT_FAILED, 6u, now, &detour));
    rs_detour_begin(&rs, now, 1u, 4u);
    move(10, 10);
    assert(rs_run_end(&rs, now, RM_STOP_MQTT_ESTOP, 4u, 9u, &summary, &detour));
    assert(detour.outcome == RM_DETOUR_ABORTED && detour.sequence == 4u);
    assert(summary.line_losses == 2u && summary.obstacles == 1u);
    assert(summary.detours_started == 4u && summary.detours_done == 2u);
    assert(summary.detours_succeeded == 1u);
    assert((summary.flags & RM_SUMMARY_DETOUR_ABORTED) != 0u);
    assert(summary.stop_source == RM_STOP_SOURCE_MQTT);
    assert(summary.shell_fault == 4u && summary.shell_fault_detail == 9u);
    assert(summary.detour_total_ms == 40u + 0u);
    /* Outcomes after the run end are ignored. */
    assert(!rs_outcome(&rs, VSM_MOD_LINE, VSM_OUT_LINE_LOST, 1u, now, &detour));
    /* A new run clears totals but keeps ids and the detour sequence. */
    rs_run_begin(&rs, now + 1000u);
    rs_step(&rs, now + 1000u, counts, measured, command, 0u);
    rs_run_end(&rs, now + 2000u, RM_STOP_TIME_LIMIT, 0u, 0u, &summary, &detour);
    assert(summary.run_id == 2u && summary.line_losses == 0u);
    assert(summary.detours_started == 0u && summary.duration_ms == 1000u);
    assert(summary.stop_source == RM_STOP_SOURCE_LIMIT);
    rs_run_begin(&rs, 0u);
    rs_detour_begin(&rs, 1u, 1u, 9u);
    rs_step(&rs, 2u, counts, measured, command, 0u);
    assert(rs_outcome(&rs, VSM_MOD_DETOUR, VSM_OUT_FAILED, 1u, 3u, &detour));
    assert(detour.sequence == 5u); /* continues after the first run */
    assert(detour.flags == 0u);    /* the first step is the origin: no data */
}

static void test_speed_error(void)
{
    rm_summary_t summary;
    rm_detour_t detour;

    setup();
    command[0] = 200;
    command[1] = 200;
    measured[0] = 0;
    measured[1] = 0;
    move(0, 0); /* command changed: the clock restarts */
    for (int i = 0; i < 29; ++i) {
        move(0, 0); /* ramp: error 200 is not counted before 300 ms */
    }
    rs_run_end(&rs, now, RM_STOP_UNKNOWN, 0u, 0u, &summary, &detour);
    assert(summary.max_speed_error_mm_s == RM_SUMMARY_NONE_U16);
    assert(summary.stop_source == RM_STOP_SOURCE_NONE);
    setup();
    command[0] = 200;
    command[1] = 200;
    measured[0] = 190;
    measured[1] = 215;
    for (int i = 0; i < 40; ++i) {
        move(0, 0);
    }
    measured[0] = 0; /* stall after a command change is not steady */
    command[0] = 100;
    move(0, 0);
    rs_run_end(&rs, now, RM_STOP_SHELL, 0u, 0u, &summary, &detour);
    assert(summary.max_speed_error_mm_s == 15u);
    assert((summary.flags & RM_SUMMARY_SPEED_ERROR_VALID) != 0u);
    assert(summary.stop_source == RM_STOP_SOURCE_FAULT);
}

static void test_faults_and_sources(void)
{
    rm_summary_t summary;
    rm_detour_t detour;

    setup();
    rs_step(&rs, now + 10u, counts, measured, command, 0x01u);
    rs_step(&rs, now + 20u, counts, measured, command, 0x08u);
    rs_run_end(&rs, now, RM_STOP_DEADLINE, 0u, 0u, &summary, &detour);
    assert(summary.motion_fault_bits == 0x09u);
    for (unsigned int reason = RM_STOP_DEADLINE; reason <= RM_STOP_OTHER; ++reason) {
        assert(rs_stop_source((uint8_t)reason) == RM_STOP_SOURCE_FAULT);
    }
    assert(rs_stop_source(RM_STOP_SHELL) == RM_STOP_SOURCE_FAULT);
    assert(rs_stop_source(RM_STOP_TIME_LIMIT) == RM_STOP_SOURCE_LIMIT);
}

int main(void)
{
    test_distance_and_spin();
    test_lateral_offset();
    test_out_and_back();
    test_straight_detour_has_no_lateral();
    test_counters_and_outcomes();
    test_speed_error();
    test_faults_and_sources();
    return 0;
}

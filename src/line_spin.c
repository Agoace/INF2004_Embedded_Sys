#include "line_spin.h"

#include <stdint.h>
#include <stdio.h>

#include "drive.h"
#include "line_sensors.h"
#include "pico/stdlib.h"

#define SPIN_DUTY_PERCENT 20 /* 15 % stalled on the floor (2026-10-27) */
#define SPIN_DUTY_LIMIT_PERCENT 20
_Static_assert(SPIN_DUTY_PERCENT <= SPIN_DUTY_LIMIT_PERCENT,
               "USB power: keep the spin duty at or below 20 %");
#define SPIN_STAGGER_MS 50u
#define STALL_GRACE_MS 1000u
#define STALL_TIMEOUT_MS 500u
#define SPIN_STEP_COUNTS (DRIVE_COUNTS_PER_WHEEL_TURN / 4) /* a quarter wheel turn */
/* Last step: stop when the line is this close to the centre (-100..+100). */
#define SPIN_CENTRE_POSITION 20
#define SPIN_STEP_TIMEOUT_MS 6000u
#define SPIN_PAUSE_MS 200u
#define SAMPLE_PERIOD_US 1000u

/* What ends a spin step for each wheel, measured from the spin start
 * ("home") so that coasting and errors do not add up step by step. */
typedef enum {
    GOAL_OUT,  /* wheel is SPIN_STEP_COUNTS away from home */
    GOAL_HOME, /* wheel has come back to (or just past) home */
} spin_goal_t;

static uint32_t abs_count(int32_t value) {
    return value < 0 ? (uint32_t)(-value) : (uint32_t)value;
}

static bool wheel_done(spin_goal_t goal, int32_t from_home, int32_t from_home_at_start) {
    if (goal == GOAL_OUT) {
        /* Starting on one side: first cross home, then go out on the other. */
        const bool started_out =
            abs_count(from_home_at_start) >= (uint32_t)SPIN_STEP_COUNTS / 2u;
        const bool other_side = (from_home < 0) != (from_home_at_start < 0);
        return (!started_out || other_side) &&
               abs_count(from_home) >= (uint32_t)SPIN_STEP_COUNTS;
    }
    return from_home == 0 || (from_home < 0) != (from_home_at_start < 0);
}

static void counts_from_home(const int32_t home[2], int32_t out[2]) {
    drive_counts(&out[DRIVE_LEFT], &out[DRIVE_RIGHT]);
    out[DRIVE_LEFT] -= home[DRIVE_LEFT];
    out[DRIVE_RIGHT] -= home[DRIVE_RIGHT];
}

/* Turns on the spot, sampling the line sensors every 1 ms.
 * direction +1: left wheel forward, right wheel reverse (robot turns right).
 * Each wheel stops by itself when it reaches its goal. With stop_on_line
 * (the calibration is already valid), both wheels stop as soon as the line
 * is under the centre sensor; the encoder goal is then only a limit.
 * Returns false (motors stopped) on a stall or the time limit. */
static bool spin_step(lp_t *lp, int direction, spin_goal_t goal, const int32_t home[2],
                      bool stop_on_line) {
    int32_t start[2];
    counts_from_home(home, start);
    const uint32_t t0 = to_ms_since_boot(get_absolute_time());
    const int wheel_sign[2] = {direction, -direction};
    bool running[2] = {true, false}; /* the right wheel starts SPIN_STAGGER_MS later */
    bool right_started = false;
    int32_t last[2] = {start[0], start[1]};
    uint32_t moved_ms[2] = {t0, t0};
    bool on_line = false;

    drive_set_wheel(DRIVE_LEFT, wheel_sign[DRIVE_LEFT] * SPIN_DUTY_PERCENT);
    absolute_time_t next_sample = get_absolute_time();

    while (running[0] || running[1] || !right_started) {
        const uint32_t now = to_ms_since_boot(get_absolute_time());
        if (!right_started && now - t0 >= SPIN_STAGGER_MS) {
            right_started = true;
            running[DRIVE_RIGHT] = true;
            moved_ms[DRIVE_RIGHT] = now;
            drive_set_wheel(DRIVE_RIGHT, wheel_sign[DRIVE_RIGHT] * SPIN_DUTY_PERCENT);
        }
        if (on_line) {
            right_started = true; /* line found before the right wheel started */
        }

        int32_t c[2];
        counts_from_home(home, c);
        bool stalled[2] = {false, false};
        for (unsigned int w = 0u; w < 2u; ++w) {
            if (c[w] != last[w]) {
                last[w] = c[w];
                moved_ms[w] = now;
            }
            if (running[w] && (on_line || wheel_done(goal, c[w], start[w]))) {
                running[w] = false;
                drive_set_wheel((drive_wheel_t)w, 0);
            }
            stalled[w] = running[w] && now - t0 >= STALL_GRACE_MS &&
                         now - moved_ms[w] >= STALL_TIMEOUT_MS;
        }
        if (stalled[0] || stalled[1] || now - t0 >= SPIN_STEP_TIMEOUT_MS) {
            drive_stop();
            printf("[spin] STOPPED: %s  from start L %ld  R %ld\n",
                   stalled[0]   ? "left wheel stalled"
                   : stalled[1] ? "right wheel stalled"
                                : "time limit",
                   (long)c[0], (long)c[1]);
            return false;
        }

        if (time_reached(next_sample)) {
            next_sample = delayed_by_us(next_sample, SAMPLE_PERIOD_US);
            uint16_t raw[LP_SENSORS];
            line_sensors_read(raw);
            if (stop_on_line) {
                (void)lp_update(lp, raw, now);
                on_line = lp->norm[LP_CENTRE] >= LP_FOUND_PCT &&
                          lp->position >= -SPIN_CENTRE_POSITION &&
                          lp->position <= SPIN_CENTRE_POSITION;
            } else {
                lp_calib_add(lp, raw);
            }
        }
    }
    int32_t end[2];
    counts_from_home(home, end);
    printf("[spin] %+d done in %lu ms%s; from start L %ld  R %ld\n", direction,
           (unsigned long)(to_ms_since_boot(get_absolute_time()) - t0),
           on_line ? ", stopped on the line" : "", (long)end[0], (long)end[1]);
    return true;
}

/* Sampling during the short pause between steps (robot still). */
static void spin_pause(lp_t *lp) {
    const absolute_time_t end = make_timeout_time_ms(SPIN_PAUSE_MS);
    while (!time_reached(end)) {
        uint16_t raw[LP_SENSORS];
        line_sensors_read(raw);
        lp_calib_add(lp, raw);
        sleep_us(SAMPLE_PERIOD_US);
    }
}

bool line_calib_end_and_report(lp_t *lp) {
    printf("[calib] seen range L %u-%u  C %u-%u  R %u-%u\n", lp->cal_min[0],
           lp->cal_max[0], lp->cal_min[1], lp->cal_max[1], lp->cal_min[2],
           lp->cal_max[2]);
    unsigned int bad = 0u;
    const lp_calib_result_t result = lp_calib_end(lp, &bad);
    if (result == LP_CALIB_OK) {
        printf("[calib] OK. Line reads %s than the floor.\n",
               lp_line_reads_high(lp) ? "HIGHER" : "LOWER");
        for (unsigned int s = 0u; s < LP_SENSORS; ++s) {
            printf("[calib]   %c: floor %4u  line %4u\n", "LCR"[s], lp_floor(lp, s),
                   lp_line(lp, s));
        }
    } else if (result == LP_CALIB_LOW_CONTRAST) {
        printf("[calib] FAILED: range below %u ADC on:%s%s%s.\n", LP_MIN_CONTRAST_ADC,
               (bad & 1u) ? " L" : "", (bad & 2u) ? " C" : "", (bad & 4u) ? " R" : "");
        if (lp_calibrated(lp)) {
            printf("[calib] The previous calibration stays in use.\n");
        }
    } else {
        printf("[calib] FAILED: no data.\n");
    }
    return result == LP_CALIB_OK;
}

line_spin_result_t line_spin_calibrate(lp_t *lp) {
    printf("\n[spin] Spin calibration at %d %% duty, %d counts (a quarter wheel turn).\n",
           SPIN_DUTY_PERCENT, SPIN_STEP_COUNTS);
    int32_t home[2];
    drive_counts(&home[DRIVE_LEFT], &home[DRIVE_RIGHT]);
    lp_calib_begin(lp);

    printf("[spin] step 1: left\n");
    bool ok = spin_step(lp, -1, GOAL_OUT, home, false);
    if (ok) {
        spin_pause(lp);
        printf("[spin] step 2: back through the centre to the right\n");
        ok = spin_step(lp, +1, GOAL_OUT, home, false);
    }
    if (ok) {
        spin_pause(lp);
    }
    drive_stop();
    if (!ok) {
        printf("[spin] Spin not complete. The calibration is not changed.\n");
        return LINE_SPIN_STOPPED;
    }

    const bool calibrated = line_calib_end_and_report(lp);
    printf("[spin] step 3: back to the centre%s\n",
           calibrated ? ", stop on the line" : " by encoder count");
    ok = spin_step(lp, -1, GOAL_HOME, home, calibrated);
    drive_stop();
    if (!ok) {
        return LINE_SPIN_STOPPED;
    }
    return calibrated ? LINE_SPIN_OK : LINE_SPIN_CALIB_FAILED;
}

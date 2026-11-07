/*
 * Host tests for odometry.c: heading, turn rate, distance, sine and cosine.
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "../app_program/vehicle/odometry.h"

#define CPM 12414u
#define CPD_X100 2405u
#define PI_VALUE 3.14159265358979323846

static const odo_config_t robot = {CPM, CPD_X100};

static void test_sine_table(void)
{
    double worst = 0.0;

    for (int32_t cdeg = -72000; cdeg <= 72000; ++cdeg) {
        const double angle = (double)cdeg * PI_VALUE / 18000.0;
        const double sine = (double)odo_sin_q15(cdeg) / 32768.0;
        const double cosine = (double)odo_cos_q15(cdeg) / 32768.0;
        double error = fabs(sine - sin(angle));

        if (fabs(cosine - cos(angle)) > error) {
            error = fabs(cosine - cos(angle));
        }
        if (error > worst) {
            worst = error;
        }
        assert(abs(odo_sin_q15(cdeg)) <= 32768);
    }
    printf("sine worst error %.6f\n", worst);
    assert(worst < 0.0001); /* 3.3 q15 units; 0.01 percent of one mm per m */
    assert(odo_sin_q15(0) == 0 && odo_sin_q15(9000) == 32767);
    assert(odo_sin_q15(18000) == 0 && odo_sin_q15(27000) == -32767);
    assert(odo_cos_q15(0) == 32767 && odo_cos_q15(18000) == -32767);
}

static void test_heading(void)
{
    /* One degree of turn is 24.05 counts: 100 cdeg per 24.05 counts. */
    assert(odo_heading_cdeg(&robot, 0, 0) == 0);
    assert(odo_heading_cdeg(&robot, -1202, 1203) == 10000); /* 2405 / 24.05 */
    assert(odo_heading_cdeg(&robot, 1203, -1202) == -10000);
    assert(odo_heading_cdeg(&robot, 100, 100) == 0);
    assert(odo_heading_cdeg(&robot, INT32_MIN, INT32_MAX) > 0); /* no overflow */
    /* One count is 4.16 cdeg: the 0.01 degree unit is finer than the encoder. */
    assert(odo_heading_cdeg(&robot, 0, 1) == 4);
    assert(odo_heading_cdeg(&robot, 0, -1) == -4);
}

static void test_turn_rate(void)
{
    /* Spin on the spot at 131 mm/s: 262 mm/s difference = 135.2 deg/s. */
    assert(odo_turn_rate_ddps(&robot, -131, 131) == 1352);
    assert(odo_turn_rate_ddps(&robot, 131, -131) == -1352);
    assert(odo_turn_rate_ddps(&robot, 200, 200) == 0);
    assert(odo_turn_rate_ddps(&robot, 0, 0) == 0);
    assert(odo_turn_rate_ddps(&robot, -460, 460) == 4749); /* maximum spin */
    /* Same constants as the track width: 0.5162 deg/s per mm/s. */
    for (int dv = -920; dv <= 920; ++dv) {
        const double expected = (double)dv * 180.0 / PI_VALUE / 111.0 * 10.0;
        assert(fabs((double)odo_turn_rate_ddps(&robot, 0, (int16_t)dv) -
                    expected) < 0.3 * fabs(expected) / 100.0 + 1.0);
    }
    /* Limits of the int16 range. */
    assert(odo_turn_rate_ddps(&robot, INT16_MIN, INT16_MAX) == INT16_MAX);
    assert(odo_turn_rate_ddps(&robot, INT16_MAX, INT16_MIN) == INT16_MIN);
}

static void test_distance(void)
{
    assert(odo_counts_to_mm(&robot, 0u) == 0u);
    assert(odo_counts_to_mm(&robot, CPM) == 1000u);
    assert(odo_counts_to_mm(&robot, CPM - 1u) == 999u);
    assert(odo_counts_to_mm(&robot, 5000000000ull) == 402771064u);
}

int main(void)
{
    test_sine_table();
    test_heading();
    test_turn_rate();
    test_distance();
    return 0;
}

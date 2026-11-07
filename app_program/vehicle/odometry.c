/*
 * odometry.c - pure encoder odometry helpers. See odometry.h.
 */
#include "odometry.h"

#define ODO_TABLE_LAST    90
#define ODO_CDEG_PER_DEG  100
#define ODO_HEADING_SCALE 10000
#define ODO_MM_PER_M      1000

/* sin(0 deg) to sin(90 deg), one degree apart, scaled by 32768. */
static const int16_t sine_table[ODO_TABLE_LAST + 1] = {
    0,     572,   1144,  1715,  2286,  2856,  3425,  3993,  4560,  5126,  5690,
    6252,  6813,  7371,  7927,  8481,  9032,  9580,  10126, 10668, 11207, 11743,
    12275, 12803, 13328, 13848, 14365, 14876, 15384, 15886, 16384, 16877, 17364,
    17847, 18324, 18795, 19261, 19720, 20174, 20622, 21063, 21498, 21926, 22348,
    22763, 23170, 23571, 23965, 24351, 24730, 25102, 25466, 25822, 26170, 26510,
    26842, 27166, 27482, 27789, 28088, 28378, 28660, 28932, 29197, 29452, 29698,
    29935, 30163, 30382, 30592, 30792, 30983, 31164, 31336, 31499, 31651, 31795,
    31928, 32052, 32166, 32270, 32365, 32449, 32524, 32588, 32643, 32688, 32723,
    32748, 32763, 32767,
};

/* Sine of an angle from 0 to 90 degrees, linear between table entries. */
static int32_t quarter_sine(int32_t cdeg)
{
    const int32_t index = cdeg / ODO_CDEG_PER_DEG;
    const int32_t part = cdeg % ODO_CDEG_PER_DEG;

    if (index >= ODO_TABLE_LAST) {
        return sine_table[ODO_TABLE_LAST];
    }
    return sine_table[index] +
           ((sine_table[index + 1] - sine_table[index]) * part +
            ODO_CDEG_PER_DEG / 2) /
               ODO_CDEG_PER_DEG;
}

int32_t odo_heading_cdeg(const odo_config_t *cfg, int32_t left_counts,
                         int32_t right_counts)
{
    const int64_t diff = (int64_t)right_counts - (int64_t)left_counts;

    return (int32_t)(diff * ODO_HEADING_SCALE /
                     (int64_t)cfg->counts_per_deg_x100);
}

int16_t odo_turn_rate_ddps(const odo_config_t *cfg, int16_t left_mm_s,
                           int16_t right_mm_s)
{
    /*
     * degrees per second = dv * (counts_per_m / 1000) / (cpd_x100 / 100)
     * so tenths of a degree per second = dv * counts_per_m / cpd_x100.
     */
    const int64_t dv = (int64_t)right_mm_s - (int64_t)left_mm_s;
    const int64_t num = dv * (int64_t)cfg->counts_per_m;
    const int64_t den = (int64_t)cfg->counts_per_deg_x100;
    const int64_t half = den / 2;
    const int64_t value =
        (num >= 0) ? (num + half) / den : -((-num + half) / den);

    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    if (value < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)value;
}

uint32_t odo_counts_to_mm(const odo_config_t *cfg, uint64_t counts)
{
    return (uint32_t)(counts * ODO_MM_PER_M / cfg->counts_per_m);
}

int32_t odo_sin_q15(int32_t cdeg)
{
    int32_t angle = cdeg % ODO_FULL_TURN_CDEG;

    if (angle < 0) {
        angle += ODO_FULL_TURN_CDEG;
    }
    if (angle <= ODO_SIN_QUARTER_CDEG) {
        return quarter_sine(angle);
    }
    if (angle <= ODO_SIN_HALF_CDEG) {
        return quarter_sine(ODO_SIN_HALF_CDEG - angle);
    }
    if (angle <= ODO_SIN_THREE_QUARTER_CDEG) {
        return -quarter_sine(angle - ODO_SIN_HALF_CDEG);
    }
    return -quarter_sine(ODO_FULL_TURN_CDEG - angle);
}

int32_t odo_cos_q15(int32_t cdeg)
{
    return odo_sin_q15(cdeg + ODO_SIN_QUARTER_CDEG);
}

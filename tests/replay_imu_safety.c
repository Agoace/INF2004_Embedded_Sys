/* Replay a recorded IMU trace through the shipped safety detector.
 *
 * RT-0091 asks for a replay harness before the 0.4 g / 0.5 g bounds are
 * treated as device thresholds. No recorded trace exists in the tree yet, so
 * this tool is the analyser to use when one arrives; the checked-in fixture
 * is SYNTHETIC and proves only that the tool and the rule wiring work.
 *
 * Input CSV, one observation per line, in the order the detector consumes
 * them. Lines starting with '#' and blank lines are ignored. The header is
 * not required.
 *
 *     t_ms,ax_mg,ay_mg,az_mg,valid,calibrated,wheels_still
 *
 * The acceleration columns are the MAPPED, BIAS-CORRECTED values: exactly
 * what imu_service.c hands the detector (imu_core_output_t.accel_mg). The
 * axis mapping and the bias must already be applied; this tool does not
 * re-apply them, so it cannot silently analyse a different signal.
 *
 * Output: one line per confirmed trigger, then a summary line. Exit 0 when
 * the file parsed, 1 on a malformed row, 2 on a usage error.
 */
#include "../app_program/subsystem4/imu_safety.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LINE 256u
#define MAX_EVENTS 64u

static const char *reason_name(uint8_t trigger)
{
    if (trigger == (uint8_t)IMU_SAFETY_LIFT) {
        return "lift";
    }
    if (trigger == (uint8_t)IMU_SAFETY_OVERTURN) {
        return "overturn";
    }
    if (trigger == (uint8_t)IMU_SAFETY_IMU_UNAVAILABLE) {
        return "unavailable";
    }
    return "none";
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: %s <trace.csv>\n", argv[0]);
        return 2;
    }
    FILE *in = fopen(argv[1], "r");
    if (in == NULL) {
        fprintf(stderr, "cannot open %s\n", argv[1]);
        return 2;
    }
    imu_safety_t detector;
    imu_safety_init(&detector);

    unsigned long rows = 0ul;
    unsigned long events = 0ul;
    unsigned long bad = 0ul;
    uint32_t first_ms = 0u;
    uint8_t first_reason = (uint8_t)IMU_SAFETY_NONE;
    float min_g = 1.0e9f;
    float max_g = -1.0e9f;
    int32_t max_tilt = 0;
    char line[MAX_LINE];

    while (fgets(line, (int)sizeof(line), in) != NULL) {
        /* Skip blank lines, '#' comments and a column header: anything that
         * does not begin with a number is not an observation. A line that
         * does begin with a number but does not parse is reported. */
        const char *first = line;
        while (*first == ' ' || *first == '\t') {
            ++first;
        }
        if (*first == '\0' || *first == '\n' || *first == '\r' ||
            *first == '#') {
            continue;
        }
        if (!((*first >= '0' && *first <= '9') || *first == '-' ||
              *first == '+')) {
            continue;
        }
        unsigned long t_ms = 0ul;
        double ax = 0.0;
        double ay = 0.0;
        double az = 0.0;
        int valid = 0;
        int calibrated = 0;
        int still = 0;
        const int fields = sscanf(line, "%lu,%lf,%lf,%lf,%d,%d,%d",
                                  &t_ms, &ax, &ay, &az, &valid, &calibrated,
                                  &still);

        if (fields != 7) {
            ++bad;
            continue;
        }
        imu_safety_input_t sample = {
            .accel_mg = {(float)ax, (float)ay, (float)az},
            .now_ms = (uint32_t)t_ms,
            .accel_valid = valid != 0,
            .calibrated = calibrated != 0,
            .wheels_still = still != 0};
        imu_safety_step(&detector, &sample);
        ++rows;

        const imu_safety_output_t *o = &detector.output;
        if ((o->flags & IMU_SAFETY_FLAG_VALID) != 0u) {
            if (o->g_mg < min_g) {
                min_g = o->g_mg;
            }
            if (o->g_mg > max_g) {
                max_g = o->g_mg;
            }
            if (o->tilt_ddeg > max_tilt) {
                max_tilt = o->tilt_ddeg;
            }
        }
        if (o->current_trigger != (uint8_t)IMU_SAFETY_NONE &&
            events < MAX_EVENTS) {
            ++events;
            if (first_reason == (uint8_t)IMU_SAFETY_NONE) {
                first_reason = o->current_trigger;
                first_ms = o->observation_ms;
            }
            printf("trigger t_ms=%lu reason=%s g_mg=%.1f tilt_ddeg=%ld "
                   "vertical_mg=%.1f\n",
                   (unsigned long)o->observation_ms,
                   reason_name(o->current_trigger), (double)o->g_mg,
                   (long)o->tilt_ddeg, (double)o->vertical_mg);
        }
    }
    fclose(in);

    if (bad != 0ul) {
        fprintf(stderr, "%lu malformed rows\n", bad);
        return 1;
    }
    printf("summary rows=%lu events=%lu first_reason=%s first_ms=%lu "
           "latched=%s stops=%u min_g_mg=%.1f max_g_mg=%.1f "
           "max_tilt_ddeg=%ld clear_ms=%lu\n",
           rows, events, reason_name(first_reason), (unsigned long)first_ms,
           reason_name(detector.output.latched_trigger),
           (unsigned int)detector.output.stops,
           rows == 0ul ? 0.0 : (double)min_g,
           rows == 0ul ? 0.0 : (double)max_g, (long)max_tilt,
           (unsigned long)detector.output.clear_ms);
    return 0;
}

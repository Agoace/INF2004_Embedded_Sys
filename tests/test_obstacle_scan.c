/*
 * Host unit test for the pure parts of src/obstacle_scan.h: the angle and
 * pulse tables and the reading-combine rule. It needs no Pico hardware.
 *
 *   cc -std=c11 -Wall -Wextra -Wconversion -Isrc \
 *      tests/test_obstacle_scan.c -o build/test_obstacle_scan
 *   ./build/test_obstacle_scan
 */
#include <stdio.h>

#include "obstacle_scan.h"
#include "servo.h"

static int failures;

#define CHECK(cond, ...)                \
    do {                                \
        if (!(cond)) {                  \
            printf("FAIL: " __VA_ARGS__); \
            printf("\n");               \
            ++failures;                 \
        }                               \
    } while (0)

static ultrasonic_result_t word(ultrasonic_status_t status, uint16_t distance,
                                uint8_t streak) {
    return (ultrasonic_result_t)(
        ((uint32_t)distance << ULTRASONIC_DISTANCE_SHIFT) |
        ((uint32_t)status << ULTRASONIC_STATUS_SHIFT) |
        ((uint32_t)streak << ULTRASONIC_STREAK_SHIFT));
}

#define VALID(mm) word(ULTRASONIC_STATUS_VALID, (mm), 0u)
#define NO_RETURN word(ULTRASONIC_STATUS_NO_RETURN, 0u, 0u)
#define NO_RISE(s) word(ULTRASONIC_STATUS_NO_RISE, 0u, (s))
#define STUCK(s) word(ULTRASONIC_STATUS_STUCK_HIGH, 0u, (s))
#define EMPTY OBSTACLE_SCAN_POINT_EMPTY

static obstacle_scan_point_t two(ultrasonic_result_t a, ultrasonic_result_t b) {
    return obstacle_scan_combine(obstacle_scan_combine(EMPTY, a), b);
}

int main(void) {
    /* Angles and pulses. */
    for (unsigned int i = 0; i < OBSTACLE_SCAN_POINTS; ++i) {
        const int angle = obstacle_scan_angle_deg(i);
        const int expect = 1500 + (angle * 1000 + (angle < 0 ? -45 : 45)) / 90;
        const uint16_t pulse = obstacle_scan_pulse_us(i);
        CHECK(pulse == expect, "idx %u pulse %u, want %d", i, pulse, expect);
        CHECK(pulse >= SERVO_SAFE_MIN_US && pulse <= SERVO_SAFE_MAX_US,
              "idx %u pulse %u outside the safe range", i, pulse);
        if (i > 0u) {
            CHECK(pulse > obstacle_scan_pulse_us(i - 1u),
                  "pulses must rise from right to left");
        }
    }
    CHECK(obstacle_scan_angle_deg(0u) == -30, "idx 0 must be 30 deg right");
    CHECK(obstacle_scan_angle_deg(6u) == 0, "idx 6 must be straight ahead");
    CHECK(obstacle_scan_angle_deg(12u) == 30, "idx 12 must be 30 deg left");
    CHECK(obstacle_scan_pulse_us(13u) == 1500u, "unused idx 13 must be centre");
    CHECK(OBSTACLE_SCAN_POINTS <= 16u, "index must fit in 4 bits");

    /* Combine rule. */
    CHECK(two(VALID(300), VALID(250)) == VALID(250), "keep the nearer");
    CHECK(two(VALID(250), VALID(300)) == VALID(250), "keep the nearer");
    CHECK(two(VALID(250), VALID(0)) == VALID(0), "TOO_NEAR is the nearest");
    CHECK(two(VALID(250), NO_RETURN) == VALID(250), "VALID beats NO_RETURN");
    CHECK(two(NO_RETURN, VALID(250)) == VALID(250), "VALID beats NO_RETURN");
    CHECK(two(NO_RETURN, NO_RETURN) == NO_RETURN, "two NO_RETURN");
    CHECK(two(NO_RISE(1), NO_RETURN) == NO_RETURN, "NO_RETURN beats a failure");
    CHECK(two(NO_RETURN, STUCK(1)) == NO_RETURN, "a failure adds nothing");
    CHECK(two(NO_RISE(1), NO_RISE(2)) == EMPTY, "two failures give no data");
    CHECK(two(STUCK(3), VALID(120)) == VALID(120), "VALID after a failure");
    CHECK(obstacle_scan_combine(EMPTY, word(ULTRASONIC_STATUS_VALID, 120u, 0u)) ==
              0x0078u,
          "point word layout");

    if (failures == 0) {
        printf("test_obstacle_scan: all tests passed\n");
        return 0;
    }
    printf("test_obstacle_scan: %d failure(s)\n", failures);
    return 1;
}

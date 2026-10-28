/*
 * Host unit test for src/obstacle.c. It needs no Pico hardware.
 *
 *   cc -std=c11 -Wall -Wextra -Wconversion -Isrc \
 *      tests/test_obstacle.c src/obstacle.c -o build/test_obstacle
 *   ./build/test_obstacle
 */
#include <stdio.h>

#include "obstacle.h"

static int failures;

static ultrasonic_result_t word(ultrasonic_status_t status, uint16_t distance,
                                uint8_t streak) {
    return (ultrasonic_result_t)(
        ((uint32_t)distance << ULTRASONIC_DISTANCE_SHIFT) |
        ((uint32_t)status << ULTRASONIC_STATUS_SHIFT) |
        ((uint32_t)streak << ULTRASONIC_STREAK_SHIFT));
}

static ultrasonic_result_t mm(uint16_t distance) {
    return word(ULTRASONIC_STATUS_VALID, distance, 0u);
}

#define TOO_NEAR mm(ULTRASONIC_DISTANCE_TOO_NEAR)
#define NO_RETURN word(ULTRASONIC_STATUS_NO_RETURN, 0u, 0u)
#define NO_RISE(streak) word(ULTRASONIC_STATUS_NO_RISE, 0u, streak)

static void expect(const char *test, int step, obstacle_monitor_t *m,
                   ultrasonic_result_t in, obstacle_event_t want_event,
                   obstacle_state_t want_state) {
    const obstacle_event_word_t out = obstacle_update(m, in);
    const obstacle_event_t got_event = obstacle_event(out);
    const obstacle_state_t got_state = obstacle_state(m);

    if (got_event != want_event || got_state != want_state) {
        printf("FAIL %s step %d: input 0x%04X -> event %d state %d, "
               "want event %d state %d\n",
               test, step, in, got_event, got_state, want_event, want_state);
        ++failures;
    }
}

typedef struct {
    ultrasonic_result_t in;
    obstacle_event_t event;
    obstacle_state_t state;
} step_t;

static void run(const char *test, const step_t *steps, int count) {
    obstacle_monitor_t m;
    obstacle_init(&m);
    for (int i = 0; i < count; ++i) {
        expect(test, i, &m, steps[i].in, steps[i].event, steps[i].state);
    }
}

#define RUN(name, ...)                                                    \
    do {                                                                  \
        const step_t steps_[] = {__VA_ARGS__};                            \
        run(name, steps_, (int)(sizeof steps_ / sizeof steps_[0]));       \
    } while (0)

#define S(in, ev, st) {(in), OBSTACLE_EVENT_##ev, OBSTACLE_##st}

int main(void) {
    /* Thresholds at the sensor: STOP below 258, NEAR below 400,
     * release STOP at 278 or more, clear NEAR at 420 or more. */
    RUN("approach and stop",
        S(mm(500), NONE, MONITORING),
        S(mm(390), NONE, MONITORING),
        S(mm(380), NEAR, APPROACHING),
        S(mm(300), NONE, APPROACHING),
        S(mm(250), NONE, APPROACHING),
        S(mm(240), STOP, CONFIRMED),
        S(mm(230), NONE, CONFIRMED));

    RUN("single reading does not confirm",
        S(mm(380), NONE, MONITORING),
        S(mm(500), NONE, MONITORING),
        S(mm(250), NONE, MONITORING),
        S(mm(500), NONE, MONITORING));

    RUN("fast approach goes straight to stop",
        S(mm(200), NONE, MONITORING),
        S(mm(190), STOP, CONFIRMED));

    RUN("TOO_NEAR is a stop reading",
        S(TOO_NEAR, NONE, MONITORING),
        S(TOO_NEAR, STOP, CONFIRMED));

    RUN("NO_RETURN never releases a stop",
        S(mm(200), NONE, MONITORING),
        S(mm(200), STOP, CONFIRMED),
        S(NO_RETURN, NONE, CONFIRMED),
        S(NO_RETURN, NONE, CONFIRMED),
        S(NO_RETURN, NONE, CONFIRMED),
        S(NO_RETURN, NONE, CONFIRMED));

    RUN("release stop in two steps",
        S(mm(200), NONE, MONITORING),
        S(mm(200), STOP, CONFIRMED),
        S(mm(500), NONE, CONFIRMED),
        S(mm(500), NEAR, APPROACHING),
        S(mm(500), CLEAR, MONITORING));

    RUN("NO_RETURN between valid echoes restarts the release",
        S(mm(200), NONE, MONITORING),
        S(mm(200), STOP, CONFIRMED),
        S(mm(300), NONE, CONFIRMED),
        S(NO_RETURN, NONE, CONFIRMED),
        S(mm(300), NONE, CONFIRMED),
        S(mm(300), NEAR, APPROACHING));

    RUN("dead band at the stop threshold",
        S(mm(256), NONE, MONITORING),
        S(mm(256), STOP, CONFIRMED),
        S(mm(260), NONE, CONFIRMED),
        S(mm(260), NONE, CONFIRMED),
        S(mm(256), NONE, CONFIRMED),
        S(mm(277), NONE, CONFIRMED),
        S(mm(278), NONE, CONFIRMED),
        S(mm(278), NEAR, APPROACHING));

    RUN("dead band at the near threshold",
        S(mm(398), NONE, MONITORING),
        S(mm(398), NEAR, APPROACHING),
        S(mm(402), NONE, APPROACHING),
        S(mm(402), NONE, APPROACHING),
        S(mm(420), NONE, APPROACHING),
        S(mm(420), CLEAR, MONITORING));

    RUN("NO_RETURN can end approaching",
        S(mm(350), NONE, MONITORING),
        S(mm(350), NEAR, APPROACHING),
        S(NO_RETURN, NONE, APPROACHING),
        S(NO_RETURN, CLEAR, MONITORING));

    RUN("single fault keeps the counts",
        S(mm(200), NONE, MONITORING),
        S(NO_RISE(1), NONE, MONITORING),
        S(mm(200), STOP, CONFIRMED));

    RUN("fault streak latches",
        S(mm(200), NONE, MONITORING),
        S(mm(200), STOP, CONFIRMED),
        S(NO_RISE(1), NONE, CONFIRMED),
        S(NO_RISE(4), NONE, CONFIRMED),
        S(NO_RISE(5), SENSOR_FAULT, SENSOR_FAULT),
        S(NO_RISE(5), NONE, SENSOR_FAULT),
        S(mm(500), NONE, SENSOR_FAULT),
        S(mm(500), NONE, SENSOR_FAULT));

    RUN("no result yet is ignored",
        S(ULTRASONIC_RESULT_NONE, NONE, MONITORING),
        S(mm(380), NONE, MONITORING),
        S(ULTRASONIC_RESULT_NONE, NONE, MONITORING),
        S(mm(380), NEAR, APPROACHING));

    /* Event word layout. */
    {
        obstacle_monitor_t m;
        obstacle_init(&m);
        (void)obstacle_update(&m, mm(240));
        const obstacle_event_word_t w = obstacle_update(&m, mm(237));
        if (w != (obstacle_event_word_t)((3u << 9) | 237u) ||
            obstacle_event_distance(w) != 237u) {
            printf("FAIL event word: 0x%04X\n", w);
            ++failures;
        }
    }

    if (failures == 0) {
        printf("test_obstacle: all tests passed\n");
        return 0;
    }
    printf("test_obstacle: %d failure(s)\n", failures);
    return 1;
}

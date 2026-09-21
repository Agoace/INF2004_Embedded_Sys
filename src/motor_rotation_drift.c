#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#include "telemetry.h"

#define MOTOR_1_PWM_PIN 8
#define MOTOR_1_BRAKE_PIN 9
#define MOTOR_2_PWM_PIN 10
#define MOTOR_2_BRAKE_PIN 11

#define ENCODER_1_A_PIN 16
#define ENCODER_1_B_PIN 17
#define ENCODER_2_A_PIN 4
#define ENCODER_2_B_PIN 5

#define MOTOR_1_BUTTON_PIN 20
#define MOTOR_2_BUTTON_PIN 21

/* Measured 2535.000 counts/revolution by a 50-revolution continuous
 * forward run with count-based forward jog to visual mark alignment, motor 1,
 * wheel lifted, 10% duty, Raspberry Pi 400 USB supply. A second 51-revolution
 * run measured 2534.96. This supersedes 2540, which came from a chosen
 * 2500-count single-revolution stop plus observed coast and roughly +/-20
 * counts of visual resolution, not an independent measurement. 2535 is not
 * divisible by 4, so the encoder is ahead of a non-integer gear ratio and the
 * true value is not expected to be a round number. */
#define COUNTS_PER_REVOLUTION 2535
/* Raise to 100 for highest precision. At 10% duty on USB power, each
 * revolution takes about 2.62 seconds: 50 takes about 131 seconds and 100
 * takes about 262 seconds. */
#define TEST_REVOLUTIONS 50
/* Coast was measured at 24-25 counts with the wheel lifted at 10% duty on
 * USB power. It does not transfer to a loaded wheel, different supply, or
 * different duty, so it is deliberately not applied to the target. Coast is
 * reported separately. A future nonzero value can be applied once per run;
 * never scale it by TEST_REVOLUTIONS. */
#define COAST_COMPENSATION_COUNTS 0
/* Deliberately stop short of the mark so the operator jogs forward to
 * alignment at exactly TEST_REVOLUTIONS. 300 counts is about 42 degrees,
 * larger than the plausible difference between the 2535 and 2540 cases. */
#define UNDERSHOOT_COUNTS 300
#define IDEAL_COUNTS (COUNTS_PER_REVOLUTION * TEST_REVOLUTIONS)
#define TARGET_COUNTS \
    (IDEAL_COUNTS - COAST_COMPENSATION_COUNTS - UNDERSHOOT_COUNTS)
#define PWM_WRAP 6249
#define PWM_DUTY_PERCENT 10
#define JOG_DUTY_PERCENT 10
#define JOG_COARSE_COUNTS 40
#define JOG_FINE_COUNTS 3
#define JOG_PULSE_TIMEOUT_MS 500
#define BUTTON_POLL_MS 20
/* At 10% duty the wheel advances roughly 19 counts per 20 ms, so 500 ms
 * without any count change means the motor is not turning. */
#define STALL_TIMEOUT_MS 500
/* Allow the motor to start from rest before the stall timer is checked. */
#define STALL_GRACE_MS 1000
#define BRAKE_SETTLE_MS 100
#define TEST_TIMEOUT_MS (TEST_REVOLUTIONS * 4000u)

typedef struct {
    uint a_pin;
    uint b_pin;
    volatile uint32_t a_edges;
    volatile uint32_t b_edges;
    volatile int32_t quadrature_count;
    volatile uint8_t previous_state;
} encoder_state_t;

static encoder_state_t encoder_1 = {
    .a_pin = ENCODER_1_A_PIN,
    .b_pin = ENCODER_1_B_PIN,
};

static encoder_state_t encoder_2 = {
    .a_pin = ENCODER_2_A_PIN,
    .b_pin = ENCODER_2_B_PIN,
};

static const int8_t quadrature_table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0,
};

static uint motor_1_slice;
static uint motor_2_slice;
static volatile bool stop_armed;
static volatile bool stop_triggered;
static volatile uint32_t stop_target_counts;
static volatile uint8_t active_motor;
static volatile int32_t stop_count_snapshot;
static volatile uint32_t boundary_us[TEST_REVOLUTIONS];
static volatile uint32_t boundary_count[TEST_REVOLUTIONS];
static volatile uint32_t boundary_index;
static volatile uint32_t boundary_start_us;

typedef struct {
    bool valid;
    bool completed;
    bool emergency_stop;
    bool timed_out;
    bool stalled;
    bool jog_complete;
    unsigned int motor_number;
    bool reverse;
    unsigned int duty_percent;
    unsigned int revolutions_requested;
    int32_t count_at_stop;
    uint32_t absolute_count_at_stop;
    int32_t count_at_trigger;
    uint32_t absolute_count_at_trigger;
    int32_t count_after_settle;
    uint32_t absolute_count_after_settle;
    int32_t overshoot_counts;
    uint32_t boundary_count_reached;
    int32_t jog_counts;
    uint32_t jog_final_absolute_count;
    uint32_t jog_combined_count;
    bool jog_movement_seen;
    uint32_t smallest_jog_movement;
    uint32_t elapsed_us;
    uint32_t mean_us_per_revolution;
    int32_t drift_counts;
    int32_t drift_millidegrees;
    int32_t drift_per_revolution_millidegrees;
    uint32_t elapsed_ms;
    uint32_t mean_ms_per_revolution;
} drift_result_t;

static drift_result_t last_result;
static bool telemetry_enabled;

static uint8_t read_encoder_state(const encoder_state_t *encoder) {
    return (uint8_t)((gpio_get(encoder->a_pin) << 1u) |
                     gpio_get(encoder->b_pin));
}

static void update_encoder(encoder_state_t *encoder, uint changed_gpio) {
    if (changed_gpio == encoder->a_pin) {
        ++encoder->a_edges;
    } else {
        ++encoder->b_edges;
    }

    const uint8_t current_state = read_encoder_state(encoder);
    const uint8_t table_index =
        (uint8_t)((encoder->previous_state << 2u) | current_state);
    encoder->quadrature_count += quadrature_table[table_index];
    encoder->previous_state = current_state;

    const bool active_encoder =
        (encoder == &encoder_1 && active_motor == 1) ||
        (encoder == &encoder_2 && active_motor == 2);
    if (active_encoder) {
        const uint32_t count = (uint32_t)(
            encoder->quadrature_count < 0
                ? -encoder->quadrature_count
                : encoder->quadrature_count);

        while (boundary_index < TEST_REVOLUTIONS &&
               count >= (boundary_index + 1u) * COUNTS_PER_REVOLUTION) {
            boundary_us[boundary_index] =
                (uint32_t)to_us_since_boot(get_absolute_time()) -
                boundary_start_us;
            boundary_count[boundary_index] = count;
            ++boundary_index;
        }

        if (stop_armed && count >= stop_target_counts) {
            stop_armed = false;
            stop_triggered = true;
            stop_count_snapshot = encoder->quadrature_count;
            if (active_motor == 1) {
                pwm_set_gpio_level(MOTOR_1_PWM_PIN, 0);
                pwm_set_gpio_level(MOTOR_1_BRAKE_PIN, 0);
            } else {
                pwm_set_gpio_level(MOTOR_2_PWM_PIN, 0);
                pwm_set_gpio_level(MOTOR_2_BRAKE_PIN, 0);
            }
        }
    }
}

static void encoder_irq_callback(uint gpio, uint32_t events) {
    (void)events;

    if (gpio == encoder_1.a_pin || gpio == encoder_1.b_pin) {
        update_encoder(&encoder_1, gpio);
    } else if (gpio == encoder_2.a_pin || gpio == encoder_2.b_pin) {
        update_encoder(&encoder_2, gpio);
    }
}

static void initialise_encoder(encoder_state_t *encoder, bool set_callback) {
    gpio_init(encoder->a_pin);
    gpio_set_dir(encoder->a_pin, GPIO_IN);
    gpio_disable_pulls(encoder->a_pin);
    gpio_init(encoder->b_pin);
    gpio_set_dir(encoder->b_pin, GPIO_IN);
    gpio_disable_pulls(encoder->b_pin);
    encoder->previous_state = read_encoder_state(encoder);

    const uint32_t events = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    if (set_callback) {
        gpio_set_irq_enabled_with_callback(
            encoder->a_pin, events, true, encoder_irq_callback);
    } else {
        gpio_set_irq_enabled(encoder->a_pin, events, true);
    }
    gpio_set_irq_enabled(encoder->b_pin, events, true);
}

static void reset_encoder(encoder_state_t *encoder) {
    const uint32_t interrupt_state = save_and_disable_interrupts();
    encoder->a_edges = 0;
    encoder->b_edges = 0;
    encoder->quadrature_count = 0;
    encoder->previous_state = read_encoder_state(encoder);
    restore_interrupts(interrupt_state);
}

static void initialise_motor_pwm(uint pin, uint *slice) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    *slice = pwm_gpio_to_slice_num(pin);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.0f);
    pwm_config_set_wrap(&config, PWM_WRAP);
    pwm_init(*slice, &config, true);
    pwm_set_gpio_level(pin, 0);
}

static void initialise_brake_pin(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, false);
}

static uint16_t motor_duty_level(void) {
    return (uint16_t)(((PWM_WRAP + 1u) * PWM_DUTY_PERCENT) / 100u);
}

static uint16_t jog_duty_level(void) {
    return (uint16_t)(((PWM_WRAP + 1u) * JOG_DUTY_PERCENT) / 100u);
}

static void stop_all_motors(void) {
    pwm_set_gpio_level(MOTOR_1_PWM_PIN, 0);
    pwm_set_gpio_level(MOTOR_1_BRAKE_PIN, 0);
    pwm_set_gpio_level(MOTOR_2_PWM_PIN, 0);
    pwm_set_gpio_level(MOTOR_2_BRAKE_PIN, 0);
}

static void drive_motor_1(bool reverse) {
    if (reverse) {
        pwm_set_gpio_level(MOTOR_1_PWM_PIN, 0);
        pwm_set_gpio_level(MOTOR_1_BRAKE_PIN, motor_duty_level());
    } else {
        pwm_set_gpio_level(MOTOR_1_BRAKE_PIN, 0);
        pwm_set_gpio_level(MOTOR_1_PWM_PIN, motor_duty_level());
    }
}

static void drive_motor_2(bool reverse) {
    if (reverse) {
        pwm_set_gpio_level(MOTOR_2_PWM_PIN, 0);
        pwm_set_gpio_level(MOTOR_2_BRAKE_PIN, motor_duty_level());
    } else {
        pwm_set_gpio_level(MOTOR_2_BRAKE_PIN, 0);
        pwm_set_gpio_level(MOTOR_2_PWM_PIN, motor_duty_level());
    }
}

static int32_t read_quadrature_count(const encoder_state_t *encoder) {
    const uint32_t interrupt_state = save_and_disable_interrupts();
    const int32_t count = encoder->quadrature_count;
    restore_interrupts(interrupt_state);
    return count;
}

static uint32_t absolute_count(int32_t count) {
    return (uint32_t)(count < 0 ? -count : count);
}

static void send_motor_telemetry(const telemetry_motor_sample_t *sample) {
    if (telemetry_enabled && telemetry_is_connected()) {
        (void)telemetry_send_motor_line(sample);
    }
}

static void send_run_progress_telemetry(unsigned int motor_number,
                                         uint32_t observed_count,
                                         uint32_t elapsed_ms,
                                         bool stall_monitoring) {
    telemetry_motor_sample_t sample = {0};
    sample.test_type = "rotation_drift";
    sample.state = "running";
    sample.direction = "forward";
    sample.startup_state = stall_monitoring ? "complete" : "grace";
    sample.stall_state = "clear";
    sample.emergency_state = "clear";
    sample.error_state = "none";
    sample.has_motor_number = true;
    sample.motor_number = motor_number;
    sample.has_duty_percent = true;
    sample.duty_percent = PWM_DUTY_PERCENT;
    sample.has_revolutions_requested = true;
    sample.revolutions_requested = TEST_REVOLUTIONS;
    sample.has_revolution_index = true;
    sample.revolution_index = observed_count / COUNTS_PER_REVOLUTION;
    sample.has_absolute_count = true;
    sample.absolute_count = observed_count;
    sample.has_elapsed_ms = true;
    sample.elapsed_ms = elapsed_ms;
    sample.has_completed = true;
    sample.completed = false;
    send_motor_telemetry(&sample);
}

static const char *result_state(const drift_result_t *result) {
    if (result->completed) {
        return "completed";
    }
    if (result->emergency_stop) {
        return "emergency_stop";
    }
    if (result->stalled) {
        return "stalled";
    }
    if (result->timed_out) {
        return "safety_timeout";
    }
    return "incomplete";
}

static void send_completed_telemetry(bool include_splits) {
    if (!last_result.valid) {
        return;
    }

    const drift_result_t *result = &last_result;
    telemetry_motor_sample_t sample = {0};
    sample.test_type = "rotation_drift";
    sample.state = result_state(result);
    sample.direction = result->reverse ? "reverse" : "forward";
    sample.startup_state = "complete";
    sample.stall_state = result->stalled ? "active" : "clear";
    sample.emergency_state = result->emergency_stop ? "active" : "clear";
    sample.error_state = result->timed_out ? "timeout" :
                         (result->stalled ? "stall" : "none");
    sample.has_motor_number = true;
    sample.motor_number = result->motor_number;
    sample.has_duty_percent = true;
    sample.duty_percent = result->duty_percent;
    sample.has_revolutions_requested = true;
    sample.revolutions_requested = result->revolutions_requested;
    sample.has_ideal_counts = true;
    sample.ideal_counts = IDEAL_COUNTS;
    sample.has_target_counts = true;
    sample.target_counts = TARGET_COUNTS;
    sample.has_count_at_stop = true;
    sample.count_at_stop = result->count_at_stop;
    sample.has_absolute_count_at_stop = true;
    sample.absolute_count_at_stop = result->absolute_count_at_stop;
    sample.has_count_after_settle = true;
    sample.count_after_settle = result->count_after_settle;
    sample.has_absolute_count_after_settle = true;
    sample.absolute_count_after_settle = result->absolute_count_after_settle;
    sample.has_coast_overshoot_counts = true;
    sample.coast_overshoot_counts = result->overshoot_counts;
    sample.has_boundary_count_reached = true;
    sample.boundary_count_reached = result->boundary_count_reached;
    sample.has_elapsed_ms = true;
    sample.elapsed_ms = result->elapsed_ms;
    sample.has_completed = true;
    sample.completed = true;

    if (result->completed) {
        sample.has_count_at_trigger = true;
        sample.count_at_trigger = result->count_at_trigger;
        sample.has_absolute_count_at_trigger = true;
        sample.absolute_count_at_trigger = result->absolute_count_at_trigger;
        sample.has_undershoot_counts = true;
        sample.undershoot_counts = UNDERSHOOT_COUNTS;
        sample.has_drift_counts = true;
        sample.drift_counts = result->drift_counts;
        sample.has_drift_millidegrees = true;
        sample.drift_millidegrees = result->drift_millidegrees;
        sample.has_drift_per_revolution_millidegrees = true;
        sample.drift_per_revolution_millidegrees =
            result->drift_per_revolution_millidegrees;
        sample.has_elapsed_us = true;
        sample.elapsed_us = result->elapsed_us;
        sample.has_mean_us_per_revolution = true;
        sample.mean_us_per_revolution = result->mean_us_per_revolution;
        sample.has_mean_ms_per_revolution = true;
        sample.mean_ms_per_revolution = result->mean_ms_per_revolution;
        sample.has_counts_per_rev_milli = true;
        sample.counts_per_rev_milli =
            (result->absolute_count_after_settle * 1000u) /
            TEST_REVOLUTIONS;
        sample.has_sensitivity_millicounts = true;
        sample.sensitivity_millicounts =
            (IDEAL_COUNTS * 1000u) / (360u * TEST_REVOLUTIONS);
    }
    if (result->jog_complete) {
        sample.has_jog_counts = true;
        sample.jog_counts = result->jog_counts;
        sample.has_jog_final_absolute_count = true;
        sample.jog_final_absolute_count = result->jog_final_absolute_count;
        sample.has_jog_combined_count = true;
        sample.jog_combined_count = result->jog_combined_count;
        sample.has_smallest_jog_movement = true;
        sample.smallest_jog_movement = result->smallest_jog_movement;
        sample.has_alignment_resolution_cpr_milli = true;
        sample.alignment_resolution_cpr_milli =
            (result->smallest_jog_movement * 1000u) /
            TEST_REVOLUTIONS;
        sample.has_jog_primary_cpr_milli = true;
        sample.jog_primary_cpr_milli =
            (result->jog_combined_count * 1000u) /
            TEST_REVOLUTIONS;
        sample.has_jog_secondary_cpr_milli = true;
        sample.jog_secondary_cpr_milli =
            (result->jog_combined_count * 1000u) /
            (TEST_REVOLUTIONS + 1u);
    }
    send_motor_telemetry(&sample);

    if (!include_splits || !result->completed) {
        return;
    }
    for (uint32_t index = 0;
         index < result->boundary_count_reached && index < TEST_REVOLUTIONS;
         ++index) {
        const uint32_t previous_count = index == 0 ? 0 : boundary_count[index - 1];
        const uint32_t previous_us = index == 0 ? 0 : boundary_us[index - 1];
        telemetry_motor_sample_t split = {0};
        split.test_type = "rotation_drift";
        split.state = "completed_split";
        split.direction = result->reverse ? "reverse" : "forward";
        split.has_motor_number = true;
        split.motor_number = result->motor_number;
        split.has_split_index = true;
        split.split_index = index;
        split.has_split_total = true;
        split.split_total = result->boundary_count_reached;
        split.has_split_revolution = true;
        split.split_revolution = index + 1u;
        split.has_split_absolute_count = true;
        split.split_absolute_count = boundary_count[index];
        split.has_split_elapsed_us = true;
        split.split_elapsed_us = boundary_us[index];
        split.has_split_delta_counts = true;
        split.split_delta_counts = boundary_count[index] - previous_count;
        split.has_split_delta_us = true;
        split.split_delta_us = boundary_us[index] - previous_us;
        split.has_completed = true;
        split.completed = true;
        send_motor_telemetry(&split);
    }
}

static void print_millicounts(const char *label, uint32_t numerator,
                              uint32_t denominator) {
    const uint64_t millicounts =
        ((uint64_t)numerator * 1000u) / denominator;
    printf("%s%lu.%03lu counts/rev\n",
           label,
           (unsigned long)(millicounts / 1000u),
           (unsigned long)(millicounts % 1000u));
}

static void print_result(const drift_result_t *result) {
    printf("\nSaved motor rotation drift result:\n");
    printf("  motor number: %u\n", result->motor_number);
    printf("  direction: %s\n", result->reverse ? "reverse" : "forward");
    printf("  duty: %u%%\n", result->duty_percent);
    printf("  revolutions requested: %u\n", result->revolutions_requested);
    printf("  ideal counts: %u\n", IDEAL_COUNTS);
    printf("  target counts (ideal minus compensation): %u\n", TARGET_COUNTS);
    printf("  signed count at main-loop read: %ld\n",
           (long)result->count_at_stop);
    printf("  absolute count at main-loop read: %lu\n",
           (unsigned long)result->absolute_count_at_stop);
    if (result->completed) {
        printf("  signed count at ISR trigger: %ld\n",
               (long)result->count_at_trigger);
        printf("  absolute count at ISR trigger: %lu\n",
               (unsigned long)result->absolute_count_at_trigger);
        printf("  outcome: completed\n");
    } else if (result->emergency_stop) {
        printf("  outcome: emergency stop\n");
    } else if (result->stalled) {
        printf("  outcome: stalled\n");
    } else if (result->timed_out) {
        printf("  outcome: safety timeout\n");
    } else {
        printf("  outcome: incomplete\n");
    }
    printf("  signed count after coast settle: %ld\n",
           (long)result->count_after_settle);
    printf("  absolute count after coast settle: %lu\n",
           (unsigned long)result->absolute_count_after_settle);
    printf("  coast overshoot counts: %ld\n", (long)result->overshoot_counts);
    printf("  boundary splits reached: %lu/%u\n",
           (unsigned long)result->boundary_count_reached,
           TEST_REVOLUTIONS);
    if (result->completed) {
        for (uint32_t index = 0;
             index < result->boundary_count_reached &&
             index < TEST_REVOLUTIONS;
             ++index) {
            const uint32_t elapsed_us = boundary_us[index];
            const uint32_t previous_count =
                index == 0 ? 0 : boundary_count[index - 1];
            const uint32_t previous_us =
                index == 0 ? 0 : boundary_us[index - 1];
            printf("  revolution %lu/%u: absolute count %lu at %lu us; "
                   "delta %lu counts, %lu us\n",
                   (unsigned long)(index + 1), TEST_REVOLUTIONS,
                   (unsigned long)boundary_count[index],
                   (unsigned long)elapsed_us,
                   (unsigned long)(boundary_count[index] - previous_count),
                   (unsigned long)(boundary_us[index] - previous_us));
        }
        printf("  deliberate undershoot counts: %u\n", UNDERSHOOT_COUNTS);
        printf("  drift vs ideal (includes deliberate undershoot): %ld\n",
               (long)result->drift_counts);
        printf("  drift millidegrees: %ld\n",
               (long)result->drift_millidegrees);
        printf("  drift per revolution millidegrees: %ld\n",
               (long)result->drift_per_revolution_millidegrees);
        printf("  total elapsed us: %lu\n",
               (unsigned long)result->elapsed_us);
        printf("  mean us per revolution: %lu\n",
               (unsigned long)result->mean_us_per_revolution);
        printf("  total elapsed ms: %lu\n",
               (unsigned long)result->elapsed_ms);
        printf("  mean ms per revolution: %lu\n",
               (unsigned long)result->mean_ms_per_revolution);
        printf("\nCPR measurement guidance:\n");
        printf("  revolutions commanded: %u\n", TEST_REVOLUTIONS);
        printf("  final absolute count: %lu\n",
               (unsigned long)result->absolute_count_after_settle);
        printf("  deliberate undershoot counts: %u\n", UNDERSHOOT_COUNTS);
        printf("  coast counts (lifted-wheel measurement): %ld\n",
               (long)result->overshoot_counts);
        printf("  true counts/rev = final absolute count / "
               "(revolutions + visual offset degrees / 360)\n");
        print_millicounts("  counts/rev assuming zero visual offset: ",
                          result->absolute_count_after_settle,
                          TEST_REVOLUTIONS);
        const uint32_t sensitivity_millicounts =
            (IDEAL_COUNTS * 1000u) /
            (360u * TEST_REVOLUTIONS);
        printf("  one degree of visual offset changes counts/rev by "
               "%lu millicounts\n",
               (unsigned long)sensitivity_millicounts);
        if (result->jog_complete) {
            printf("\nJog alignment result:\n");
            printf("  revolutions commanded: %u\n", TEST_REVOLUTIONS);
            printf("  final absolute count after run: %lu\n",
                   (unsigned long)result->absolute_count_after_settle);
            printf("  total jog counts: %ld\n", (long)result->jog_counts);
            printf("  combined total: %lu\n",
                   (unsigned long)result->jog_combined_count);
            printf("  measured alignment resolution: %lu counts "
                   "(smallest single-pulse movement)\n",
                   (unsigned long)result->smallest_jog_movement);
            print_millicounts(
                "  alignment resolution at N revolutions: ",
                result->smallest_jog_movement, TEST_REVOLUTIONS);
            print_millicounts(
                "  primary counts/rev if aligned at N revolutions: ",
                result->jog_combined_count, TEST_REVOLUTIONS);
            print_millicounts(
                "  secondary counts/rev at N+1 (overshoot guard): ",
                result->jog_combined_count, TEST_REVOLUTIONS + 1u);
        }
        printf("Align a wheel-rim mark with a fixed chassis mark before "
               "starting; photograph both start and end from the same "
               "position. Report the wheel marker's visual final position; "
               "encoder counts cannot detect wheel slip or gearbox backlash.\n");
    } else {
        printf("  Run did not finish; drift and derived timing are not "
               "meaningful.\n");
        if (result->stalled) {
            printf("  Check for a mechanical jam or snagged cable and "
                   "confirm the wheel is free before rerunning.\n");
        }
    }
}

static void print_saved_result(void) {
    if (!last_result.valid) {
        return;
    }

    print_result(&last_result);
    printf("Result remains in RAM while VIN power stays on.\n");
}

static void run_motor_test(unsigned int motor_number,
                           encoder_state_t *encoder,
                           uint emergency_button_pin) {
    last_result.valid = false;
    last_result.jog_complete = false;
    last_result.stalled = false;
    last_result.jog_counts = 0;
    last_result.jog_final_absolute_count = 0;
    last_result.jog_combined_count = 0;
    last_result.jog_movement_seen = false;
    last_result.smallest_jog_movement = 0;
    reset_encoder(encoder);

    printf("\nMotor %u: forward continuous run for %u revolutions at %u%% duty.\n",
           motor_number, TEST_REVOLUTIONS, PWM_DUTY_PERCENT);
    printf("The other motor remains disabled.\n");

    const uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    const uint32_t start_us =
        (uint32_t)to_us_since_boot(get_absolute_time());
    stop_target_counts = TARGET_COUNTS;
    stop_triggered = false;
    stop_count_snapshot = 0;
    boundary_start_us = start_us;
    boundary_index = 0;
    active_motor = (uint8_t)motor_number;
    stop_armed = true;
    if (motor_number == 1) {
        drive_motor_1(false);
    } else {
        drive_motor_2(false);
    }

    bool emergency_stop = false;
    bool timed_out = false;
    bool stalled = false;
    bool target_reached = false;
    uint32_t elapsed_ms = 0;
    uint32_t last_observed_count = absolute_count(read_quadrature_count(encoder));
    uint32_t last_count_change_ms = 0;
    bool stall_monitoring = false;
    uint32_t next_telemetry_ms = 1000;

    while (true) {
        const uint32_t now_ms =
            to_ms_since_boot(get_absolute_time()) - start_ms;
        if (!gpio_get(emergency_button_pin)) {
            emergency_stop = true;
            elapsed_ms = now_ms;
            break;
        }

        const uint32_t observed_count =
            absolute_count(read_quadrature_count(encoder));
        if (observed_count != last_observed_count) {
            last_observed_count = observed_count;
            last_count_change_ms = now_ms;
        }
        if (!stall_monitoring && now_ms >= STALL_GRACE_MS) {
            stall_monitoring = true;
        }
        if (stall_monitoring &&
            now_ms - last_count_change_ms >= STALL_TIMEOUT_MS) {
            stalled = true;
            elapsed_ms = now_ms;
            break;
        }

        if (stop_triggered) {
            target_reached = true;
            elapsed_ms = now_ms;
            break;
        }
        if (now_ms >= TEST_TIMEOUT_MS) {
            timed_out = true;
            elapsed_ms = now_ms;
            break;
        }

        /* Send only after emergency, stall, stop, and timeout checks. */
        if (now_ms >= next_telemetry_ms) {
            send_run_progress_telemetry(motor_number, observed_count,
                                         now_ms, stall_monitoring);
            do {
                next_telemetry_ms += 1000u;
            } while (next_telemetry_ms <= now_ms);
        }
        sleep_ms(BUTTON_POLL_MS);
    }

    stop_armed = false;
    if (!target_reached) {
        active_motor = 0;
    }
    const int32_t count_at_stop = read_quadrature_count(encoder);
    const uint32_t absolute_count_at_stop = absolute_count(count_at_stop);

    stop_all_motors();
    sleep_ms(BRAKE_SETTLE_MS);
    active_motor = 0;
    const int32_t count_after_settle = read_quadrature_count(encoder);
    const uint32_t absolute_count_after_settle =
        absolute_count(count_after_settle);

    last_result.completed = target_reached;
    last_result.emergency_stop = emergency_stop;
    last_result.timed_out = timed_out;
    last_result.stalled = stalled;
    last_result.count_at_trigger = 0;
    last_result.absolute_count_at_trigger = 0;
    if (target_reached) {
        last_result.count_at_trigger = stop_count_snapshot;
        last_result.absolute_count_at_trigger =
            absolute_count(stop_count_snapshot);
    }
    last_result.motor_number = motor_number;
    last_result.reverse = false;
    last_result.duty_percent = PWM_DUTY_PERCENT;
    last_result.revolutions_requested = TEST_REVOLUTIONS;
    last_result.count_at_stop = count_at_stop;
    last_result.absolute_count_at_stop = absolute_count_at_stop;
    last_result.count_after_settle = count_after_settle;
    last_result.absolute_count_after_settle = absolute_count_after_settle;
    last_result.boundary_count_reached = boundary_index;
    last_result.overshoot_counts = 0;
    if (last_result.completed) {
        last_result.overshoot_counts =
            (int32_t)absolute_count_after_settle -
            (int32_t)last_result.absolute_count_at_trigger;
    }
    last_result.drift_counts = 0;
    last_result.drift_millidegrees = 0;
    last_result.drift_per_revolution_millidegrees = 0;
    if (last_result.completed) {
        last_result.drift_counts =
            (int32_t)absolute_count_after_settle - IDEAL_COUNTS;
        last_result.drift_millidegrees =
            (last_result.drift_counts * 360000) / COUNTS_PER_REVOLUTION;
        last_result.drift_per_revolution_millidegrees =
            last_result.drift_millidegrees / TEST_REVOLUTIONS;
    }
    last_result.elapsed_us = elapsed_ms * 1000u;
    if (target_reached && boundary_index > 0) {
        last_result.elapsed_us = boundary_us[boundary_index - 1];
    }
    last_result.mean_us_per_revolution =
        last_result.elapsed_us / TEST_REVOLUTIONS;
    last_result.elapsed_ms = last_result.elapsed_us / 1000u;
    last_result.mean_ms_per_revolution =
        last_result.mean_us_per_revolution / 1000u;
    last_result.valid = true;

    if (emergency_stop) {
        printf("Emergency stop activated.\n");
    } else if (stalled) {
        printf("Stall guard activated after %u ms without count change.\n",
               STALL_TIMEOUT_MS);
    } else if (timed_out) {
        printf("Safety timeout after %u ms.\n", TEST_TIMEOUT_MS);
    } else if (target_reached) {
        printf("Target reached after approximately %lu ms.\n",
               (unsigned long)last_result.elapsed_ms);
    }
    print_result(&last_result);
}

static void drive_jog_motor(unsigned int motor_number) {
    const uint16_t duty_level = jog_duty_level();
    stop_all_motors();
    if (motor_number == 1) {
        pwm_set_gpio_level(MOTOR_1_BRAKE_PIN, 0);
        pwm_set_gpio_level(MOTOR_1_PWM_PIN, duty_level);
    } else {
        pwm_set_gpio_level(MOTOR_2_BRAKE_PIN, 0);
        pwm_set_gpio_level(MOTOR_2_PWM_PIN, duty_level);
    }
}

static void perform_jog_pulse(unsigned int motor_number,
                              encoder_state_t *encoder,
                              uint32_t increment_counts) {
    const int32_t count_before = read_quadrature_count(encoder);
    const uint32_t target_count =
        absolute_count(count_before) + increment_counts;
    const uint32_t start_ms = to_ms_since_boot(get_absolute_time());

    stop_target_counts = target_count;
    stop_triggered = false;
    active_motor = (uint8_t)motor_number;
    stop_armed = true;
    drive_jog_motor(motor_number);

    bool pulse_reached = false;
    while (true) {
        if (stop_triggered) {
            pulse_reached = true;
            break;
        }
        const uint32_t now_ms =
            to_ms_since_boot(get_absolute_time()) - start_ms;
        if (now_ms >= JOG_PULSE_TIMEOUT_MS) {
            break;
        }
        sleep_ms(BUTTON_POLL_MS);
    }

    stop_armed = false;
    active_motor = 0;
    stop_all_motors();
    sleep_ms(BRAKE_SETTLE_MS);

    const int32_t current_count = read_quadrature_count(encoder);
    const int32_t pulse_counts = current_count - count_before;
    last_result.jog_counts =
        current_count - last_result.count_after_settle;
    last_result.jog_final_absolute_count = absolute_count(current_count);
    last_result.jog_combined_count = absolute_count(
        last_result.count_after_settle + last_result.jog_counts);
    const uint32_t movement = absolute_count(pulse_counts);
    if (!last_result.jog_movement_seen ||
        movement < last_result.smallest_jog_movement) {
        last_result.smallest_jog_movement = movement;
        last_result.jog_movement_seen = true;
    }

    if (!pulse_reached) {
        printf("WARNING: jog pulse did not reach its %lu-count target; "
               "the motor may be stalled or the duty may be too low.\n",
               (unsigned long)increment_counts);
    }
    printf("\nJog pulse target %lu counts; moved %ld counts.\n",
           (unsigned long)increment_counts, (long)pulse_counts);
    printf("  cumulative absolute count: %lu\n",
           (unsigned long)last_result.jog_final_absolute_count);
    printf("  cumulative jog counts: %ld\n", (long)last_result.jog_counts);
    printf("  combined total: %lu\n",
           (unsigned long)last_result.jog_combined_count);
    print_millicounts("  running primary counts/rev at N: ",
                      last_result.jog_combined_count, TEST_REVOLUTIONS);
    print_millicounts("  running secondary counts/rev at N+1: ",
                      last_result.jog_combined_count,
                      TEST_REVOLUTIONS + 1u);

    telemetry_motor_sample_t sample = {0};
    sample.test_type = "rotation_drift";
    sample.state = "jogging";
    sample.direction = "forward";
    sample.has_motor_number = true;
    sample.motor_number = motor_number;
    sample.has_duty_percent = true;
    sample.duty_percent = JOG_DUTY_PERCENT;
    sample.has_absolute_count = true;
    sample.absolute_count = last_result.jog_final_absolute_count;
    sample.has_jog_counts = true;
    sample.jog_counts = last_result.jog_counts;
    sample.has_jog_final_absolute_count = true;
    sample.jog_final_absolute_count = last_result.jog_final_absolute_count;
    sample.has_jog_combined_count = true;
    sample.jog_combined_count = last_result.jog_combined_count;
    sample.has_smallest_jog_movement = true;
    sample.smallest_jog_movement = last_result.smallest_jog_movement;
    sample.has_alignment_resolution_cpr_milli = true;
    sample.alignment_resolution_cpr_milli =
        (last_result.smallest_jog_movement * 1000u) / TEST_REVOLUTIONS;
    sample.has_jog_primary_cpr_milli = true;
    sample.jog_primary_cpr_milli =
        (last_result.jog_combined_count * 1000u) / TEST_REVOLUTIONS;
    sample.has_jog_secondary_cpr_milli = true;
    sample.jog_secondary_cpr_milli =
        (last_result.jog_combined_count * 1000u) /
        (TEST_REVOLUTIONS + 1u);
    sample.has_completed = true;
    sample.completed = false;
    send_motor_telemetry(&sample);
}

static void run_jog_mode(unsigned int motor_number,
                         encoder_state_t *encoder,
                         uint jog_button_pin,
                         uint fine_button_pin) {
    printf("\nForward jog alignment mode\n");
    printf("Align the wheel-rim mark with the fixed chassis mark.\n");
    printf("Tap GP%u for a %u-count coarse jog pulse; it fires on release.\n",
           jog_button_pin, JOG_COARSE_COUNTS);
    printf("Tap GP%u for a %u-count fine jog pulse; it fires on release.\n",
           fine_button_pin, JOG_FINE_COUNTS);
    printf("Jog forward only. Holding both buttons finishes without moving.\n");

    while (!gpio_get(jog_button_pin) || !gpio_get(fine_button_pin)) {
        sleep_ms(BUTTON_POLL_MS);
    }

    uint8_t held_button = 0;
    bool exit_requested = false;
    bool wait_for_release = false;
    while (true) {
        const bool jog_pressed = !gpio_get(jog_button_pin);
        const bool fine_pressed = !gpio_get(fine_button_pin);

        if (wait_for_release) {
            if (jog_pressed && fine_pressed) {
                exit_requested = true;
            }
            if (!jog_pressed && !fine_pressed) {
                wait_for_release = false;
                if (exit_requested) {
                    last_result.jog_complete = true;
                    printf("\nJog alignment complete.\n");
                    print_result(&last_result);
                    return;
                }
                exit_requested = false;
            }
            sleep_ms(BUTTON_POLL_MS);
            continue;
        }

        if (held_button == 0) {
            if (jog_pressed && fine_pressed) {
                held_button = 3;
                exit_requested = true;
            } else if (jog_pressed) {
                held_button = 1;
            } else if (fine_pressed) {
                held_button = 2;
            }
        } else if (jog_pressed && fine_pressed) {
            exit_requested = true;
        }

        if (held_button != 0 && !jog_pressed && !fine_pressed) {
            if (exit_requested || held_button == 3) {
                last_result.jog_complete = true;
                printf("\nJog alignment complete.\n");
                print_result(&last_result);
                return;
            }

            const uint32_t increment_counts =
                held_button == 1 ? JOG_COARSE_COUNTS : JOG_FINE_COUNTS;
            perform_jog_pulse(motor_number, encoder, increment_counts);
            held_button = 0;
            exit_requested = false;
            wait_for_release = true;
        }

        sleep_ms(BUTTON_POLL_MS);
    }
}

static void initialise_button(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

int main(void) {
    stdio_init_all();

    initialise_motor_pwm(MOTOR_1_PWM_PIN, &motor_1_slice);
    initialise_brake_pin(MOTOR_1_BRAKE_PIN);
    gpio_set_function(MOTOR_1_BRAKE_PIN, GPIO_FUNC_PWM);
    pwm_set_gpio_level(MOTOR_1_BRAKE_PIN, 0);
    initialise_motor_pwm(MOTOR_2_PWM_PIN, &motor_2_slice);
    initialise_brake_pin(MOTOR_2_BRAKE_PIN);
    gpio_set_function(MOTOR_2_BRAKE_PIN, GPIO_FUNC_PWM);
    pwm_set_gpio_level(MOTOR_2_BRAKE_PIN, 0);
    stop_all_motors();

    initialise_encoder(&encoder_1, true);
    initialise_encoder(&encoder_2, false);
    initialise_button(MOTOR_1_BUTTON_PIN);
    initialise_button(MOTOR_2_BUTTON_PIN);

    sleep_ms(1000);
    printf("\n%u-revolution continuous motor drift test\n",
           TEST_REVOLUTIONS);
    printf("WARNING: power motors from VIN and never from Raspberry Pi 400 USB.\n");
    printf("The shared 1.2 A USB limit caused a previous test to over-current "
           "and drop the /srv SSD.\n");
    printf("Before starting, align a wheel-rim mark with a fixed chassis "
           "mark. Lift the wheel; keep hands and cables clear.\n");
    printf("Tap GP20 to run motor 1 only; GP21 is its emergency stop.\n");
    printf("Tap GP21 to run motor 2 only; GP20 is its emergency stop.\n");
    printf("Each run is %u continuous forward revolutions at %u%% duty.\n",
           TEST_REVOLUTIONS, PWM_DUTY_PERCENT);
    printf("Ideal counts: %u; target counts: %u; timeout: %u ms.\n",
           IDEAL_COUNTS, TARGET_COUNTS, TEST_TIMEOUT_MS);
    printf("Deliberate undershoot: %u counts; jog forward to alignment.\n",
           UNDERSHOOT_COUNTS);

    telemetry_enabled = telemetry_init("pico-w-rotation-drift");
    printf("Telemetry startup: %s.\n",
           telemetry_enabled ? "enabled" : "unavailable; continuing without it");

    bool previous_motor_1_pressed = false;
    bool previous_motor_2_pressed = false;
    absolute_time_t next_result_report = make_timeout_time_ms(2000);
    absolute_time_t next_split_report = make_timeout_time_ms(10000);

    while (true) {
        const bool motor_1_pressed = !gpio_get(MOTOR_1_BUTTON_PIN);
        const bool motor_2_pressed = !gpio_get(MOTOR_2_BUTTON_PIN);

        if (motor_1_pressed && !previous_motor_1_pressed) {
            run_motor_test(1, &encoder_1, MOTOR_2_BUTTON_PIN);
            send_completed_telemetry(true);
            if (last_result.completed && !last_result.stalled) {
                run_jog_mode(1, &encoder_1,
                             MOTOR_1_BUTTON_PIN, MOTOR_2_BUTTON_PIN);
                send_completed_telemetry(true);
            }
            printf("Tap GP20 or GP21 to run another test.\n");
            next_result_report = make_timeout_time_ms(2000);
            next_split_report = make_timeout_time_ms(10000);
            while (!gpio_get(MOTOR_1_BUTTON_PIN) ||
                   !gpio_get(MOTOR_2_BUTTON_PIN)) {
                sleep_ms(BUTTON_POLL_MS);
            }
            previous_motor_1_pressed = false;
            previous_motor_2_pressed = false;
            continue;
        }

        if (motor_2_pressed && !previous_motor_2_pressed) {
            run_motor_test(2, &encoder_2, MOTOR_1_BUTTON_PIN);
            send_completed_telemetry(true);
            if (last_result.completed && !last_result.stalled) {
                run_jog_mode(2, &encoder_2,
                             MOTOR_2_BUTTON_PIN, MOTOR_1_BUTTON_PIN);
                send_completed_telemetry(true);
            }
            printf("Tap GP20 or GP21 to run another test.\n");
            next_result_report = make_timeout_time_ms(2000);
            next_split_report = make_timeout_time_ms(10000);
            while (!gpio_get(MOTOR_1_BUTTON_PIN) ||
                   !gpio_get(MOTOR_2_BUTTON_PIN)) {
                sleep_ms(BUTTON_POLL_MS);
            }
            previous_motor_1_pressed = false;
            previous_motor_2_pressed = false;
            continue;
        }

        if (last_result.valid && time_reached(next_result_report)) {
            print_saved_result();
            send_completed_telemetry(false);
            next_result_report = make_timeout_time_ms(2000);
        }
        if (last_result.valid && time_reached(next_split_report)) {
            /* Summary repeats every 2 s; split packets repeat every 10 s. */
            send_completed_telemetry(true);
            next_split_report = make_timeout_time_ms(10000);
        }

        previous_motor_1_pressed = motor_1_pressed;
        previous_motor_2_pressed = motor_2_pressed;
        sleep_ms(BUTTON_POLL_MS);
    }
}

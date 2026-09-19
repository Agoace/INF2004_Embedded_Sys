#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1
#define ULTRASONIC_ECHO_PIN 2
#define ULTRASONIC_TRIGGER_PIN 3
#define ENCODER_1_A_PIN 4
#define ENCODER_1_B_PIN 5
#define GY511_DRDY_PIN 6
#define SERVO_PIN 15
#define ENCODER_2_A_PIN 16
#define ENCODER_2_B_PIN 17
#define IR_ADC_PIN 26
#define IR_ADC_INPUT 0

#define I2C_BAUD_HZ (100 * 1000)
#define REPORT_INTERVAL_MS 1000
#define ULTRASONIC_TIMEOUT_US 30000

static volatile uint32_t encoder_1_a_edges;
static volatile uint32_t encoder_1_b_edges;
static volatile uint32_t encoder_2_a_edges;
static volatile uint32_t encoder_2_b_edges;
static volatile uint32_t drdy_edges;

static void gpio_edge_callback(uint gpio, uint32_t events) {
    (void)events;

    switch (gpio) {
        case ENCODER_1_A_PIN:
            ++encoder_1_a_edges;
            break;
        case ENCODER_1_B_PIN:
            ++encoder_1_b_edges;
            break;
        case ENCODER_2_A_PIN:
            ++encoder_2_a_edges;
            break;
        case ENCODER_2_B_PIN:
            ++encoder_2_b_edges;
            break;
        case GY511_DRDY_PIN:
            ++drdy_edges;
            break;
        default:
            break;
    }
}

static void initialise_edge_input(uint pin, bool install_callback) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);

    const uint32_t events = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    if (install_callback) {
        gpio_set_irq_enabled_with_callback(pin, events, true, gpio_edge_callback);
    } else {
        gpio_set_irq_enabled(pin, events, true);
    }
}

static bool i2c_address_is_reserved(uint8_t address) {
    return (address & 0x78u) == 0u || (address & 0x78u) == 0x78u;
}

static void scan_i2c_bus(void) {
    unsigned int devices = 0;

    printf("I2C0 scan on GP0=SDA, GP1=SCL:\n");
    for (uint8_t address = 0; address < 0x80; ++address) {
        if (i2c_address_is_reserved(address)) {
            continue;
        }

        uint8_t data;
        const int result = i2c_read_timeout_us(
            i2c0, address, &data, 1, false, 2000);
        if (result >= 0) {
            printf("  device acknowledged at 0x%02X", address);
            if (address == 0x19) {
                printf(" (common GY-511 accelerometer address)");
            } else if (address == 0x1E) {
                printf(" (common GY-511 magnetometer address)");
            }
            printf("\n");
            ++devices;
        }
    }

    if (devices == 0) {
        printf("  no devices found; check SDA/SCL, power, and ground\n");
    }
}

static bool wait_for_gpio_level(uint pin, bool level, uint32_t timeout_us) {
    const absolute_time_t deadline = make_timeout_time_us(timeout_us);
    while (gpio_get(pin) != level) {
        if (time_reached(deadline)) {
            return false;
        }
        tight_loop_contents();
    }
    return true;
}

static int32_t measure_ultrasonic_distance_mm(void) {
    if (!wait_for_gpio_level(ULTRASONIC_ECHO_PIN, false,
                             ULTRASONIC_TIMEOUT_US)) {
        return -2;
    }

    gpio_put(ULTRASONIC_TRIGGER_PIN, true);
    sleep_us(10);
    gpio_put(ULTRASONIC_TRIGGER_PIN, false);

    if (!wait_for_gpio_level(ULTRASONIC_ECHO_PIN, true,
                             ULTRASONIC_TIMEOUT_US)) {
        return -1;
    }

    const uint64_t rise_time = time_us_64();
    if (!wait_for_gpio_level(ULTRASONIC_ECHO_PIN, false,
                             ULTRASONIC_TIMEOUT_US)) {
        return -2;
    }

    const uint64_t pulse_us = time_us_64() - rise_time;
    return (int32_t)((pulse_us * 343u) / 2000u);
}

static uint16_t read_ir_millivolts(void) {
    uint32_t total = 0;
    adc_select_input(IR_ADC_INPUT);
    for (unsigned int sample = 0; sample < 16; ++sample) {
        total += adc_read();
        sleep_us(20);
    }

    const uint32_t average = total / 16u;
    return (uint16_t)((average * 3300u) / 4095u);
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    i2c_init(i2c0, I2C_BAUD_HZ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    gpio_init(ULTRASONIC_TRIGGER_PIN);
    gpio_set_dir(ULTRASONIC_TRIGGER_PIN, GPIO_OUT);
    gpio_put(ULTRASONIC_TRIGGER_PIN, false);
    gpio_init(ULTRASONIC_ECHO_PIN);
    gpio_set_dir(ULTRASONIC_ECHO_PIN, GPIO_IN);
    gpio_disable_pulls(ULTRASONIC_ECHO_PIN);

    initialise_edge_input(ENCODER_1_A_PIN, true);
    initialise_edge_input(ENCODER_1_B_PIN, false);
    initialise_edge_input(GY511_DRDY_PIN, false);
    initialise_edge_input(ENCODER_2_A_PIN, false);
    initialise_edge_input(ENCODER_2_B_PIN, false);

    adc_init();
    adc_gpio_init(IR_ADC_PIN);

    printf("\nRobo Pico connection diagnostics\n");
    printf("GP15 servo is not driven to prevent unexpected movement.\n");
    printf("WARNING: GP2 must not receive more than 3.3 V.\n\n");
    scan_i2c_bus();

    uint32_t previous_encoder_1_a = 0;
    uint32_t previous_encoder_1_b = 0;
    uint32_t previous_encoder_2_a = 0;
    uint32_t previous_encoder_2_b = 0;
    uint32_t previous_drdy = 0;

    while (true) {
        const uint32_t current_encoder_1_a = encoder_1_a_edges;
        const uint32_t current_encoder_1_b = encoder_1_b_edges;
        const uint32_t current_encoder_2_a = encoder_2_a_edges;
        const uint32_t current_encoder_2_b = encoder_2_b_edges;
        const uint32_t current_drdy = drdy_edges;
        const int32_t distance_mm = measure_ultrasonic_distance_mm();
        const uint16_t ir_mv = read_ir_millivolts();

        printf("\n--- live report ---\n");
        if (distance_mm >= 0) {
            printf("HC-SR04 GP3->GP2: %ld mm\n", (long)distance_mm);
        } else if (distance_mm == -1) {
            printf("HC-SR04 GP3->GP2: no echo started\n");
        } else {
            printf("HC-SR04 GP3->GP2: echo stuck HIGH or too long\n");
        }

        printf("Motor 2 encoder GP4/GP5: levels %u/%u, edges +%lu/+%lu\n",
               gpio_get(ENCODER_1_A_PIN), gpio_get(ENCODER_1_B_PIN),
               (unsigned long)(current_encoder_1_a - previous_encoder_1_a),
               (unsigned long)(current_encoder_1_b - previous_encoder_1_b));
        printf("Motor 1 encoder GP16/GP17: levels %u/%u, edges +%lu/+%lu\n",
               gpio_get(ENCODER_2_A_PIN), gpio_get(ENCODER_2_B_PIN),
               (unsigned long)(current_encoder_2_a - previous_encoder_2_a),
               (unsigned long)(current_encoder_2_b - previous_encoder_2_b));
        printf("GY-511 DRDY GP6: level %u, edges +%lu\n",
               gpio_get(GY511_DRDY_PIN),
               (unsigned long)(current_drdy - previous_drdy));
        printf("IR sensor GP26/ADC0: %u mV\n", ir_mv);
        printf("Servo GP15: not driven\n");

        previous_encoder_1_a = current_encoder_1_a;
        previous_encoder_1_b = current_encoder_1_b;
        previous_encoder_2_a = current_encoder_2_a;
        previous_encoder_2_b = current_encoder_2_b;
        previous_drdy = current_drdy;

        sleep_ms(REPORT_INTERVAL_MS);
    }
}

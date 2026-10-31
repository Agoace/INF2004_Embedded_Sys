#include "sim_runtime.h"
#include "sim_world.h"
#include <inttypes.h>
#include <stdarg.h>
#include <stdlib.h>
#include "hardware/gpio.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

typedef struct { uint64_t us; unsigned int pin; unsigned int down; } button_t;
static button_t *buttons;
static size_t button_count, next_button;
static bool down[2];
static uint64_t now_us, end_us, quantum;
static uint64_t next_snapshot = 10000u;
uint16_t sim_floor[3];

uint64_t sim_now(void) { return now_us; }

_Noreturn void sim_error(const char *message) {
    fprintf(stderr, "sim: %s\n", message);
    exit(2);
}

static void json_string(const char *text, size_t size) {
    putchar('"');
    for (size_t i = 0; i < size; ++i) {
        const unsigned char c = (unsigned char)text[i];
        if (c == '"' || c == '\\') { putchar('\\'); putchar((int)c); }
        else if (c < 32u) { printf("\\u%04x", (unsigned int)c); }
        else { putchar((int)c); }
    }
    putchar('"');
}

static void event_args(uint64_t time_us, const char *kind, const char *fields, va_list args) {
    printf("{\"t_us\":%" PRIu64 ",\"type\":\"%s\"", time_us, kind);
    if (fields[0] != '\0') { putchar(','); vprintf(fields, args); }
    puts("}");
}
void sim_event(const char *kind, const char *fields, ...) {
    va_list args; va_start(args, fields);
    event_args(now_us, kind, fields, args);
    va_end(args);
}
void sim_event_at(uint64_t time_us, const char *kind, const char *fields, ...) {
    va_list args; va_start(args, fields);
    event_args(time_us, kind, fields, args);
    va_end(args);
}

int sim_printf(const char *format, ...) {
    va_list args, copy;
    va_start(args, format);
    va_copy(copy, args);
    const int n = vsnprintf(NULL, 0, format, copy);
    va_end(copy);
    if (n < 0) { sim_error("console formatting failed"); }
    char *text = malloc((size_t)n + 1u);
    if (text == NULL) { sim_error("console allocation failed"); }
    (void)vsnprintf(text, (size_t)n + 1u, format, args);
    va_end(args);
    printf("{\"t_us\":%" PRIu64 ",\"type\":\"text\",\"text\":", now_us);
    json_string(text, (size_t)n);
    puts("}");
    free(text);
    return n;
}

static void advance(uint64_t delta) {
    const uint64_t target = delta >= end_us - now_us ? end_us : now_us + delta;
    while (now_us < target || (next_button < button_count && buttons[next_button].us == now_us)) {
        uint64_t span = target - now_us;
        if (world_is_active() && span > 100u) { span = 100u; }
        uint64_t next = now_us + span;
        if (next_button < button_count && buttons[next_button].us < next) { next = buttons[next_button].us; }
        if (next_snapshot < next) { next = next_snapshot; }
        world_advance(now_us, next);
        now_us = next;
        while (next_button < button_count && buttons[next_button].us == now_us) {
            const button_t b = buttons[next_button++];
            down[b.pin - 20u] = b.down != 0u;
            sim_event("button", "\"pin\":%u,\"down\":%s", b.pin, b.down ? "true" : "false");
        }
        if (now_us == next_snapshot) {
            world_snapshot(now_us);
            next_snapshot += 10000u;
        }
    }
    if (now_us == end_us) {
        if (now_us % 10000u != 0u) { world_snapshot(now_us); }
        sim_event("end", "\"reason\":\"scenario_end\",\"complete\":true");
        free(buttons);
        world_destroy();
        if (fflush(stdout) != 0 || ferror(stdout)) { exit(2); }
        exit(0);
    }
}

absolute_time_t get_absolute_time(void) { advance(quantum); return (absolute_time_t){now_us}; }
uint32_t to_ms_since_boot(absolute_time_t time) { return (uint32_t)(time.us / 1000u); }
uint32_t time_us_32(void) { advance(quantum); return (uint32_t)now_us; }
bool time_reached(absolute_time_t deadline) { advance(quantum); return now_us >= deadline.us; }
absolute_time_t delayed_by_us(absolute_time_t time, uint64_t us) {
    if (us > UINT64_MAX - time.us) { sim_error("deadline overflow"); }
    return (absolute_time_t){time.us + us};
}
absolute_time_t make_timeout_time_ms(uint32_t ms) {
    return delayed_by_us(get_absolute_time(), (uint64_t)ms * 1000u);
}
void sleep_ms(uint32_t ms) { advance((uint64_t)ms * 1000u); }
void sleep_us(uint64_t us) { advance(us); }
bool stdio_init_all(void) { return true; }
bool stdio_usb_connected(void) { return false; }
static void check_pin(unsigned int pin) {
    if (pin != 20u && pin != 21u) { sim_error("unsupported GPIO pin"); }
}
void gpio_init(unsigned int pin) { check_pin(pin); }
void gpio_set_dir(unsigned int pin, bool out) {
    check_pin(pin);
    if (out) { sim_error("only input GPIO is supported"); }
}
void gpio_pull_up(unsigned int pin) { check_pin(pin); }
bool gpio_get(unsigned int pin) { check_pin(pin); return !down[pin - 20u]; }

int main(void) {
    unsigned int floor[3];
    if (scanf("%" SCNu64 " %" SCNu64 " %u %u %u %zu", &end_us, &quantum,
              &floor[0], &floor[1], &floor[2], &button_count) != 6 ||
        end_us == 0u || quantum == 0u || button_count > 100000u) {
        sim_error("invalid normalised scenario");
    }
    for (size_t i = 0; i < 3u; ++i) {
        if (floor[i] > 4095u) { sim_error("invalid floor ADC"); }
        sim_floor[i] = (uint16_t)floor[i];
    }
    buttons = calloc(button_count == 0u ? 1u : button_count, sizeof *buttons);
    if (buttons == NULL) { sim_error("button allocation failed"); }
    for (size_t i = 0; i < button_count; ++i) {
        button_t *b = &buttons[i];
        if (scanf("%" SCNu64 " %u %u", &b->us, &b->pin, &b->down) != 3 ||
            b->us >= end_us || (i != 0u && b->us < buttons[i - 1u].us) || b->down > 1u) {
            sim_error("invalid button event");
        }
        check_pin(b->pin);
    }
    world_read(stdin);
    sim_event("start", "\"milestone\":2,\"devices\":\"motion_ir_with_placeholder_ultrasonic\"");
    world_snapshot(0u);
    (void)firmware_main();
    sim_error("firmware returned before scenario end");
    return 2;
}

/*
 * CR1 F: sonar owner word. Bit 5 mirrors the existing ultrasonic fault
 * streak (>= ULTRASONIC_FAULT_STREAK) of the last ECHO result. The sonar
 * task is the single writer. DEADLINE and CANCELLED do not change it.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static void sonar_fault_read_hook(const uint32_t *sequence,
                                  unsigned int index);
#define APP_BOARD_READ_HOOK(s, i) sonar_fault_read_hook((s), (i))
#include "../app_program/subsystem5/sonar_service.c"
#undef APP_BOARD_READ_HOOK
#define main original_service_test
#include "test_sonar_service.c"
#undef main

static bool tear_next;
static void sonar_fault_read_hook(const uint32_t *sequence,
                                  unsigned int index)
{
    (void)index;
    if (tear_next) {
        tear_next = false;
        __atomic_fetch_add((uint32_t *)sequence, 2u, __ATOMIC_SEQ_CST);
    }
}

static ultrasonic_result_t echo_word(ultrasonic_status_t status,
                                     unsigned int streak, unsigned int mm)
{
    return (ultrasonic_result_t)(
        (mm & ULTRASONIC_DISTANCE_MASK) |
        ((unsigned int)status << ULTRASONIC_STATUS_SHIFT) |
        ((streak << ULTRASONIC_STREAK_SHIFT) & ULTRASONIC_STREAK_MASK));
}

static uint32_t read_word(void)
{
    uint32_t value = 0xDEADBEEFu;
    assert(sonar_fault_mask_try_read(&value));
    assert((value & ~FAULT_OWNER_SONAR) == 0u);
    return value;
}

/* Every echo word: bit 5 is set exactly when the streak is at the limit. */
static void test_all_echo_words(void)
{
    for (uint32_t e = 0u; e <= UINT16_MAX; ++e) {
        const ultrasonic_result_t r = (ultrasonic_result_t)e;
        publish(SONAR_ECHO, r, 1000u);
        const uint32_t want =
            ultrasonic_result_fault(r) ? FAULT_SONAR_SENSOR : 0u;
        assert(read_word() == want);
    }
}

/* Driver-like sequence: NO_RETURN is not a fault; no new latch. */
static void test_sequence(void)
{
    for (unsigned int s = 1u; s < ULTRASONIC_FAULT_STREAK; ++s) {
        publish(SONAR_ECHO, echo_word(ULTRASONIC_STATUS_NO_RISE, s, 0u), 0u);
        assert(read_word() == 0u);
    }
    publish(SONAR_ECHO,
            echo_word(ULTRASONIC_STATUS_NO_RISE, ULTRASONIC_FAULT_STREAK, 0u),
            0u);
    assert(read_word() == FAULT_SONAR_SENSOR);
    /* No new reading: the word keeps the last real reading. */
    publish(SONAR_DEADLINE, ULTRASONIC_RESULT_NONE, 0u);
    assert(read_word() == FAULT_SONAR_SENSOR);
    publish(SONAR_CANCELLED, ULTRASONIC_RESULT_NONE, 0u);
    assert(read_word() == FAULT_SONAR_SENSOR);
    publish(SONAR_ECHO, echo_word(ULTRASONIC_STATUS_NO_RETURN, 0u, 0u), 0u);
    assert(read_word() == 0u);
    publish(SONAR_ECHO,
            echo_word(ULTRASONIC_STATUS_STUCK_HIGH, ULTRASONIC_FAULT_STREAK,
                      0u),
            0u);
    assert(read_word() == FAULT_SONAR_SENSOR);
    publish(SONAR_ECHO, echo_word(ULTRASONIC_STATUS_VALID, 0u, 300u), 0u);
    assert(read_word() == 0u);
    publish(SONAR_CANCELLED, ULTRASONIC_RESULT_NONE, 0u);
    assert(read_word() == 0u);
}

/* A read torn by a writer fails and leaves out unchanged. */
static void test_torn_read(void)
{
    uint32_t value = 0x12345678u;

    publish(SONAR_ECHO,
            echo_word(ULTRASONIC_STATUS_NO_RISE, ULTRASONIC_FAULT_STREAK, 0u),
            0u);
    tear_next = true;
    assert(!sonar_fault_mask_try_read(&value));
    assert(!tear_next && value == 0x12345678u);
    assert(sonar_fault_mask_try_read(&value) && value == FAULT_SONAR_SENSOR);
    assert(!sonar_fault_mask_try_read(NULL));
}

/* One writer: one publish call site in the sonar service source, on a
 * file-local board. */
static void test_one_writer(void)
{
    FILE *f = fopen(SONAR_SERVICE_SOURCE, "r");
    char line[512];
    unsigned int calls = 0u;
    unsigned int boards = 0u;

    assert(f != NULL);
    while (fgets(line, sizeof(line), f) != NULL) {
        if (strstr(line, "fault_mask_publish(") != NULL) {
            ++calls;
        }
        if (strstr(line, "static fault_mask_board_t fault_board;") != NULL) {
            ++boards;
        }
    }
    (void)fclose(f);
    assert(calls == 1u && boards == 1u);
}

int main(void)
{
    uint32_t value = 0xA5A5A5A5u;

    /* Unpublished before the first ECHO: unavailable, not all-clear. */
    assert(!sonar_fault_mask_try_read(&value) && value == 0xA5A5A5A5u);
    assert(sonar_service_init() == E_OK);
    publish(SONAR_CANCELLED, ULTRASONIC_RESULT_NONE, 0u);
    assert(!sonar_fault_mask_try_read(&value) && value == 0xA5A5A5A5u);
    test_sequence();
    test_all_echo_words();
    test_torn_read();
    test_one_writer();
    assert(sizeof(fault_board) == 8u);
    return 0;
}

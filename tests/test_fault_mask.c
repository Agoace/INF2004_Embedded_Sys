/* Diagnostic owner words: no actuator input or replacement fault latch. */
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

static void read_hook(const uint32_t *sequence, unsigned int index);
#define APP_BOARD_READ_HOOK(sequence, index) read_hook(sequence, index)
#include "../app_program/platform/fault_mask.h"

static bool interrupt_read;
static void read_hook(const uint32_t *sequence, unsigned int index)
{
    (void)index;
    if (interrupt_read) {
        interrupt_read = false;
        __atomic_fetch_add((uint32_t *)sequence, 2u, __ATOMIC_SEQ_CST);
    }
}

static void test_board(void)
{
    fault_mask_board_t b = {0};
    uint32_t out = UINT32_C(0xdeadbeef);
    assert(sizeof(b) == 8u && offsetof(fault_mask_board_t, word) == 4u);
    assert(!fault_mask_try_read(&b, FAULT_OWNER_VEHICLE, &out));
    assert(out == UINT32_C(0xdeadbeef));
    for (uint32_t value = 0u; value <= FAULT_MASK_ALL; ++value) {
        assert(fault_mask_publish(&b, FAULT_MASK_ALL, value));
        assert(fault_mask_try_read(&b, FAULT_MASK_ALL, &out));
        assert(out == value);
        assert((b.word & ~FAULT_MASK_ALL) == 0u);
    }
    assert(fault_mask_publish(&b, FAULT_OWNER_VEHICLE, FAULT_IMU_SAFETY));
    const uint32_t seq = b.sequence;
    assert(!fault_mask_publish(&b, FAULT_OWNER_VEHICLE, FAULT_SONAR_SENSOR));
    assert(!fault_mask_publish(&b, FAULT_MASK_ALL, UINT32_C(0x400)));
    assert(b.sequence == seq && b.word == FAULT_IMU_SAFETY);
    assert(!fault_mask_try_read(&b, FAULT_OWNER_SONAR, &out));
    out = UINT32_C(0xdeadbeef);
    __atomic_store_n(&b.sequence, 3u, __ATOMIC_SEQ_CST);
    assert(!fault_mask_try_read(&b, FAULT_OWNER_VEHICLE, &out));
    assert(out == UINT32_C(0xdeadbeef));
    __atomic_store_n(&b.sequence, 4u, __ATOMIC_SEQ_CST);
    interrupt_read = true;
    assert(!fault_mask_try_read(&b, FAULT_OWNER_VEHICLE, &out));
    assert(out == UINT32_C(0xdeadbeef));
    __atomic_store_n(&b.sequence, UINT32_MAX - 1u, __ATOMIC_SEQ_CST);
    assert(fault_mask_publish(&b, FAULT_OWNER_VEHICLE, 0u));
    assert(b.sequence == 0u);
    assert(!fault_mask_try_read(&b, FAULT_OWNER_VEHICLE, &out));
    assert(fault_mask_publish(&b, FAULT_OWNER_VEHICLE, FAULT_IMU_SAFETY));
    assert(fault_mask_try_read(&b, FAULT_OWNER_VEHICLE, &out));
    assert(out == FAULT_IMU_SAFETY);
    assert(!fault_mask_publish(NULL, FAULT_OWNER_SONAR, 0u));
    assert(!fault_mask_publish(&b, 0u, 0u));
    assert(!fault_mask_publish(&b, UINT32_MAX, 0u));
    assert(!fault_mask_try_read(NULL, FAULT_OWNER_SONAR, &out));
    assert(!fault_mask_try_read(&b, FAULT_OWNER_SONAR, NULL));
}

static void test_combiner(void)
{
    fault_mask_snapshot_t s = {0};
    fault_mask_collect_begin(&s, 1u);
    uint32_t value = FAULT_MOTION_STALE | FAULT_MOTION_ESTOP;
    assert(fault_mask_combine(&s, FAULT_OWNER_MOTION, &value));
    value = FAULT_IMU_SAFETY;
    assert(fault_mask_combine(&s, FAULT_OWNER_VEHICLE, &value));
    assert(s.now == (FAULT_MOTION_STALE | FAULT_MOTION_ESTOP | FAULT_IMU_SAFETY));
    assert(s.run == s.now && s.epoch == 1u);
    /* Clearing a current condition never clears the run OR. */
    value = 0u;
    assert(fault_mask_combine(&s, FAULT_OWNER_MOTION, &value));
    assert(s.now == FAULT_IMU_SAFETY && (s.run & FAULT_MOTION_STALE) != 0u);
    const uint32_t run = s.run;
    fault_mask_collect_begin(&s, 1u);
    assert(s.run == run && s.valid == 0u);
    assert(!fault_mask_combine(&s, FAULT_OWNER_VEHICLE, NULL));
    assert(s.now == FAULT_IMU_SAFETY && s.run == run && s.valid == 0u);
    /* Invalid owner data is not a successful zero/current clear. */
    value = FAULT_SONAR_SENSOR;
    assert(!fault_mask_combine(&s, FAULT_OWNER_VEHICLE, &value));
    assert(s.now == FAULT_IMU_SAFETY && s.run == run && s.valid == 0u);
    fault_mask_collect_begin(&s, 2u);
    assert(s.run == 0u && s.epoch == 2u && s.valid == 0u);
    assert(s.now == FAULT_IMU_SAFETY); /* last-known, invalid until copied */
    value = FAULT_SONAR_SENSOR;
    assert(fault_mask_combine(&s, FAULT_OWNER_SONAR, &value));
    assert(s.run == FAULT_SONAR_SENSOR && s.valid == FAULT_OWNER_SONAR);
    /* Disjoint owner masks prevent a source from clearing another source. */
    value = 0u;
    assert(fault_mask_combine(&s, FAULT_OWNER_NETWORK, &value));
    assert((s.now & FAULT_SONAR_SENSOR) != 0u);
    assert(s.valid == (FAULT_OWNER_SONAR | FAULT_OWNER_NETWORK));
    fault_mask_collect_begin(&s, UINT32_MAX);
    fault_mask_collect_begin(&s, 0u);
    assert(s.run == 0u && s.epoch == 0u);
    assert(!fault_mask_combine(&s, 0u, &value));
    assert(!fault_mask_combine(&s, UINT32_MAX, &value));
}

int main(void)
{
    assert(FAULT_MOTION_STALL_LEFT == (1u << 0));
    assert(FAULT_MOTION_STALL_RIGHT == (1u << 1));
    assert(FAULT_MOTION_ESTOP == (1u << 2));
    assert(FAULT_MOTION_STALE == (1u << 3));
    assert(FAULT_IMU_SAFETY == (1u << 4));
    assert(FAULT_SONAR_SENSOR == (1u << 5));
    assert(FAULT_SHELL_OTHER == (1u << 6));
    assert(FAULT_LOG_FULL == (1u << 7));
    assert(FAULT_WIFI_LINK_DOWN == (1u << 8));
    assert(FAULT_MQTT_DISCONNECTED == (1u << 9));
    assert((FAULT_OWNER_MOTION | FAULT_OWNER_VEHICLE | FAULT_OWNER_SONAR |
            FAULT_OWNER_TELEMETRY | FAULT_OWNER_NETWORK) == FAULT_MASK_ALL);
    test_board();
    test_combiner();
    puts("test_fault_mask: PASS");
    return 0;
}

/* Exact diagnostic mirror: never a latch, clear request or control input. */
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

static bool interrupt_read;
static void fault_read_hook(const uint32_t *sequence, unsigned int index);
#define APP_BOARD_READ_HOOK(sequence, index) fault_read_hook(sequence, index)
#include "../app_program/vehicle/vehicle_fault.h"

static fault_mask_board_t *shared_board;
static uint32_t replacement;
static void fault_read_hook(const uint32_t *sequence, unsigned int index)
{
    assert(index == 0u);
    if (interrupt_read) {
        interrupt_read = false;
        assert(sequence == &shared_board->sequence);
        assert(fault_mask_publish(shared_board, FAULT_OWNER_VEHICLE, replacement));
    }
}

static void board_reads(void)
{
    fault_mask_board_t board = {0};
    shared_board = &board;
    uint32_t cached = UINT32_C(0xdeadbeef);
    assert(!fault_mask_try_read(&board, FAULT_OWNER_VEHICLE, &cached));
    assert(cached == UINT32_C(0xdeadbeef));
    for (unsigned int flags = 0u; flags < 4u; ++flags) {
        const uint32_t value = ((flags & 1u) ? FAULT_IMU_SAFETY : 0u) |
                               ((flags & 2u) ? FAULT_SHELL_OTHER : 0u);
        assert(fault_mask_publish(&board, FAULT_OWNER_VEHICLE, value));
        replacement = value ^ FAULT_OWNER_VEHICLE;
        interrupt_read = true;
        assert(!fault_mask_try_read(&board, FAULT_OWNER_VEHICLE, &cached));
        assert(!interrupt_read && cached == UINT32_C(0xdeadbeef));
        assert(fault_mask_try_read(&board, FAULT_OWNER_VEHICLE, &cached));
        assert(cached == replacement);
        cached = UINT32_C(0xdeadbeef);
    }
    board.sequence = 3u;
    assert(!fault_mask_try_read(&board, FAULT_OWNER_VEHICLE, &cached));
    assert(cached == UINT32_C(0xdeadbeef));
}

int main(void)
{
    board_reads();
    assert(vehicle_fault_value(IMU_SAFETY_LIFT, VSM_FAULT, VSM_FAULT_IMU) ==
           FAULT_IMU_SAFETY);
    for (unsigned int latch = 0u; latch <= UINT8_MAX; ++latch)
    for (unsigned int state = 0u; state <= VSM_FAULT; ++state)
    for (unsigned int cause = 0u; cause <= UINT8_MAX; ++cause) {
        uint32_t expected = latch != IMU_SAFETY_NONE ? FAULT_IMU_SAFETY : 0u;
        if (state == VSM_FAULT && cause != VSM_FAULT_SAFETY &&
            cause != VSM_FAULT_IMU) expected |= FAULT_SHELL_OTHER;
        const uint32_t actual = vehicle_fault_value((uint8_t)latch,
            (uint8_t)state, (uint8_t)cause);
        assert(actual == expected);
        assert((actual & ~FAULT_OWNER_VEHICLE) == 0u);
    }
    /* Current values only: latch clearing must immediately clear bit 4. */
    assert(vehicle_fault_value(IMU_SAFETY_OVERTURN, VSM_FAULT,
                              VSM_FAULT_FAILED) == FAULT_OWNER_VEHICLE);
    assert(vehicle_fault_value(IMU_SAFETY_NONE, VSM_FAULT,
                              VSM_FAULT_FAILED) == FAULT_SHELL_OTHER);
    assert(vehicle_fault_value(IMU_SAFETY_NONE, VSM_IDLE,
                              VSM_FAULT_NONE) == 0u);
    return 0;
}

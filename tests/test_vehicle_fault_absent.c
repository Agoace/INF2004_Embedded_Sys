/* TEST/BENCH do not start a Vehicle owner; unavailable is not all-clear. */
#include <assert.h>
#include "../app_program/platform/fault_mask.h"

int main(void)
{
    uint32_t value = UINT32_C(0xdeadbeef);
    assert(!vehicle_fault_mask_try_read(&value));
    assert(value == UINT32_C(0xdeadbeef));
    assert(!vehicle_fault_mask_try_read(0));
    return 0;
}

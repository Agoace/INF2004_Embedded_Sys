/* Operator marker debounce; no hardware or task state. */
#include "imu_mark.h"
bool imu_mark_update(imu_mark_t *state, bool down, uint32_t now_ms)
{
    if (down != state->sample) {
        state->sample = down;
        state->changed_at = now_ms;
    }
    if ((down != state->stable) &&
        ((uint32_t)(now_ms - state->changed_at) >= 30u)) {
        state->stable = down;
        return down;
    }
    return false;
}

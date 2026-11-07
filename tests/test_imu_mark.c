#include "../app_program/subsystem4/imu_mark.h"
#include <assert.h>
int main(void)
{
    imu_mark_t state = {0};
    assert(!imu_mark_update(&state, true, 100u));
    assert(!imu_mark_update(&state, false, 110u)); /* bounce */
    assert(!imu_mark_update(&state, true, 120u));
    assert(!imu_mark_update(&state, true, 149u));
    assert(imu_mark_update(&state, true, 150u));
    assert(!imu_mark_update(&state, true, 5000u)); /* held */
    assert(!imu_mark_update(&state, false, 5010u));
    assert(!imu_mark_update(&state, false, 5040u)); /* rearm */
    assert(!imu_mark_update(&state, true, UINT32_MAX - 10u));
    assert(!imu_mark_update(&state, true, 18u));
    assert(imu_mark_update(&state, true, 19u)); /* wrap-safe 30 ms */
    return 0;
}

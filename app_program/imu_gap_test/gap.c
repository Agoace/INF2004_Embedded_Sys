/* IMU_GAP_TEST=1 TEST image only. Bench check for IMU FIFO option D
 * (docs/s4-imu/IMU.md): after a 10 s warm-up the IMU task sleeps ONCE for
 * 30 ms instead of waiting for its 10 ms release, so 12 or more samples
 * queue in the accel FIFO. Expected [S4_FIFO]: partial >= 1,
 * depth_max >= 12, backlog 0, reset 0. */
#include <tk/tkernel.h>
#include <stdbool.h>
#include "../platform/drv_time.h"

#define IMU_GAP_TEST_AFTER_MS 10000u
#define IMU_GAP_TEST_MS 30

ER imu_gap_test_sleep(TMO timeout)
{
    static bool done;

    if (!done && drv_time_ms() >= IMU_GAP_TEST_AFTER_MS) {
        done = true;
        const ER er = tk_dly_tsk(IMU_GAP_TEST_MS);
        /* Releases in the window queued wakeups; cancel them so the next
         * sample is the late one and missed_wakes stays unchanged. */
        (void)tk_can_wup(TSK_SELF);
        return er;
    }
    return tk_slp_tsk(timeout);
}
